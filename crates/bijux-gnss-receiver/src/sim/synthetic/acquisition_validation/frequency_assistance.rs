/// Measure truth-guided acquisition Doppler accuracy from a synthetic capture.
pub fn validate_truth_guided_acquisition_doppler(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_bins: usize,
) -> SyntheticAcquisitionDopplerValidationReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let tolerance_hz = tolerance_bins as f64 * doppler_step_hz as f64;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition
                .run_fft_for_requests(
                    &isolated_frame,
                    &[truth_guided_acquisition_request(config, sat_truth)],
                )
                .remove(0);
            let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                result.carrier_hz.0,
            );
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let doppler_error_hz = (measured_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_error_bins = doppler_error_hz / doppler_step_hz as f64;
            let pass =
                measured_doppler_hz.is_finite() && doppler_error_hz <= tolerance_hz + f64::EPSILON;

            SyntheticAcquisitionDopplerValidationSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                doppler_error_hz,
                doppler_step_hz,
                doppler_error_bins,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionDopplerValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_bins,
        tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Validate acquisition receiver clock-offset recovery from a synthetic capture.
pub fn validate_truth_guided_acquisition_receiver_clock_offset(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_bins: usize,
) -> SyntheticAcquisitionReceiverClockOffsetValidationReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let tolerance_hz = tolerance_bins as f64 * doppler_step_hz as f64;
    let injected_receiver_clock_frequency_bias_hz = truth.receiver_clock_frequency_bias_hz;
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition
                .run_fft_for_requests(
                    &isolated_frame,
                    &[truth_guided_acquisition_request(config, sat_truth)],
                )
                .remove(0);
            let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                result.carrier_hz.0,
            );
            let expected_measured_doppler_hz =
                sat_truth.doppler_hz + injected_receiver_clock_frequency_bias_hz;
            let measured_receiver_clock_frequency_bias_hz =
                measured_doppler_hz - sat_truth.doppler_hz;
            let receiver_clock_frequency_bias_error_hz = (measured_receiver_clock_frequency_bias_hz
                - injected_receiver_clock_frequency_bias_hz)
                .abs();
            let pass = measured_doppler_hz.is_finite()
                && receiver_clock_frequency_bias_error_hz <= tolerance_hz + f64::EPSILON;

            SyntheticAcquisitionReceiverClockOffsetSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                injected_receiver_clock_frequency_bias_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                measured_receiver_clock_frequency_bias_hz,
                receiver_clock_frequency_bias_error_hz,
                doppler_step_hz,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let min_measured_receiver_clock_frequency_bias_hz = satellites
        .iter()
        .map(|row| row.measured_receiver_clock_frequency_bias_hz)
        .min_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(0.0);
    let max_measured_receiver_clock_frequency_bias_hz = satellites
        .iter()
        .map(|row| row.measured_receiver_clock_frequency_bias_hz)
        .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(0.0);
    let measured_receiver_clock_frequency_bias_spread_hz =
        max_measured_receiver_clock_frequency_bias_hz
            - min_measured_receiver_clock_frequency_bias_hz;
    let mean_measured_receiver_clock_frequency_bias_hz = if satellites.is_empty() {
        0.0
    } else {
        satellites.iter().map(|row| row.measured_receiver_clock_frequency_bias_hz).sum::<f64>()
            / satellites.len() as f64
    };
    let pass = !satellites.is_empty()
        && satellites.iter().all(|row| row.pass)
        && measured_receiver_clock_frequency_bias_spread_hz <= tolerance_hz + f64::EPSILON;

    SyntheticAcquisitionReceiverClockOffsetValidationReport {
        scenario_id: truth.scenario_id.clone(),
        injected_receiver_clock_frequency_bias_hz,
        tolerance_bins,
        tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        doppler_step_hz,
        mean_measured_receiver_clock_frequency_bias_hz,
        min_measured_receiver_clock_frequency_bias_hz,
        max_measured_receiver_clock_frequency_bias_hz,
        measured_receiver_clock_frequency_bias_spread_hz,
        pass,
        satellites,
    }
}

/// Validate common oscillator bias estimation and assisted follow-up acquisition from one capture.
pub fn validate_truth_guided_common_oscillator_bias_follow_up(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    requests: &[crate::api::core::AcqRequest],
    tolerance_bins: usize,
) -> SyntheticCommonOscillatorBiasFollowUpReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let tolerance_hz = tolerance_bins as f64 * doppler_step_hz as f64;
    let acquisition = crate::pipeline::acquisition::Acquisition::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    let initial_rows = acquisition.run_fft_topn_for_requests(frame, requests, 1);
    let estimate = crate::pipeline::acquisition_assistance::estimate_common_oscillator_bias(
        requests,
        &initial_rows,
    );
    let follow_up_requests = estimate
        .as_ref()
        .map(|estimate| {
            crate::pipeline::acquisition_assistance::build_common_oscillator_bias_follow_up_requests(
                requests,
                &initial_rows,
                estimate,
            )
        })
        .unwrap_or_default();
    let follow_up_rows = if follow_up_requests.is_empty() {
        Vec::new()
    } else {
        acquisition.run_fft_topn_for_requests(
            frame,
            &follow_up_requests.iter().map(|request| request.request).collect::<Vec<_>>(),
            1,
        )
    };
    let mut follow_up_primary_by_request_index = vec![None; requests.len()];
    for (follow_up_request, row) in follow_up_requests.iter().zip(follow_up_rows.into_iter()) {
        follow_up_primary_by_request_index[follow_up_request.request_index] = row.first().cloned();
    }

    let satellites = requests
        .iter()
        .enumerate()
        .map(|(request_index, request)| {
            let truth_satellite = truth.satellites.iter().find(|sat_truth| {
                sat_truth.sat == request.sat
                    && sat_truth.signal_band == request.signal_band
                    && sat_truth.signal_code == request.signal_code
                    && sat_truth.glonass_frequency_channel == request.glonass_frequency_channel
            });
            let initial_primary = initial_rows.get(request_index).and_then(|row| row.first());
            let follow_up_primary = follow_up_primary_by_request_index[request_index].as_ref();
            let follow_up_request = follow_up_requests
                .iter()
                .find(|follow_up_request| follow_up_request.request_index == request_index);
            let initial_trackable = initial_primary.is_some_and(|candidate| {
                matches!(
                    candidate.hypothesis,
                    crate::api::core::AcqHypothesis::Accepted
                        | crate::api::core::AcqHypothesis::Ambiguous
                )
            });
            let follow_up_trackable = follow_up_primary.is_some_and(|candidate| {
                matches!(
                    candidate.hypothesis,
                    crate::api::core::AcqHypothesis::Accepted
                        | crate::api::core::AcqHypothesis::Ambiguous
                )
            });

            SyntheticCommonOscillatorBiasFollowUpSatellite {
                sat: request.sat,
                expected_line_of_sight_doppler_hz: request
                    .expected_line_of_sight_doppler_hz
                    .or_else(|| truth_satellite.map(|sat_truth| sat_truth.doppler_hz)),
                initial_doppler_center_hz: request.doppler_center_hz,
                initial_doppler_search_hz: request.doppler_search_hz,
                initial_hypothesis: initial_primary
                    .map(|candidate| candidate.hypothesis.to_string())
                    .unwrap_or_else(|| "deferred".to_string()),
                initial_measured_doppler_hz: initial_primary
                    .map(|candidate| candidate.doppler_hz.0),
                follow_up_requested: follow_up_request.is_some(),
                follow_up_doppler_center_hz: follow_up_request
                    .as_ref()
                    .map(|follow_up_request| follow_up_request.request.doppler_center_hz),
                follow_up_doppler_search_hz: follow_up_request
                    .as_ref()
                    .map(|follow_up_request| follow_up_request.request.doppler_search_hz),
                follow_up_hypothesis: follow_up_primary
                    .map(|candidate| candidate.hypothesis.to_string()),
                follow_up_measured_doppler_hz: follow_up_primary
                    .map(|candidate| candidate.doppler_hz.0),
                improved: !initial_trackable && follow_up_trackable,
            }
        })
        .collect::<Vec<_>>();
    let improved_follow_up_count = satellites.iter().filter(|satellite| satellite.improved).count();
    let common_oscillator_bias_error_hz = estimate.as_ref().map(|estimate| {
        (estimate.estimated_bias_hz - truth.receiver_clock_frequency_bias_hz).abs()
    });
    let pass = common_oscillator_bias_error_hz
        .is_some_and(|error_hz| error_hz <= tolerance_hz + f64::EPSILON)
        && !follow_up_requests.is_empty()
        && improved_follow_up_count > 0;

    SyntheticCommonOscillatorBiasFollowUpReport {
        scenario_id: truth.scenario_id.clone(),
        injected_receiver_clock_frequency_bias_hz: truth.receiver_clock_frequency_bias_hz,
        tolerance_bins,
        tolerance_hz,
        doppler_step_hz,
        estimated_common_oscillator_bias_hz: estimate
            .as_ref()
            .map(|estimate| estimate.estimated_bias_hz),
        common_oscillator_bias_error_hz,
        support_count: estimate.as_ref().map_or(0, |estimate| estimate.support_count),
        support_bias_spread_hz: estimate.as_ref().map(|estimate| estimate.residual_spread_hz),
        follow_up_count: follow_up_requests.len(),
        improved_follow_up_count,
        pass,
        satellites,
    }
}

/// Validate assisted acquisition bounds reduction and safety fallback behavior from one capture.
pub fn validate_truth_guided_assisted_acquisition_bounds(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    requests: &[crate::api::core::AcqRequest],
) -> SyntheticAssistedAcquisitionBoundsReport {
    let acquisition = crate::pipeline::acquisition::Acquisition::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    let results = acquisition.run_fft_for_requests(frame, requests);
    let satellites = requests
        .iter()
        .copied()
        .zip(results)
        .map(|(request, result)| {
            let signal_code = if request.signal_code != crate::api::core::SignalCode::Unknown {
                request.signal_code
            } else {
                crate::engine::signal_selection::default_signal_code_for_band(
                    request.sat.constellation,
                    request.signal_band,
                )
            };
            let resolved_bounds = bijux_gnss_signal::api::AcquisitionSignalModel::for_sat_signal(
                request.sat,
                Some(request.signal_band),
                signal_code,
                request.glonass_frequency_channel,
            )
            .ok()
            .flatten()
            .map(|signal_model| {
                crate::pipeline::acquisition_assistance::resolve_acquisition_search_bounds(
                    config,
                    &signal_model,
                    request,
                )
            });
            let final_assumptions = result.assumptions.as_ref();
            let search_domain_reduced =
                resolved_bounds.as_ref().is_some_and(|bounds| bounds.search_domain_reduced);
            let final_code_phase_search_mode =
                final_assumptions.map(|assumptions| assumptions.code_phase_search_mode.clone());
            let assistance_fallback_triggered = request.assistance_bounds.is_some()
                && search_domain_reduced
                && final_assumptions.is_some_and(|assumptions| {
                    assumptions.assistance_bounds.is_none()
                        && assumptions.code_phase_search_mode == "full_code"
                });
            let pass = search_domain_reduced
                && matches!(
                    result.hypothesis,
                    crate::api::core::AcqHypothesis::Accepted
                        | crate::api::core::AcqHypothesis::Ambiguous
                );

            SyntheticAssistedAcquisitionBoundsSatellite {
                sat: request.sat,
                requested_doppler_search_hz: request.doppler_search_hz,
                bounded_doppler_search_hz: resolved_bounds
                    .as_ref()
                    .map(|bounds| bounds.doppler_search_hz),
                bounded_code_phase_search_bins: resolved_bounds
                    .as_ref()
                    .map(|bounds| bounds.code_phase_search_bins),
                bounded_code_phase_search_mode: resolved_bounds
                    .as_ref()
                    .map(|bounds| bounds.code_phase_search_mode.clone()),
                final_doppler_search_hz: final_assumptions
                    .map(|assumptions| assumptions.doppler_search_hz),
                final_code_phase_search_bins: final_assumptions
                    .map(|assumptions| assumptions.code_phase_search_bins),
                final_code_phase_search_mode,
                final_hypothesis: result.hypothesis.to_string(),
                assistance_fallback_triggered,
                search_domain_reduced,
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|satellite| satellite.pass);

    SyntheticAssistedAcquisitionBoundsReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: config.sampling_freq_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
        pass,
        satellites,
    }
}
