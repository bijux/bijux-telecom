fn truth_guided_acquisition_request(
    config: &ReceiverPipelineConfig,
    sat_truth: &SyntheticSatelliteTruth,
) -> crate::api::core::AcqRequest {
    crate::api::core::AcqRequest {
        sat: sat_truth.sat,
        glonass_frequency_channel: sat_truth.glonass_frequency_channel,
        signal_band: sat_truth.signal_band,
        signal_code: sat_truth.signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: Some(sat_truth.doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }
}

/// Build a truth-guided acquisition table from a synthetic capture.
pub fn validate_truth_guided_acquisition_table(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    doppler_tolerance_bins: usize,
    code_phase_tolerance_samples: usize,
) -> SyntheticAcquisitionTruthTableReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let doppler_tolerance_hz = doppler_tolerance_bins as f64 * doppler_step_hz as f64;
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
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                result.carrier_hz.0,
            );
            let doppler_error_hz = (measured_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_error_bins = doppler_error_hz / doppler_step_hz as f64;
            let doppler_pass = measured_doppler_hz.is_finite()
                && doppler_error_hz <= doppler_tolerance_hz + f64::EPSILON;

            let expected_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                )
                .round() as usize;
            let measured_code_phase_samples = result.code_phase_samples;
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                measured_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let code_phase_pass = code_phase_error_samples <= code_phase_tolerance_samples;

            SyntheticAcquisitionTruthTableSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                expected_measured_doppler_hz,
                measured_doppler_hz,
                doppler_error_hz,
                doppler_error_bins,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                doppler_pass,
                code_phase_pass,
                pass: doppler_pass && code_phase_pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionTruthTableReport {
        scenario_id: truth.scenario_id.clone(),
        doppler_tolerance_bins,
        doppler_tolerance_hz,
        code_phase_tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        doppler_step_hz,
        pass,
        satellites,
    }
}

/// Measure truth-guided acquisition code-phase accuracy from a synthetic capture.
pub fn validate_truth_guided_acquisition_code_phase(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    tolerance_samples: usize,
) -> SyntheticAcquisitionCodePhaseValidationReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let mut acquisition_config = config.clone();
            acquisition_config.intermediate_freq_hz = synthetic_carrier_hz(
                truth.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                synthetic_truth_measured_doppler_hz(truth, sat_truth),
            );
            acquisition_config.acquisition_doppler_search_hz = 0;
            acquisition_config.acquisition_doppler_step_hz = 1;
            let acquisition_request =
                truth_guided_acquisition_request(&acquisition_config, sat_truth);
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                acquisition_config,
                crate::engine::runtime::ReceiverRuntime::default(),
            )
            .with_doppler(0, 1);
            let result =
                acquisition.run_fft_for_requests(&isolated_frame, &[acquisition_request]).remove(0);
            let expected_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                )
                .round() as usize;
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                result.code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted
                    | crate::api::core::AcqHypothesis::Ambiguous
            ) && code_phase_error_samples <= tolerance_samples;

            SyntheticAcquisitionCodePhaseValidationSatellite {
                sat: sat_truth.sat,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                measured_code_phase_samples: result.code_phase_samples,
                code_phase_error_samples,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCodePhaseValidationReport {
        scenario_id: truth.scenario_id.clone(),
        tolerance_samples,
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        pass,
        satellites,
    }
}

/// Measure whether acquisition code-phase refinement improves pseudorange initialization.
pub fn validate_truth_guided_acquisition_code_phase_refinement(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticAcquisitionCodePhaseRefinementReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let satellites = truth
        .satellites
        .iter()
        .map(|sat_truth| {
            let isolated_frame = regenerate_isolated_scaled_satellite_signal_only_frame(
                config, frame, truth, sat_truth,
            );
            let mut acquisition_config = config.clone();
            acquisition_config.intermediate_freq_hz = synthetic_carrier_hz(
                truth.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                synthetic_truth_measured_doppler_hz(truth, sat_truth),
            );
            acquisition_config.acquisition_doppler_search_hz = 0;
            acquisition_config.acquisition_doppler_step_hz = 1;
            let acquisition_request =
                truth_guided_acquisition_request(&acquisition_config, sat_truth);
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                acquisition_config,
                crate::engine::runtime::ReceiverRuntime::default(),
            )
            .with_doppler(0, 1);
            let result =
                acquisition.run_fft_for_requests(&isolated_frame, &[acquisition_request]).remove(0);
            let expected_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                );
            let coarse_code_phase_samples = result.code_phase_samples;
            let refined_code_phase_samples = result.resolved_code_phase_samples();
            let coarse_error_samples = wrapped_code_phase_error_samples_f64(
                coarse_code_phase_samples as f64,
                expected_code_phase_samples,
                period_samples,
            );
            let refined_error_samples = wrapped_code_phase_error_samples_f64(
                refined_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let coarse_pseudorange_error_m = code_phase_error_samples_to_pseudorange_m(
                coarse_error_samples,
                config.sampling_freq_hz,
            );
            let refined_pseudorange_error_m = code_phase_error_samples_to_pseudorange_m(
                refined_error_samples,
                config.sampling_freq_hz,
            );
            let improvement_samples = coarse_error_samples - refined_error_samples;
            let improvement_m = coarse_pseudorange_error_m - refined_pseudorange_error_m;
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted
                    | crate::api::core::AcqHypothesis::Ambiguous
            ) && refined_error_samples <= coarse_error_samples + f64::EPSILON;

            SyntheticAcquisitionCodePhaseRefinementSatellite {
                sat: sat_truth.sat,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_code_phase_samples,
                coarse_code_phase_samples,
                refined_code_phase_samples,
                coarse_error_samples,
                refined_error_samples,
                coarse_pseudorange_error_m,
                refined_pseudorange_error_m,
                improvement_samples,
                improvement_m,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCodePhaseRefinementReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        pass,
        satellites,
    }
}

/// Measure whether joint acquisition refinement improves both code phase and Doppler.
pub fn validate_truth_guided_acquisition_joint_refinement(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
) -> SyntheticAcquisitionJointRefinementReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1);
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
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
            let result = acquisition.run_fft(&isolated_frame, &[sat_truth.sat]).remove(0);
            let expected_measured_doppler_hz =
                synthetic_truth_measured_doppler_hz(truth, sat_truth);
            let refined_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                config.intermediate_freq_hz,
                sat_truth.sat,
                sat_truth.signal_band,
                sat_truth.signal_code,
                sat_truth.glonass_frequency_channel,
                result.carrier_hz.0,
            );
            let (coarse_doppler_hz, doppler_refined) = result
                .doppler_refinement
                .as_ref()
                .map(|refinement| {
                    (
                        synthetic_measured_doppler_hz_from_carrier_hz(
                            config.intermediate_freq_hz,
                            sat_truth.sat,
                            sat_truth.signal_band,
                            sat_truth.signal_code,
                            sat_truth.glonass_frequency_channel,
                            refinement.coarse_carrier_hz.0,
                        ),
                        true,
                    )
                })
                .unwrap_or((refined_doppler_hz, false));
            let coarse_doppler_error_hz = (coarse_doppler_hz - expected_measured_doppler_hz).abs();
            let refined_doppler_error_hz =
                (refined_doppler_hz - expected_measured_doppler_hz).abs();
            let doppler_improvement_hz = coarse_doppler_error_hz - refined_doppler_error_hz;

            let expected_code_phase_samples =
                expected_truth_guided_acquisition_code_phase_samples_f64(
                    config,
                    &isolated_frame,
                    truth,
                    sat_truth.code_phase_chips,
                );
            let coarse_code_phase_samples = result.code_phase_samples;
            let refined_code_phase_samples = result.resolved_code_phase_samples();
            let coarse_code_phase_error_samples = wrapped_code_phase_error_samples_f64(
                coarse_code_phase_samples as f64,
                expected_code_phase_samples,
                period_samples,
            );
            let refined_code_phase_error_samples = wrapped_code_phase_error_samples_f64(
                refined_code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let code_phase_improvement_samples =
                coarse_code_phase_error_samples - refined_code_phase_error_samples;
            let pass = matches!(
                result.hypothesis,
                crate::api::core::AcqHypothesis::Accepted
                    | crate::api::core::AcqHypothesis::Ambiguous
            ) && doppler_refined
                && result.code_phase_refinement.is_some()
                && refined_doppler_error_hz + f64::EPSILON < coarse_doppler_error_hz
                && refined_code_phase_error_samples + f64::EPSILON
                    < coarse_code_phase_error_samples;

            SyntheticAcquisitionJointRefinementSatellite {
                sat: sat_truth.sat,
                injected_doppler_hz: sat_truth.doppler_hz,
                injected_code_phase_chips: sat_truth.code_phase_chips,
                expected_measured_doppler_hz,
                expected_code_phase_samples,
                coarse_doppler_hz,
                refined_doppler_hz,
                coarse_doppler_error_hz,
                refined_doppler_error_hz,
                doppler_improvement_hz,
                coarse_code_phase_samples,
                refined_code_phase_samples,
                coarse_code_phase_error_samples,
                refined_code_phase_error_samples,
                code_phase_improvement_samples,
                doppler_step_hz,
                peak_mean_ratio: result.peak_mean_ratio,
                hypothesis: result.hypothesis.to_string(),
                pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionJointRefinementReport {
        scenario_id: truth.scenario_id.clone(),
        sample_rate_hz: truth.sample_rate_hz,
        period_samples,
        doppler_step_hz,
        pass,
        satellites,
    }
}

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
        .zip(results.into_iter())
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

/// Validate acquisition code phase and Doppler for a coherent integration profile.
pub fn validate_truth_guided_acquisition_coherent_integration(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    coherent_ms: u32,
    noncoherent: u32,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionCoherentIntegrationReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;

    let code_phase_report = validate_truth_guided_acquisition_code_phase(
        &profile_config,
        frame,
        truth,
        code_phase_tolerance_samples,
    );
    let doppler_report = validate_truth_guided_acquisition_doppler(
        &profile_config,
        frame,
        truth,
        doppler_tolerance_bins,
    );

    let satellites = code_phase_report
        .satellites
        .iter()
        .zip(doppler_report.satellites.iter())
        .map(|(code_phase_row, doppler_row)| {
            debug_assert_eq!(code_phase_row.sat.prn, doppler_row.sat.prn);
            SyntheticAcquisitionCoherentIntegrationSatellite {
                sat: code_phase_row.sat,
                coherent_ms,
                noncoherent,
                code_phase_error_samples: code_phase_row.code_phase_error_samples,
                doppler_error_hz: doppler_row.doppler_error_hz,
                doppler_error_bins: doppler_row.doppler_error_bins,
                peak_mean_ratio: code_phase_row.peak_mean_ratio,
                hypothesis: code_phase_row.hypothesis.clone(),
                pass: code_phase_row.pass && doppler_row.pass,
            }
        })
        .collect::<Vec<_>>();
    let pass = !satellites.is_empty() && satellites.iter().all(|row| row.pass);

    SyntheticAcquisitionCoherentIntegrationReport {
        scenario_id: truth.scenario_id.clone(),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_tolerance_hz: doppler_report.tolerance_hz,
        sample_rate_hz: truth.sample_rate_hz,
        pass,
        satellites,
    }
}

/// Measure low-C/N0 detection probability for an acquisition integration profile.
pub fn measure_truth_guided_acquisition_detection_probability(
    config: &ReceiverPipelineConfig,
    signal: SyntheticSignalParams,
    coherent_ms: u32,
    noncoherent: u32,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionSensitivityReport {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;
    let doppler_step_hz = profile_config.acquisition_doppler_step_hz.max(1);
    let duration_s = (coherent_ms.saturating_mul(noncoherent).max(1) as f64) / 1000.0;

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: profile_config.sampling_freq_hz,
                intermediate_freq_hz: profile_config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: *seed,
                satellites: vec![signal.clone()],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let frame = generate_l1_ca_multi(&profile_config, &scenario);
            let bundle = build_iq16_capture_bundle(
                &scenario.id,
                &scenario,
                &frame,
                "2026-07-09T00:00:00Z",
                Some("synthetic acquisition sensitivity trial".to_string()),
            );
            let scaled_frame = SamplesFrame::new(
                frame.t0,
                frame.dt_s,
                frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                profile_config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&scaled_frame, &[signal.sat]).remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                &profile_config,
                &scaled_frame,
                signal.code_phase_chips,
            );
            let period_samples = samples_per_code(
                profile_config.sampling_freq_hz,
                profile_config.code_freq_basis_hz,
                profile_config.code_length,
            );
            let code_phase_error_samples = wrapped_code_phase_error_samples(
                result.code_phase_samples,
                expected_code_phase_samples,
                period_samples,
            );
            let measured_doppler_hz = crate::pipeline::doppler::doppler_hz_from_carrier_hz(
                profile_config.intermediate_freq_hz,
                result.carrier_hz.0,
            );
            let doppler_error_bins =
                ((measured_doppler_hz - signal.doppler_hz).abs()) / doppler_step_hz as f64;
            let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);
            let detected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected)
                && code_phase_error_samples <= code_phase_tolerance_samples
                && doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON;

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                hypothesis: result.hypothesis.to_string(),
                accepted,
                detected,
                code_phase_error_samples: Some(code_phase_error_samples),
                doppler_error_bins: Some(doppler_error_bins),
                peak_mean_ratio: result.peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    synthetic_acquisition_sensitivity_report(
        scenario_id_prefix,
        signal.sat,
        Some(signal.cn0_db_hz),
        coherent_ms,
        noncoherent,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        trials,
    )
}

/// Measure acquisition detection rate across multiple C/N0, Doppler, and integration settings.
pub fn measure_truth_guided_acquisition_detection_rate(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionDetectionRateCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionDetectionRateReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let points = cases
        .iter()
        .map(|case| {
            let sensitivity = measure_truth_guided_acquisition_detection_probability(
                config,
                case.signal.clone(),
                case.coherent_ms,
                case.noncoherent,
                trial_seeds,
                &detection_rate_case_id(scenario_id_prefix, case),
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );

            SyntheticAcquisitionDetectionRatePoint {
                sat: case.signal.sat,
                cn0_db_hz: case.signal.cn0_db_hz,
                doppler_hz: case.signal.doppler_hz,
                coherent_ms: case.coherent_ms,
                noncoherent: case.noncoherent,
                trial_count: sensitivity.trial_count,
                accepted_count: sensitivity.accepted_count,
                detected_count: sensitivity.detected_count,
                acceptance_probability: sensitivity.acceptance_probability,
                detection_probability: sensitivity.detection_probability,
                mean_peak_mean_ratio: sensitivity.mean_peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionDetectionRateReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        points,
    }
}

#[derive(Debug, Clone)]
struct SyntheticAcquisitionTrialMeasurement {
    hypothesis: String,
    accepted: bool,
    detected: bool,
    code_phase_error_samples: Option<usize>,
    doppler_error_bins: Option<f64>,
    peak_mean_ratio: f32,
}

const GAUSSIAN_ONE_SIGMA_COVERAGE_RATE: f64 = 0.682_689_492_137_085_9;

fn synthetic_trial_seed(base: u64, trial_index: usize) -> u64 {
    let mixed = base ^ ((trial_index as u64).wrapping_add(0x9E37_79B9_7F4A_7C15));
    mixed.rotate_left(27).wrapping_mul(0x94D0_49BB_1331_11EB)
}

/// Measure same-band acquisition interference against isolated and thermal-noise baselines.
pub fn measure_truth_guided_acquisition_interference(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionInterferenceCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionInterferenceReport {
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1);
    let points = cases
        .iter()
        .map(|case| {
            measure_truth_guided_acquisition_interference_point(
                config,
                case,
                trial_seeds,
                &interference_case_id(scenario_id_prefix, case),
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            )
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionInterferenceReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        doppler_step_hz,
        points,
    }
}

fn measure_truth_guided_acquisition_interference_point(
    config: &ReceiverPipelineConfig,
    case: &SyntheticAcquisitionInterferenceCase,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionInterferencePoint {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = case.coherent_ms;
    profile_config.acquisition_noncoherent = case.noncoherent;
    let duration_s =
        (case.coherent_ms.saturating_mul(case.noncoherent).max(1) as f64) / 1000.0;

    let trials = trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let isolated_frame = scaled_synthetic_acquisition_frame(
                &profile_config,
                &SyntheticScenario {
                    sample_rate_hz: profile_config.sampling_freq_hz,
                    intermediate_freq_hz: profile_config.intermediate_freq_hz,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s,
                    seed: *seed,
                    satellites: vec![case.target_signal.clone()],
                    ephemerides: Vec::new(),
                    id: format!("{scenario_id}_isolated"),
                },
                "synthetic isolated acquisition interference trial",
            );
            let interfered_frame = scaled_synthetic_acquisition_frame(
                &profile_config,
                &SyntheticScenario {
                    sample_rate_hz: profile_config.sampling_freq_hz,
                    intermediate_freq_hz: profile_config.intermediate_freq_hz,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s,
                    seed: *seed,
                    satellites: std::iter::once(case.target_signal.clone())
                        .chain(case.interfering_signals.iter().cloned())
                        .collect(),
                    ephemerides: Vec::new(),
                    id: format!("{scenario_id}_interfered"),
                },
                "synthetic interfered acquisition trial",
            );
            let thermal_noise_frame = scaled_synthetic_acquisition_frame(
                &profile_config,
                &SyntheticScenario {
                    sample_rate_hz: profile_config.sampling_freq_hz,
                    intermediate_freq_hz: profile_config.intermediate_freq_hz,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s,
                    seed: *seed,
                    satellites: Vec::new(),
                    ephemerides: Vec::new(),
                    id: format!("{scenario_id}_thermal_noise"),
                },
                "synthetic thermal-noise acquisition interference trial",
            );
            let interference_only_frame = scaled_synthetic_acquisition_frame(
                &profile_config,
                &SyntheticScenario {
                    sample_rate_hz: profile_config.sampling_freq_hz,
                    intermediate_freq_hz: profile_config.intermediate_freq_hz,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s,
                    seed: *seed,
                    satellites: case.interfering_signals.clone(),
                    ephemerides: Vec::new(),
                    id: format!("{scenario_id}_interference_only"),
                },
                "synthetic interference-only acquisition trial",
            );

            let isolated = measure_truth_guided_acquisition_trial(
                &profile_config,
                &isolated_frame,
                &case.target_signal,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );
            let interfered = measure_truth_guided_acquisition_trial(
                &profile_config,
                &interfered_frame,
                &case.target_signal,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );
            let thermal_noise =
                measure_target_absent_acquisition_trial(&profile_config, &thermal_noise_frame, &case.target_signal);
            let interference_only = measure_target_absent_acquisition_trial(
                &profile_config,
                &interference_only_frame,
                &case.target_signal,
            );

            let failure_class = if interfered.detected {
                SyntheticAcquisitionInterferenceFailureClass::Detected
            } else if isolated.detected {
                SyntheticAcquisitionInterferenceFailureClass::CrossSignalInterference
            } else {
                SyntheticAcquisitionInterferenceFailureClass::ThermalNoiseLimited
            };
            let false_alarm_class = if thermal_noise.accepted {
                SyntheticAcquisitionFalseAlarmClass::ThermalNoise
            } else if interference_only.accepted {
                SyntheticAcquisitionFalseAlarmClass::CrossSignalInterference
            } else {
                SyntheticAcquisitionFalseAlarmClass::None
            };

            SyntheticAcquisitionInterferenceTrial {
                scenario_id,
                seed: *seed,
                isolated_detected: isolated.detected,
                interfered_detected: interfered.detected,
                failure_class,
                thermal_noise_false_alarm: thermal_noise.accepted,
                interference_only_false_alarm: interference_only.accepted,
                false_alarm_class,
                isolated_hypothesis: isolated.hypothesis,
                interfered_hypothesis: interfered.hypothesis,
                thermal_noise_hypothesis: thermal_noise.hypothesis,
                interference_only_hypothesis: interference_only.hypothesis,
                isolated_code_phase_error_samples: isolated.code_phase_error_samples,
                interfered_code_phase_error_samples: interfered.code_phase_error_samples,
                isolated_doppler_error_bins: isolated.doppler_error_bins,
                interfered_doppler_error_bins: interfered.doppler_error_bins,
                isolated_peak_mean_ratio: isolated.peak_mean_ratio,
                interfered_peak_mean_ratio: interfered.peak_mean_ratio,
                thermal_noise_peak_mean_ratio: thermal_noise.peak_mean_ratio,
                interference_only_peak_mean_ratio: interference_only.peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    let trial_count = trials.len();
    let isolated_detection_count = trials.iter().filter(|trial| trial.isolated_detected).count();
    let interfered_detection_count = trials.iter().filter(|trial| trial.interfered_detected).count();
    let thermal_noise_failure_count = trials
        .iter()
        .filter(|trial| {
            matches!(
                trial.failure_class,
                SyntheticAcquisitionInterferenceFailureClass::ThermalNoiseLimited
            )
        })
        .count();
    let cross_signal_interference_failure_count = trials
        .iter()
        .filter(|trial| {
            matches!(
                trial.failure_class,
                SyntheticAcquisitionInterferenceFailureClass::CrossSignalInterference
            )
        })
        .count();
    let thermal_noise_false_alarm_count = trials
        .iter()
        .filter(|trial| {
            matches!(
                trial.false_alarm_class,
                SyntheticAcquisitionFalseAlarmClass::ThermalNoise
            )
        })
        .count();
    let cross_signal_false_alarm_count = trials
        .iter()
        .filter(|trial| {
            matches!(
                trial.false_alarm_class,
                SyntheticAcquisitionFalseAlarmClass::CrossSignalInterference
            )
        })
        .count();
    let mean_isolated_peak_mean_ratio = mean_peak_mean_ratio(
        trials.iter().map(|trial| trial.isolated_peak_mean_ratio),
    );
    let mean_interfered_peak_mean_ratio = mean_peak_mean_ratio(
        trials.iter().map(|trial| trial.interfered_peak_mean_ratio),
    );
    let isolated_detection_probability = probability(isolated_detection_count, trial_count);
    let interfered_detection_probability = probability(interfered_detection_count, trial_count);

    SyntheticAcquisitionInterferencePoint {
        case: case.clone(),
        trial_count,
        isolated_detection_count,
        interfered_detection_count,
        thermal_noise_failure_count,
        cross_signal_interference_failure_count,
        thermal_noise_false_alarm_count,
        cross_signal_false_alarm_count,
        isolated_detection_probability,
        interfered_detection_probability,
        detection_probability_loss: isolated_detection_probability - interfered_detection_probability,
        thermal_noise_false_alarm_rate: probability(thermal_noise_false_alarm_count, trial_count),
        cross_signal_false_alarm_rate: probability(cross_signal_false_alarm_count, trial_count),
        mean_isolated_peak_mean_ratio,
        mean_interfered_peak_mean_ratio,
        trials,
    }
}

fn measure_truth_guided_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples(config, frame, signal.code_phase_chips);
    let code_phase_error_samples = wrapped_code_phase_error_samples(
        result.code_phase_samples,
        expected_code_phase_samples,
        period_samples,
    );
    let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
        config.intermediate_freq_hz,
        signal.sat,
        signal.signal_band,
        signal.signal_code,
        signal.glonass_frequency_channel,
        result.carrier_hz.0,
    );
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1) as f64;
    let doppler_error_bins = (measured_doppler_hz - signal.doppler_hz).abs() / doppler_step_hz;
    let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);
    let detected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected)
        && code_phase_error_samples <= code_phase_tolerance_samples
        && doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON;

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted,
        detected,
        code_phase_error_samples: Some(code_phase_error_samples),
        doppler_error_bins: Some(doppler_error_bins),
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

fn measure_target_absent_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    let non_rejected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected);

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted: non_rejected,
        detected: false,
        code_phase_error_samples: None,
        doppler_error_bins: None,
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

/// Measure whether reported acquisition uncertainty contains empirical error at one sigma.
pub fn measure_synthetic_acquisition_uncertainty_coverage(
    config: &ReceiverPipelineConfig,
    cases: &[SyntheticAcquisitionUncertaintyCoverageCase],
    trial_count: usize,
) -> SyntheticAcquisitionUncertaintyCoverageReport {
    let points = cases
        .iter()
        .map(|case| {
            let mut case_config = config.clone();
            case_config.acquisition_integration_ms = case.coherent_ms;
            case_config.acquisition_noncoherent = case.noncoherent;
            let period_samples = samples_per_code(
                case_config.sampling_freq_hz,
                case_config.code_freq_basis_hz,
                case_config.code_length,
            )
            .max(1);
            let trials = (0..trial_count)
                .map(|trial_index| {
                    let seed = synthetic_trial_seed(
                        case.signal.sat.prn as u64
                            ^ case.coherent_ms as u64
                            ^ ((case.noncoherent as u64) << 8)
                            ^ trial_count as u64,
                        trial_index,
                    );
                    let frame = case.doppler_rate_hz_per_s.map_or_else(
                        || generate_l1_ca(&case_config, case.signal.clone(), seed, case.duration_s),
                        |doppler_rate_hz_per_s| {
                            generate_l1_ca_with_doppler_ramp(
                                &case_config,
                                SyntheticDopplerRampParams {
                                    signal: case.signal.clone(),
                                    doppler_rate_hz_per_s,
                                },
                                seed,
                                case.duration_s,
                            )
                        },
                    );
                    let scaled_frame = scale_synthetic_capture_frame(&frame);
                    let result = acquisition_result_for_target_signal(
                        &case_config,
                        &scaled_frame,
                        &case.signal,
                    );
                    let uncertainty = result.uncertainty.as_ref();
                    let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
                        case_config.intermediate_freq_hz,
                        case.signal.sat,
                        case.signal.signal_band,
                        case.signal.signal_code,
                        case.signal.glonass_frequency_channel,
                        result.carrier_hz.0,
                    );
                    let expected_doppler_hz = case.signal.doppler_hz;
                    let doppler_error_hz = (measured_doppler_hz - expected_doppler_hz).abs();
                    let expected_code_phase_samples = expected_acquisition_code_phase_samples_f64(
                        &case_config,
                        &scaled_frame,
                        case.signal.code_phase_chips,
                    );
                    let measured_code_phase_samples = result.resolved_code_phase_samples();
                    let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
                        measured_code_phase_samples,
                        expected_code_phase_samples,
                        period_samples,
                    );
                    let expected_doppler_rate_hz_per_s = case.doppler_rate_hz_per_s;
                    let measured_doppler_rate_hz_per_s =
                        expected_doppler_rate_hz_per_s.map(|_| result.doppler_rate_hz_per_s);
                    let doppler_rate_error_hz_per_s = expected_doppler_rate_hz_per_s
                        .zip(measured_doppler_rate_hz_per_s)
                        .map(|(expected, measured)| (measured - expected).abs());

                    SyntheticAcquisitionUncertaintyCoverageTrial {
                        scenario_id: format!(
                            "synthetic_acquisition_uncertainty_{}_trial_{}",
                            case.signal.sat.prn, trial_index
                        ),
                        seed,
                        hypothesis: result.hypothesis.to_string(),
                        measured_doppler_hz,
                        expected_doppler_hz,
                        doppler_error_hz,
                        reported_doppler_sigma_hz: uncertainty.map(|u| u.doppler_hz),
                        doppler_within_one_sigma: uncertainty
                            .map(|u| doppler_error_hz <= u.doppler_hz + f64::EPSILON),
                        measured_code_phase_samples,
                        expected_code_phase_samples,
                        code_phase_error_samples,
                        reported_code_phase_sigma_samples: uncertainty
                            .map(|u| u.code_phase_samples),
                        code_phase_within_one_sigma: uncertainty.map(|u| {
                            code_phase_error_samples <= u.code_phase_samples + f64::EPSILON
                        }),
                        measured_doppler_rate_hz_per_s,
                        expected_doppler_rate_hz_per_s,
                        doppler_rate_error_hz_per_s,
                        reported_doppler_rate_sigma_hz_per_s: uncertainty
                            .and_then(|u| u.doppler_rate_hz_per_s),
                        doppler_rate_within_one_sigma: uncertainty
                            .and_then(|u| u.doppler_rate_hz_per_s)
                            .zip(doppler_rate_error_hz_per_s)
                            .map(|(sigma, error)| error <= sigma + f64::EPSILON),
                    }
                })
                .collect::<Vec<_>>();
            let successful_trial_count =
                trials.iter().filter(|trial| trial.reported_doppler_sigma_hz.is_some()).count();
            let doppler_within_one_sigma_count = trials
                .iter()
                .filter(|trial| trial.doppler_within_one_sigma == Some(true))
                .count();
            let code_phase_within_one_sigma_count = trials
                .iter()
                .filter(|trial| trial.code_phase_within_one_sigma == Some(true))
                .count();
            let doppler_rate_within_one_sigma_count = case.doppler_rate_hz_per_s.map(|_| {
                trials
                    .iter()
                    .filter(|trial| trial.doppler_rate_within_one_sigma == Some(true))
                    .count()
            });

            SyntheticAcquisitionUncertaintyCoveragePoint {
                case: case.clone(),
                trial_count,
                successful_trial_count,
                expected_one_sigma_rate: GAUSSIAN_ONE_SIGMA_COVERAGE_RATE,
                doppler_within_one_sigma_count,
                doppler_within_one_sigma_rate: probability(
                    doppler_within_one_sigma_count,
                    successful_trial_count,
                ),
                code_phase_within_one_sigma_count,
                code_phase_within_one_sigma_rate: probability(
                    code_phase_within_one_sigma_count,
                    successful_trial_count,
                ),
                doppler_rate_within_one_sigma_count,
                doppler_rate_within_one_sigma_rate: doppler_rate_within_one_sigma_count
                    .map(|count| probability(count, successful_trial_count)),
                trials,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionUncertaintyCoverageReport {
        scenario_id_prefix: "synthetic_acquisition_uncertainty".to_string(),
        doppler_step_hz: config.acquisition_doppler_step_hz.max(1),
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s.max(1),
        points,
    }
}

fn acquisition_result_for_target_signal(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqResult {
    let acquisition = crate::pipeline::acquisition::Acquisition::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    acquisition
        .run_fft_for_requests(frame, &[acquisition_request_for_signal(config, signal)])
        .remove(0)
}

fn acquisition_request_for_signal(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqRequest {
    crate::api::core::AcqRequest {
        sat: signal.sat,
        glonass_frequency_channel: signal.glonass_frequency_channel,
        signal_band: signal.signal_band,
        signal_code: signal.signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: Some(signal.doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }
}

fn scale_synthetic_capture_frame(frame: &SamplesFrame) -> SamplesFrame {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}

fn scaled_synthetic_acquisition_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    description: &str,
) -> SamplesFrame {
    let frame = generate_l1_ca_multi(config, scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        scenario,
        &frame,
        "2026-07-14T00:00:00Z",
        Some(description.to_string()),
    );
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    )
}

fn interference_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionInterferenceCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_band_{:?}_code_{:?}_interferers_{}_coherent_{}ms_noncoherent_{}",
        case.target_signal.sat.prn,
        case.target_signal.signal_band,
        case.target_signal.signal_code,
        case.interfering_signals.len(),
        case.coherent_ms,
        case.noncoherent,
    )
}

fn mean_peak_mean_ratio(values: impl Iterator<Item = f32>) -> f64 {
    let values = values.map(|value| value as f64).collect::<Vec<_>>();
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn probability(count: usize, total: usize) -> f64 {
    if total == 0 {
        0.0
    } else {
        count as f64 / total as f64
    }
}
