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
    const REFINEMENT_REGRESSION_SLACK_SAMPLES: f64 = 0.05;

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
            ) && refined_error_samples
                <= coarse_error_samples + REFINEMENT_REGRESSION_SLACK_SAMPLES;

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
