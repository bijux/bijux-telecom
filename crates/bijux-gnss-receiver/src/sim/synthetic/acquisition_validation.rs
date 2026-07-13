fn truth_guided_acquisition_request(
    config: &ReceiverPipelineConfig,
    sat_truth: &SyntheticSatelliteTruth,
) -> crate::api::core::AcqRequest {
    crate::api::core::AcqRequest {
        sat: sat_truth.sat,
        glonass_frequency_channel: sat_truth.glonass_frequency_channel,
        signal_band: sat_truth.signal_band,
        signal_code: sat_truth.signal_code,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
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

            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                config,
                &isolated_frame,
                sat_truth.code_phase_chips,
            );
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
            let result = acquisition
                .run_fft_for_requests(&isolated_frame, &[acquisition_request])
                .remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples(
                config,
                &isolated_frame,
                sat_truth.code_phase_chips,
            );
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
            let result = acquisition
                .run_fft_for_requests(&isolated_frame, &[acquisition_request])
                .remove(0);
            let expected_code_phase_samples = expected_acquisition_code_phase_samples_f64(
                config,
                &isolated_frame,
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
