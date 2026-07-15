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
    let trials = measure_truth_guided_acquisition_sensitivity_trials(
        &profile_config,
        &signal,
        0.0,
        trial_seeds,
        scenario_id_prefix,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
    );

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
