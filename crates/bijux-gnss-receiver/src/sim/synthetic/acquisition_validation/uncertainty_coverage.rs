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
            let doppler_within_one_sigma_count =
                trials.iter().filter(|trial| trial.doppler_within_one_sigma == Some(true)).count();
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
