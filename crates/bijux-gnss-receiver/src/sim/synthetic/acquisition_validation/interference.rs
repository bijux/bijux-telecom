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
    let duration_s = (case.coherent_ms.saturating_mul(case.noncoherent).max(1) as f64) / 1000.0;

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
            let thermal_noise = measure_target_absent_acquisition_trial(
                &profile_config,
                &thermal_noise_frame,
                &case.target_signal,
            );
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
    let interfered_detection_count =
        trials.iter().filter(|trial| trial.interfered_detected).count();
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
            matches!(trial.false_alarm_class, SyntheticAcquisitionFalseAlarmClass::ThermalNoise)
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
    let mean_isolated_peak_mean_ratio =
        mean_peak_mean_ratio(trials.iter().map(|trial| trial.isolated_peak_mean_ratio));
    let mean_interfered_peak_mean_ratio =
        mean_peak_mean_ratio(trials.iter().map(|trial| trial.interfered_peak_mean_ratio));
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
        detection_probability_loss: isolated_detection_probability
            - interfered_detection_probability,
        thermal_noise_false_alarm_rate: probability(thermal_noise_false_alarm_count, trial_count),
        cross_signal_false_alarm_rate: probability(cross_signal_false_alarm_count, trial_count),
        mean_isolated_peak_mean_ratio,
        mean_interfered_peak_mean_ratio,
        trials,
    }
}
