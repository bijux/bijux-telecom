fn measure_truth_guided_acquisition_sensitivity_trials(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
    receiver_clock_frequency_bias_hz: f64,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> Vec<SyntheticAcquisitionSensitivityTrial> {
    let duration_s =
        (config.acquisition_integration_ms.saturating_mul(config.acquisition_noncoherent).max(1)
            as f64)
            / 1000.0;

    trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz,
                duration_s,
                seed: *seed,
                satellites: vec![signal.clone()],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let scaled_frame = scaled_synthetic_acquisition_frame(
                config,
                &scenario,
                "synthetic acquisition sensitivity trial",
            );
            let acquisition = crate::pipeline::acquisition::Acquisition::new(
                config.clone(),
                crate::engine::runtime::ReceiverRuntime::default(),
            );
            let result = acquisition.run_fft(&scaled_frame, &[signal.sat]).remove(0);
            let trial = acquisition_trial_measurement_from_result(
                config,
                &scaled_frame,
                signal,
                result,
                signal.doppler_hz + receiver_clock_frequency_bias_hz,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                hypothesis: trial.hypothesis,
                accepted: trial.accepted,
                detected: trial.detected,
                code_phase_error_samples: trial.code_phase_error_samples,
                doppler_error_bins: trial.doppler_error_bins,
                peak_mean_ratio: trial.peak_mean_ratio,
            }
        })
        .collect()
}

fn measure_truth_guided_targeted_acquisition_sensitivity_trials(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
    receiver_clock_frequency_bias_hz: f64,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> Vec<SyntheticAcquisitionSensitivityTrial> {
    let duration_s =
        (config.acquisition_integration_ms.saturating_mul(config.acquisition_noncoherent).max(1)
            as f64)
            / 1000.0;

    trial_seeds
        .iter()
        .enumerate()
        .map(|(trial_index, seed)| {
            let scenario_id = format!("{scenario_id_prefix}_trial_{trial_index}");
            let scenario = SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz,
                duration_s,
                seed: *seed,
                satellites: vec![signal.clone()],
                ephemerides: Vec::new(),
                id: scenario_id.clone(),
            };
            let scaled_frame = scaled_synthetic_acquisition_frame(
                config,
                &scenario,
                "synthetic targeted acquisition sensitivity trial",
            );
            let result = acquisition_result_for_target_signal(config, &scaled_frame, signal);
            let trial = acquisition_trial_measurement_from_result(
                config,
                &scaled_frame,
                signal,
                result,
                signal.doppler_hz + receiver_clock_frequency_bias_hz,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );

            SyntheticAcquisitionSensitivityTrial {
                scenario_id,
                seed: *seed,
                sat: signal.sat,
                hypothesis: trial.hypothesis,
                accepted: trial.accepted,
                detected: trial.detected,
                code_phase_error_samples: trial.code_phase_error_samples,
                doppler_error_bins: trial.doppler_error_bins,
                peak_mean_ratio: trial.peak_mean_ratio,
            }
        })
        .collect()
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

#[derive(Debug, Clone, Copy)]
struct SyntheticAcquisitionFalseAlarmProfileMeasurement {
    trial_count: usize,
    false_alarm_count: usize,
    false_alarm_rate: f64,
}

#[derive(Debug, Clone)]
struct SyntheticAcquisitionOperatingEnvelopeVariant {
    axis: SyntheticAcquisitionOperatingEnvelopeAxis,
    signal: SyntheticSignalParams,
    coherent_ms: u32,
    noncoherent: u32,
    receiver_clock_frequency_bias_hz: f64,
}

const GAUSSIAN_ONE_SIGMA_COVERAGE_RATE: f64 = 0.682_689_492_137_085_9;

fn synthetic_trial_seed(base: u64, trial_index: usize) -> u64 {
    let mixed = base ^ ((trial_index as u64).wrapping_add(0x9E37_79B9_7F4A_7C15));
    mixed.rotate_left(27).wrapping_mul(0x94D0_49BB_1331_11EB)
}
