include!("acquisition_validation/truth_table.rs");

include!("acquisition_validation/code_phase_refinement.rs");

include!("acquisition_validation/frequency_assistance.rs");

include!("acquisition_validation/detection_rate.rs");

include!("acquisition_validation/operating_envelope.rs");

include!("acquisition_validation/operating_envelope_support.rs");

include!("acquisition_validation/acquisition_trials.rs");

include!("acquisition_validation/acquisition_execution.rs");

include!("acquisition_validation/interference.rs");

include!("acquisition_validation/uncertainty_coverage.rs");

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
