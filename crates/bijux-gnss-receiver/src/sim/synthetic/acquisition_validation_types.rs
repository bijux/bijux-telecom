/// Per-satellite acquisition code-phase comparison between clean synthetic truth and receiver output.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition-reported code phase sample under the receiver search convention.
    pub expected_code_phase_samples: usize,
    /// Measured acquisition-reported code phase sample.
    pub measured_code_phase_samples: usize,
    /// Wrapped absolute error between expected and measured code phase samples.
    pub code_phase_error_samples: usize,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured code phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition code-phase validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed wrapped absolute error in samples.
    pub tolerance_samples: usize,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionCodePhaseValidationSatellite>,
}

/// Per-satellite comparison between coarse acquisition code phase and refined sub-sample phase.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseRefinementSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Continuous expected acquisition-reported code phase sample.
    pub expected_code_phase_samples: f64,
    /// Coarse acquisition-reported code phase sample.
    pub coarse_code_phase_samples: usize,
    /// Refined acquisition-reported code phase sample.
    pub refined_code_phase_samples: f64,
    /// Wrapped absolute coarse error in samples.
    pub coarse_error_samples: f64,
    /// Wrapped absolute refined error in samples.
    pub refined_error_samples: f64,
    /// Pseudorange initialization error from the coarse phase, in meters.
    pub coarse_pseudorange_error_m: f64,
    /// Pseudorange initialization error from the refined phase, in meters.
    pub refined_pseudorange_error_m: f64,
    /// Improvement from refinement, in samples.
    pub improvement_samples: f64,
    /// Improvement from refinement, in meters.
    pub improvement_m: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the refined phase preserved or improved coarse pseudorange initialization.
    pub pass: bool,
}

/// Truth-guided report for acquisition code-phase refinement on a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCodePhaseRefinementReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Whether every measured satellite passed the refinement criterion.
    pub pass: bool,
    /// Per-satellite refinement rows.
    pub satellites: Vec<SyntheticAcquisitionCodePhaseRefinementSatellite>,
}

/// Per-satellite comparison between coarse acquisition bins and the joint refined estimate.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionJointRefinementSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition Doppler after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Continuous expected acquisition-reported code phase sample.
    pub expected_code_phase_samples: f64,
    /// Coarse acquisition Doppler in Hz relative to the configured IF.
    pub coarse_doppler_hz: f64,
    /// Refined acquisition Doppler in Hz relative to the configured IF.
    pub refined_doppler_hz: f64,
    /// Absolute coarse Doppler error in Hz.
    pub coarse_doppler_error_hz: f64,
    /// Absolute refined Doppler error in Hz.
    pub refined_doppler_error_hz: f64,
    /// Improvement from Doppler refinement, in Hz.
    pub doppler_improvement_hz: f64,
    /// Coarse acquisition-reported code phase sample.
    pub coarse_code_phase_samples: usize,
    /// Refined acquisition-reported code phase sample.
    pub refined_code_phase_samples: f64,
    /// Wrapped absolute coarse code-phase error in samples.
    pub coarse_code_phase_error_samples: f64,
    /// Wrapped absolute refined code-phase error in samples.
    pub refined_code_phase_error_samples: f64,
    /// Improvement from code-phase refinement, in samples.
    pub code_phase_improvement_samples: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the joint refinement improved both axes.
    pub pass: bool,
}

/// Truth-guided report for joint acquisition code-phase and Doppler refinement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionJointRefinementReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite improved both axes.
    pub pass: bool,
    /// Per-satellite refinement rows.
    pub satellites: Vec<SyntheticAcquisitionJointRefinementSatellite>,
}

/// Per-satellite acquisition Doppler comparison between clean synthetic truth and receiver output.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerValidationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected acquisition Doppler after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Doppler error expressed in acquisition bins.
    pub doppler_error_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured Doppler stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition Doppler validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed Doppler error in acquisition bins.
    pub tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the requested tolerance.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionDopplerValidationSatellite>,
}

/// Per-satellite acquisition truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected acquisition Doppler after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Doppler error expressed in acquisition bins.
    pub doppler_error_bins: f64,
    /// Injected code phase at the start of the validation frame, in chips.
    pub injected_code_phase_chips: f64,
    /// Expected acquisition-reported code phase sample under the receiver search convention.
    pub expected_code_phase_samples: usize,
    /// Measured acquisition-reported code phase sample.
    pub measured_code_phase_samples: usize,
    /// Wrapped absolute error between expected and measured code phase samples.
    pub code_phase_error_samples: usize,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured Doppler stayed within tolerance.
    pub doppler_pass: bool,
    /// Whether the measured code phase stayed within tolerance.
    pub code_phase_pass: bool,
    /// Whether both Doppler and code phase stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition truth table for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionTruthTableReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Allowed wrapped absolute code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticAcquisitionTruthTableSatellite>,
}

/// Per-satellite acquisition receiver clock-offset comparison row.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionReceiverClockOffsetSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Receiver clock frequency bias injected into the synthetic capture, in Hz.
    pub injected_receiver_clock_frequency_bias_hz: f64,
    /// Expected Doppler measurement after the common receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Measured acquisition Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Receiver clock frequency bias implied by the measured acquisition Doppler, in Hz.
    pub measured_receiver_clock_frequency_bias_hz: f64,
    /// Absolute error between measured and injected receiver clock frequency bias, in Hz.
    pub receiver_clock_frequency_bias_error_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the measured receiver clock bias stayed within tolerance.
    pub pass: bool,
}

/// Truth-guided acquisition receiver clock-offset validation report for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionReceiverClockOffsetValidationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Receiver clock frequency bias injected into the synthetic capture, in Hz.
    pub injected_receiver_clock_frequency_bias_hz: f64,
    /// Allowed receiver clock bias error in acquisition bins.
    pub tolerance_bins: usize,
    /// Allowed receiver clock bias error in Hz.
    pub tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Mean recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub mean_measured_receiver_clock_frequency_bias_hz: f64,
    /// Minimum recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub min_measured_receiver_clock_frequency_bias_hz: f64,
    /// Maximum recovered receiver clock frequency bias across all measured satellites, in Hz.
    pub max_measured_receiver_clock_frequency_bias_hz: f64,
    /// Spread between the minimum and maximum recovered receiver clock frequency bias, in Hz.
    pub measured_receiver_clock_frequency_bias_spread_hz: f64,
    /// Whether every measured satellite recovered the injected receiver clock bias consistently.
    pub pass: bool,
    /// Per-satellite comparison rows.
    pub satellites: Vec<SyntheticAcquisitionReceiverClockOffsetSatellite>,
}

/// Per-satellite acquisition validation row for a specific coherent integration profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCoherentIntegrationSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Code-phase error in samples.
    pub code_phase_error_samples: usize,
    /// Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Doppler error in acquisition bins.
    pub doppler_error_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether both code-phase and Doppler checks passed for this profile.
    pub pass: bool,
}

/// Truth-guided acquisition validation report for a coherent integration profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionCoherentIntegrationReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite validation rows.
    pub satellites: Vec<SyntheticAcquisitionCoherentIntegrationSatellite>,
}

/// Per-trial acquisition outcome for a synthetic sensitivity profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSensitivityTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the receiver accepted this trial.
    pub accepted: bool,
    /// Whether the selected non-rejected result stayed within truth tolerances.
    pub detected: bool,
    /// Wrapped code-phase error in samples when truth was available.
    pub code_phase_error_samples: Option<usize>,
    /// Doppler error in acquisition bins when truth was available.
    pub doppler_error_bins: Option<f64>,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
}

/// Detection-probability summary for a synthetic acquisition profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSensitivityReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Number of synthetic trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub accepted_count: usize,
    /// Number of trials that produced non-rejected results within truth tolerances.
    pub detected_count: usize,
    /// Accepted-trial probability across the measured trials.
    pub acceptance_probability: f64,
    /// Detection probability across the measured trials.
    pub detection_probability: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
    /// Per-trial acquisition rows.
    pub trials: Vec<SyntheticAcquisitionSensitivityTrial>,
}

/// Acquisition detection-rate input for one synthetic measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRateCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Detection-rate summary for one synthetic acquisition measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRatePoint {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Injected Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub accepted_count: usize,
    /// Number of trials that produced non-rejected results within truth tolerances.
    pub detected_count: usize,
    /// Accepted-trial probability across the measured trials.
    pub acceptance_probability: f64,
    /// Detection probability across the measured trials.
    pub detection_probability: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
}

/// Detection-rate report across multiple C/N0, Doppler, and integration settings.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDetectionRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionDetectionRatePoint>,
}

/// Per-trial tracking outcome for a synthetic lock-sensitivity profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Whether the receiver reached a sustained stable tracking window.
    pub stable_lock: bool,
    /// Whether the receiver explicitly refused to declare lock under weak signal conditions.
    pub refused_lock: bool,
    /// First epoch index of the sustained stable tracking window, when one exists.
    pub first_lock_epoch_index: Option<usize>,
    /// Count of epochs in the sustained stable tracking window.
    pub locked_epoch_count: usize,
    /// Final per-epoch tracking state emitted by the receiver.
    pub final_lock_state: String,
    /// Final per-epoch tracking state reason emitted by the receiver.
    pub final_lock_state_reason: Option<String>,
}

/// Lock-probability summary for one synthetic tracking profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingSensitivityReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
    /// Per-trial tracking rows.
    pub trials: Vec<SyntheticTrackingSensitivityTrial>,
}

/// Tracking lock-rate input for one synthetic measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
}

/// Lock-rate summary for one synthetic tracking measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRatePoint {
    /// Satellite identifier under test.
    pub sat: SatId,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub cn0_db_hz: f32,
    /// Duration of each synthetic tracking trial in seconds.
    pub duration_s: f64,
    /// Seeded Doppler error relative to truth, in Hz.
    pub seeded_doppler_error_hz: f64,
    /// Seeded code-phase error relative to truth, in samples.
    pub seeded_code_phase_error_samples: isize,
    /// Minimum sustained locked-epoch count required to declare stable lock.
    pub min_locked_epochs: usize,
    /// Number of synthetic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that reached a sustained stable lock window.
    pub stable_lock_count: usize,
    /// Number of trials that explicitly refused lock under weak-signal conditions.
    pub refused_lock_count: usize,
    /// Stable-lock probability across the measured trials.
    pub lock_probability: f64,
    /// Mean count of epochs inside the sustained lock window.
    pub mean_locked_epochs: f64,
}

/// Lock-rate report across multiple C/N0 tracking measurement points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingLockRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticTrackingLockRatePoint>,
}
