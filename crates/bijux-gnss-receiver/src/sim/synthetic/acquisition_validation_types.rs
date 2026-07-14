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

/// Per-satellite result row for oscillator-bias-assisted acquisition follow-up.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCommonOscillatorBiasFollowUpSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Expected line-of-sight Doppler used to separate common bias from per-satellite motion.
    pub expected_line_of_sight_doppler_hz: Option<f64>,
    /// Initial request Doppler center in Hz.
    pub initial_doppler_center_hz: f64,
    /// Initial request half-search width in Hz.
    pub initial_doppler_search_hz: i32,
    /// Initial primary acquisition hypothesis.
    pub initial_hypothesis: String,
    /// Initial measured Doppler in Hz when a primary candidate exists.
    pub initial_measured_doppler_hz: Option<f64>,
    /// Whether this satellite received a follow-up request centered by the estimated common bias.
    pub follow_up_requested: bool,
    /// Follow-up Doppler center in Hz when a follow-up request was issued.
    pub follow_up_doppler_center_hz: Option<f64>,
    /// Follow-up Doppler half-search width in Hz when a follow-up request was issued.
    pub follow_up_doppler_search_hz: Option<i32>,
    /// Follow-up primary acquisition hypothesis when a follow-up request was issued.
    pub follow_up_hypothesis: Option<String>,
    /// Follow-up measured Doppler in Hz when a follow-up request was issued.
    pub follow_up_measured_doppler_hz: Option<f64>,
    /// Whether the follow-up improved this satellite from unacquired to acquired.
    pub improved: bool,
}

/// Truth-guided report for common oscillator bias estimation and assisted follow-up acquisition.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticCommonOscillatorBiasFollowUpReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Receiver clock frequency bias injected into the synthetic capture, in Hz.
    pub injected_receiver_clock_frequency_bias_hz: f64,
    /// Allowed common-bias estimation error in acquisition bins.
    pub tolerance_bins: usize,
    /// Allowed common-bias estimation error in Hz.
    pub tolerance_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Estimated common oscillator bias in Hz when estimation succeeded.
    pub estimated_common_oscillator_bias_hz: Option<f64>,
    /// Absolute error between the estimated and injected common bias, in Hz.
    pub common_oscillator_bias_error_hz: Option<f64>,
    /// Number of supporting satellites used for the estimate.
    pub support_count: usize,
    /// Spread across the supporting per-satellite implied biases, in Hz.
    pub support_bias_spread_hz: Option<f64>,
    /// Number of satellites that received assisted follow-up requests.
    pub follow_up_count: usize,
    /// Number of satellites improved from unacquired to acquired by the follow-up pass.
    pub improved_follow_up_count: usize,
    /// Whether the estimate matched truth and at least one narrowed follow-up search improved.
    pub pass: bool,
    /// Per-satellite validation rows.
    pub satellites: Vec<SyntheticCommonOscillatorBiasFollowUpSatellite>,
}

/// Per-satellite result row for truth-guided assisted acquisition bounds.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAssistedAcquisitionBoundsSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Requested half-search width in Hz before assistance-derived narrowing.
    pub requested_doppler_search_hz: i32,
    /// Assisted half-search width derived from uncertainty bounds, in Hz.
    pub bounded_doppler_search_hz: Option<i32>,
    /// Assisted code-phase search bins derived from uncertainty bounds.
    pub bounded_code_phase_search_bins: Option<usize>,
    /// Assisted code-phase search mode derived from uncertainty bounds.
    pub bounded_code_phase_search_mode: Option<String>,
    /// Final executed half-search width reported by acquisition, in Hz.
    pub final_doppler_search_hz: Option<i32>,
    /// Final executed code-phase search bins reported by acquisition.
    pub final_code_phase_search_bins: Option<usize>,
    /// Final executed code-phase search mode reported by acquisition.
    pub final_code_phase_search_mode: Option<String>,
    /// Final acquisition hypothesis after any safety fallback.
    pub final_hypothesis: String,
    /// Whether the result widened back to the full request after assisted bounds proved unsafe.
    pub assistance_fallback_triggered: bool,
    /// Whether the assistance math reduced the initial search domain.
    pub search_domain_reduced: bool,
    /// Whether the assisted or widened search still produced a trackable result.
    pub pass: bool,
}

/// Truth-guided report for assisted acquisition bounds and safety fallback behavior.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAssistedAcquisitionBoundsReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every assisted request reduced its intended search domain and still ended trackable.
    pub pass: bool,
    /// Per-satellite validation rows.
    pub satellites: Vec<SyntheticAssistedAcquisitionBoundsSatellite>,
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

/// Failure classification for a target-present acquisition trial under interference measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum SyntheticAcquisitionInterferenceFailureClass {
    /// The target signal remained detectable within the requested truth tolerances.
    Detected,
    /// The isolated baseline also failed, so the dominant limiter was thermal noise or low signal strength.
    ThermalNoiseLimited,
    /// The isolated baseline succeeded but the interfered trial failed, so cross-signal interference dominated.
    CrossSignalInterference,
}

/// False-alarm classification for a target-absent acquisition trial under interference measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum SyntheticAcquisitionFalseAlarmClass {
    /// Neither the thermal-noise baseline nor the interference-only trial produced a false alarm.
    None,
    /// The thermal-noise baseline already produced the false alarm, so interference was not required.
    ThermalNoise,
    /// Only the interference-only trial produced the false alarm, indicating cross-correlation interference.
    CrossSignalInterference,
}

/// Same-band acquisition-interference measurement input for one synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionInterferenceCase {
    /// Desired signal searched by the receiver.
    pub target_signal: SyntheticSignalParams,
    /// Additional same-band signals present during the interfered trials.
    pub interfering_signals: Vec<SyntheticSignalParams>,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Per-trial acquisition outcome for a same-band interference measurement.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionInterferenceTrial {
    /// Stable scenario identifier for this trial family.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Whether the isolated target-only baseline detected the desired signal.
    pub isolated_detected: bool,
    /// Whether the target-plus-interference trial detected the desired signal.
    pub interfered_detected: bool,
    /// Failure classification comparing isolated and interfered detection outcomes.
    pub failure_class: SyntheticAcquisitionInterferenceFailureClass,
    /// Whether the noise-only baseline produced a false alarm on the target search.
    pub thermal_noise_false_alarm: bool,
    /// Whether the interference-only trial produced a false alarm on the target search.
    pub interference_only_false_alarm: bool,
    /// False-alarm classification comparing thermal-noise and interference-only outcomes.
    pub false_alarm_class: SyntheticAcquisitionFalseAlarmClass,
    /// Hypothesis returned by the isolated target-only baseline.
    pub isolated_hypothesis: String,
    /// Hypothesis returned by the target-plus-interference trial.
    pub interfered_hypothesis: String,
    /// Hypothesis returned by the noise-only target-absent baseline.
    pub thermal_noise_hypothesis: String,
    /// Hypothesis returned by the interference-only target-absent trial.
    pub interference_only_hypothesis: String,
    /// Wrapped code-phase error in samples for the isolated target-only baseline.
    pub isolated_code_phase_error_samples: Option<usize>,
    /// Wrapped code-phase error in samples for the target-plus-interference trial.
    pub interfered_code_phase_error_samples: Option<usize>,
    /// Doppler error in acquisition bins for the isolated target-only baseline.
    pub isolated_doppler_error_bins: Option<f64>,
    /// Doppler error in acquisition bins for the target-plus-interference trial.
    pub interfered_doppler_error_bins: Option<f64>,
    /// Peak-to-mean ratio for the isolated target-only baseline.
    pub isolated_peak_mean_ratio: f32,
    /// Peak-to-mean ratio for the target-plus-interference trial.
    pub interfered_peak_mean_ratio: f32,
    /// Peak-to-mean ratio for the noise-only target-absent baseline.
    pub thermal_noise_peak_mean_ratio: f32,
    /// Peak-to-mean ratio for the interference-only target-absent trial.
    pub interference_only_peak_mean_ratio: f32,
}

/// Aggregate same-band acquisition-interference summary for one synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionInterferencePoint {
    /// Signal and interferer profile used for this point.
    pub case: SyntheticAcquisitionInterferenceCase,
    /// Number of deterministic trials measured for this point.
    pub trial_count: usize,
    /// Count of isolated target-only trials that detected the desired signal.
    pub isolated_detection_count: usize,
    /// Count of target-plus-interference trials that detected the desired signal.
    pub interfered_detection_count: usize,
    /// Count of target-present failures attributable to thermal-noise limitations.
    pub thermal_noise_failure_count: usize,
    /// Count of target-present failures attributable to cross-signal interference.
    pub cross_signal_interference_failure_count: usize,
    /// Count of target-absent false alarms already present in the thermal-noise baseline.
    pub thermal_noise_false_alarm_count: usize,
    /// Count of target-absent false alarms introduced only by the interference-only trial.
    pub cross_signal_false_alarm_count: usize,
    /// Detection probability for the isolated target-only baseline.
    pub isolated_detection_probability: f64,
    /// Detection probability for the target-plus-interference trial.
    pub interfered_detection_probability: f64,
    /// Detection-probability loss attributable to the interference environment.
    pub detection_probability_loss: f64,
    /// False-alarm rate for the thermal-noise target-absent baseline.
    pub thermal_noise_false_alarm_rate: f64,
    /// False-alarm rate introduced only by the interference-only environment.
    pub cross_signal_false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across isolated target-only trials.
    pub mean_isolated_peak_mean_ratio: f64,
    /// Mean peak-to-mean ratio across target-plus-interference trials.
    pub mean_interfered_peak_mean_ratio: f64,
    /// Per-trial classification rows.
    pub trials: Vec<SyntheticAcquisitionInterferenceTrial>,
}

/// Same-band acquisition-interference report across multiple synthetic profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionInterferenceReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Allowed code-phase error in samples.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins.
    pub doppler_tolerance_bins: usize,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionInterferencePoint>,
}

/// Synthetic acquisition-uncertainty coverage input for one signal profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageCase {
    /// Signal configuration under test.
    pub signal: SyntheticSignalParams,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Duration of each synthetic acquisition frame, in seconds.
    pub duration_s: f64,
    /// Optional linear Doppler-rate term applied during generation, in Hz/s.
    #[serde(default)]
    pub doppler_rate_hz_per_s: Option<f64>,
}

/// Per-trial acquisition uncertainty coverage row for a synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageTrial {
    /// Stable scenario identifier for this trial.
    pub scenario_id: String,
    /// Deterministic seed used for the synthetic noise realization.
    pub seed: u64,
    /// Hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Measured Doppler in Hz relative to the configured IF.
    pub measured_doppler_hz: f64,
    /// Expected Doppler in Hz relative to the configured IF.
    pub expected_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Receiver-reported Doppler one-sigma uncertainty in Hz, when available.
    pub reported_doppler_sigma_hz: Option<f64>,
    /// Whether the Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma: Option<bool>,
    /// Measured code phase in samples after refinement.
    pub measured_code_phase_samples: f64,
    /// Expected code phase in samples under the receiver search convention.
    pub expected_code_phase_samples: f64,
    /// Wrapped absolute code-phase error in samples.
    pub code_phase_error_samples: f64,
    /// Receiver-reported code-phase one-sigma uncertainty in samples, when available.
    pub reported_code_phase_sigma_samples: Option<f64>,
    /// Whether the code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma: Option<bool>,
    /// Measured Doppler rate in Hz/s, when the acquisition search exposed one.
    pub measured_doppler_rate_hz_per_s: Option<f64>,
    /// Expected Doppler rate in Hz/s, when the synthetic profile injected one.
    pub expected_doppler_rate_hz_per_s: Option<f64>,
    /// Absolute Doppler-rate error in Hz/s, when both expected and measured values exist.
    pub doppler_rate_error_hz_per_s: Option<f64>,
    /// Receiver-reported Doppler-rate one-sigma uncertainty in Hz/s, when available.
    pub reported_doppler_rate_sigma_hz_per_s: Option<f64>,
    /// Whether the Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma: Option<bool>,
}

/// Aggregate acquisition uncertainty coverage summary for one synthetic profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoveragePoint {
    /// Signal profile used for this point.
    pub case: SyntheticAcquisitionUncertaintyCoverageCase,
    /// Number of deterministic trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted acquisitions with reported uncertainty.
    pub successful_trial_count: usize,
    /// Expected one-sigma containment rate under a Gaussian local model.
    pub expected_one_sigma_rate: f64,
    /// Count of successful trials whose Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma_count: usize,
    /// Fraction of successful trials whose Doppler error stayed within one reported sigma.
    pub doppler_within_one_sigma_rate: f64,
    /// Count of successful trials whose code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma_count: usize,
    /// Fraction of successful trials whose code-phase error stayed within one reported sigma.
    pub code_phase_within_one_sigma_rate: f64,
    /// Count of successful trials whose Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma_count: Option<usize>,
    /// Fraction of successful trials whose Doppler-rate error stayed within one reported sigma.
    pub doppler_rate_within_one_sigma_rate: Option<f64>,
    /// Per-trial coverage rows.
    pub trials: Vec<SyntheticAcquisitionUncertaintyCoverageTrial>,
}

/// Acquisition uncertainty coverage report across multiple synthetic profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionUncertaintyCoverageReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Effective acquisition Doppler-rate bin width in Hz/s.
    pub doppler_rate_step_hz_per_s: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionUncertaintyCoveragePoint>,
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
