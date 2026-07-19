/// Noise-only false-alarm summary for a synthetic acquisition profile.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmReport {
    /// Scenario identifier prefix shared across the trials.
    pub scenario_id_prefix: String,
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic noise-only trials measured for this profile.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub false_alarm_count: usize,
    /// False-alarm probability across the measured trials.
    pub false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
    /// Per-trial acquisition rows.
    pub trials: Vec<SyntheticAcquisitionSensitivityTrial>,
}

/// Noise-only false-alarm measurement input for one acquisition profile.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticAcquisitionFalseAlarmRateCase {
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
}

/// Noise-only false-alarm summary for one acquisition measurement point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmRatePoint {
    /// Satellite identifier searched during the noise-only trials.
    pub sat: SatId,
    /// Coherent integration length under test, in milliseconds.
    pub coherent_ms: u32,
    /// Noncoherent integration count under test.
    pub noncoherent: u32,
    /// Number of synthetic noise-only trials measured for this point.
    pub trial_count: usize,
    /// Number of trials that produced accepted results.
    pub false_alarm_count: usize,
    /// False-alarm probability across the measured trials.
    pub false_alarm_rate: f64,
    /// Mean peak-to-mean ratio across the measured trials.
    pub mean_peak_mean_ratio: f64,
}

/// Noise-only false-alarm report across multiple acquisition profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionFalseAlarmRateReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Configured acquisition Doppler search half-width in Hz.
    pub acquisition_doppler_search_hz: i32,
    /// Effective acquisition Doppler bin width in Hz.
    pub acquisition_doppler_step_hz: i32,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticAcquisitionFalseAlarmRatePoint>,
}

/// Per-satellite comparison between coarse acquisition Doppler bins and refined Doppler estimates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerRefinementSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Coarse acquisition Doppler in Hz relative to the configured IF.
    pub coarse_doppler_hz: f64,
    /// Refined acquisition Doppler in Hz relative to the configured IF.
    pub refined_doppler_hz: f64,
    /// Absolute coarse Doppler error in Hz.
    pub coarse_error_hz: f64,
    /// Absolute refined Doppler error in Hz.
    pub refined_error_hz: f64,
    /// Improvement from refinement, in Hz.
    pub improvement_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Refined Doppler offset relative to the coarse grid, in bins.
    pub refinement_bins: f64,
    /// Peak-to-mean ratio for the selected acquisition result.
    pub peak_mean_ratio: f32,
    /// Acquisition hypothesis returned by the receiver.
    pub hypothesis: String,
    /// Whether the refined Doppler stayed measurably closer than the coarse bin.
    pub pass: bool,
}

/// Truth-guided report for acquisition Doppler refinement on a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionDopplerRefinementReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Effective acquisition Doppler bin width in Hz.
    pub doppler_step_hz: i32,
    /// Whether every measured satellite passed the refinement criterion.
    pub pass: bool,
    /// Per-satellite refinement rows.
    pub satellites: Vec<SyntheticAcquisitionDopplerRefinementSatellite>,
}

/// Validation input for one acquisition sample-rate profile.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticAcquisitionSampleRateValidationCase<'a> {
    /// Receiver configuration to validate.
    pub config: &'a ReceiverPipelineConfig,
    /// Synthetic sample frame captured at the configured rate.
    pub frame: &'a SamplesFrame,
    /// Machine-readable synthetic truth for the frame.
    pub truth: &'a SyntheticIqTruthBundle,
}

/// Per-sample-rate acquisition validation summary across code-phase and Doppler checks.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSampleRateValidationPoint {
    /// Stable scenario identifier for this sample-rate profile.
    pub scenario_id: String,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Truth-guided acquisition code-phase validation for this profile.
    pub code_phase_validation: SyntheticAcquisitionCodePhaseValidationReport,
    /// Truth-guided acquisition Doppler validation for this profile.
    pub doppler_validation: SyntheticAcquisitionDopplerValidationReport,
    /// Whether both code-phase and Doppler checks passed for this profile.
    pub pass: bool,
}

/// Aggregate acquisition validation across multiple sample-rate profiles.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionSampleRateValidationReport {
    /// Allowed code-phase error in samples for every profile.
    pub code_phase_tolerance_samples: usize,
    /// Allowed Doppler error in acquisition bins for every profile.
    pub doppler_tolerance_bins: usize,
    /// Allowed Doppler error in Hz for every profile.
    pub doppler_tolerance_hz: f64,
    /// Number of distinct sample rates covered by the report.
    pub distinct_sample_rate_count: usize,
    /// Lowest sample rate validated by this report, in Hz.
    pub min_sample_rate_hz: f64,
    /// Highest sample rate validated by this report, in Hz.
    pub max_sample_rate_hz: f64,
    /// Whether at least two distinct sample rates were validated and every profile passed.
    pub pass: bool,
    /// Per-profile validation results.
    pub points: Vec<SyntheticAcquisitionSampleRateValidationPoint>,
}
