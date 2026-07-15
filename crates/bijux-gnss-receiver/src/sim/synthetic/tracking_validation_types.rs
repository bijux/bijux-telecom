/// Per-epoch tracking truth-table row for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableEpoch {
    /// Zero-based epoch index inside the satellite track.
    pub epoch_index: usize,
    /// Absolute sample index at the start of the epoch.
    pub sample_index: u64,
    /// Expected carrier frequency in Hz at this epoch.
    pub expected_carrier_hz: f64,
    /// Measured carrier frequency in Hz at this epoch.
    pub measured_carrier_hz: f64,
    /// Absolute carrier-frequency error in Hz.
    pub carrier_error_hz: f64,
    /// Expected Doppler in Hz at this epoch.
    pub expected_doppler_hz: f64,
    /// Measured Doppler in Hz at this epoch.
    pub measured_doppler_hz: f64,
    /// Absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// PLL discriminator residual in radians for this tracking epoch.
    #[serde(default)]
    pub pll_phase_error_rad: f64,
    /// Absolute PLL discriminator residual in cycles for this tracking epoch.
    #[serde(default)]
    pub pll_phase_error_cycles: f64,
    /// Expected code phase in samples at this epoch.
    pub expected_code_phase_samples: f64,
    /// Measured code phase in samples at this epoch.
    pub measured_code_phase_samples: f64,
    /// Wrapped absolute code-phase error in samples.
    pub code_phase_error_samples: f64,
    /// Expected C/N0 in dB-Hz at this epoch.
    pub expected_cn0_db_hz: f64,
    /// Measured C/N0 in dB-Hz at this epoch.
    pub measured_cn0_dbhz: f64,
    /// Absolute C/N0 error in dB-Hz.
    pub cn0_error_db: f64,
    /// Whether the prompt detector reported signal lock.
    pub lock: bool,
    /// Whether the PLL reported lock.
    pub pll_lock: bool,
    /// Whether the DLL reported lock.
    pub dll_lock: bool,
    /// Whether the FLL reported lock.
    pub fll_lock: bool,
    /// Whether the receiver reported a cycle slip at this epoch.
    pub cycle_slip: bool,
    /// Receiver lock-state label for this epoch.
    pub lock_state: String,
    /// Receiver lock-state reason for this epoch, when one was emitted.
    pub lock_state_reason: Option<String>,
    /// Whether this epoch belongs to a stable tracking window.
    pub stable_tracking_epoch: bool,
    /// Whether every tracked error stayed within tolerance for this stable epoch.
    pub pass: bool,
}

/// Per-satellite tracking truth table for synthetic validation.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableSatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Injected Doppler shift in Hz.
    pub injected_doppler_hz: f64,
    /// Expected measured Doppler after the receiver clock bias is applied, in Hz.
    pub expected_measured_doppler_hz: f64,
    /// Injected code phase at sample zero, in chips.
    pub injected_code_phase_chips: f64,
    /// Injected carrier-to-noise density ratio in dB-Hz.
    pub injected_cn0_db_hz: f32,
    /// Count of recorded epochs for this satellite.
    pub epoch_count: usize,
    /// Count of epochs inside stable tracking windows.
    pub stable_epoch_count: usize,
    /// First stable tracking epoch index, when one exists.
    pub first_stable_epoch_index: Option<usize>,
    /// Whether every stable tracking epoch stayed within tolerance.
    pub pass: bool,
    /// Per-epoch truth-table rows.
    pub epochs: Vec<SyntheticTrackingTruthTableEpoch>,
}

/// Truth-guided tracking truth table for a synthetic capture.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingTruthTableReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Allowed carrier-frequency error in Hz.
    pub carrier_tolerance_hz: f64,
    /// Allowed Doppler error in Hz.
    pub doppler_tolerance_hz: f64,
    /// Allowed wrapped absolute code-phase error in samples.
    pub code_phase_tolerance_samples: f64,
    /// Allowed absolute C/N0 error in dB-Hz.
    pub cn0_tolerance_db_hz: f64,
    /// Capture sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of samples in one code period at the configured rate.
    pub period_samples: usize,
    /// Output scale applied before quantization.
    pub output_scale_applied: f32,
    /// Whether every measured satellite passed the requested tolerances.
    pub pass: bool,
    /// Per-satellite truth-table rows.
    pub satellites: Vec<SyntheticTrackingTruthTableSatellite>,
}

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
