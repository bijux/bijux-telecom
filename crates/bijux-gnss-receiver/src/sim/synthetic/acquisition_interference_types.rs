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
