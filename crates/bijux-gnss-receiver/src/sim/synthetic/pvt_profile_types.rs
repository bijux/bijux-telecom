/// One synthetic PVT accuracy measurement point indexed by observed signal strength.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtCn0ProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of observation epochs that contributed C/N0 measurements.
    pub observation_epoch_count: usize,
    /// Mean observation C/N0 across the contributing epochs, in dB-Hz.
    pub mean_observation_cn0_dbhz: f64,
    /// Minimum per-epoch mean observation C/N0, in dB-Hz.
    pub min_observation_cn0_dbhz: f64,
    /// Maximum per-epoch mean observation C/N0, in dB-Hz.
    pub max_observation_cn0_dbhz: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Maximum PDOP across matched PVT epochs.
    pub max_pdop: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had both truth-ready PVT comparisons and observed C/N0 support.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple signal-strength points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtCn0ProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtCn0ProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by satellite geometry quality.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtGeometryProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of matched PVT epochs that contributed geometry measurements.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Mean PDOP across matched PVT epochs.
    pub mean_pdop: Option<f64>,
    /// Minimum PDOP across matched PVT epochs.
    pub min_pdop: Option<f64>,
    /// Maximum PDOP across matched PVT epochs.
    pub max_pdop: Option<f64>,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one geometry measurement.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple satellite-geometry points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtGeometryProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtGeometryProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by injected multipath severity.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMultipathProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of satellites carrying injected multipath bias.
    pub affected_satellite_count: usize,
    /// Mean absolute injected pseudorange bias across affected satellites, in meters.
    pub mean_abs_pseudorange_bias_m: f64,
    /// Maximum absolute injected pseudorange bias across affected satellites, in meters.
    pub max_abs_pseudorange_bias_m: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// Number of truth-matched epochs whose solution validity diverged.
    pub diverging_epoch_count: usize,
    /// Diverging-solution fraction across truth-matched epochs.
    pub diverging_epoch_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one matched epoch.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple synthetic multipath points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMultipathProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtMultipathProfilePoint>,
}

/// One synthetic PVT accuracy measurement point indexed by receiver-motion truth path.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMotionProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of truth epochs in the receiver-motion reference path.
    pub truth_epoch_count: usize,
    /// Whether the truth path contains measurable receiver displacement.
    pub moving: bool,
    /// Total receiver path length across the truth epochs, in meters.
    pub path_length_m: f64,
    /// Mean receiver speed across the truth epochs, in meters per second.
    pub mean_speed_mps: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// RMS 3D position error across matched PVT epochs, in meters.
    pub rms_position_error_3d_m: Option<f64>,
    /// Maximum 3D position error across matched PVT epochs, in meters.
    pub max_position_error_3d_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one matched epoch.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple receiver-motion truth paths.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtMotionProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtMotionProfilePoint>,
}

/// One truth-guided PVT accuracy measurement point indexed by injected receiver clock drift.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtClockProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Injected receiver clock drift carried by the scenario truth, in seconds per second.
    pub injected_clock_drift_s_per_s: f64,
    /// Expected observation-Doppler offset induced by the injected clock drift, in hertz.
    pub expected_observation_doppler_offset_hz: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of observation Doppler pairs compared against the stable reference observations.
    pub observation_doppler_pair_count: usize,
    /// Mean observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_mean_observation_doppler_offset_hz: Option<f64>,
    /// Minimum observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_min_observation_doppler_offset_hz: Option<f64>,
    /// Maximum observed observation-Doppler offset relative to the stable reference observations, in hertz.
    pub observed_max_observation_doppler_offset_hz: Option<f64>,
    /// Largest absolute observation-Doppler offset error relative to the expected clock-induced offset, in hertz.
    pub max_observation_doppler_offset_error_hz: Option<f64>,
    /// Final solved receiver clock drift from the navigation solution sequence, in seconds per second.
    pub final_solved_clock_drift_s_per_s: Option<f64>,
    /// Mean solved receiver clock drift across the navigation solution sequence, in seconds per second.
    pub mean_solved_clock_drift_s_per_s: Option<f64>,
    /// Largest absolute solved-drift error across the navigation solution sequence, in seconds per second.
    pub max_clock_drift_error_s_per_s: Option<f64>,
    /// RMS clock-bias error across matched PVT epochs, in meters.
    pub rms_clock_bias_error_m: Option<f64>,
    /// Maximum clock-bias error across matched PVT epochs, in meters.
    pub max_clock_bias_error_m: Option<f64>,
    /// RMS residual RMS across matched PVT epochs, in meters.
    pub rms_residual_rms_m: Option<f64>,
    /// Maximum residual RMS across matched PVT epochs, in meters.
    pub max_residual_rms_m: Option<f64>,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and at least one solved clock-drift estimate.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across multiple receiver clock-drift points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtClockProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtClockProfilePoint>,
}

/// Time-evolution trend classification for a truth-guided PVT accuracy run.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum SyntheticPvtTimeTrend {
    /// Errors settle or remain bounded over the measured interval.
    Stabilizing,
    /// Errors grow over time without the receiver validity diverging.
    Drifting,
    /// Errors diverge strongly enough that receiver validity becomes diverging.
    Diverging,
}

/// One truth-guided PVT accuracy measurement point indexed by time evolution.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTimeProfilePoint {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Number of truth-matched epochs in the time series.
    pub truth_epoch_count: usize,
    /// Time covered by the matched epochs, in seconds.
    pub duration_s: f64,
    /// Number of matched PVT epochs compared against truth.
    pub epoch_count: usize,
    /// Number of matched PVT epochs that satisfied the hard accuracy budget.
    pub passing_epoch_count: usize,
    /// Passing-epoch fraction across the matched PVT epochs.
    pub pass_rate: f64,
    /// Number of truth-matched epochs whose solution validity remained stable.
    pub stable_epoch_count: usize,
    /// Stable-solution fraction across truth-matched epochs.
    pub stable_epoch_rate: f64,
    /// Number of truth-matched epochs whose solution validity diverged.
    pub diverging_epoch_count: usize,
    /// Diverging-solution fraction across truth-matched epochs.
    pub diverging_epoch_rate: f64,
    /// Number of epochs contributing to each leading and trailing analysis window.
    pub analysis_window_epoch_count: usize,
    /// Mean 3D position error over the first analysis window, in meters.
    pub first_window_mean_position_error_3d_m: Option<f64>,
    /// Mean 3D position error over the last analysis window, in meters.
    pub last_window_mean_position_error_3d_m: Option<f64>,
    /// Difference between the trailing and leading mean 3D position error, in meters.
    pub position_error_growth_m: Option<f64>,
    /// Linear 3D position-error drift slope over receive time, in meters per second.
    pub position_error_drift_m_per_s: Option<f64>,
    /// Mean residual RMS over the first analysis window, in meters.
    pub first_window_mean_residual_rms_m: Option<f64>,
    /// Mean residual RMS over the last analysis window, in meters.
    pub last_window_mean_residual_rms_m: Option<f64>,
    /// Difference between the trailing and leading mean residual RMS, in meters.
    pub residual_rms_growth_m: Option<f64>,
    /// Linear residual-RMS drift slope over receive time, in meters per second.
    pub residual_rms_drift_m_per_s: Option<f64>,
    /// Classified time-evolution trend.
    pub trend: SyntheticPvtTimeTrend,
    /// Whether synthetic truth coverage remained sufficient for a hard claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether the point had truth-ready PVT comparisons and enough epochs to classify time trend.
    pub ready: bool,
}

/// Truth-guided PVT accuracy profile across long-run time-evolution points.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtTimeProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Measurement points captured in the report.
    pub points: Vec<SyntheticPvtTimeProfilePoint>,
}

/// Aggregated multi-stage accuracy summary at one signal-strength point.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAccuracyCn0ProfilePoint {
    /// Shared signal-strength coordinate for the merged stage metrics, in dB-Hz.
    pub cn0_db_hz: f64,
    /// Number of acquisition cases contributing to this point.
    pub acquisition_case_count: usize,
    /// Total synthetic acquisition trials contributing to this point.
    pub acquisition_trial_count: usize,
    /// Mean acceptance probability across acquisition cases at this point.
    pub acquisition_acceptance_probability_mean: Option<f64>,
    /// Mean detection probability across acquisition cases at this point.
    pub acquisition_detection_probability_mean: Option<f64>,
    /// Number of tracking cases contributing to this point.
    pub tracking_case_count: usize,
    /// Total synthetic tracking trials contributing to this point.
    pub tracking_trial_count: usize,
    /// Mean stable-lock probability across tracking cases at this point.
    pub tracking_lock_probability_mean: Option<f64>,
    /// Mean refused-lock rate across tracking cases at this point.
    pub tracking_refused_lock_rate_mean: Option<f64>,
    /// Number of PVT cases contributing to this point.
    pub pvt_case_count: usize,
    /// Total matched PVT epochs contributing to this point.
    pub pvt_epoch_count: usize,
    /// Mean PVT pass rate across cases at this point.
    pub pvt_pass_rate_mean: Option<f64>,
    /// Mean RMS 3D position error across cases at this point, in meters.
    pub pvt_rms_position_error_3d_m_mean: Option<f64>,
    /// Maximum 3D position error observed across cases at this point, in meters.
    pub pvt_max_position_error_3d_m_max: Option<f64>,
}

/// Merged acquisition, tracking, and PVT performance profile across signal strength.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAccuracyCn0ProfileReport {
    /// Scenario identifier prefix shared across the measurement points.
    pub scenario_id_prefix: String,
    /// Merged per-C/N0 measurement points.
    pub points: Vec<SyntheticAccuracyCn0ProfilePoint>,
}
