/// Hard accuracy threshold for truth-guided acquisition validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracyBudget {
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error in samples.
    pub max_code_phase_error_samples: usize,
}

/// Hard accuracy threshold for truth-guided tracking validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracyBudget {
    /// Maximum absolute carrier-frequency error in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: f64,
}

/// Hard accuracy threshold for truth-guided observation validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracyBudget {
    /// Maximum absolute pseudorange error in meters.
    pub max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error in cycles.
    pub max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: f64,
}

/// Hard accuracy threshold for truth-guided PVT validation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyBudget {
    /// Maximum 3D position error in meters.
    pub max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error in meters.
    pub max_clock_bias_error_m: f64,
    /// Maximum residual RMS in meters.
    pub max_residual_rms_m: f64,
    /// Maximum PDOP for an in-budget solution epoch.
    pub max_pdop: f64,
}

/// Hard truth-guided accuracy thresholds for receiver stages.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SyntheticReceiverAccuracyBudgets {
    /// Acquisition-stage hard thresholds.
    pub acquisition: SyntheticAcquisitionAccuracyBudget,
    /// Tracking-stage hard thresholds.
    pub tracking: SyntheticTrackingAccuracyBudget,
    /// Observation-stage hard thresholds.
    pub observation: SyntheticObservationAccuracyBudget,
    /// PVT-stage hard thresholds.
    pub pvt: SyntheticPvtAccuracyBudget,
}

/// Machine-checkable reason a stage accuracy report lacks sufficient synthetic truth coverage.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticTruthCoverageIssue {
    /// Satellite identifier when the issue is scoped to one tracked satellite.
    pub sat: Option<SatId>,
    /// Navigation epoch index when the issue is scoped to one solution epoch.
    pub epoch_index: Option<u64>,
    /// Stable issue code describing the missing or incomplete truth coverage.
    pub code: String,
}

/// Per-satellite acquisition budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Measured absolute Doppler error in Hz.
    pub doppler_error_hz: f64,
    /// Measured wrapped code-phase error in samples.
    pub code_phase_error_samples: usize,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Acquisition-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticAcquisitionAccuracyReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub max_code_phase_error_samples: usize,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticAcquisitionAccuracySatellite>,
}

/// Per-satellite tracking budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Number of stable tracking epochs compared against truth.
    pub stable_epoch_count: usize,
    /// Maximum carrier error across stable tracking epochs, in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum Doppler error across stable tracking epochs, in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error across stable tracking epochs, in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum C/N0 error across stable tracking epochs, in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Tracking-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticTrackingAccuracyReport {
    /// Stable scenario identifier for this capture.
    pub scenario_id: String,
    /// Maximum absolute carrier-frequency error allowed in Hz.
    pub max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticTrackingAccuracySatellite>,
}

/// Per-satellite observation budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracySatellite {
    /// Satellite identifier.
    pub sat: SatId,
    /// Maximum absolute pseudorange error in meters.
    pub max_pseudorange_error_m: Option<f64>,
    /// Maximum absolute carrier-phase error in cycles.
    pub max_carrier_phase_error_cycles: Option<f64>,
    /// Maximum absolute Doppler error in Hz.
    pub max_doppler_error_hz: Option<f64>,
    /// Maximum absolute C/N0 error in dB-Hz.
    pub max_cn0_error_db_hz: Option<f64>,
    /// Whether this satellite stayed within the configured hard budget.
    pub pass: bool,
}

/// Observation-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticObservationAccuracyReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Maximum absolute pseudorange error allowed in meters.
    pub max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error allowed in cycles.
    pub max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub max_cn0_error_db_hz: f64,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the budget.
    pub passing_satellite_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every measured satellite satisfied the budget.
    pub pass: bool,
    /// Per-satellite budget outcomes.
    pub satellites: Vec<SyntheticObservationAccuracySatellite>,
}

/// Per-epoch PVT budget outcome.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyEpoch {
    /// Receiver navigation epoch index.
    pub epoch_index: u64,
    /// 3D position error in meters.
    pub position_error_3d_m: f64,
    /// Absolute clock-bias error in meters.
    pub clock_bias_error_m: f64,
    /// Residual RMS in meters.
    pub residual_rms_m: f64,
    /// PDOP reported for this epoch.
    pub pdop: f64,
    /// Whether this epoch stayed within the configured hard budget.
    pub pass: bool,
}

/// PVT-stage hard-budget validation report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticPvtAccuracyReport {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: String,
    /// Maximum 3D position error allowed in meters.
    pub max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error allowed in meters.
    pub max_clock_bias_error_m: f64,
    /// Maximum residual RMS allowed in meters.
    pub max_residual_rms_m: f64,
    /// Maximum PDOP allowed for an in-budget epoch.
    pub max_pdop: f64,
    /// Number of matched solution epochs.
    pub epoch_count: usize,
    /// Number of epochs that satisfied the budget.
    pub passing_epoch_count: usize,
    /// Whether this report had enough synthetic truth coverage to make a hard accuracy claim.
    pub truth_coverage_ready: bool,
    /// Machine-checkable truth-coverage issues that forced or should force validation failure.
    pub truth_coverage_issues: Vec<SyntheticTruthCoverageIssue>,
    /// Whether every matched epoch satisfied the budget.
    pub pass: bool,
    /// Per-epoch budget outcomes.
    pub epochs: Vec<SyntheticPvtAccuracyEpoch>,
}

