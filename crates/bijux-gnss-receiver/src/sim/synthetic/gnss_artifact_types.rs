/// Data-source summary for one GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyDataSource {
    /// Stable source kind used to interpret the run origin.
    pub source_kind: String,
    /// Sample rate used for the validation run, in Hz.
    pub sample_rate_hz: f64,
    /// Intermediate frequency used for the validation run, in Hz.
    pub intermediate_freq_hz: f64,
    /// Total synthetic capture duration, in seconds.
    pub duration_s: f64,
    /// Number of satellites present in the source scenario.
    pub satellite_count: usize,
}

/// Reference-truth summary for one GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyReferenceTruth {
    /// Stable truth kind used to interpret the reference source.
    pub truth_kind: String,
    /// Receiver ECEF reference coordinates when available, in meters.
    pub receiver_ecef_m: Option<[f64; 3]>,
    /// Absolute receive-time anchor for the truth reference, in seconds.
    pub reference_receive_time_s: Option<f64>,
    /// Number of satellites covered by the reference truth.
    pub satellite_count: usize,
    /// Number of truth epochs available to the validation run.
    pub reference_epoch_count: usize,
}

/// Compact acquisition-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAcquisitionStageSummary {
    /// Whether the acquisition stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest absolute Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest wrapped code-phase error observed across measured satellites, in samples.
    pub observed_max_code_phase_error_samples: Option<usize>,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub threshold_max_code_phase_error_samples: usize,
}

/// Compact tracking-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssTrackingStageSummary {
    /// Whether the tracking stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest carrier-frequency error observed across measured satellites, in Hz.
    pub observed_max_carrier_error_hz: Option<f64>,
    /// Largest Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest wrapped code-phase error observed across measured satellites, in samples.
    pub observed_max_code_phase_error_samples: Option<f64>,
    /// Largest C/N0 error observed across measured satellites, in dB-Hz.
    pub observed_max_cn0_error_db_hz: Option<f64>,
    /// Maximum absolute carrier-frequency error allowed in Hz.
    pub threshold_max_carrier_error_hz: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum wrapped code-phase error allowed in samples.
    pub threshold_max_code_phase_error_samples: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub threshold_max_cn0_error_db_hz: f64,
}

/// Compact observation-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssObservationStageSummary {
    /// Whether the observation stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured satellites.
    pub satellite_count: usize,
    /// Number of satellites that satisfied the stage budget.
    pub passing_satellite_count: usize,
    /// Largest pseudorange error observed across measured satellites, in meters.
    pub observed_max_pseudorange_error_m: Option<f64>,
    /// Largest carrier-phase error observed across measured satellites, in cycles.
    pub observed_max_carrier_phase_error_cycles: Option<f64>,
    /// Largest Doppler error observed across measured satellites, in Hz.
    pub observed_max_doppler_error_hz: Option<f64>,
    /// Largest C/N0 error observed across measured satellites, in dB-Hz.
    pub observed_max_cn0_error_db_hz: Option<f64>,
    /// Maximum absolute pseudorange error allowed in meters.
    pub threshold_max_pseudorange_error_m: f64,
    /// Maximum absolute carrier-phase error allowed in cycles.
    pub threshold_max_carrier_phase_error_cycles: f64,
    /// Maximum absolute Doppler error allowed in Hz.
    pub threshold_max_doppler_error_hz: f64,
    /// Maximum absolute C/N0 error allowed in dB-Hz.
    pub threshold_max_cn0_error_db_hz: f64,
}

/// Compact PVT-stage summary carried by a final GNSS accuracy artifact.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssPvtStageSummary {
    /// Whether the PVT stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether synthetic truth coverage was sufficient for the stage claim.
    pub truth_coverage_ready: bool,
    /// Number of measured navigation epochs.
    pub epoch_count: usize,
    /// Number of epochs that satisfied the stage budget.
    pub passing_epoch_count: usize,
    /// Largest 3D position error observed across measured epochs, in meters.
    pub observed_max_position_error_3d_m: Option<f64>,
    /// Largest absolute clock-bias error observed across measured epochs, in meters.
    pub observed_max_clock_bias_error_m: Option<f64>,
    /// Largest residual RMS observed across measured epochs, in meters.
    pub observed_max_residual_rms_m: Option<f64>,
    /// Largest PDOP observed across measured epochs.
    pub observed_max_pdop: Option<f64>,
    /// Maximum 3D position error allowed in meters.
    pub threshold_max_position_error_3d_m: f64,
    /// Maximum absolute clock-bias error allowed in meters.
    pub threshold_max_clock_bias_error_m: f64,
    /// Maximum residual RMS allowed in meters.
    pub threshold_max_residual_rms_m: f64,
    /// Maximum PDOP allowed for an in-budget epoch.
    pub threshold_max_pdop: f64,
}

/// Final acquisition-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAcquisitionStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssAcquisitionStageSummary,
    /// Detailed acquisition accuracy report for per-satellite review.
    pub report: SyntheticAcquisitionAccuracyReport,
}

/// Final tracking-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssTrackingStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssTrackingStageSummary,
    /// Detailed tracking accuracy report for per-satellite review.
    pub report: SyntheticTrackingAccuracyReport,
}

/// Final observation-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssObservationStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssObservationStageSummary,
    /// Detailed observation accuracy report for per-satellite review.
    pub report: SyntheticObservationAccuracyReport,
}

/// Final PVT-stage artifact payload with both compact summary and detailed report.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssPvtStageArtifact {
    /// Compact stage summary used for run-level pass/fail review.
    pub summary: SyntheticGnssPvtStageSummary,
    /// Detailed PVT accuracy report for per-epoch review.
    pub report: SyntheticPvtAccuracyReport,
}

/// Ordered receiver-chain stage covered by a synthetic closure validation run.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticClosureStageKind {
    /// Raw IQ input and front-end metadata were explicit and consumed by the receiver.
    IqInput,
    /// Acquisition produced truth-checked satellite detections.
    Acquisition,
    /// Tracking produced truth-checked channel histories.
    Tracking,
    /// Observation timing carried explicit navigation-data receive/transmit timing evidence.
    NavigationDataTiming,
    /// Observation epochs carried truth-checked pseudorange, phase, Doppler, and C/N0 evidence.
    Observations,
    /// Navigation consumed explicit satellite-state evidence from declared broadcast ephemerides.
    SatelliteStates,
    /// Navigation applied explicit correction evidence before estimation.
    Corrections,
    /// The configured positioning estimator produced truth-checked navigation solutions.
    Estimator,
    /// Carrier-ambiguity processing was either executed or explicitly not applicable.
    AmbiguityProcessing,
    /// Integrity monitoring emitted an explicit decision and protection evidence.
    IntegrityDecision,
}

/// Closure status for one stage in the synthetic validation chain.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SyntheticClosureStageStatus {
    /// The stage executed and carried enough truth/runtime evidence for a hard closure claim.
    Passed,
    /// The stage was required for the configured path but did not carry enough evidence.
    Failed,
    /// The stage does not apply to the configured estimator path.
    NotApplicable,
}

impl SyntheticClosureStageStatus {
    fn is_closure_success(self) -> bool {
        matches!(self, Self::Passed | Self::NotApplicable)
    }
}

/// Machine-checkable closure evidence for one stage in the validation chain.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticClosureStageEvidence {
    /// Ordered receiver-chain stage.
    pub stage: SyntheticClosureStageKind,
    /// Closure status for this stage.
    pub status: SyntheticClosureStageStatus,
    /// Whether the receiver produced runtime artifacts for this stage.
    pub executed: bool,
    /// Whether the stage had enough independent synthetic truth or explicit runtime evidence.
    pub truth_backed: bool,
    /// Number of runtime or truth rows used as evidence for this stage.
    pub evidence_count: usize,
    /// Stable reason code for reviewer and machine interpretation.
    pub reason: String,
}

impl SyntheticClosureStageEvidence {
    /// Whether this stage contributes a successful closure outcome.
    pub fn is_closure_success(&self) -> bool {
        self.status.is_closure_success()
    }
}

/// Final closure summary for the configured synthetic receiver path.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SyntheticClosureSummary {
    /// Whether every applicable stage closed with hard evidence.
    pub pass: bool,
    /// Count of stages that applied to this configured path.
    pub applicable_stage_count: usize,
    /// Count of applicable stages that passed.
    pub passed_stage_count: usize,
    /// Count of explicitly non-applicable stages.
    pub not_applicable_stage_count: usize,
    /// Ordered per-stage closure evidence.
    pub stages: Vec<SyntheticClosureStageEvidence>,
}

/// Borrowed inputs for building one final GNSS accuracy artifact.
#[derive(Debug, Clone)]
pub struct SyntheticGnssAccuracyArtifactCase<'a> {
    /// Stable scenario identifier for the validation run.
    pub scenario_id: &'a str,
    /// Data-source summary for the validation run.
    pub data_source: SyntheticGnssAccuracyDataSource,
    /// Reference-truth summary for the validation run.
    pub reference_truth: SyntheticGnssAccuracyReferenceTruth,
    /// Detailed acquisition-stage accuracy report.
    pub acquisition: &'a SyntheticAcquisitionAccuracyReport,
    /// Detailed tracking-stage accuracy report.
    pub tracking: &'a SyntheticTrackingAccuracyReport,
    /// Detailed observation-stage accuracy report.
    pub observation: &'a SyntheticObservationAccuracyReport,
    /// Detailed PVT-stage accuracy report.
    pub pvt: &'a SyntheticPvtAccuracyReport,
    /// Machine-checkable closure summary for the configured receiver path.
    pub closure: SyntheticClosureSummary,
}

/// Final truth-guided GNSS accuracy artifact for one validation run.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SyntheticGnssAccuracyArtifact {
    /// Schema version for the artifact payload.
    pub schema_version: u32,
    /// Stable scenario identifier for the validation run.
    pub scenario_id: String,
    /// Whether every stage satisfied its hard thresholds.
    pub pass: bool,
    /// Whether every stage had enough truth coverage for a hard claim.
    pub truth_coverage_ready: bool,
    /// Whether the configured receiver path closed every applicable stage with evidence.
    pub closure_ready: bool,
    /// Validation-run source summary.
    pub data_source: SyntheticGnssAccuracyDataSource,
    /// Validation-run reference-truth summary.
    pub reference_truth: SyntheticGnssAccuracyReferenceTruth,
    /// Acquisition-stage artifact payload.
    pub acquisition: SyntheticGnssAcquisitionStageArtifact,
    /// Tracking-stage artifact payload.
    pub tracking: SyntheticGnssTrackingStageArtifact,
    /// Observation-stage artifact payload.
    pub observation: SyntheticGnssObservationStageArtifact,
    /// PVT-stage artifact payload.
    pub pvt: SyntheticGnssPvtStageArtifact,
    /// Ordered closure evidence for the configured receiver path.
    pub closure: SyntheticClosureSummary,
}
