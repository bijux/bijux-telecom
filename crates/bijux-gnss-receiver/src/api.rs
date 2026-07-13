//! Public API for bijux-gnss-receiver.
#![cfg_attr(docsrs, feature(doc_cfg))]

/// Receiver configuration and schema.
pub use crate::engine::receiver_config::{
    ConstellationSelectionPolicy, NavigationMotionClass, NavigationWeightingMode, ReceiverConfig,
    ReceiverError, ReceiverPipelineConfig,
};
/// Receiver runtime options (side-effectful controls).
pub use crate::engine::runtime::{
    Metric, MetricsSink, NullLogger, ReceiverRuntime, ReceiverRuntimeConfig, TraceRecord, TraceSink,
};

/// I/O helpers.
pub use crate::io::data::{FileSamples, MemorySamples, SampleSourceError};
pub use bijux_gnss_signal::api::SignalSource;

/// Re-export core API for downstream crates.
pub use bijux_gnss_core::api as core;

/// Re-export signal API for downstream crates.
pub use bijux_gnss_signal::api as signal;

/// Re-export nav API for downstream crates.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_nav::api as nav;

/// Runtime logging helpers.
pub use crate::engine::logging;
pub use crate::ports::clock::{Clock, SystemClock};
/// Port traits (I/O boundaries).
pub use crate::ports::{ArtifactSink, SampleSource};

/// Acquisition engine.
pub use crate::pipeline::acquisition::Acquisition as AcquisitionEngine;
/// Carrier/Doppler conversion helpers.
pub use crate::pipeline::doppler::{carrier_hz_from_doppler_hz, doppler_hz_from_carrier_hz};
/// Navigation engine and helpers.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::pipeline::navigation::{EkfState, Navigation, NavigationEngine};
/// Filtered navigation execution helpers.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::pipeline::navigation_filter::{NavigationFilter, NavigationFilterThresholds};
/// Carrier-smoothed code validation helpers.
pub use crate::pipeline::observation_validation::{
    summarize_carrier_smoothed_code, validate_carrier_smoothed_code,
    validate_carrier_smoothed_code_from_artifacts, CarrierSmoothedCodeValidationReport,
};
/// Observation-building helpers.
pub use crate::pipeline::observations::{
    observation_artifacts_from_tracking_results,
    observation_artifacts_from_tracking_results_with_gps_anchor,
    observation_measurement_quality_from_tracking_results,
    observation_measurement_quality_from_tracking_results_with_gps_anchor,
    observation_residuals_from_tracking_results,
    observation_residuals_from_tracking_results_with_gps_anchor, observations_from_tracking,
    observations_from_tracking_results, observations_from_tracking_results_with_gps_anchor,
    ObservationMeasurementQualityEpochReport, ObservationMeasurementQualitySatellite,
    ObservationPipelineArtifacts, ObservationResidualEpochReport, ObservationResidualSatellite,
    ObservationResidualValue,
};
/// Tracking engine and related types.
pub use crate::pipeline::tracking::{
    Channel, ChannelEvent, ChannelState, CorrelatorOutput, Tracking as TrackingEngine,
    TrackingArtifacts, TrackingChannelState, TrackingChannelStateEvent, TrackingChannelStateReport,
    TrackingResult, TrackingSession,
};
/// Pipeline step report helpers.
pub use crate::pipeline::{StepReport, StepStats};

#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::rtk::core::{
    build_dd, build_dd_per_constellation, build_sd, choose_ref_sat,
    choose_ref_sat_per_constellation, dd_covariance, innovation_diagnostics, los_unit,
    solve_baseline_dd, solve_float_baseline_dd, AlignmentDiagnostic, AlignmentReport,
    BaselineConfig, DdCovarianceModel, DdObservation, EpochAligner, RefSatPolicy, RefSatSelector,
    SdObservation, SolutionSeparation,
};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::rtk::differencing::{double_difference, single_difference};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::rtk::metrics::{
    apply_fix_hold, baseline_from_ecef, dd_residual_metrics, enu_to_ecef,
    evaluate_rtk_fixed_baseline_guard, jitter_summary, sd_residual_metrics, solution_separation,
    BaselineSolution, JitterSummary, RtkBaselineQuality, RtkFixedBaselineGuardDecision,
    RtkFixedBaselineGuardPolicy, RtkPrecision,
};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::rtk::status::{
    apply_downgrade_policy, evaluate_prerequisites, support_status_matrix, AdvancedMaturity,
    AdvancedMode, AdvancedPrereqDecision, AdvancedPrerequisites, AdvancedRefusalClass,
    AdvancedSolutionArtifact, AdvancedSolutionClaim, AdvancedSolutionProvenance,
    AdvancedSupportMatrix, AdvancedSupportRow, AmbiguityStateArtifact, CorrectionInputArtifact,
    ADVANCED_SUPPORT_MATRIX_VERSION,
};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_core::api::rtk::{
    RtkBaselineEpochV1, RtkBaselineQualityV1, RtkDdEpochV1, RtkFixAuditV1, RtkPrecisionV1,
    RtkSdEpochV1,
};
/// RTK differencing and baseline helpers.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_nav::api::{
    rtk_ambiguity_state_from_fixed_solution, rtk_candidate_ratio,
    rtk_conditioned_baseline_from_fixed_ambiguities,
    rtk_float_ambiguity_state_from_baseline_solution, rtk_float_ambiguity_state_from_filter_state,
    rtk_integer_ambiguity_candidates, rtk_lambda_decorrelate, rtk_ratio_test_acceptance,
    rtk_select_partial_ambiguity_fix, RtkAmbiguityFixAudit, RtkAmbiguityFixPolicy,
    RtkAmbiguityFixResult, RtkAmbiguityFixState, RtkAmbiguityFixStatus, RtkAmbiguityTracker,
    RtkConditionedBaselineSolution, RtkDecorrelatedAmbiguityState, RtkDoubleDifferenceAmbiguityId,
    RtkFloatAmbiguityState, RtkIntegerAmbiguityCandidate, RtkRatioTestFixer,
};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_nav::api::{RtkFloatAmbiguityEstimate, RtkFloatBaselineSolution};

/// Synthetic signal generation and canonical scenario execution for tests, validation, and demos.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::sim::synthetic as sim;

#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::covariance_realism::{
    CovarianceCoverageClass, CovarianceCoverageRate, CovarianceRealismReport,
};
/// Validation report helpers.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use crate::validation_report::{
    build_validation_report, build_validation_report_from_observation_artifacts,
    build_validation_report_from_observation_artifacts_with_budgets,
    build_validation_report_with_budgets, check_time_consistency, ConvergenceReport,
    DiagnosticPartitionReport, FixTimelineEntry, NavConstellationResidualReport, NavIntegrityClass,
    NavIntegrityReport, NavResidualReport, PppReadinessReport, ProtectionLevelValidationReport,
    ReferencePositionErrorEpoch, TimeConsistencyReport, ValidationAssumptionReport,
    ValidationBudgets, ValidationErrorStats, ValidationReport, ValidationSciencePolicy,
};

/// Artifacts produced by a receiver pipeline run.
#[derive(Debug, Default, Clone)]
pub struct RunArtifacts {
    /// Total complex samples consumed from the input source.
    pub processed_input_samples: u64,
    /// Total code-period epochs consumed from the input source.
    pub processed_input_epochs: u64,
    /// Acquisition candidates captured during the run.
    pub acquisitions: Vec<bijux_gnss_core::api::AcqResult>,
    /// Acquisition explain artifacts captured during the run.
    pub acquisition_explain: Vec<bijux_gnss_core::api::AcqExplain>,
    /// Track transition artifacts captured during the run.
    pub track_transitions: Vec<bijux_gnss_core::api::TrackTransition>,
    /// Per-channel tracking state reports captured during the run.
    pub channel_state_reports: Vec<TrackingChannelStateReport>,
    /// Tracking reports captured during the run.
    pub tracking: Vec<TrackingResult>,
    /// Observation decision artifacts captured during the run.
    pub observation_decisions: Vec<bijux_gnss_core::api::ObsDecisionArtifact>,
    /// Observation epochs captured during the run.
    pub observations: Vec<bijux_gnss_core::api::ObsEpoch>,
    /// Observation residual reports captured during the run.
    pub observation_residuals: Vec<ObservationResidualEpochReport>,
    /// Per-signal observation measurement quality reports captured during the run.
    pub observation_measurement_quality: Vec<ObservationMeasurementQualityEpochReport>,
    /// Signal support matrix artifact.
    pub support_matrix: Option<bijux_gnss_core::api::SupportMatrix>,
    /// Navigation solution epochs captured during the run.
    pub navigation: Vec<bijux_gnss_core::api::NavEpoch>,
}

impl RunArtifacts {
    /// Reconstruct the observation-stage artifacts emitted during this run.
    pub fn observation_artifacts(&self) -> ObservationPipelineArtifacts {
        ObservationPipelineArtifacts {
            epochs: self.observations.clone(),
            residuals: self.observation_residuals.clone(),
            measurement_quality: self.observation_measurement_quality.clone(),
        }
    }
}

/// Receiver engine boundary trait.
pub trait ReceiverEngine {
    /// Run the receiver pipeline against a signal source.
    fn run(
        &mut self,
        input: &mut dyn bijux_gnss_signal::api::SignalSource<
            Error = crate::io::data::SampleSourceError,
        >,
    ) -> Result<RunArtifacts, ReceiverError>;
}

/// High-level receiver pipeline entrypoint.
pub struct Receiver {
    pub(crate) config: ReceiverPipelineConfig,
    pub(crate) runtime: ReceiverRuntime,
}
