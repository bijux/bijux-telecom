//! Public API for bijux-gnss-receiver.

/// Receiver configuration and schema.
pub use crate::engine::receiver_config::{ReceiverConfig, ReceiverError, ReceiverProfile};

/// I/O helpers.
pub use crate::io::data::{FileSamples, MemorySamples, SampleSourceError};
pub use bijux_gnss_signal::api::SignalSource;

/// Re-export core API for downstream crates.
pub use bijux_gnss_core::api as core;

/// Re-export signal API for downstream crates.
pub use bijux_gnss_signal::api as signal;

/// Re-export nav API for downstream crates.
#[cfg(feature = "nav")]
pub use bijux_gnss_nav::api as nav;

/// Runtime logging helpers.
pub use crate::engine::logging;
/// Port traits (I/O boundaries).
pub use crate::ports::{ArtifactSink, Clock, SampleSource, SystemClock};

/// Acquisition engine.
pub use crate::pipeline::acquisition::Acquisition as AcquisitionEngine;
/// Tracking engine and related types.
pub use crate::pipeline::tracking::{
    Channel, ChannelEvent, ChannelState, CorrelatorOutput, Tracking as TrackingEngine,
    TrackingResult,
};
/// Observation-building helpers.
pub use crate::pipeline::observations::{
    observations_from_tracking, observations_from_tracking_results,
};
/// Navigation engine and helpers.
#[cfg(feature = "nav")]
pub use crate::pipeline::navigation::{EkfState, Navigation, NavigationEngine};

/// RTK differencing and baseline helpers.
#[cfg(feature = "nav")]
pub mod rtk {
    pub use crate::rtk::ambiguity::{
        decorrelate_lambda, float_from_state, ratio_from_candidates, ratio_test,
        search_integer_candidates, select_partial_fix, AmbiguityFixResult, AmbiguityManager,
        DecorrelatedAmbiguities, FixAuditEvent, FixPolicy, FixState, FloatAmbiguitySolution,
        IntegerCandidate, NaiveFixer,
    };
    pub use crate::rtk::core::{
        build_dd, build_dd_per_constellation, build_sd, choose_ref_sat,
        choose_ref_sat_per_constellation, dd_covariance, innovation_diagnostics, los_unit,
        solve_baseline_dd, AlignmentDiagnostic, AlignmentReport, BaselineConfig, DdCovarianceModel,
        DdObservation, EpochAligner, RefSatPolicy, RefSatSelector, SdObservation,
        SolutionSeparation,
    };
    pub use crate::rtk::differencing::{double_difference, single_difference};
    pub use crate::rtk::metrics::{
        apply_fix_hold, baseline_from_ecef, dd_residual_metrics, enu_to_ecef, jitter_summary,
        solution_separation, BaselineSolution, JitterSummary, RtkBaselineQuality, RtkPrecision,
    };
    pub use bijux_gnss_core::api::rtk::{
        RtkBaselineEpochV1, RtkBaselineQualityV1, RtkDdEpochV1, RtkFixAuditV1, RtkPrecisionV1,
        RtkSdEpochV1,
    };
}

/// Synthetic signal generation for tests and demos.
#[cfg(feature = "nav")]
pub use crate::sim::synthetic as sim;

/// Validation report helpers.
#[cfg(feature = "nav")]
pub mod validation_report {
    pub use crate::validation_report::{
        build_validation_report, check_time_consistency, ConvergenceReport, FixTimelineEntry,
        NavResidualReport, PppReadinessReport, TimeConsistencyReport, ValidationBudgets,
        ValidationErrorStats, ValidationReport,
    };
}

/// Artifacts produced by a receiver pipeline run.
#[derive(Debug, Default, Clone)]
pub struct RunArtifacts {
    /// Acquisition candidates captured during the run.
    pub acquisitions: Vec<bijux_gnss_core::api::AcqResult>,
    /// Tracking reports captured during the run.
    pub tracking: Vec<TrackingResult>,
    /// Observation epochs captured during the run.
    pub observations: Vec<bijux_gnss_core::api::ObsEpoch>,
    /// Navigation solution epochs captured during the run.
    pub navigation: Vec<bijux_gnss_core::api::NavEpoch>,
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
    config: ReceiverConfig,
}

impl Receiver {
    /// Create a new receiver with the provided configuration.
    pub fn new(config: ReceiverConfig) -> Self {
        Self { config }
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverConfig {
        &self.config
    }
}
