//! Public API for bijux-gnss-infra.
#![cfg_attr(docsrs, feature(doc_cfg))]

pub use crate::artifact_tools::{
    artifact_explain, artifact_validate, ArtifactExplainResult, ArtifactValidationResult,
};
pub use crate::commands::prepare_run;
pub use crate::validate_reference::validate_reference;
pub use bijux_gnss_receiver::api::core::{
    align_reference_by_time, check_solution_consistency, reference_compare, reference_ecef,
    ReferenceAlign, ReferenceCompareStats, SolutionConsistencyReport, ValidationReferenceEpoch,
};
pub use bijux_gnss_receiver::api::core::{lla_to_ecef, stats, StatsSummary};
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_receiver::api::{
    build_validation_report, check_time_consistency, ConvergenceReport, FixTimelineEntry,
    NavResidualReport, PppReadinessReport, TimeConsistencyReport, ValidationBudgets,
    ValidationErrorStats, ValidationReport,
};

/// Receiver API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api as receiver;

/// Core API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api::core;

/// Run layout helpers.
pub use crate::run_layout::{
    append_run_index, artifact_header, artifacts_dir, run_dir, run_report_schema_version,
    write_manifest, write_run_report, RunContextArgs, RunDirLayout, RunIndexEntry, RunManifest,
    RunReport,
};

/// Dataset helpers.
pub use crate::datasets::{parse_ecef, DatasetEntry, DatasetRegistry};

/// Experiment helpers.
pub use crate::experiments::{ExperimentSpec, SweepParameter};
pub use crate::sweep::{expand_sweep, parse_sweep};

/// Override helpers.
pub use crate::overrides::{
    apply_common_overrides, apply_overrides, apply_sweep_value, CommonOverrides,
};

/// Hash helpers.
pub use crate::hash::{cpu_features, git_dirty, git_hash, hash_config};

/// Signal API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api::signal;

/// Nav API re-exports for CLI convenience.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_receiver::api::nav;
