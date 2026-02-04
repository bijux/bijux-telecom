//! Public API for bijux-gnss-infra.

pub use crate::dataset::{DatasetEntry, DatasetRegistry};
pub use crate::commands::prepare_run;
pub use crate::artifact_tools::{
    artifact_explain, artifact_validate, ArtifactExplainResult, ArtifactValidationResult,
};
pub use crate::errors::{InfraError, InfraResult};
pub use crate::experiment::{ExperimentSpec, SweepParameter};
pub use crate::hash::{cpu_features, git_dirty, git_hash, hash_config};
pub use crate::overrides::{
    apply_common_overrides, apply_overrides, apply_sweep_value, CommonOverrides,
};
pub use crate::parse::parse_ecef;
pub use crate::reference_validation::{
    align_reference_by_time, check_solution_consistency, check_time_consistency, reference_compare,
    reference_ecef, ReferenceAlign, ReferenceCompareStats, SolutionConsistencyReport,
    TimeConsistencyReport, ValidationReferenceEpoch,
};
pub use crate::run_layout::{
    append_run_index, artifact_header, artifacts_dir, run_dir, write_manifest, RunContextArgs,
    RunDirLayout, RunIndexEntry, RunManifest,
};
pub use crate::stats::{lla_to_ecef, stats, StatsSummary};
pub use crate::sweep::{expand_sweep, parse_sweep};
