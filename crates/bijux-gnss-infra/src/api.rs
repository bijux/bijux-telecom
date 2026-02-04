//! Public API for bijux-gnss-infra.

pub use crate::artifact_tools::{
    artifact_explain, artifact_validate, ArtifactExplainResult, ArtifactValidationResult,
};
pub use crate::commands::prepare_run;
pub use crate::dataset::{DatasetEntry, DatasetRegistry};
pub use crate::run_layout::{
    append_run_index, artifact_header, artifacts_dir, run_dir, write_manifest, RunContextArgs,
    RunDirLayout, RunIndexEntry, RunManifest,
};
pub use crate::sweep::{expand_sweep, parse_sweep};
pub use crate::validate_reference::validate_reference;
pub use bijux_gnss_receiver::api::core::{
    align_reference_by_time, check_solution_consistency, reference_compare, reference_ecef,
    ReferenceAlign, ReferenceCompareStats, SolutionConsistencyReport, ValidationReferenceEpoch,
};
pub use bijux_gnss_receiver::api::core::{lla_to_ecef, stats, StatsSummary};
#[cfg(feature = "nav")]
pub use bijux_gnss_receiver::api::validation_report::{
    build_validation_report, check_time_consistency, ConvergenceReport, FixTimelineEntry,
    NavResidualReport, PppReadinessReport, TimeConsistencyReport, ValidationBudgets,
    ValidationErrorStats, ValidationReport,
};

/// Receiver API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api as receiver;

/// Core API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api::core as core;

/// Hash a config file or profile snapshot.
pub fn hash_config(
    path: Option<&std::path::PathBuf>,
    profile: &bijux_gnss_receiver::api::ReceiverProfile,
) -> Result<String, bijux_gnss_receiver::api::core::InputError> {
    crate::hash::core::hash_config(path, profile)
}

/// Return current git hash if available.
pub fn git_hash() -> Option<String> {
    crate::hash::core::git_hash()
}

/// Return true when git workspace is dirty.
pub fn git_dirty() -> bool {
    crate::hash::core::git_dirty()
}

/// CPU feature detection summary.
pub fn cpu_features() -> Vec<String> {
    crate::hash::core::cpu_features()
}

/// Parse ECEF coordinates in x,y,z format.
pub fn parse_ecef(text: &str) -> Result<[f64; 3], bijux_gnss_receiver::api::core::InputError> {
    crate::parse::core::parse_ecef(text)
}

/// Common override values from CLI.
#[derive(Debug, Clone, Copy)]
pub struct CommonOverrides {
    /// Deterministic seed override.
    pub seed: Option<u64>,
    /// Force deterministic run.
    pub deterministic: bool,
}

/// Sweep parameter definition.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SweepParameter {
    /// Parameter key.
    pub key: String,
    /// Values to sweep.
    pub values: Vec<String>,
}

/// Experiment specification for batch runs.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ExperimentSpec {
    /// Dataset or scenario id.
    pub dataset_id: String,
    /// Optional config path.
    pub config_path: Option<String>,
    /// Sweep parameters.
    pub sweep: Vec<SweepParameter>,
    /// Deterministic seed.
    pub seed: Option<u64>,
    /// Output directory.
    pub out_dir: Option<String>,
}

/// Apply config overrides from CLI options.
pub fn apply_overrides(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
    code_hz: Option<f64>,
    code_length: Option<usize>,
) {
    crate::overrides::core::apply_overrides(profile, sampling_hz, if_hz, code_hz, code_length);
}

/// Apply seed/determinism overrides.
pub fn apply_common_overrides(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    common: CommonOverrides,
) {
    crate::overrides::core::apply_common_overrides(profile, common);
}

/// Apply sweep parameter overrides.
pub fn apply_sweep_value(
    profile: &mut bijux_gnss_receiver::api::ReceiverProfile,
    key: &str,
    value: &str,
) -> Result<(), bijux_gnss_receiver::api::core::InputError> {
    crate::overrides::core::apply_sweep_value(profile, key, value)
}

/// Signal API re-exports for CLI convenience.
pub use bijux_gnss_receiver::api::signal as signal;

/// Nav API re-exports for CLI convenience.
#[cfg(feature = "nav")]
pub use bijux_gnss_receiver::api::nav as nav;
