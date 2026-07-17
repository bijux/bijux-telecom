//! Run report persistence helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;

use crate::datasets::DatasetEntry;
use crate::hash::{git_hash, hash_config};

use crate::run_layout::directories::context::{resolve_run_context, RunContextArgs};
use crate::run_layout::identity::{
    dataset_hash, run_id, RUN_LAYOUT_SCHEMA_VERSION, RUN_REPORT_SCHEMA_VERSION,
};
use crate::run_layout::provenance::{front_end_provenance, replay_scope};
use crate::run_layout::{FrontEndProvenance, ReplayScope};

/// Run report persisted for each execution.
#[derive(Debug, Serialize)]
pub struct RunReport {
    /// Schema version.
    pub schema_version: u32,
    /// Deterministic run id.
    pub run_id: String,
    /// Config hash.
    pub config_hash: String,
    /// Dataset id.
    pub dataset_id: Option<String>,
    /// Dataset hash.
    pub dataset_hash: Option<String>,
    /// Git commit hash.
    pub git_hash: String,
    /// Build profile.
    pub build_profile: String,
    /// Toolchain string.
    pub toolchain: String,
    /// Build version string.
    pub build_version: String,
    /// Build git hash.
    pub build_git: String,
    /// Build timestamp (optional).
    pub build_timestamp: String,
    /// Layout schema version for run directory contracts.
    pub layout_schema_version: u32,
    /// Replay scope required to re-run with matching context.
    pub replay_scope: ReplayScope,
    /// Front-end provenance that can affect scientific interpretation.
    pub front_end_provenance: FrontEndProvenance,
}

/// Load run report schema version.
pub fn run_report_schema_version() -> u32 {
    RUN_REPORT_SCHEMA_VERSION
}

/// Write a run report to disk.
pub fn write_run_report(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<RunReport, InputError> {
    let config_hash = hash_config(args.config, profile)?;
    let dataset_hash = dataset.map(dataset_hash).transpose()?;
    let build_version = env!("CARGO_PKG_VERSION");
    let report = RunReport {
        schema_version: RUN_REPORT_SCHEMA_VERSION,
        run_id: run_id(&config_hash, dataset_hash.as_deref(), build_version),
        config_hash,
        dataset_id: dataset.map(|entry| entry.id.clone()),
        dataset_hash,
        git_hash: git_hash().unwrap_or_else(|| "unknown".to_string()),
        build_profile: std::env::var("BIJUX_BUILD_PROFILE").unwrap_or_else(|_| "dev".to_string()),
        toolchain: std::env::var("BIJUX_BUILD_RUSTC").unwrap_or_else(|_| "unknown".to_string()),
        build_version: build_version.to_string(),
        build_git: std::env::var("BIJUX_BUILD_GIT_SHA").unwrap_or_else(|_| "unknown".to_string()),
        build_timestamp: std::env::var("BIJUX_BUILD_TIMESTAMP")
            .unwrap_or_else(|_| "unknown".to_string()),
        layout_schema_version: RUN_LAYOUT_SCHEMA_VERSION,
        replay_scope: replay_scope(args),
        front_end_provenance: front_end_provenance(args, profile, dataset)?,
    };
    let context = resolve_run_context(args, command, dataset)?;
    let data = serde_json::to_string_pretty(&report).map_err(map_err)?;
    let path = context.layout.run_dir.join("run_report.json");
    std::fs::write(path, data).map_err(map_err)?;
    std::env::set_var("BIJUX_RUN_ID", &report.run_id);
    Ok(report)
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
