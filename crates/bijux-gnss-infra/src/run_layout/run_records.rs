//! Run manifest and reporting helpers.

use std::io::Write;

use bijux_gnss_receiver::api::core::{ArtifactHeaderV1, ArtifactReadPolicy, InputError};
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;

use crate::datasets::DatasetEntry;
use crate::hash::{cpu_features, git_dirty, git_hash, hash_config};

use super::context::{resolve_run_context, RunContextArgs};
use super::provenance::{enabled_features, front_end_provenance, replay_scope};
use super::run_identity::{
    dataset_hash, now_unix_ms, run_id, RUN_LAYOUT_SCHEMA_VERSION, RUN_REPORT_SCHEMA_VERSION,
};
use super::{FrontEndProvenance, ReplayScope};

/// Run manifest persisted for each execution.
#[derive(Debug, Serialize)]
pub struct RunManifest {
    /// Layout schema version for run directory contracts.
    pub layout_schema_version: u32,
    /// Command name.
    pub command: String,
    /// Timestamp in Unix milliseconds.
    pub timestamp_unix_ms: u128,
    /// Git commit hash.
    pub git_hash: String,
    /// Dirty flag.
    pub git_dirty: bool,
    /// Config hash.
    pub config_hash: String,
    /// Optional config snapshot.
    pub config_snapshot: Option<String>,
    /// Dataset id.
    pub dataset_id: Option<String>,
    /// Dataset metadata.
    pub dataset_metadata: Option<DatasetEntry>,
    /// Build profile.
    pub build_profile: String,
    /// CPU features.
    pub cpu_features: Vec<String>,
    /// Toolchain string.
    pub toolchain: String,
    /// Enabled features.
    pub features: Vec<String>,
    /// Replay scope required to re-run with matching context.
    pub replay_scope: ReplayScope,
    /// Front-end provenance that can affect scientific interpretation.
    pub front_end_provenance: FrontEndProvenance,
    /// Summary payload.
    pub summary: serde_json::Value,
}

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

/// Run index entry appended to `artifacts/runs/index.jsonl`.
#[derive(Debug, Serialize)]
pub struct RunIndexEntry {
    /// Run directory path.
    pub run_dir: String,
    /// Command name.
    pub command: String,
    /// Timestamp in Unix milliseconds.
    pub timestamp_unix_ms: u128,
    /// Git hash.
    pub git_hash: String,
    /// Dataset id.
    pub dataset_id: Option<String>,
    /// Config hash.
    pub config_hash: String,
    /// Summary payload.
    pub summary: serde_json::Value,
}

/// Append a run index entry.
pub fn append_run_index(
    run_dir: &std::path::Path,
    manifest: &RunManifest,
) -> Result<(), InputError> {
    let index_path = std::path::PathBuf::from("artifacts").join("runs").join("index.jsonl");
    std::fs::create_dir_all(index_path.parent().unwrap_or(std::path::Path::new("artifacts")))
        .map_err(map_err)?;
    let entry = RunIndexEntry {
        run_dir: run_dir.display().to_string(),
        command: manifest.command.clone(),
        timestamp_unix_ms: manifest.timestamp_unix_ms,
        git_hash: manifest.git_hash.clone(),
        dataset_id: manifest.dataset_id.clone(),
        config_hash: manifest.config_hash.clone(),
        summary: manifest.summary.clone(),
    };
    let line = serde_json::to_string(&entry).map_err(map_err)?;
    let mut file =
        std::fs::OpenOptions::new().create(true).append(true).open(index_path).map_err(map_err)?;
    writeln!(file, "{line}").map_err(map_err)?;
    Ok(())
}

/// Build an artifact header for outputs.
pub fn artifact_header(
    args: &RunContextArgs<'_>,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<ArtifactHeaderV1, InputError> {
    let config_hash = hash_config(args.config, profile)?;
    Ok(ArtifactHeaderV1 {
        schema_version: ArtifactReadPolicy::LATEST,
        producer: env!("CARGO_PKG_NAME").to_string(),
        producer_version: env!("CARGO_PKG_VERSION").to_string(),
        created_at_unix_ms: now_unix_ms(args.deterministic),
        git_sha: git_hash().unwrap_or_else(|| "unknown".to_string()),
        config_hash,
        dataset_id: dataset.map(|entry| entry.id.clone()),
        toolchain: std::env::var("BIJUX_BUILD_RUSTC").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        deterministic: args.deterministic,
        git_dirty: git_dirty(),
    })
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

/// Write a run manifest to disk.
pub fn write_manifest(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
    summary: &serde_json::Value,
) -> Result<RunManifest, InputError> {
    let config_hash = hash_config(args.config, profile)?;
    let config_snapshot = if let Some(path) = args.config {
        std::fs::read_to_string(path).ok()
    } else {
        toml::to_string(profile).ok()
    };
    let manifest = RunManifest {
        layout_schema_version: RUN_LAYOUT_SCHEMA_VERSION,
        command: command.to_string(),
        timestamp_unix_ms: now_unix_ms(args.deterministic),
        git_hash: git_hash().unwrap_or_else(|| "unknown".to_string()),
        git_dirty: git_dirty(),
        config_hash,
        config_snapshot,
        dataset_id: dataset.map(|entry| entry.id.clone()),
        dataset_metadata: dataset.cloned(),
        build_profile: std::env::var("BIJUX_BUILD_PROFILE").unwrap_or_else(|_| "dev".to_string()),
        cpu_features: cpu_features(),
        toolchain: std::env::var("RUSTC_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        replay_scope: replay_scope(args),
        front_end_provenance: front_end_provenance(args, profile, dataset)?,
        summary: summary.clone(),
    };
    let context = resolve_run_context(args, command, dataset)?;
    let data = serde_json::to_string_pretty(&manifest).map_err(map_err)?;
    std::fs::write(&context.layout.manifest_path, data).map_err(map_err)?;
    append_run_index(&context.layout.run_dir, &manifest)?;
    Ok(manifest)
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
