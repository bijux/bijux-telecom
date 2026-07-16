//! Run manifest persistence helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;

use crate::datasets::DatasetEntry;
use crate::hash::{cpu_features, git_dirty, git_hash, hash_config};

use super::context::{resolve_run_context, RunContextArgs};
use super::identity::{now_unix_ms, RUN_LAYOUT_SCHEMA_VERSION};
use super::provenance::{enabled_features, front_end_provenance, replay_scope};
use super::run_records::append_run_index;
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
