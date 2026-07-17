//! Run manifest persistence helpers.

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;

use crate::datasets::DatasetEntry;
use crate::hash::hash_config;

use super::build_metadata::build_metadata;
use super::history::append_run_history_entry;
use crate::run_layout::directories::context::{resolve_run_context, RunContextArgs};
use crate::run_layout::identity::{now_unix_ms, RUN_LAYOUT_SCHEMA_VERSION};
use crate::run_layout::provenance::{front_end_provenance, replay_scope};
use crate::run_layout::{FrontEndProvenance, ReplayScope};

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
    let build_metadata = build_metadata();
    let config_snapshot = if let Some(path) = args.config {
        std::fs::read_to_string(path).ok()
    } else {
        toml::to_string(profile).ok()
    };
    let manifest = RunManifest {
        layout_schema_version: RUN_LAYOUT_SCHEMA_VERSION,
        command: command.to_string(),
        timestamp_unix_ms: now_unix_ms(args.deterministic),
        git_hash: build_metadata.git_hash,
        git_dirty: build_metadata.git_dirty,
        config_hash,
        config_snapshot,
        dataset_id: dataset.map(|entry| entry.id.clone()),
        dataset_metadata: dataset.cloned(),
        build_profile: build_metadata.build_profile,
        cpu_features: build_metadata.cpu_features,
        toolchain: build_metadata.toolchain,
        features: build_metadata.features,
        replay_scope: replay_scope(args),
        front_end_provenance: front_end_provenance(args, profile, dataset)?,
        summary: summary.clone(),
    };
    let context = resolve_run_context(args, command, dataset)?;
    let data = serde_json::to_string_pretty(&manifest).map_err(map_err)?;
    std::fs::write(&context.layout.manifest_path, data).map_err(map_err)?;
    append_run_history_entry(&context.layout.run_dir, &manifest)?;
    Ok(manifest)
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
