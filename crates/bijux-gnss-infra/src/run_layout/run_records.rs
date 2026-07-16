//! Run index and artifact header helpers.

use std::io::Write;

use bijux_gnss_receiver::api::core::{ArtifactHeaderV1, ArtifactReadPolicy, InputError};
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;

use crate::datasets::DatasetEntry;
use crate::hash::{git_dirty, git_hash, hash_config};

use super::identity::now_unix_ms;
use super::manifest::RunManifest;
use super::provenance::enabled_features;
use super::RunContextArgs;

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

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
