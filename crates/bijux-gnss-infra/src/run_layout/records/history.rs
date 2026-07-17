//! Run history persistence helpers.

use std::io::Write;

use bijux_gnss_receiver::api::core::InputError;
use serde::Serialize;

use super::manifest::RunManifest;

/// Run history entry appended to `artifacts/runs/index.jsonl`.
#[derive(Debug, Serialize)]
pub struct RunHistoryEntry {
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

/// Append a run history entry.
pub fn append_run_history_entry(
    run_dir: &std::path::Path,
    manifest: &RunManifest,
) -> Result<(), InputError> {
    let index_path = std::path::PathBuf::from("artifacts").join("runs").join("index.jsonl");
    std::fs::create_dir_all(index_path.parent().unwrap_or(std::path::Path::new("artifacts")))
        .map_err(map_err)?;
    let entry = RunHistoryEntry {
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

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
