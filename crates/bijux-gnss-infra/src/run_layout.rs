//! Run directory layout and manifest utilities.

use crate::dataset::DatasetEntry;
use crate::errors::{InfraError, InfraResult};
use crate::hash::{cpu_features, git_dirty, git_hash, hash_config};
use bijux_gnss_core::{ArtifactHeaderV1, ArtifactReadPolicy};
use bijux_gnss_receiver::ReceiverProfile;
use serde::Serialize;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};

/// Run directory layout.
#[derive(Debug, Clone)]
pub struct RunDirLayout {
    /// Root run directory.
    pub run_dir: PathBuf,
    /// Artifacts directory.
    pub artifacts_dir: PathBuf,
    /// Logs directory.
    pub logs_dir: PathBuf,
    /// Summary output path.
    pub summary_path: PathBuf,
    /// Manifest output path.
    pub manifest_path: PathBuf,
}

impl RunDirLayout {
    /// Create a layout from a run directory.
    pub fn new(run_dir: PathBuf) -> Self {
        let artifacts_dir = run_dir.join("artifacts");
        let logs_dir = run_dir.join("logs");
        let summary_path = run_dir.join("summary.json");
        let manifest_path = run_dir.join("manifest.json");
        Self {
            run_dir,
            artifacts_dir,
            logs_dir,
            summary_path,
            manifest_path,
        }
    }

    /// Create directories on disk.
    pub fn create(&self) -> InfraResult<()> {
        fs::create_dir_all(&self.artifacts_dir)?;
        fs::create_dir_all(&self.logs_dir)?;
        Ok(())
    }
}

/// Run context arguments.
#[derive(Debug, Clone)]
pub struct RunContextArgs<'a> {
    /// Config path.
    pub config: Option<&'a PathBuf>,
    /// Dataset id.
    pub dataset_id: Option<&'a str>,
    /// Allow unregistered dataset.
    pub unregistered_dataset: bool,
    /// Output path override.
    pub out: Option<&'a PathBuf>,
    /// Resume path.
    pub resume: Option<&'a PathBuf>,
    /// Deterministic flag.
    pub deterministic: bool,
}

/// Run manifest persisted for each execution.
#[derive(Debug, Serialize)]
pub struct RunManifest {
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
    /// Summary payload.
    pub summary: serde_json::Value,
}

/// Run index entry appended to runs/index.jsonl.
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

#[derive(Debug, Clone)]
struct RunContext {
    layout: RunDirLayout,
}

static RUN_CONTEXT: OnceLock<RunContext> = OnceLock::new();

fn now_unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn resolve_run_context(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> InfraResult<&'static RunContext> {
    if let Some(ctx) = RUN_CONTEXT.get() {
        return Ok(ctx);
    }
    if dataset.is_none() && !args.unregistered_dataset {
        return Err(InfraError::InvalidInput(
            "dataset id is required (use --dataset or --unregistered-dataset)".to_string(),
        ));
    }
    let run_dir = if let Some(resume) = args.resume {
        resume.clone()
    } else if let Some(out) = args.out {
        out.clone()
    } else {
        let dataset_tag = dataset
            .map(|d| d.id.clone())
            .unwrap_or_else(|| "unknown".to_string());
        let stamp = now_unix_ms();
        PathBuf::from("runs").join(format!("{stamp}_{dataset_tag}_{command}"))
    };
    let layout = RunDirLayout::new(run_dir);
    layout.create()?;
    let ctx = RunContext { layout };
    let _ = RUN_CONTEXT.set(ctx);
    Ok(RUN_CONTEXT.get().expect("run context set"))
}

/// Resolve run directory path.
pub fn run_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> InfraResult<PathBuf> {
    Ok(resolve_run_context(args, command, dataset)?
        .layout
        .run_dir
        .clone())
}

/// Resolve artifacts directory path.
pub fn artifacts_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> InfraResult<PathBuf> {
    Ok(resolve_run_context(args, command, dataset)?
        .layout
        .artifacts_dir
        .clone())
}

/// Append a run index entry.
pub fn append_run_index(run_dir: &Path, manifest: &RunManifest) -> InfraResult<()> {
    let index_path = PathBuf::from("runs").join("index.jsonl");
    fs::create_dir_all(index_path.parent().unwrap_or(Path::new("runs")))?;
    let entry = RunIndexEntry {
        run_dir: run_dir.display().to_string(),
        command: manifest.command.clone(),
        timestamp_unix_ms: manifest.timestamp_unix_ms,
        git_hash: manifest.git_hash.clone(),
        dataset_id: manifest.dataset_id.clone(),
        config_hash: manifest.config_hash.clone(),
        summary: manifest.summary.clone(),
    };
    let line = serde_json::to_string(&entry)?;
    let mut file = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(index_path)?;
    writeln!(file, "{line}")?;
    Ok(())
}

/// Build an artifact header for outputs.
pub fn artifact_header(
    args: &RunContextArgs<'_>,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> InfraResult<ArtifactHeaderV1> {
    let config_hash = hash_config(args.config, profile)?;
    Ok(ArtifactHeaderV1 {
        schema_version: ArtifactReadPolicy::LATEST,
        producer: env!("CARGO_PKG_NAME").to_string(),
        producer_version: env!("CARGO_PKG_VERSION").to_string(),
        created_at_unix_ms: now_unix_ms(),
        git_sha: git_hash().unwrap_or_else(|| "unknown".to_string()),
        config_hash,
        dataset_id: dataset.map(|d| d.id.clone()),
        toolchain: std::env::var("RUSTC_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        deterministic: args.deterministic,
        git_dirty: git_dirty(),
    })
}

fn enabled_features() -> Vec<String> {
    let mut features = Vec::new();
    if let Ok(value) = std::env::var("BIJUX_GNSS_FEATURES") {
        for item in value.split(',') {
            let trimmed = item.trim();
            if !trimmed.is_empty() {
                features.push(trimmed.to_string());
            }
        }
    }
    features
}

/// Write a run manifest to disk.
pub fn write_manifest(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
    summary: &serde_json::Value,
) -> InfraResult<RunManifest> {
    let config_hash = hash_config(args.config, profile)?;
    let config_snapshot = if let Some(path) = args.config {
        fs::read_to_string(path).ok()
    } else {
        toml::to_string(profile).ok()
    };
    let manifest = RunManifest {
        command: command.to_string(),
        timestamp_unix_ms: now_unix_ms(),
        git_hash: git_hash().unwrap_or_else(|| "unknown".to_string()),
        git_dirty: git_dirty(),
        config_hash,
        config_snapshot,
        dataset_id: dataset.map(|d| d.id.clone()),
        dataset_metadata: dataset.cloned(),
        build_profile: std::env::var("BIJUX_BUILD_PROFILE").unwrap_or_else(|_| "dev".to_string()),
        cpu_features: cpu_features(),
        toolchain: std::env::var("RUSTC_VERSION").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        summary: summary.clone(),
    };
    let ctx = resolve_run_context(args, command, dataset)?;
    let data = serde_json::to_string_pretty(&manifest)?;
    fs::write(&ctx.layout.manifest_path, data)?;
    append_run_index(&ctx.layout.run_dir, &manifest)?;
    Ok(manifest)
}
