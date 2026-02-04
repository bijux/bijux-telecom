//! Run directory layout and manifest utilities.

use crate::dataset::DatasetEntry;
use crate::hash::core::{cpu_features, git_dirty, git_hash, hash_config};
use bijux_gnss_receiver::api::core::{ArtifactHeaderV1, ArtifactReadPolicy, InputError};
use bijux_gnss_receiver::api::ReceiverConfig;
use serde::Serialize;
use sha2::{Digest, Sha256};
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
    pub fn create(&self) -> Result<(), InputError> {
        fs::create_dir_all(&self.artifacts_dir).map_err(map_err)?;
        fs::create_dir_all(&self.logs_dir).map_err(map_err)?;
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
const RUN_REPORT_SCHEMA_VERSION: u32 = 1;

fn now_unix_ms(deterministic: bool) -> u128 {
    if deterministic {
        return 0;
    }
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

fn dataset_hash(dataset: &DatasetEntry) -> Result<String, InputError> {
    let serialized = serde_json::to_vec(dataset).map_err(map_err)?;
    let mut hasher = Sha256::new();
    hasher.update(serialized);
    Ok(hex::encode(hasher.finalize()))
}

fn run_id(config_hash: &str, dataset_hash: Option<&str>, build_version: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(config_hash.as_bytes());
    if let Some(hash) = dataset_hash {
        hasher.update(hash.as_bytes());
    }
    hasher.update(build_version.as_bytes());
    hex::encode(hasher.finalize())
}

fn resolve_run_context(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<&'static RunContext, InputError> {
    if let Some(ctx) = RUN_CONTEXT.get() {
        return Ok(ctx);
    }
    if dataset.is_none() && !args.unregistered_dataset {
        return Err(InputError {
            message: "dataset id is required (use --dataset or --unregistered-dataset)".to_string(),
        });
    }
    let run_dir = if let Some(resume) = args.resume {
        resume.clone()
    } else if let Some(out) = args.out {
        out.clone()
    } else {
        let dataset_tag = dataset
            .map(|d| d.id.clone())
            .unwrap_or_else(|| "unknown".to_string());
        let config_hash = hash_config(args.config, &ReceiverConfig::default())?;
        let dataset_hash = dataset
            .map(dataset_hash)
            .transpose()?
            .unwrap_or_else(|| "unknown".to_string());
        let build_version = env!("CARGO_PKG_VERSION");
        let id = run_id(&config_hash, Some(&dataset_hash), build_version);
        let mut name = format!("{id}_{dataset_tag}_{command}");
        if !args.deterministic {
            let stamp = now_unix_ms(false);
            name = format!("{name}_{stamp}");
        }
        PathBuf::from("runs").join(name)
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
) -> Result<PathBuf, InputError> {
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
) -> Result<PathBuf, InputError> {
    Ok(resolve_run_context(args, command, dataset)?
        .layout
        .artifacts_dir
        .clone())
}

/// Append a run index entry.
pub fn append_run_index(run_dir: &Path, manifest: &RunManifest) -> Result<(), InputError> {
    let index_path = PathBuf::from("runs").join("index.jsonl");
    fs::create_dir_all(index_path.parent().unwrap_or(Path::new("runs"))).map_err(map_err)?;
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
    let mut file = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(index_path)
        .map_err(map_err)?;
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
        dataset_id: dataset.map(|d| d.id.clone()),
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
    let id = run_id(&config_hash, dataset_hash.as_deref(), build_version);
    let report = RunReport {
        schema_version: RUN_REPORT_SCHEMA_VERSION,
        run_id: id,
        config_hash,
        dataset_id: dataset.map(|d| d.id.clone()),
        dataset_hash,
        git_hash: git_hash().unwrap_or_else(|| "unknown".to_string()),
        build_profile: std::env::var("BIJUX_BUILD_PROFILE").unwrap_or_else(|_| "dev".to_string()),
        toolchain: std::env::var("BIJUX_BUILD_RUSTC").unwrap_or_else(|_| "unknown".to_string()),
        build_version: build_version.to_string(),
        build_git: std::env::var("BIJUX_BUILD_GIT_SHA").unwrap_or_else(|_| "unknown".to_string()),
        build_timestamp: std::env::var("BIJUX_BUILD_TIMESTAMP")
            .unwrap_or_else(|_| "unknown".to_string()),
    };
    let ctx = resolve_run_context(args, command, dataset)?;
    let data = serde_json::to_string_pretty(&report).map_err(map_err)?;
    let path = ctx.layout.run_dir.join("run_report.json");
    fs::write(path, data).map_err(map_err)?;
    std::env::set_var("BIJUX_RUN_ID", &report.run_id);
    Ok(report)
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
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
    summary: &serde_json::Value,
) -> Result<RunManifest, InputError> {
    let config_hash = hash_config(args.config, profile)?;
    let config_snapshot = if let Some(path) = args.config {
        fs::read_to_string(path).ok()
    } else {
        toml::to_string(profile).ok()
    };
    let manifest = RunManifest {
        command: command.to_string(),
        timestamp_unix_ms: now_unix_ms(args.deterministic),
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
    let data = serde_json::to_string_pretty(&manifest).map_err(map_err)?;
    fs::write(&ctx.layout.manifest_path, data).map_err(map_err)?;
    append_run_index(&ctx.layout.run_dir, &manifest)?;
    Ok(manifest)
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError {
        message: err.to_string(),
    }
}
