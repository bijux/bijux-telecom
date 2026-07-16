//! Run context resolution helpers.

use std::sync::OnceLock;

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::datasets::DatasetEntry;
use crate::hash::hash_config;

use super::identity::{dataset_hash, now_unix_ms, run_id};
use super::layout::RunDirLayout;

/// Run context arguments.
#[derive(Debug, Clone)]
pub struct RunContextArgs<'a> {
    /// Config path.
    pub config: Option<&'a std::path::PathBuf>,
    /// Dataset id.
    pub dataset_id: Option<&'a str>,
    /// Allow unregistered dataset.
    pub unregistered_dataset: bool,
    /// Output path override.
    pub out: Option<&'a std::path::PathBuf>,
    /// Resume path.
    pub resume: Option<&'a std::path::PathBuf>,
    /// Deterministic flag.
    pub deterministic: bool,
    /// Optional sidecar metadata path.
    pub sidecar: Option<&'a std::path::PathBuf>,
}

#[derive(Debug, Clone)]
pub(super) struct RunContext {
    pub(super) layout: RunDirLayout,
}

static RUN_CONTEXT: OnceLock<RunContext> = OnceLock::new();

pub(super) fn resolve_run_context(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<RunContext, InputError> {
    let should_reuse_cached_context = args.out.is_none() && args.resume.is_none();
    if should_reuse_cached_context {
        if let Some(context) = RUN_CONTEXT.get() {
            return Ok(context.clone());
        }
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
        let dataset_tag =
            dataset.map(|entry| entry.id.clone()).unwrap_or_else(|| "unknown".to_string());
        let config_hash = hash_config(args.config, &ReceiverConfig::default())?;
        let dataset_hash =
            dataset.map(dataset_hash).transpose()?.unwrap_or_else(|| "unknown".to_string());
        let build_version = env!("CARGO_PKG_VERSION");
        let id = run_id(&config_hash, Some(&dataset_hash), build_version);
        let mut name = format!("{id}_{dataset_tag}_{command}");
        if !args.deterministic {
            let stamp = now_unix_ms(false);
            name = format!("{name}_{stamp}");
        }
        std::path::PathBuf::from("runs").join(name)
    };
    let layout = RunDirLayout::new(run_dir);
    layout.create()?;
    let context = RunContext { layout };
    if should_reuse_cached_context {
        let _ = RUN_CONTEXT.set(context.clone());
    }
    Ok(context)
}

/// Resolve run directory path.
pub fn run_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<std::path::PathBuf, InputError> {
    Ok(resolve_run_context(args, command, dataset)?.layout.run_dir.clone())
}

/// Resolve artifacts directory path.
pub fn artifacts_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<std::path::PathBuf, InputError> {
    Ok(resolve_run_context(args, command, dataset)?.layout.artifacts_dir.clone())
}
