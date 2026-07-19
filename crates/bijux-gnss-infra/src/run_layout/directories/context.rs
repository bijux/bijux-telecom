//! Run context resolution helpers.

use std::sync::OnceLock;

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::datasets::DatasetEntry;
use crate::hash::hash_config;

use super::layout::RunDirectoryLayout;
use crate::run_layout::identity::{dataset_hash, now_unix_ms, run_id};

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
pub(crate) struct RunContext {
    pub(crate) layout: RunDirectoryLayout,
}

static RUN_CONTEXT: OnceLock<RunContext> = OnceLock::new();
const DEFAULT_RUNS_DIR: &str = "artifacts/runs";

pub(crate) fn resolve_run_context(
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
    let run_dir = resolve_run_dir(args, command, dataset)?;
    let layout = RunDirectoryLayout::new(run_dir);
    layout.create()?;
    let context = RunContext { layout };
    if should_reuse_cached_context {
        let _ = RUN_CONTEXT.set(context.clone());
    }
    Ok(context)
}

fn resolve_run_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<std::path::PathBuf, InputError> {
    Ok(if let Some(resume) = args.resume {
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
        std::path::PathBuf::from(DEFAULT_RUNS_DIR).join(name)
    })
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

#[cfg(test)]
mod tests {
    use super::{resolve_run_dir, RunContextArgs, DEFAULT_RUNS_DIR};

    fn args<'a>(
        out: Option<&'a std::path::PathBuf>,
        resume: Option<&'a std::path::PathBuf>,
    ) -> RunContextArgs<'a> {
        RunContextArgs {
            config: None,
            dataset_id: None,
            unregistered_dataset: true,
            out,
            resume,
            deterministic: true,
            sidecar: None,
        }
    }

    #[test]
    fn implicit_run_directory_stays_under_artifacts() {
        let run_dir = resolve_run_dir(&args(None, None), "inspect", None).unwrap();

        assert!(run_dir.starts_with(DEFAULT_RUNS_DIR));
    }

    #[test]
    fn explicit_output_directory_is_preserved() {
        let output = std::path::PathBuf::from("declared/output");

        assert_eq!(resolve_run_dir(&args(Some(&output), None), "inspect", None).unwrap(), output);
    }

    #[test]
    fn resume_directory_takes_precedence_over_output() {
        let output = std::path::PathBuf::from("declared/output");
        let resume = std::path::PathBuf::from("declared/resume");

        assert_eq!(
            resolve_run_dir(&args(Some(&output), Some(&resume)), "inspect", None).unwrap(),
            resume
        );
    }
}
