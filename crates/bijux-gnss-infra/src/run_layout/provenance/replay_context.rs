//! Replay context helpers.

use crate::run_layout::directories::context::RunContextArgs;

/// Replay scope persisted in run manifests and reports.
#[derive(Debug, serde::Serialize, Clone)]
pub struct ReplayScope {
    /// Whether deterministic mode was requested.
    pub deterministic: bool,
    /// Whether run resumed from a prior run directory.
    pub resume: bool,
    /// Whether output directory was explicitly requested.
    pub explicit_output_dir: bool,
    /// Requested dataset id from CLI args.
    pub requested_dataset_id: Option<String>,
    /// Whether unregistered datasets were allowed.
    pub allow_unregistered_dataset: bool,
}

pub(crate) fn replay_scope(args: &RunContextArgs<'_>) -> ReplayScope {
    ReplayScope {
        deterministic: args.deterministic,
        resume: args.resume.is_some(),
        explicit_output_dir: args.out.is_some(),
        requested_dataset_id: args.dataset_id.map(str::to_string),
        allow_unregistered_dataset: args.unregistered_dataset,
    }
}
