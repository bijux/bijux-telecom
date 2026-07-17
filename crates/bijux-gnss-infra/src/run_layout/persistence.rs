use crate::datasets::DatasetEntry;
use crate::run_layout::{RunContextArgs, RunManifest, RunReport};
use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverConfig;

/// Write a run report to disk.
pub fn write_run_report(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<RunReport, InputError> {
    super::records::report::write_run_report(args, command, profile, dataset)
}

/// Write a run manifest to disk.
pub fn write_manifest(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
    summary: &serde_json::Value,
) -> Result<RunManifest, InputError> {
    super::records::manifest::write_manifest(args, command, profile, dataset, summary)
}
