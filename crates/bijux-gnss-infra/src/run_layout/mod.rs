//! Run directory layout and manifest utilities.

mod directories;
mod identity;
mod provenance;
mod records;

/// Run directory layout.
pub type RunDirectoryLayout = directories::layout::RunDirectoryLayout;

/// Run context arguments.
pub type RunContextArgs<'a> = directories::context::RunContextArgs<'a>;

/// Run manifest persisted for each execution.
pub type RunManifest = records::manifest::RunManifest;

/// Run report persisted for each execution.
pub type RunReport = records::report::RunReport;

/// Replay scope persisted in run manifests and reports.
pub type ReplayScope = provenance::ReplayScope;

/// Front-end provenance captured at run time.
pub type FrontEndProvenance = provenance::FrontEndProvenance;

/// Run index entry appended to `artifacts/runs/index.jsonl`.
pub type RunIndexEntry = records::index::RunIndexEntry;

/// Resolve run directory path.
pub fn run_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&crate::datasets::DatasetEntry>,
) -> Result<std::path::PathBuf, bijux_gnss_receiver::api::core::InputError> {
    directories::context::run_dir(args, command, dataset)
}

/// Resolve artifacts directory path.
pub fn artifacts_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&crate::datasets::DatasetEntry>,
) -> Result<std::path::PathBuf, bijux_gnss_receiver::api::core::InputError> {
    directories::context::artifacts_dir(args, command, dataset)
}

/// Append a run index entry.
pub fn append_run_index(
    run_dir: &std::path::Path,
    manifest: &RunManifest,
) -> Result<(), bijux_gnss_receiver::api::core::InputError> {
    records::index::append_run_index(run_dir, manifest)
}

/// Build an artifact header for outputs.
pub fn artifact_header(
    args: &RunContextArgs<'_>,
    profile: &bijux_gnss_receiver::api::ReceiverConfig,
    dataset: Option<&crate::datasets::DatasetEntry>,
) -> Result<
    bijux_gnss_receiver::api::core::ArtifactHeaderV1,
    bijux_gnss_receiver::api::core::InputError,
> {
    records::artifact_header::artifact_header(args, profile, dataset)
}

/// Load run report schema version.
pub fn run_report_schema_version() -> u32 {
    records::report::run_report_schema_version()
}

/// Write a run report to disk.
pub fn write_run_report(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &bijux_gnss_receiver::api::ReceiverConfig,
    dataset: Option<&crate::datasets::DatasetEntry>,
) -> Result<RunReport, bijux_gnss_receiver::api::core::InputError> {
    records::report::write_run_report(args, command, profile, dataset)
}

/// Write a run manifest to disk.
pub fn write_manifest(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &bijux_gnss_receiver::api::ReceiverConfig,
    dataset: Option<&crate::datasets::DatasetEntry>,
    summary: &serde_json::Value,
) -> Result<RunManifest, bijux_gnss_receiver::api::core::InputError> {
    records::manifest::write_manifest(args, command, profile, dataset, summary)
}
