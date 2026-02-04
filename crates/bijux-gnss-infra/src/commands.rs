//! Command helpers for standardized runs.

use crate::dataset::DatasetEntry;
use crate::run_layout::{artifact_header, run_dir, RunContextArgs, RunDirLayout};
use bijux_gnss_receiver::api::core::ArtifactHeaderV1;
use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_receiver::api::ReceiverProfile;

/// Prepare a standard run layout and artifact header.
pub fn prepare_run(
    args: &RunContextArgs<'_>,
    command: &str,
    profile: &ReceiverProfile,
    dataset: Option<&DatasetEntry>,
) -> Result<(RunDirLayout, ArtifactHeaderV1), InputError> {
    let run_dir = run_dir(args, command, dataset)?;
    let layout = RunDirLayout::new(run_dir);
    let header = artifact_header(args, profile, dataset)?;
    Ok((layout, header))
}
