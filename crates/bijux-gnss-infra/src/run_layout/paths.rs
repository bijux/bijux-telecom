use crate::datasets::DatasetEntry;
use crate::run_layout::RunContextArgs;
use bijux_gnss_receiver::api::core::InputError;

/// Resolve run directory path.
pub fn run_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<std::path::PathBuf, InputError> {
    super::directories::context::run_dir(args, command, dataset)
}

/// Resolve artifacts directory path.
pub fn artifacts_dir(
    args: &RunContextArgs<'_>,
    command: &str,
    dataset: Option<&DatasetEntry>,
) -> Result<std::path::PathBuf, InputError> {
    super::directories::context::artifacts_dir(args, command, dataset)
}
