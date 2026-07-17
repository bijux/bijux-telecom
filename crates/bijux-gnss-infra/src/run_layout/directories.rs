//! Run directory resolution and layout helpers.

mod context;
mod layout;

pub(crate) use context::{artifacts_dir, resolve_run_context, run_dir, RunContextArgs};
pub(crate) use layout::RunDirectoryLayout;
