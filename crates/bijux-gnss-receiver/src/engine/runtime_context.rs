//! Runtime options for receiver execution (side-effectful controls).
#![allow(missing_docs)]

use std::path::PathBuf;

#[derive(Debug, Clone, Default)]
pub struct ReceiverRuntimeConfig {
    pub run_id: Option<String>,
    pub trace_dir: Option<PathBuf>,
    pub run_dir: Option<PathBuf>,
    pub diagnostics_dump: bool,
}
