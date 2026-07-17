//! Source-tree discovery and topology checks for guardrail evaluation.

use std::path::PathBuf;

use super::error::{GuardrailError, Result};

mod discovery;
mod topology;

pub(crate) use discovery::collect_rs_files;
pub(crate) use topology::{
    check_depth, check_empty_modules, check_mod_only_dirs, check_mod_reexports_only,
};

pub(crate) fn check_forbidden_filenames(files: &[PathBuf]) -> Result<()> {
    for path in files {
        let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
        if name.ends_with(".proptest-regressions") {
            continue;
        }
        if name == "helpers.rs" || name == "support.rs" || name == "misc.rs" {
            return Err(GuardrailError::Violation(format!(
                "forbidden filename: {}",
                path.display()
            )));
        }
    }
    Ok(())
}
