use std::path::PathBuf;

use crate::guardrails::error::{GuardrailError, Result};

pub(crate) fn check_forbidden_filenames(files: &[PathBuf]) -> Result<()> {
    for path in files {
        let name = path.file_name().and_then(|segment| segment.to_str()).unwrap_or("");
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
