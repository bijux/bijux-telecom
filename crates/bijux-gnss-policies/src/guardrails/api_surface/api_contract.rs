use regex::Regex;
use std::fs;
use std::path::PathBuf;

use crate::guardrails::error::{GuardrailError, Result};

pub(crate) fn check_api_purity(files: &[PathBuf]) -> Result<()> {
    let implementation_re = Regex::new(r"^\s*impl\b")?;
    let public_module_re = Regex::new(r"^\s*pub\s+mod\b")?;
    let public_export_re = Regex::new(r"^\s*pub\s+(use|mod)\b")?;
    let pipeline_glob_reexport_re = Regex::new(r"^\s*pub\s+use\s+crate::pipeline::.*\*")?;

    for path in files {
        if path.file_name().and_then(|segment| segment.to_str()) != Some("api.rs") {
            continue;
        }

        let content = fs::read_to_string(path)?;
        for (index, line) in content.lines().enumerate() {
            if implementation_re.is_match(line) {
                return Err(GuardrailError::Violation(format!(
                    "impl block not allowed in api.rs at {}:{}",
                    path.display(),
                    index + 1
                )));
            }
            if public_module_re.is_match(line) {
                return Err(GuardrailError::Violation(format!(
                    "pub mod not allowed in api.rs at {}:{}",
                    path.display(),
                    index + 1
                )));
            }
            if public_export_re.is_match(line) && line.contains("_internal") {
                return Err(GuardrailError::Violation(format!(
                    "public export of _internal not allowed in api.rs at {}:{}",
                    path.display(),
                    index + 1
                )));
            }
            if pipeline_glob_reexport_re.is_match(line) {
                return Err(GuardrailError::Violation(format!(
                    "wildcard re-export of pipeline not allowed in api.rs at {}:{}",
                    path.display(),
                    index + 1
                )));
            }
        }
    }

    Ok(())
}
