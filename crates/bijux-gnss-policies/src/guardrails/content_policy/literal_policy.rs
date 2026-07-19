use regex::Regex;
use std::fs;
use std::path::PathBuf;

use crate::guardrails::config::GuardrailConfig;
use crate::guardrails::error::{GuardrailError, Result};

pub(crate) fn check_panic_expect(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let panic_re = Regex::new(r"\bpanic!\(")?;
    let expect_re = Regex::new(r"\.expect\(")?;

    for path in files {
        let path_str = path.to_string_lossy();
        if config.allow_panic_expect_paths.iter().any(|allowed| path_str.contains(allowed)) {
            continue;
        }

        let content = fs::read_to_string(path)?;
        for (index, line) in content.lines().enumerate() {
            if panic_re.is_match(line) || expect_re.is_match(line) {
                return Err(GuardrailError::Violation(format!(
                    "panic/expect found in {}:{}",
                    path.display(),
                    index + 1
                )));
            }
        }
    }

    Ok(())
}

pub(crate) fn check_stage_id_strings(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let stage_identifier_re = Regex::new(r#"\"(fastq|bam|vcf)\.[^\"]+\""#)?;

    for path in files {
        let path_str = path.to_string_lossy();
        if config.allow_stage_id_paths.iter().any(|allowed| path_str.contains(allowed)) {
            continue;
        }

        let content = fs::read_to_string(path)?;
        if stage_identifier_re.is_match(&content) {
            return Err(GuardrailError::Violation(format!(
                "stage id literal found in {}",
                path.display()
            )));
        }
    }

    Ok(())
}
