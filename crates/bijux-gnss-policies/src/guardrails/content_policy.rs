//! Content-policy checks for guardrail evaluation.

use regex::Regex;
use std::fs;
use std::path::PathBuf;

use super::config::GuardrailConfig;
use super::error::{GuardrailError, Result};

pub(crate) fn check_purity_zones(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    if config.purity_zones.is_empty() || config.purity_forbidden.is_empty() {
        return Ok(());
    }
    let mut forbidden = Vec::new();
    for pattern in &config.purity_forbidden {
        forbidden.push(Regex::new(pattern)?);
    }
    for path in files {
        let path_str = path.display().to_string();
        if !config.purity_zones.iter().any(|zone| path_str.contains(zone)) {
            continue;
        }
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            for regex in &forbidden {
                if regex.is_match(line) {
                    return Err(GuardrailError::Violation(format!(
                        "purity violation at {}:{} -> {}",
                        path.display(),
                        idx + 1,
                        line.trim()
                    )));
                }
            }
        }
    }
    Ok(())
}

pub(crate) fn check_panic_expect(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let panic_re = Regex::new(r"\bpanic!\(")?;
    let expect_re = Regex::new(r"\.expect\(")?;
    for path in files {
        let path_str = path.to_string_lossy();
        if config.allow_panic_expect_paths.iter().any(|allowed| path_str.contains(allowed)) {
            continue;
        }
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            if panic_re.is_match(line) || expect_re.is_match(line) {
                return Err(GuardrailError::Violation(format!(
                    "panic/expect found in {}:{}",
                    path.display(),
                    idx + 1
                )));
            }
        }
    }
    Ok(())
}

pub(crate) fn check_stage_id_strings(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let stage_re = Regex::new(r#"\"(fastq|bam|vcf)\.[^\"]+\""#)?;
    for path in files {
        let path_str = path.to_string_lossy();
        if config.allow_stage_id_paths.iter().any(|allowed| path_str.contains(allowed)) {
            continue;
        }
        let content = fs::read_to_string(path)?;
        if stage_re.is_match(&content) {
            return Err(GuardrailError::Violation(format!(
                "stage id literal found in {}",
                path.display()
            )));
        }
    }
    Ok(())
}
