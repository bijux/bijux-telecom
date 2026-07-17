use regex::Regex;
use std::fs;
use std::path::PathBuf;

use crate::guardrails::config::GuardrailConfig;
use crate::guardrails::error::{GuardrailError, Result};

pub(crate) fn check_purity_zones(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    if config.purity_zones.is_empty() || config.purity_forbidden.is_empty() {
        return Ok(());
    }

    let mut forbidden_patterns = Vec::new();
    for pattern in &config.purity_forbidden {
        forbidden_patterns.push(Regex::new(pattern)?);
    }

    for path in files {
        let path_str = path.display().to_string();
        if !config.purity_zones.iter().any(|zone| path_str.contains(zone)) {
            continue;
        }

        let content = fs::read_to_string(path)?;
        for (index, line) in content.lines().enumerate() {
            for pattern in &forbidden_patterns {
                if pattern.is_match(line) {
                    return Err(GuardrailError::Violation(format!(
                        "purity violation at {}:{} -> {}",
                        path.display(),
                        index + 1,
                        line.trim()
                    )));
                }
            }
        }
    }

    Ok(())
}
