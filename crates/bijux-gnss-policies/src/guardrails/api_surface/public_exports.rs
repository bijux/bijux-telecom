use regex::Regex;
use std::fs;
use std::path::PathBuf;

use crate::guardrails::config::GuardrailConfig;
use crate::guardrails::error::{GuardrailError, Result};

pub(crate) fn check_pub_items(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let public_item_re =
        Regex::new(r"^\s*pub(\s*\(crate\))?\s+(struct|enum|fn|type|trait|const|static|use|mod)\b")?;

    for path in files {
        let content = fs::read_to_string(path)?;
        let count = content.lines().filter(|line| public_item_re.is_match(line)).count();
        if count > config.max_pub_items_per_file {
            return Err(GuardrailError::Violation(format!(
                "{} has {} pub items (max {})",
                path.display(),
                count,
                config.max_pub_items_per_file
            )));
        }
    }

    Ok(())
}

pub(crate) fn check_pub_items_outside_api(files: &[PathBuf]) -> Result<()> {
    let public_item_re =
        Regex::new(r"^\s*pub\s+(struct|enum|fn|type|trait|const|static|use|mod)\b")?;

    for path in files {
        if path.file_name().and_then(|segment| segment.to_str()) != Some("lib.rs") {
            continue;
        }

        let content = fs::read_to_string(path)?;
        for (index, line) in content.lines().enumerate() {
            if !public_item_re.is_match(line) {
                continue;
            }
            if line.trim() == "pub mod api;" {
                continue;
            }
            return Err(GuardrailError::Violation(format!(
                "public items must be confined to api.rs (found in {}:{})",
                path.display(),
                index + 1
            )));
        }
    }

    Ok(())
}

pub(crate) fn check_pub_use_spam(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let public_use_re = Regex::new(r"^\s*pub\s+use\b")?;

    for path in files {
        let content = fs::read_to_string(path)?;
        let count = content.lines().filter(|line| public_use_re.is_match(line)).count();
        if count > config.max_pub_use_per_file {
            return Err(GuardrailError::Violation(format!(
                "{} has {} pub use re-exports (max {})",
                path.display(),
                count,
                config.max_pub_use_per_file
            )));
        }
    }

    Ok(())
}

pub(crate) fn check_pub_use_locations_if_enabled(
    files: &[PathBuf],
    config: &GuardrailConfig,
) -> Result<()> {
    if !config.enforce_pub_use_api_only {
        return Ok(());
    }
    check_pub_use_locations(files)
}

fn check_pub_use_locations(files: &[PathBuf]) -> Result<()> {
    let public_use_re = Regex::new(r"^\s*pub\s+use\b")?;

    for path in files {
        if path.extension().and_then(|extension| extension.to_str()) != Some("rs") {
            continue;
        }

        let is_api_file = path.file_name().and_then(|segment| segment.to_str()) == Some("api.rs");
        let is_library_root = path.file_name().and_then(|segment| segment.to_str()) == Some("lib.rs");
        let content = fs::read_to_string(path)?;

        for (index, line) in content.lines().enumerate() {
            if !public_use_re.is_match(line) {
                continue;
            }
            if is_api_file {
                continue;
            }
            if is_library_root && line.contains("api::") {
                continue;
            }
            return Err(GuardrailError::Violation(format!(
                "pub use outside api.rs at {}:{}",
                path.display(),
                index + 1
            )));
        }
    }

    Ok(())
}
