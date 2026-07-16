//! Source-tree discovery and topology checks for guardrail evaluation.

use std::fs;
use std::path::{Path, PathBuf};

use walkdir::WalkDir;

use super::config::GuardrailConfig;
use super::error::{GuardrailError, Result};

pub(crate) fn collect_rs_files(root: &Path) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in WalkDir::new(root) {
        let entry = entry?;
        if entry.file_type().is_file()
            && entry.path().extension().and_then(|s| s.to_str()) == Some("rs")
        {
            files.push(entry.into_path());
        }
    }
    Ok(files)
}

pub(crate) fn check_depth(
    src_dir: &Path,
    files: &[PathBuf],
    config: &GuardrailConfig,
) -> Result<()> {
    for path in files {
        let rel = path.strip_prefix(src_dir).unwrap_or(path.as_path());
        let components: Vec<_> = rel.components().collect();
        if components.len() <= config.max_depth {
            continue;
        }
        let is_mod_rs =
            path.file_name().and_then(|name| name.to_str()).is_some_and(|name| name == "mod.rs");
        if components.len() == config.max_depth + 1 && is_mod_rs {
            continue;
        }
        return Err(GuardrailError::Violation(format!(
            "module depth exceeds allowed rule (src/a/b/c.rs or mod.rs at each level): {}",
            path.display()
        )));
    }
    Ok(())
}

pub(crate) fn check_mod_only_dirs(src_dir: &Path) -> Result<()> {
    for entry in WalkDir::new(src_dir).min_depth(1).max_depth(10) {
        let entry = entry?;
        if !entry.file_type().is_dir() {
            continue;
        }
        if entry.path() == src_dir {
            continue;
        }
        if entry.path().components().any(|component| component.as_os_str() == "tests") {
            continue;
        }
        let mut rs_files = Vec::new();
        for child in fs::read_dir(entry.path())? {
            let child = child?;
            let path = child.path();
            if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("rs") {
                if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                    rs_files.push(name.to_string());
                }
            }
        }
        if rs_files.is_empty() {
            continue;
        }
        let allowed = rs_files.iter().all(|name| name == "mod.rs" || name == "tests.rs");
        if allowed && rs_files.iter().any(|name| name == "mod.rs") && rs_files.len() <= 2 {
            return Err(GuardrailError::Violation(format!(
                "module directory contains only mod.rs (and optionally tests.rs): {}",
                entry.path().display()
            )));
        }
    }
    Ok(())
}

pub(crate) fn check_empty_modules(files: &[PathBuf]) -> Result<()> {
    for path in files {
        if path.file_name().and_then(|s| s.to_str()) != Some("mod.rs") {
            continue;
        }
        let content = fs::read_to_string(path)?;
        let mut meaningful = 0usize;
        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("//") {
                continue;
            }
            if trimmed.starts_with("pub mod ") || trimmed.starts_with("mod ") {
                continue;
            }
            meaningful += 1;
        }
        if meaningful == 0 {
            return Err(GuardrailError::Violation(format!(
                "empty module file (only mod re-exports): {}",
                path.display()
            )));
        }
    }
    Ok(())
}

pub(crate) fn check_mod_reexports_only(files: &[PathBuf]) -> Result<()> {
    for path in files {
        if path.file_name().and_then(|s| s.to_str()) != Some("mod.rs") {
            continue;
        }
        if !path.to_string_lossy().ends_with("stages/mod.rs") {
            continue;
        }
        let content = fs::read_to_string(path)?;
        let mut meaningful = 0usize;
        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("//") {
                continue;
            }
            if trimmed.starts_with("#[") {
                continue;
            }
            if trimmed.starts_with("pub mod ") || trimmed.starts_with("mod ") {
                continue;
            }
            meaningful += 1;
        }
        if meaningful == 0 {
            return Err(GuardrailError::Violation(format!(
                "stages mod.rs contains only re-exports: {}",
                path.display()
            )));
        }
    }
    Ok(())
}

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
