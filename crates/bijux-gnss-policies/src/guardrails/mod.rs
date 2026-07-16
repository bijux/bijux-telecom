//! Shared guardrail policy enforcement across crates.
//! See docs/GLOSSARY.md for acronym definitions.

#![deny(missing_docs)]

use regex::Regex;
use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

pub mod config;
pub mod error;

use self::config::GuardrailConfig;
use self::error::{GuardrailError, Result};

macro_rules! bail {
    ($($arg:tt)*) => {
        return Err(GuardrailError::Violation(format!($($arg)*)))
    };
}

/// Run guardrail checks on a crate root.
pub fn check(crate_root: &Path, config: &GuardrailConfig) -> Result<()> {
    let src_dir = crate_root.join("src");
    let files = collect_rs_files(&src_dir)?;
    check_depth(&src_dir, &files, config)?;
    check_mod_only_dirs(&src_dir)?;
    check_empty_modules(&files)?;
    check_mod_reexports_only(&files)?;
    check_pub_items(&files, config)?;
    check_api_purity(&files)?;
    if config.enforce_pub_use_api_only {
        check_pub_items_outside_api(&files)?;
    }
    check_forbidden_filenames(&files)?;
    check_pub_use_locations_if_enabled(&files, config)?;
    if config.forbid_pub_use_spam {
        check_pub_use_spam(&files, config)?;
    }
    if config.forbid_panic_expect {
        check_panic_expect(&files, config)?;
    }
    if config.forbid_stage_id_strings {
        check_stage_id_strings(&files, config)?;
    }
    check_purity_zones(&files, config)?;
    Ok(())
}

fn check_api_purity(files: &[PathBuf]) -> Result<()> {
    let impl_re = Regex::new(r"^\s*impl\b")?;
    let pub_mod_re = Regex::new(r"^\s*pub\s+mod\b")?;
    let pub_export_re = Regex::new(r"^\s*pub\s+(use|mod)\b")?;
    let pipeline_glob_re = Regex::new(r"^\s*pub\s+use\s+crate::pipeline::.*\*")?;
    for path in files {
        let is_api = path.file_name().and_then(|s| s.to_str()) == Some("api.rs");
        if !is_api {
            continue;
        }
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            if impl_re.is_match(line) {
                bail!("impl block not allowed in api.rs at {}:{}", path.display(), idx + 1);
            }
            if pub_mod_re.is_match(line) {
                bail!("pub mod not allowed in api.rs at {}:{}", path.display(), idx + 1);
            }
            if pub_export_re.is_match(line) && line.contains("_internal") {
                bail!(
                    "public export of _internal not allowed in api.rs at {}:{}",
                    path.display(),
                    idx + 1
                );
            }
            if pipeline_glob_re.is_match(line) {
                bail!(
                    "wildcard re-export of pipeline not allowed in api.rs at {}:{}",
                    path.display(),
                    idx + 1
                );
            }
        }
    }
    Ok(())
}

fn check_purity_zones(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
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
                    bail!("purity violation at {}:{} -> {}", path.display(), idx + 1, line.trim());
                }
            }
        }
    }
    Ok(())
}

fn collect_rs_files(root: &Path) -> Result<Vec<PathBuf>> {
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

fn check_depth(src_dir: &Path, files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
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
        bail!(
            "module depth exceeds allowed rule (src/a/b/c.rs or mod.rs at each level): {}",
            path.display()
        );
    }
    Ok(())
}

fn check_mod_only_dirs(src_dir: &Path) -> Result<()> {
    for entry in WalkDir::new(src_dir).min_depth(1).max_depth(10) {
        let entry = entry?;
        if !entry.file_type().is_dir() {
            continue;
        }
        if entry.path() == src_dir {
            continue;
        }
        if entry.path().components().any(|c| c.as_os_str() == "tests") {
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
            bail!(
                "module directory contains only mod.rs (and optionally tests.rs): {}",
                entry.path().display()
            );
        }
    }
    Ok(())
}

fn check_empty_modules(files: &[PathBuf]) -> Result<()> {
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
            bail!("empty module file (only mod re-exports): {}", path.display());
        }
    }
    Ok(())
}

fn check_mod_reexports_only(files: &[PathBuf]) -> Result<()> {
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
            bail!("stages mod.rs contains only re-exports: {}", path.display());
        }
    }
    Ok(())
}

fn check_pub_items(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let pub_re =
        Regex::new(r"^\s*pub(\s*\(crate\))?\s+(struct|enum|fn|type|trait|const|static|use|mod)\b")?;
    for path in files {
        let content = fs::read_to_string(path)?;
        let count = content.lines().filter(|line| pub_re.is_match(line)).count();
        if count > config.max_pub_items_per_file {
            bail!(
                "{} has {} pub items (max {})",
                path.display(),
                count,
                config.max_pub_items_per_file
            );
        }
    }
    Ok(())
}

fn check_pub_items_outside_api(files: &[PathBuf]) -> Result<()> {
    let pub_item_re = Regex::new(r"^\s*pub\s+(struct|enum|fn|type|trait|const|static|use|mod)\b")?;
    for path in files {
        let is_lib = path.file_name().and_then(|s| s.to_str()) == Some("lib.rs");
        if !is_lib {
            continue;
        }
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            if !pub_item_re.is_match(line) {
                continue;
            }
            if line.trim() == "pub mod api;" {
                continue;
            }
            bail!(
                "public items must be confined to api.rs (found in {}:{})",
                path.display(),
                idx + 1
            );
        }
    }
    Ok(())
}

fn check_pub_use_spam(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let pub_use_re = Regex::new(r"^\s*pub\s+use\b")?;
    for path in files {
        let content = fs::read_to_string(path)?;
        let count = content.lines().filter(|line| pub_use_re.is_match(line)).count();
        if count > config.max_pub_use_per_file {
            bail!(
                "{} has {} pub use re-exports (max {})",
                path.display(),
                count,
                config.max_pub_use_per_file
            );
        }
    }
    Ok(())
}

fn check_pub_use_locations(files: &[PathBuf]) -> Result<()> {
    let pub_use_re = Regex::new(r"^\s*pub\s+use\b")?;
    for path in files {
        if path.extension().and_then(|ext| ext.to_str()) != Some("rs") {
            continue;
        }
        let is_api = path.file_name().and_then(|s| s.to_str()) == Some("api.rs");
        let is_lib = path.file_name().and_then(|s| s.to_str()) == Some("lib.rs");
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            if !pub_use_re.is_match(line) {
                continue;
            }
            if is_api {
                continue;
            }
            if is_lib && line.contains("api::") {
                continue;
            }
            bail!("pub use outside api.rs at {}:{}", path.display(), idx + 1);
        }
    }
    Ok(())
}

fn check_pub_use_locations_if_enabled(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    if !config.enforce_pub_use_api_only {
        return Ok(());
    }
    check_pub_use_locations(files)
}

fn check_forbidden_filenames(files: &[PathBuf]) -> Result<()> {
    for path in files {
        let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
        if name.ends_with(".proptest-regressions") {
            continue;
        }
        if name == "helpers.rs" || name == "support.rs" || name == "misc.rs" {
            bail!("forbidden filename: {}", path.display());
        }
    }
    Ok(())
}

fn check_panic_expect(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
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
                bail!("panic/expect found in {}:{}", path.display(), idx + 1);
            }
        }
    }
    Ok(())
}

fn check_stage_id_strings(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let stage_re = Regex::new(r#"\"(fastq|bam|vcf)\.[^\"]+\""#)?;
    for path in files {
        let path_str = path.to_string_lossy();
        if config.allow_stage_id_paths.iter().any(|allowed| path_str.contains(allowed)) {
            continue;
        }
        let content = fs::read_to_string(path)?;
        if stage_re.is_match(&content) {
            bail!("stage id literal found in {}", path.display());
        }
    }
    Ok(())
}
