//! Shared guardrail policy enforcement across crates.
//! See docs/GLOSSARY.md for acronym definitions.

#![deny(missing_docs)]

use regex::Regex;
use std::fs;
use std::path::{Path, PathBuf};

pub mod config;
pub mod error;
mod source_tree;

use self::config::GuardrailConfig;
use self::error::{GuardrailError, Result};
use self::source_tree::{
    check_depth, check_empty_modules, check_forbidden_filenames, check_mod_only_dirs,
    check_mod_reexports_only, collect_rs_files,
};

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
