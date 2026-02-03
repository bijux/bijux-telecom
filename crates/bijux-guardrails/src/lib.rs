//! Shared guardrail checks across crates.

use anyhow::Result;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GuardrailConfig {
    pub max_loc: usize,
    pub max_depth: usize,
    pub max_modules_per_dir: usize,
    pub max_rs_files_per_dir: usize,
    pub max_pub_items_per_file: usize,
    pub max_pub_use_per_file: usize,
    pub forbid_pub_use_spam: bool,
    pub forbid_panic_expect: bool,
    pub forbid_stage_id_strings: bool,
    pub allow_panic_expect_paths: Vec<String>,
    pub allow_stage_id_paths: Vec<String>,
}

impl Default for GuardrailConfig {
    fn default() -> Self {
        Self {
            max_loc: 500,
            max_depth: 4,
            max_modules_per_dir: 10,
            max_rs_files_per_dir: 10,
            max_pub_items_per_file: 50,
            max_pub_use_per_file: 25,
            forbid_pub_use_spam: false,
            forbid_panic_expect: false,
            forbid_stage_id_strings: false,
            allow_panic_expect_paths: Vec::new(),
            allow_stage_id_paths: Vec::new(),
        }
    }
}

impl GuardrailConfig {
    #[must_use]
    pub fn for_crate(name: &str) -> Self {
        let mut config = Self::default();
        if name == "bijux-domain-bam" {
            config.allow_stage_id_paths = vec!["/src/bam_stage_registry.rs".to_string()];
        }
        config
    }
}

pub fn check(crate_root: &Path, config: &GuardrailConfig) -> Result<()> {
    let src_dir = crate_root.join("src");
    let files = collect_rs_files(&src_dir)?;
    check_loc(&files, config)?;
    check_depth(&src_dir, &files, config)?;
    check_modules_per_dir(&src_dir, config)?;
    check_rs_files_per_dir(&src_dir, config)?;
    check_mod_only_dirs(&src_dir)?;
    check_empty_modules(&files)?;
    check_mod_reexports_only(&files)?;
    check_pub_items(&files, config)?;
    if config.forbid_pub_use_spam {
        check_pub_use_spam(&files, config)?;
    }
    if config.forbid_panic_expect {
        check_panic_expect(&files, config)?;
    }
    if config.forbid_stage_id_strings {
        check_stage_id_strings(&files, config)?;
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

fn check_loc(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    for path in files {
        let content = fs::read_to_string(path)?;
        let lines = content.lines().count();
        if lines > config.max_loc {
            anyhow::bail!(
                "{} has {} lines (max {})",
                path.display(),
                lines,
                config.max_loc
            );
        }
    }
    Ok(())
}

fn check_depth(src_dir: &Path, files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    for path in files {
        let rel = path.strip_prefix(src_dir).unwrap_or(path.as_path());
        let components: Vec<_> = rel.components().collect();
        if components.len() <= config.max_depth {
            continue;
        }
        let is_mod_rs = path
            .file_name()
            .and_then(|name| name.to_str())
            .is_some_and(|name| name == "mod.rs");
        if components.len() == config.max_depth + 1 && is_mod_rs {
            continue;
        }
        anyhow::bail!(
            "module depth exceeds allowed rule (src/a/b/c.rs or mod.rs at each level): {}",
            path.display()
        );
    }
    Ok(())
}

fn check_modules_per_dir(src_dir: &Path, config: &GuardrailConfig) -> Result<()> {
    for entry in WalkDir::new(src_dir).min_depth(0).max_depth(10) {
        let entry = entry?;
        if !entry.file_type().is_dir() {
            continue;
        }
        let mut count = 0usize;
        for child in fs::read_dir(entry.path())? {
            let child = child?;
            let path = child.path();
            if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("rs") {
                count += 1;
            }
        }
        if count > config.max_modules_per_dir {
            anyhow::bail!(
                "{} has {} rust modules (max {})",
                entry.path().display(),
                count,
                config.max_modules_per_dir
            );
        }
    }
    Ok(())
}

fn check_rs_files_per_dir(src_dir: &Path, config: &GuardrailConfig) -> Result<()> {
    for entry in WalkDir::new(src_dir).min_depth(0).max_depth(10) {
        let entry = entry?;
        if !entry.file_type().is_dir() {
            continue;
        }
        let mut count = 0usize;
        for child in fs::read_dir(entry.path())? {
            let child = child?;
            let path = child.path();
            if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("rs") {
                let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
                if name == "lib.rs" || name == "main.rs" || name == "mod.rs" {
                    continue;
                }
                count += 1;
            }
        }
        if count > config.max_rs_files_per_dir {
            anyhow::bail!(
                "{} has {} rust modules (max {})",
                entry.path().display(),
                count,
                config.max_rs_files_per_dir
            );
        }
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
        let allowed = rs_files
            .iter()
            .all(|name| name == "mod.rs" || name == "tests.rs");
        if allowed && rs_files.iter().any(|name| name == "mod.rs") && rs_files.len() <= 2 {
            anyhow::bail!(
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
            anyhow::bail!(
                "empty module file (only mod re-exports): {}",
                path.display()
            );
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
            anyhow::bail!("stages mod.rs contains only re-exports: {}", path.display());
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
            anyhow::bail!(
                "{} has {} pub items (max {})",
                path.display(),
                count,
                config.max_pub_items_per_file
            );
        }
    }
    Ok(())
}

fn check_pub_use_spam(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let pub_use_re = Regex::new(r"^\s*pub\s+use\b")?;
    for path in files {
        let content = fs::read_to_string(path)?;
        let count = content
            .lines()
            .filter(|line| pub_use_re.is_match(line))
            .count();
        if count > config.max_pub_use_per_file {
            anyhow::bail!(
                "{} has {} pub use re-exports (max {})",
                path.display(),
                count,
                config.max_pub_use_per_file
            );
        }
    }
    Ok(())
}

fn check_panic_expect(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let panic_re = Regex::new(r"\bpanic!\(")?;
    let expect_re = Regex::new(r"\.expect\(")?;
    for path in files {
        let path_str = path.to_string_lossy();
        if config
            .allow_panic_expect_paths
            .iter()
            .any(|allowed| path_str.contains(allowed))
        {
            continue;
        }
        let content = fs::read_to_string(path)?;
        for (idx, line) in content.lines().enumerate() {
            if panic_re.is_match(line) || expect_re.is_match(line) {
                anyhow::bail!("panic/expect found in {}:{}", path.display(), idx + 1);
            }
        }
    }
    Ok(())
}

fn check_stage_id_strings(files: &[PathBuf], config: &GuardrailConfig) -> Result<()> {
    let stage_re = Regex::new(r#"\"(fastq|bam|vcf)\.[^\"]+\""#)?;
    for path in files {
        let path_str = path.to_string_lossy();
        if config
            .allow_stage_id_paths
            .iter()
            .any(|allowed| path_str.contains(allowed))
        {
            continue;
        }
        let content = fs::read_to_string(path)?;
        if stage_re.is_match(&content) {
            anyhow::bail!("stage id literal found in {}", path.display());
        }
    }
    Ok(())
}
