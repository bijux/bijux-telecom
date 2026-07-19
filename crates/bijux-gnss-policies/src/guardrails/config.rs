//! Configuration ownership for guardrail policy checks.

use serde::{Deserialize, Serialize};

/// Guardrail configuration for policy checks.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GuardrailConfig {
    /// Maximum lines of code per file.
    pub max_loc: usize,
    /// Maximum module path depth.
    pub max_depth: usize,
    /// Maximum number of Rust modules per directory.
    pub max_modules_per_dir: usize,
    /// Maximum number of Rust files per directory.
    pub max_rs_files_per_dir: usize,
    /// Maximum number of public items per file.
    pub max_pub_items_per_file: usize,
    /// Maximum number of pub use statements per file.
    pub max_pub_use_per_file: usize,
    /// Whether to forbid dense pub use blocks.
    pub forbid_pub_use_spam: bool,
    /// Whether to forbid panic!/expect usage.
    pub forbid_panic_expect: bool,
    /// Whether to forbid hard-coded stage id strings.
    pub forbid_stage_id_strings: bool,
    /// Whether to enforce pub use only in api.rs.
    pub enforce_pub_use_api_only: bool,
    /// Path substrings that define pure zones.
    pub purity_zones: Vec<String>,
    /// Regex patterns forbidden inside pure zones.
    pub purity_forbidden: Vec<String>,
    /// Allowlist of paths for panic/expect.
    pub allow_panic_expect_paths: Vec<String>,
    /// Allowlist of paths for stage id strings.
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
            enforce_pub_use_api_only: true,
            purity_zones: Vec::new(),
            purity_forbidden: Vec::new(),
            allow_panic_expect_paths: Vec::new(),
            allow_stage_id_paths: Vec::new(),
        }
    }
}

impl GuardrailConfig {
    #[must_use]
    /// Return a default configuration for a given crate name.
    pub fn for_crate(name: &str) -> Self {
        let mut config = Self::default();
        if name == "bijux-domain-bam" {
            config.allow_stage_id_paths = vec!["/src/bam_stage_registry.rs".to_string()];
        }
        config
    }
}
