//! Build metadata helpers shared by persisted run records.

use crate::hash::{cpu_features, git_dirty, git_hash};
use crate::run_layout::provenance::enabled_features;

#[derive(Debug, Clone)]
pub(crate) struct BuildMetadata {
    pub(crate) git_hash: String,
    pub(crate) git_dirty: bool,
    pub(crate) build_profile: String,
    pub(crate) cpu_features: Vec<String>,
    pub(crate) toolchain: String,
    pub(crate) features: Vec<String>,
    pub(crate) build_version: String,
    pub(crate) build_git: String,
    pub(crate) build_timestamp: String,
}

pub(crate) fn build_metadata() -> BuildMetadata {
    BuildMetadata {
        git_hash: git_hash().unwrap_or_else(|| "unknown".to_string()),
        git_dirty: git_dirty(),
        build_profile: std::env::var("BIJUX_BUILD_PROFILE").unwrap_or_else(|_| "dev".to_string()),
        cpu_features: cpu_features(),
        toolchain: std::env::var("BIJUX_BUILD_RUSTC").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        build_version: env!("CARGO_PKG_VERSION").to_string(),
        build_git: std::env::var("BIJUX_BUILD_GIT_SHA").unwrap_or_else(|_| "unknown".to_string()),
        build_timestamp: std::env::var("BIJUX_BUILD_TIMESTAMP")
            .unwrap_or_else(|_| "unknown".to_string()),
    }
}
