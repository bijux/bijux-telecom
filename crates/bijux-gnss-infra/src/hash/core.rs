//! Hashing and provenance helpers.

use crate::errors::InfraResult;
use bijux_gnss_receiver::ReceiverProfile;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;
use std::process::Command as ProcessCommand;

/// Hash a config file or profile snapshot.
pub fn hash_config(path: Option<&PathBuf>, profile: &ReceiverProfile) -> InfraResult<String> {
    let mut hasher = Sha256::new();
    if let Some(path) = path {
        let bytes = fs::read(path)?;
        hasher.update(bytes);
    } else {
        let serialized = toml::to_string(profile)?;
        hasher.update(serialized.as_bytes());
    }
    Ok(hex::encode(hasher.finalize()))
}

/// Return current git hash if available.
pub fn git_hash() -> Option<String> {
    let output = ProcessCommand::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let hash = String::from_utf8_lossy(&output.stdout).trim().to_string();
    Some(hash)
}

/// Return true when git workspace is dirty.
pub fn git_dirty() -> bool {
    let output = ProcessCommand::new("git")
        .args(["status", "--porcelain"])
        .output();
    if let Ok(output) = output {
        if output.status.success() {
            return !output.stdout.is_empty();
        }
    }
    false
}

/// CPU feature detection summary.
pub fn cpu_features() -> Vec<String> {
    let mut features = Vec::new();
    #[cfg(target_arch = "x86_64")]
    {
        if std::is_x86_feature_detected!("avx2") {
            features.push("avx2".to_string());
        }
        if std::is_x86_feature_detected!("sse4.2") {
            features.push("sse4.2".to_string());
        }
        if std::is_x86_feature_detected!("fma") {
            features.push("fma".to_string());
        }
    }
    features
}
