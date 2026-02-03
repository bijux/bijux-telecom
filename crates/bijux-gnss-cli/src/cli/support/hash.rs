use sha2::{Digest, Sha256};
use std::process::Command as ProcessCommand;

fn hash_config(path: Option<&PathBuf>, profile: &ReceiverProfile) -> Result<String> {
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

fn git_hash() -> Option<String> {
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

fn cpu_features() -> Vec<String> {
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
