//! Artifact header helpers.

use bijux_gnss_receiver::api::core::{ArtifactHeaderV1, ArtifactReadPolicy, InputError};
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::datasets::DatasetEntry;
use crate::hash::{git_dirty, git_hash, hash_config};

use crate::run_layout::directories::RunContextArgs;
use crate::run_layout::identity::now_unix_ms;
use crate::run_layout::provenance::enabled_features;

/// Build an artifact header for outputs.
pub fn artifact_header(
    args: &RunContextArgs<'_>,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<ArtifactHeaderV1, InputError> {
    let config_hash = hash_config(args.config, profile)?;
    Ok(ArtifactHeaderV1 {
        schema_version: ArtifactReadPolicy::LATEST,
        producer: env!("CARGO_PKG_NAME").to_string(),
        producer_version: env!("CARGO_PKG_VERSION").to_string(),
        created_at_unix_ms: now_unix_ms(args.deterministic),
        git_sha: git_hash().unwrap_or_else(|| "unknown".to_string()),
        config_hash,
        dataset_id: dataset.map(|entry| entry.id.clone()),
        toolchain: std::env::var("BIJUX_BUILD_RUSTC").unwrap_or_else(|_| "unknown".to_string()),
        features: enabled_features(),
        deterministic: args.deterministic,
        git_dirty: git_dirty(),
    })
}
