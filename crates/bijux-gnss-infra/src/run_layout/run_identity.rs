//! Run identity helpers.

use sha2::{Digest, Sha256};

use crate::datasets::DatasetEntry;

pub(super) const RUN_REPORT_SCHEMA_VERSION: u32 = 1;
pub(super) const RUN_LAYOUT_SCHEMA_VERSION: u32 = 1;

pub(super) fn now_unix_ms(deterministic: bool) -> u128 {
    if deterministic {
        return 0;
    }
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}

pub(super) fn dataset_hash(
    dataset: &DatasetEntry,
) -> Result<String, bijux_gnss_receiver::api::core::InputError> {
    let serialized = serde_json::to_vec(dataset).map_err(map_err)?;
    let mut hasher = Sha256::new();
    hasher.update(serialized);
    Ok(hex::encode(hasher.finalize()))
}

pub(super) fn run_id(config_hash: &str, dataset_hash: Option<&str>, build_version: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(config_hash.as_bytes());
    if let Some(dataset_hash) = dataset_hash {
        hasher.update(dataset_hash.as_bytes());
    }
    hasher.update(build_version.as_bytes());
    hex::encode(hasher.finalize())
}

fn map_err(err: impl std::fmt::Display) -> bijux_gnss_receiver::api::core::InputError {
    bijux_gnss_receiver::api::core::InputError { message: err.to_string() }
}
