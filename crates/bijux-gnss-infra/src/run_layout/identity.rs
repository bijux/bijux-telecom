//! Run identity helpers.

mod fingerprint;
mod schema_versions;
mod timestamp;

use crate::datasets::DatasetEntry;

pub(crate) const RUN_REPORT_SCHEMA_VERSION: u32 = schema_versions::RUN_REPORT_SCHEMA_VERSION;
pub(crate) const RUN_LAYOUT_SCHEMA_VERSION: u32 = schema_versions::RUN_LAYOUT_SCHEMA_VERSION;

pub(crate) fn now_unix_ms(deterministic: bool) -> u128 {
    timestamp::now_unix_ms(deterministic)
}

pub(crate) fn dataset_hash(
    dataset: &DatasetEntry,
) -> Result<String, bijux_gnss_receiver::api::core::InputError> {
    fingerprint::dataset_hash(dataset)
}

pub(crate) fn run_id(
    config_hash: &str,
    dataset_hash: Option<&str>,
    build_version: &str,
) -> String {
    fingerprint::run_id(config_hash, dataset_hash, build_version)
}
