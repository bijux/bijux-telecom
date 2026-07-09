//! Dataset helpers for the infra API.
#![allow(missing_docs)]

use std::path::Path;

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::RawIqMetadata;

pub type DatasetEntry = crate::dataset::DatasetEntry;
pub type DatasetRegistry = crate::dataset::DatasetRegistry;

pub fn parse_ecef(text: &str) -> Result<[f64; 3], bijux_gnss_receiver::api::core::InputError> {
    crate::parse::core::parse_ecef(text)
}

pub fn load_raw_iq_metadata(path: &Path) -> Result<RawIqMetadata, InputError> {
    crate::dataset::load_raw_iq_metadata(path)
}

pub fn resolve_raw_iq_metadata(
    dataset: Option<&DatasetEntry>,
    explicit_sidecar: Option<&Path>,
) -> Result<RawIqMetadata, InputError> {
    crate::dataset::resolve_raw_iq_metadata(dataset, explicit_sidecar)
}
