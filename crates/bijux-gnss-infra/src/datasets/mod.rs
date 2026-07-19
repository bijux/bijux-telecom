//! Dataset helpers for the infra API.
#![allow(missing_docs)]

mod raw_iq_metadata;
mod registry;

pub type DatasetEntry = registry::DatasetEntry;
pub type DatasetRegistry = registry::DatasetRegistry;
pub type RecordedCaptureProvenance = registry::RecordedCaptureProvenance;

pub fn parse_ecef(text: &str) -> Result<[f64; 3], bijux_gnss_receiver::api::core::InputError> {
    crate::parse::parse_ecef(text)
}

pub fn load_raw_iq_metadata(
    path: &std::path::Path,
) -> Result<bijux_gnss_signal::api::RawIqMetadata, bijux_gnss_receiver::api::core::InputError> {
    raw_iq_metadata::load_raw_iq_metadata(path)
}

pub fn resolve_raw_iq_metadata(
    dataset: Option<&DatasetEntry>,
    explicit_sidecar: Option<&std::path::Path>,
) -> Result<bijux_gnss_signal::api::RawIqMetadata, bijux_gnss_receiver::api::core::InputError> {
    raw_iq_metadata::resolve_raw_iq_metadata(dataset, explicit_sidecar)
}
