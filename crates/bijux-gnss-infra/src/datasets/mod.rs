//! Dataset helpers for the infra API.
#![allow(missing_docs)]

mod raw_iq_metadata;
mod registry;

pub use raw_iq_metadata::{load_raw_iq_metadata, resolve_raw_iq_metadata};
pub use registry::{DatasetEntry, DatasetRegistry, RecordedCaptureProvenance};

pub fn parse_ecef(text: &str) -> Result<[f64; 3], bijux_gnss_receiver::api::core::InputError> {
    crate::parse::core::parse_ecef(text)
}
