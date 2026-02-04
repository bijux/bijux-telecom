//! Dataset helpers for the infra API.
#![allow(missing_docs)]

pub use crate::dataset::{DatasetEntry, DatasetRegistry};

pub fn parse_ecef(text: &str) -> Result<[f64; 3], bijux_gnss_receiver::api::core::InputError> {
    crate::parse::core::parse_ecef(text)
}
