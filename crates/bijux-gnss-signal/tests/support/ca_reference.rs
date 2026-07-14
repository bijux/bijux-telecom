#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use bijux_gnss_signal::api::CA_CODE_PERIOD_CHIPS;
use serde::Deserialize;

use crate::support::period_reference::{assert_code_period_reference, ReferenceWindow};

const CATALOG_SCHEMA: &str = "ca_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct CaReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub code_length: usize,
    pub window_length: usize,
    pub middle_start: usize,
    pub boundary_start: usize,
    pub code: Vec<CaCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct CaCodeReference {
    pub prn: u8,
    pub g2_taps: (u8, u8),
    pub g2_delay_chips: u16,
    pub first_ten_chips_octal: u16,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
    pub bit_boundary: String,
}

pub fn load_reference_catalog() -> CaReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read GPS C/A reference catalog fixture");
    let catalog: CaReferenceCatalog =
        toml::from_str(&raw).expect("parse GPS C/A reference catalog fixture");
    catalog.validate();
    catalog
}

impl CaReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected GPS C/A catalog schema");
        assert_eq!(self.code_length, CA_CODE_PERIOD_CHIPS, "unexpected GPS C/A code length");
        assert_eq!(self.window_length, 32, "unexpected GPS C/A window length");
        assert_eq!(self.middle_start, 495, "unexpected GPS C/A middle window start");
        assert_eq!(self.boundary_start, 1007, "unexpected GPS C/A boundary window start");
        assert_eq!(self.code.len(), 32, "unexpected GPS C/A reference count");

        for reference in &self.code {
            assert_eq!(
                reference.bit_prefix.len(),
                self.window_length,
                "unexpected GPS C/A prefix length for PRN {}",
                reference.prn
            );
            assert_eq!(
                reference.bit_middle.len(),
                self.window_length,
                "unexpected GPS C/A middle length for PRN {}",
                reference.prn
            );
            assert_eq!(
                reference.bit_suffix.len(),
                self.window_length,
                "unexpected GPS C/A suffix length for PRN {}",
                reference.prn
            );
            assert_eq!(
                reference.bit_boundary.len(),
                self.window_length,
                "unexpected GPS C/A boundary length for PRN {}",
                reference.prn
            );
        }
    }

    pub fn code_reference(&self, prn: u8) -> &CaCodeReference {
        self.code
            .iter()
            .find(|reference| reference.prn == prn)
            .unwrap_or_else(|| panic!("missing GPS C/A reference for PRN {prn}"))
    }
}

pub fn assert_code_matches_reference(catalog: &CaReferenceCatalog, prn: u8, logical_bits: &str) {
    let reference = catalog.code_reference(prn);
    let windows = vec![
        ReferenceWindow { start: 0, bits: reference.bit_prefix.clone() },
        ReferenceWindow { start: catalog.middle_start, bits: reference.bit_middle.clone() },
        ReferenceWindow {
            start: catalog.code_length - catalog.window_length,
            bits: reference.bit_suffix.clone(),
        },
        ReferenceWindow { start: catalog.boundary_start, bits: reference.bit_boundary.clone() },
    ];
    assert_code_period_reference(
        logical_bits,
        catalog.code_length,
        &reference.bit_sha256,
        &windows,
        &format!("GPS C/A PRN {prn}"),
    );
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join("ca_reference_catalog.toml")
}
