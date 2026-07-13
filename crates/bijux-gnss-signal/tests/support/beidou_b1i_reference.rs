#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use bijux_gnss_signal::api::BEIDOU_B1I_CODE_CHIPS;
use serde::Deserialize;

use crate::support::period_reference::{assert_code_period_reference, ReferenceWindow};

const CATALOG_SCHEMA: &str = "beidou_b1i_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct BeidouB1iReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub code_length: usize,
    pub window_length: usize,
    pub middle_start: usize,
    pub boundary_start: usize,
    pub code: Vec<BeidouB1iCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct BeidouB1iCodeReference {
    pub prn: u8,
    pub g2_taps: (u8, u8),
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
    pub bit_boundary: String,
}

pub fn load_reference_catalog() -> BeidouB1iReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read BeiDou B1I reference catalog fixture");
    let catalog: BeidouB1iReferenceCatalog =
        toml::from_str(&raw).expect("parse BeiDou B1I reference catalog fixture");
    catalog.validate();
    catalog
}

impl BeidouB1iReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected BeiDou B1I catalog schema");
        assert_eq!(self.code_length, BEIDOU_B1I_CODE_CHIPS, "unexpected BeiDou B1I code length");
        assert_eq!(self.window_length, 32, "unexpected BeiDou B1I window length");
        assert_eq!(self.middle_start, 1007, "unexpected BeiDou B1I middle window start");
        assert_eq!(self.boundary_start, 2030, "unexpected BeiDou B1I boundary window start");
        assert_eq!(self.code.len(), 37, "unexpected BeiDou B1I reference count");
    }

    pub fn code_reference(&self, prn: u8) -> &BeidouB1iCodeReference {
        self.code
            .iter()
            .find(|reference| reference.prn == prn)
            .unwrap_or_else(|| panic!("missing BeiDou B1I reference for PRN {prn}"))
    }
}

pub fn assert_code_matches_reference(
    catalog: &BeidouB1iReferenceCatalog,
    prn: u8,
    logical_bits: &str,
) {
    let reference = catalog.code_reference(prn);
    let windows = vec![
        ReferenceWindow {
            start: 0,
            bits: reference.bit_prefix.clone(),
        },
        ReferenceWindow {
            start: catalog.middle_start,
            bits: reference.bit_middle.clone(),
        },
        ReferenceWindow {
            start: catalog.code_length - catalog.window_length,
            bits: reference.bit_suffix.clone(),
        },
        ReferenceWindow {
            start: catalog.boundary_start,
            bits: reference.bit_boundary.clone(),
        },
    ];
    assert_code_period_reference(
        logical_bits,
        catalog.code_length,
        &reference.bit_sha256,
        &windows,
        &format!("BeiDou B1I PRN {prn}"),
    );
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("beidou_b1i_reference_catalog.toml")
}
