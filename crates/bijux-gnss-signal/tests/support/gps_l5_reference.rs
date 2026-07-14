#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use bijux_gnss_signal::api::GPS_L5_CODE_CHIPS;
use serde::Deserialize;

use crate::support::period_reference::{assert_code_period_reference, ReferenceWindow};

const CATALOG_SCHEMA: &str = "gps_l5_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct GpsL5ReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub spec_source: String,
    pub published_tables: String,
    pub code_length: usize,
    pub window_length: usize,
    pub middle_start: usize,
    pub boundary_start: usize,
    pub l5i_nh_bits: String,
    pub l5i_nh_sha256: String,
    pub l5q_nh_bits: String,
    pub l5q_nh_sha256: String,
    pub primary_code: Vec<GpsL5PrimaryCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct GpsL5PrimaryCodeReference {
    pub channel: String,
    pub prn: u8,
    pub xb_advance_chips: usize,
    pub xb_initial_state_bits: String,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
    pub bit_boundary: String,
}

pub fn load_reference_catalog() -> GpsL5ReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read GPS L5 reference catalog fixture");
    let catalog: GpsL5ReferenceCatalog =
        toml::from_str(&raw).expect("parse GPS L5 reference catalog fixture");
    catalog.validate();
    catalog
}

impl GpsL5ReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected GPS L5 catalog schema");
        assert_eq!(self.code_length, GPS_L5_CODE_CHIPS, "unexpected GPS L5 code length");
        assert_eq!(self.window_length, 32, "unexpected GPS L5 window length");
        assert_eq!(self.middle_start, 5099, "unexpected GPS L5 middle window start");
        assert_eq!(self.boundary_start, 10214, "unexpected GPS L5 boundary window start");
        assert_eq!(self.primary_code.len(), 126, "unexpected GPS L5 reference count");
        assert_eq!(self.l5i_nh_bits.len(), 10, "unexpected GPS L5-I NH length");
        assert_eq!(self.l5q_nh_bits.len(), 20, "unexpected GPS L5-Q NH length");
    }

    pub fn primary_code_reference(&self, channel: &str, prn: u8) -> &GpsL5PrimaryCodeReference {
        self.primary_code
            .iter()
            .find(|reference| reference.channel == channel && reference.prn == prn)
            .unwrap_or_else(|| panic!("missing GPS L5 {channel} reference for PRN {prn}"))
    }
}

pub fn assert_primary_code_matches_reference(
    catalog: &GpsL5ReferenceCatalog,
    channel: &str,
    prn: u8,
    logical_bits: &str,
) {
    let reference = catalog.primary_code_reference(channel, prn);
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
        &format!("GPS L5 {channel} PRN {prn}"),
    );
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("gps_l5_reference_catalog.toml")
}
