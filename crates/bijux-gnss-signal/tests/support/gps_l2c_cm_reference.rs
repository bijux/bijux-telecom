#![allow(dead_code, missing_docs)]

use std::collections::BTreeSet;
use std::path::PathBuf;

use bijux_gnss_signal::api::GPS_L2C_CM_CODE_CHIPS;
use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::support::period_reference::wrapped_window_bits;

const CATALOG_SCHEMA: &str = "gps_l2c_cm_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct GpsL2cCmReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub spec_source: String,
    pub published_tables: String,
    pub chip_length: usize,
    pub prefix_length: usize,
    pub suffix_length: usize,
    pub middle_start: usize,
    pub published_prn_count: usize,
    pub code: Vec<GpsL2cCmCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct GpsL2cCmCodeReference {
    pub prn: u8,
    pub initial_state_octal: String,
    pub end_state_octal: String,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
}

pub fn load_reference_catalog() -> GpsL2cCmReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read GPS L2C CM reference catalog fixture");
    let catalog: GpsL2cCmReferenceCatalog =
        toml::from_str(&raw).expect("parse GPS L2C CM reference catalog fixture");
    catalog.validate();
    catalog
}

impl GpsL2cCmReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected GPS L2C CM catalog schema");
        assert_eq!(
            self.chip_length, GPS_L2C_CM_CODE_CHIPS,
            "unexpected GPS L2C CM code length in reference catalog"
        );
        assert_eq!(self.prefix_length, 64, "unexpected GPS L2C CM prefix length");
        assert_eq!(self.suffix_length, 64, "unexpected GPS L2C CM suffix length");
        assert_eq!(self.middle_start, 5_099, "unexpected GPS L2C CM middle start");
        assert_eq!(
            self.code.len(),
            self.published_prn_count,
            "unexpected GPS L2C CM reference count"
        );
        assert_eq!(self.published_prn_count, 115, "unexpected GPS L2C CM published PRN count");

        let prns: Vec<u8> = self.code.iter().map(|reference| reference.prn).collect();
        let unique_prns: BTreeSet<u8> = prns.iter().copied().collect();
        assert_eq!(unique_prns.len(), prns.len(), "duplicate GPS L2C CM PRN reference");
        assert_eq!(self.code.first().map(|reference| reference.prn), Some(1));
        assert_eq!(self.code.last().map(|reference| reference.prn), Some(210));
        assert_eq!(
            prns.iter().copied().filter(|prn| (64..=158).contains(prn)).count(),
            0,
            "unexpected GPS L2C CM gap assignment in reference catalog"
        );

        for reference in &self.code {
            assert_eq!(
                reference.bit_prefix.len(),
                self.prefix_length,
                "unexpected GPS L2C CM prefix length for PRN {}",
                reference.prn
            );
            assert_eq!(
                reference.bit_suffix.len(),
                self.suffix_length,
                "unexpected GPS L2C CM suffix length for PRN {}",
                reference.prn
            );
            assert_eq!(
                reference.bit_middle.len(),
                self.prefix_length,
                "unexpected GPS L2C CM middle length for PRN {}",
                reference.prn
            );
        }
    }

    pub fn code_reference(&self, prn: u8) -> &GpsL2cCmCodeReference {
        self.code
            .iter()
            .find(|reference| reference.prn == prn)
            .unwrap_or_else(|| panic!("missing GPS L2C CM reference for PRN {prn}"))
    }
}

pub fn assert_code_matches_reference(
    catalog: &GpsL2cCmReferenceCatalog,
    prn: u8,
    code: &[i8],
) {
    let reference = catalog.code_reference(prn);
    let logical_bits = logical_bits_from_code(code);
    let prefix = &logical_bits[..catalog.prefix_length];
    let middle = &logical_bits[catalog.middle_start..catalog.middle_start + catalog.prefix_length];
    let suffix = &logical_bits[logical_bits.len() - catalog.suffix_length..];
    let boundary = wrapped_window_bits(
        &logical_bits,
        logical_bits.len() - (catalog.prefix_length / 2),
        catalog.prefix_length,
    );
    let expected_boundary = format!(
        "{}{}",
        &reference.bit_suffix[reference.bit_suffix.len() - (catalog.prefix_length / 2)..],
        &reference.bit_prefix[..catalog.prefix_length / 2]
    );

    assert_eq!(
        logical_bits.len(),
        catalog.chip_length,
        "GPS L2C CM PRN {} length mismatch",
        prn
    );
    assert_eq!(
        sha256_hex(&logical_bits),
        reference.bit_sha256,
        "GPS L2C CM PRN {} fingerprint mismatch",
        prn
    );
    assert_eq!(prefix, reference.bit_prefix, "GPS L2C CM PRN {} prefix mismatch", prn);
    assert_eq!(middle, reference.bit_middle, "GPS L2C CM PRN {} middle mismatch", prn);
    assert_eq!(suffix, reference.bit_suffix, "GPS L2C CM PRN {} suffix mismatch", prn);
    assert_eq!(boundary, expected_boundary, "GPS L2C CM PRN {} boundary mismatch", prn);
}

pub fn logical_bits_from_code(code: &[i8]) -> String {
    assert_eq!(code.len(), GPS_L2C_CM_CODE_CHIPS, "unexpected GPS L2C CM code length");
    code.iter()
        .map(|chip| match chip {
            1 => '0',
            -1 => '1',
            _ => panic!("GPS L2C CM code must be bipolar, found {chip}"),
        })
        .collect()
}

pub fn sha256_hex(payload: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(payload.as_bytes());
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("gps_l2c_cm_reference_catalog.toml")
}
