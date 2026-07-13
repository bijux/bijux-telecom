#![allow(dead_code, missing_docs)]

use std::collections::BTreeSet;
use std::path::PathBuf;

use bijux_gnss_signal::api::GPS_L2C_CL_CODE_CHIPS;
use serde::Deserialize;
use sha2::{Digest, Sha256};

const CATALOG_SCHEMA: &str = "gps_l2c_cl_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct GpsL2cClReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub spec_source: String,
    pub published_tables: String,
    pub chip_length: usize,
    pub range_length: usize,
    pub range_offsets: Vec<usize>,
    pub published_prn_count: usize,
    pub code: Vec<GpsL2cClCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct GpsL2cClCodeReference {
    pub prn: u8,
    pub initial_state_octal: String,
    pub end_state_octal: String,
    pub bit_sha256: String,
    pub range_bits: Vec<String>,
}

pub fn load_reference_catalog() -> GpsL2cClReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read GPS L2C CL reference catalog fixture");
    let catalog: GpsL2cClReferenceCatalog =
        toml::from_str(&raw).expect("parse GPS L2C CL reference catalog fixture");
    catalog.validate();
    catalog
}

impl GpsL2cClReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected GPS L2C CL catalog schema");
        assert_eq!(
            self.chip_length, GPS_L2C_CL_CODE_CHIPS,
            "unexpected GPS L2C CL code length in reference catalog"
        );
        assert_eq!(
            self.range_length, 64,
            "unexpected GPS L2C CL range length"
        );
        assert_eq!(
            self.range_offsets,
            vec![0, 10_230, 255_731, 511_463, 767_186, 767_218],
            "unexpected GPS L2C CL range offsets"
        );
        assert_eq!(
            self.code.len(),
            self.published_prn_count,
            "unexpected GPS L2C CL reference count"
        );
        assert_eq!(self.published_prn_count, 115, "unexpected GPS L2C CL published PRN count");

        let prns: Vec<u8> = self.code.iter().map(|reference| reference.prn).collect();
        let unique_prns: BTreeSet<u8> = prns.iter().copied().collect();
        assert_eq!(unique_prns.len(), prns.len(), "duplicate GPS L2C CL PRN reference");
        assert_eq!(self.code.first().map(|reference| reference.prn), Some(1));
        assert_eq!(self.code.last().map(|reference| reference.prn), Some(210));
        assert_eq!(
            prns.iter().copied().filter(|prn| (64..=158).contains(prn)).count(),
            0,
            "unexpected GPS L2C CL gap assignment in reference catalog"
        );

        for reference in &self.code {
            assert_eq!(
                reference.range_bits.len(),
                self.range_offsets.len(),
                "unexpected GPS L2C CL range count for PRN {}",
                reference.prn
            );
            for bits in &reference.range_bits {
                assert_eq!(
                    bits.len(),
                    self.range_length,
                    "unexpected GPS L2C CL range length for PRN {}",
                    reference.prn
                );
            }
        }
    }

    pub fn code_reference(&self, prn: u8) -> &GpsL2cClCodeReference {
        self.code
            .iter()
            .find(|reference| reference.prn == prn)
            .unwrap_or_else(|| panic!("missing GPS L2C CL reference for PRN {prn}"))
    }

    pub fn range_bits(&self, prn: u8, start_chip: usize) -> &str {
        let reference = self.code_reference(prn);
        let index = self
            .range_offsets
            .iter()
            .position(|offset| *offset == start_chip)
            .unwrap_or_else(|| panic!("missing GPS L2C CL range offset {start_chip}"));
        &reference.range_bits[index]
    }
}

pub fn assert_code_matches_reference(
    catalog: &GpsL2cClReferenceCatalog,
    prn: u8,
    code: &[i8],
) {
    let reference = catalog.code_reference(prn);
    let logical_bits = logical_bits_from_code(code);

    assert_eq!(
        logical_bits.len(),
        catalog.chip_length,
        "GPS L2C CL PRN {} length mismatch",
        prn
    );
    assert_eq!(
        sha256_hex(&logical_bits),
        reference.bit_sha256,
        "GPS L2C CL PRN {} fingerprint mismatch",
        prn
    );
    for (offset, expected_bits) in catalog.range_offsets.iter().zip(reference.range_bits.iter()) {
        assert_eq!(
            extract_range_bits(&logical_bits, *offset, catalog.range_length),
            *expected_bits,
            "GPS L2C CL PRN {} range mismatch at chip {}",
            prn,
            offset
        );
    }
}

pub fn assert_range_matches_reference(
    catalog: &GpsL2cClReferenceCatalog,
    prn: u8,
    start_chip: usize,
    code: &[i8],
) {
    let logical_bits = logical_bits_from_code(code);
    assert_eq!(
        logical_bits.len(),
        catalog.range_length,
        "GPS L2C CL PRN {} range length mismatch at chip {}",
        prn,
        start_chip
    );
    assert_eq!(
        logical_bits,
        catalog.range_bits(prn, start_chip),
        "GPS L2C CL PRN {} range bits mismatch at chip {}",
        prn,
        start_chip
    );
}

pub fn logical_bits_from_code(code: &[i8]) -> String {
    code.iter()
        .map(|chip| match chip {
            1 => '0',
            -1 => '1',
            _ => panic!("GPS L2C CL code must be bipolar, found {chip}"),
        })
        .collect()
}

pub fn extract_range_bits(logical_bits: &str, start_chip: usize, length: usize) -> String {
    if start_chip + length <= logical_bits.len() {
        return logical_bits[start_chip..start_chip + length].to_owned();
    }
    let tail = &logical_bits[start_chip..];
    let head = &logical_bits[..length - tail.len()];
    format!("{tail}{head}")
}

pub fn sha256_hex(payload: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(payload.as_bytes());
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("gps_l2c_cl_reference_catalog.toml")
}
