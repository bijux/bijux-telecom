#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use bijux_gnss_signal::api::{
    GalileoE1Channel, GALILEO_E1_PRIMARY_CODE_CHIPS, GALILEO_E1_SECONDARY_CODE_CHIPS,
};
use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::support::period_reference::wrapped_window_bits;

const CATALOG_SCHEMA: &str = "galileo_e1_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct GalileoE1ReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub harshadms_source: String,
    pub mx4_source: String,
    pub primary_code_length_bits: usize,
    pub window_length: usize,
    pub middle_start: usize,
    pub secondary_code_name: String,
    pub secondary_code_bits: String,
    pub secondary_code_sha256: String,
    pub primary_code: Vec<GalileoE1PrimaryCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct GalileoE1PrimaryCodeReference {
    pub channel: String,
    pub prn: u8,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
}

pub fn load_reference_catalog() -> GalileoE1ReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read Galileo E1 reference catalog fixture");
    let catalog: GalileoE1ReferenceCatalog =
        toml::from_str(&raw).expect("parse Galileo E1 reference catalog fixture");
    catalog.validate();
    catalog
}

impl GalileoE1ReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected Galileo E1 catalog schema");
        assert_eq!(
            self.primary_code_length_bits, GALILEO_E1_PRIMARY_CODE_CHIPS,
            "unexpected Galileo E1 primary-code length in reference catalog"
        );
        assert_eq!(self.window_length, 32, "unexpected Galileo E1 window length");
        assert_eq!(self.middle_start, 2030, "unexpected Galileo E1 middle window start");
        assert_eq!(
            self.secondary_code_name, "CS25",
            "unexpected Galileo E1 secondary-code name in reference catalog"
        );
        assert_eq!(
            self.secondary_code_bits.len(),
            GALILEO_E1_SECONDARY_CODE_CHIPS,
            "unexpected Galileo E1 secondary-code length in reference catalog"
        );
        assert_eq!(
            self.secondary_code_sha256,
            sha256_hex(&self.secondary_code_bits),
            "Galileo E1 secondary-code fingerprint drift in reference catalog"
        );
        assert_eq!(
            self.primary_code.len(),
            100,
            "unexpected Galileo E1 primary-code reference count"
        );
        assert_eq!(
            self.primary_code.iter().filter(|reference| reference.channel == "E1B").count(),
            50,
            "unexpected Galileo E1-B reference count"
        );
        assert_eq!(
            self.primary_code.iter().filter(|reference| reference.channel == "E1C").count(),
            50,
            "unexpected Galileo E1-C reference count"
        );
    }

    pub fn primary_code_reference(
        &self,
        channel: GalileoE1Channel,
        prn: u8,
    ) -> &GalileoE1PrimaryCodeReference {
        let channel_name = channel_name(channel);
        self.primary_code
            .iter()
            .find(|reference| reference.channel == channel_name && reference.prn == prn)
            .unwrap_or_else(|| panic!("missing {channel_name} reference for Galileo PRN {prn}"))
    }
}

pub fn assert_primary_code_matches_reference(
    catalog: &GalileoE1ReferenceCatalog,
    channel: GalileoE1Channel,
    prn: u8,
    code: &[i8],
) {
    let reference = catalog.primary_code_reference(channel, prn);
    let logical_bits = logical_bits_from_code(code);
    let prefix = &logical_bits[..reference.bit_prefix.len()];
    let middle = &logical_bits[catalog.middle_start..catalog.middle_start + catalog.window_length];
    let suffix = &logical_bits[logical_bits.len() - reference.bit_suffix.len()..];
    let boundary = wrapped_window_bits(
        &logical_bits,
        logical_bits.len() - (catalog.window_length / 2),
        catalog.window_length,
    );
    let expected_boundary = format!(
        "{}{}",
        &reference.bit_suffix[reference.bit_suffix.len() - (catalog.window_length / 2)..],
        &reference.bit_prefix[..catalog.window_length / 2]
    );

    assert_eq!(
        logical_bits.len(),
        catalog.primary_code_length_bits,
        "{} PRN {} length mismatch",
        channel_name(channel),
        prn
    );
    assert_eq!(
        sha256_hex(&logical_bits),
        reference.bit_sha256,
        "{} PRN {} fingerprint mismatch",
        channel_name(channel),
        prn
    );
    assert_eq!(
        prefix,
        reference.bit_prefix,
        "{} PRN {} prefix mismatch",
        channel_name(channel),
        prn
    );
    assert_eq!(
        middle,
        reference.bit_middle,
        "{} PRN {} middle mismatch",
        channel_name(channel),
        prn
    );
    assert_eq!(
        suffix,
        reference.bit_suffix,
        "{} PRN {} suffix mismatch",
        channel_name(channel),
        prn
    );
    assert_eq!(
        boundary,
        expected_boundary,
        "{} PRN {} boundary mismatch",
        channel_name(channel),
        prn
    );
}

pub fn logical_bits_from_code(code: &[i8]) -> String {
    assert_eq!(
        code.len(),
        GALILEO_E1_PRIMARY_CODE_CHIPS,
        "unexpected Galileo E1 primary-code length"
    );
    logical_bits_from_bipolar(code)
}

pub fn assert_secondary_code_matches_reference(catalog: &GalileoE1ReferenceCatalog, code: &[i8]) {
    let logical_bits = logical_bits_from_bipolar(code);
    assert_eq!(logical_bits, catalog.secondary_code_bits, "Galileo E1 secondary-code bit mismatch");
    assert_eq!(
        sha256_hex(&logical_bits),
        catalog.secondary_code_sha256,
        "Galileo E1 secondary-code fingerprint mismatch"
    );
    assert_eq!(&logical_bits[..16], &catalog.secondary_code_bits[..16], "Galileo E1 secondary-code prefix mismatch");
    assert_eq!(
        &logical_bits[4..20],
        &catalog.secondary_code_bits[4..20],
        "Galileo E1 secondary-code middle mismatch"
    );
    assert_eq!(
        &logical_bits[logical_bits.len() - 16..],
        &catalog.secondary_code_bits[catalog.secondary_code_bits.len() - 16..],
        "Galileo E1 secondary-code suffix mismatch"
    );
    assert_eq!(
        wrapped_window_bits(&logical_bits, logical_bits.len() - 8, 16),
        wrapped_window_bits(&catalog.secondary_code_bits, catalog.secondary_code_bits.len() - 8, 16),
        "Galileo E1 secondary-code boundary mismatch"
    );
}

pub fn sha256_hex(payload: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(payload.as_bytes());
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}

fn logical_bits_from_bipolar(code: &[i8]) -> String {
    code.iter()
        .map(|chip| match chip {
            1 => '0',
            -1 => '1',
            _ => panic!("Galileo E1 code must be bipolar, found {chip}"),
        })
        .collect()
}

fn channel_name(channel: GalileoE1Channel) -> &'static str {
    match channel {
        GalileoE1Channel::E1B => "E1B",
        GalileoE1Channel::E1C => "E1C",
    }
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("galileo_e1_reference_catalog.toml")
}
