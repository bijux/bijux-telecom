#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use serde::Deserialize;

use crate::support::period_reference::{assert_code_period_reference, ReferenceWindow};

const CATALOG_SCHEMA: &str = "galileo_e5_reference_catalog.v1";

#[derive(Debug, Deserialize)]
pub struct GalileoE5ReferenceCatalog {
    schema: String,
    pub reference_origin: String,
    pub table_source: String,
    pub primary_code_length: usize,
    pub primary_window_length: usize,
    pub primary_middle_start: usize,
    pub primary_boundary_start: usize,
    pub primary_code: Vec<GalileoE5PrimaryCodeReference>,
    pub secondary_code: Vec<GalileoE5SecondaryCodeReference>,
}

#[derive(Debug, Deserialize)]
pub struct GalileoE5PrimaryCodeReference {
    pub component: String,
    pub prn: u8,
    pub register_2_start_octal: String,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
    pub bit_boundary: String,
}

#[derive(Debug, Deserialize)]
pub struct GalileoE5SecondaryCodeReference {
    pub component: String,
    pub prn: Option<u8>,
    pub bit_length: usize,
    pub window_length: usize,
    pub middle_start: usize,
    pub boundary_start: usize,
    pub bit_sha256: String,
    pub bit_prefix: String,
    pub bit_middle: String,
    pub bit_suffix: String,
    pub bit_boundary: String,
}

pub fn load_reference_catalog() -> GalileoE5ReferenceCatalog {
    let raw = std::fs::read_to_string(reference_catalog_path())
        .expect("read Galileo E5 reference catalog fixture");
    let catalog: GalileoE5ReferenceCatalog =
        toml::from_str(&raw).expect("parse Galileo E5 reference catalog fixture");
    catalog.validate();
    catalog
}

impl GalileoE5ReferenceCatalog {
    fn validate(&self) {
        assert_eq!(self.schema, CATALOG_SCHEMA, "unexpected Galileo E5 catalog schema");
        assert_eq!(self.primary_code_length, 10_230, "unexpected Galileo E5 primary-code length");
        assert_eq!(self.primary_window_length, 32, "unexpected Galileo E5 primary window length");
        assert_eq!(self.primary_middle_start, 5_099, "unexpected Galileo E5 primary middle start");
        assert_eq!(
            self.primary_boundary_start, 10_214,
            "unexpected Galileo E5 primary boundary start"
        );
        assert_eq!(self.primary_code.len(), 200, "unexpected Galileo E5 primary reference count");
        assert_eq!(
            self.secondary_code.len(),
            102,
            "unexpected Galileo E5 secondary reference count"
        );
    }

    pub fn primary_code_reference(
        &self,
        component: &str,
        prn: u8,
    ) -> &GalileoE5PrimaryCodeReference {
        self.primary_code
            .iter()
            .find(|reference| reference.component == component && reference.prn == prn)
            .unwrap_or_else(|| {
                panic!("missing Galileo E5 primary reference for {component} PRN {prn}")
            })
    }

    pub fn secondary_code_reference(
        &self,
        component: &str,
        prn: Option<u8>,
    ) -> &GalileoE5SecondaryCodeReference {
        self.secondary_code
            .iter()
            .find(|reference| reference.component == component && reference.prn == prn)
            .unwrap_or_else(|| {
                panic!("missing Galileo E5 secondary reference for {component} {prn:?}")
            })
    }
}

pub fn assert_primary_code_matches_reference(
    catalog: &GalileoE5ReferenceCatalog,
    component: &str,
    prn: u8,
    logical_bits: &str,
) {
    let reference = catalog.primary_code_reference(component, prn);
    let windows = vec![
        ReferenceWindow { start: 0, bits: reference.bit_prefix.clone() },
        ReferenceWindow { start: catalog.primary_middle_start, bits: reference.bit_middle.clone() },
        ReferenceWindow {
            start: catalog.primary_code_length - catalog.primary_window_length,
            bits: reference.bit_suffix.clone(),
        },
        ReferenceWindow {
            start: catalog.primary_boundary_start,
            bits: reference.bit_boundary.clone(),
        },
    ];
    assert_code_period_reference(
        logical_bits,
        catalog.primary_code_length,
        &reference.bit_sha256,
        &windows,
        &format!("Galileo E5 primary {component} PRN {prn}"),
    );
}

pub fn assert_secondary_code_matches_reference(
    catalog: &GalileoE5ReferenceCatalog,
    component: &str,
    prn: Option<u8>,
    logical_bits: &str,
) {
    let reference = catalog.secondary_code_reference(component, prn);
    let windows = vec![
        ReferenceWindow { start: 0, bits: reference.bit_prefix.clone() },
        ReferenceWindow { start: reference.middle_start, bits: reference.bit_middle.clone() },
        ReferenceWindow {
            start: reference.bit_length - reference.window_length,
            bits: reference.bit_suffix.clone(),
        },
        ReferenceWindow { start: reference.boundary_start, bits: reference.bit_boundary.clone() },
    ];
    assert_code_period_reference(
        logical_bits,
        reference.bit_length,
        &reference.bit_sha256,
        &windows,
        &format!("Galileo E5 secondary {component} {prn:?}"),
    );
}

fn reference_catalog_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join("galileo_e5_reference_catalog.toml")
}
