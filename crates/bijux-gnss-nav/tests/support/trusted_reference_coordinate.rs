#![allow(dead_code, missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_testkit::coordinates::{ecef_to_enu, geodetic_to_ecef};
use bijux_gnss_testkit::reference_data::reference_coordinate::{
    load_trusted_reference_coordinates, trusted_reference_coordinate_by_fixture,
    TrustedReferenceCoordinate, TrustedReferenceCoordinateError,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrustedReferenceEnuError {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
    pub horizontal_m: f64,
    pub three_dimensional_m: f64,
}

pub fn require_trusted_reference_coordinate_by_fixture(
    fixture_name: &str,
) -> Result<TrustedReferenceCoordinate, TrustedReferenceCoordinateError> {
    let catalog = trusted_reference_coordinate_catalog()?;
    trusted_reference_coordinate_by_fixture(catalog, fixture_name)
}

pub fn trusted_reference_coordinate_enu_error_m(
    solution_ecef_m: (f64, f64, f64),
    coordinate: &TrustedReferenceCoordinate,
) -> TrustedReferenceEnuError {
    let (east_m, north_m, up_m) = ecef_to_enu(
        solution_ecef_m.0,
        solution_ecef_m.1,
        solution_ecef_m.2,
        coordinate.lat_deg,
        coordinate.lon_deg,
        coordinate.alt_m,
    );
    TrustedReferenceEnuError {
        east_m,
        north_m,
        up_m,
        horizontal_m: (east_m * east_m + north_m * north_m).sqrt(),
        three_dimensional_m: (east_m * east_m + north_m * north_m + up_m * up_m).sqrt(),
    }
}

pub fn trusted_reference_coordinate_ecef_m(
    coordinate: &TrustedReferenceCoordinate,
) -> (f64, f64, f64) {
    geodetic_to_ecef(coordinate.lat_deg, coordinate.lon_deg, coordinate.alt_m)
}

fn trusted_reference_coordinate_catalog(
) -> Result<&'static Vec<TrustedReferenceCoordinate>, TrustedReferenceCoordinateError> {
    static CATALOG: OnceLock<
        Result<Vec<TrustedReferenceCoordinate>, TrustedReferenceCoordinateError>,
    > = OnceLock::new();
    CATALOG
        .get_or_init(|| {
            load_trusted_reference_coordinates(
                &PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                    .join("tests/data")
                    .join("public_station_truth.csv"),
            )
        })
        .as_ref()
        .map_err(Clone::clone)
}
