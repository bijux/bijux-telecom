#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::geodetic_to_ecef;

use support::public_station_truth::{public_station_truth_by_fixture, station_enu_error_m};

#[test]
fn public_station_truth_manifest_loads_ab43_by_fixture_name() {
    let truth = public_station_truth_by_fixture("unavco_ab43_20180114.obs");

    assert_eq!(truth.marker_name, "AB43");
    assert!((truth.lat_deg - 58.19884205).abs() < 1.0e-10);
    assert!((truth.lon_deg + 136.64080781).abs() < 1.0e-10);
    assert!((truth.alt_m - 26.9246).abs() < 1.0e-10);
    assert!(truth.source.contains("UNAVCO monument location comment"));
}

#[test]
fn public_station_truth_reports_zero_enu_error_at_truth_position() {
    let truth = public_station_truth_by_fixture("unavco_ab43_20180114.obs");

    let enu_error = station_enu_error_m(truth.truth_ecef_m(), &truth);

    assert!(enu_error.east_m.abs() < 1.0e-6);
    assert!(enu_error.north_m.abs() < 1.0e-6);
    assert!(enu_error.up_m.abs() < 1.0e-6);
    assert!(enu_error.horizontal_m.abs() < 1.0e-6);
    assert!(enu_error.three_dimensional_m.abs() < 1.0e-6);
}

#[test]
fn public_station_truth_preserves_up_error_for_vertical_offset() {
    let truth = public_station_truth_by_fixture("unavco_ab43_20180114.obs");
    let elevated_ecef_m = geodetic_to_ecef(truth.lat_deg, truth.lon_deg, truth.alt_m + 12.5);

    let enu_error = station_enu_error_m(elevated_ecef_m, &truth);

    assert!(enu_error.east_m.abs() < 1.0e-3);
    assert!(enu_error.north_m.abs() < 1.0e-3);
    assert!((enu_error.up_m - 12.5).abs() < 1.0e-3);
    assert!(enu_error.horizontal_m.abs() < 1.0e-3);
    assert!((enu_error.three_dimensional_m - 12.5).abs() < 1.0e-3);
}
