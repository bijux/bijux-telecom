#![allow(missing_docs)]

mod support;

use support::public_station_truth::public_station_truth_by_fixture;

#[test]
fn public_station_truth_manifest_loads_ab43_by_fixture_name() {
    let truth = public_station_truth_by_fixture("unavco_ab43_20180114.obs");

    assert_eq!(truth.marker_name, "AB43");
    assert!((truth.lat_deg - 58.19884205).abs() < 1.0e-10);
    assert!((truth.lon_deg + 136.64080781).abs() < 1.0e-10);
    assert!((truth.alt_m - 26.9246).abs() < 1.0e-10);
    assert!(truth.source.contains("UNAVCO monument location comment"));
}
