#![cfg(feature = "precise-products")]
#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};

use bijux_gnss_nav::api::Sp3Provider;

#[test]
fn sp3_interpolates_positions() {
    let data = "\
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10001.000000  20001.000000  30001.000000  0.000000
";
    let provider: Sp3Provider = data.parse().expect("parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let state = provider.sat_state(sat, 900.0).expect("state");
    assert_eq!(state.x_m, 10_001_000.0);
    assert_eq!(state.y_m, 20_001_000.0);
    assert_eq!(state.z_m, 30_001_000.0);
}

#[test]
fn sp3_interpolates_cubic_orbit_segments_between_epochs() {
    let data = "\
* 2020 01 01 00 00 00.000000
PG01  1.000000  0.000000  5.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10.000000  1.000000  6.000000  0.000000
* 2020 01 01 00 30 00.000000
PG01  49.000000  8.000000  9.000000  0.000000
* 2020 01 01 00 45 00.000000
PG01  142.000000  27.000000  14.000000  0.000000
";
    let provider: Sp3Provider = data.parse().expect("parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let state = provider.sat_state(sat, 1_350.0).expect("interpolated state");

    assert!((state.x_m - 24_250.0).abs() < 1e-6);
    assert!((state.y_m - 3_375.0).abs() < 1e-6);
    assert!((state.z_m - 7_250.0).abs() < 1e-6);
}
