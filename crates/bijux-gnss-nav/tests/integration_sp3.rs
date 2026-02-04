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
    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 1,
    };
    let state = provider.sat_state(sat, 900.0).expect("state");
    assert!(state.x_m > 10_000_000.0);
    assert!(state.y_m > 20_000_000.0);
}
