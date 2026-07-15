#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    BroadcastProductsProvider, GpsEphemeris, ProductDiagnostics, Products, ProductsProvider,
};

fn make_eph(prn: u8) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.02,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.1,
        omegadot: 0.0,
        w: 0.2,
        m0: 0.3,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 1.0e-4,
        af1: -2.0e-12,
        af2: 3.0e-20,
        tgd: 8.0e-9,
    }
}

#[test]
fn precise_orbit_products_report_sp3_interpolation_summary() {
    let eph = make_eph(1);
    let sp3_data = "\
* 2020 01 01 00 00 00.000000
PG01  1.000000  0.000000  5.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10.000000  1.000000  6.000000  0.000000
* 2020 01 01 00 30 00.000000
PG01  49.000000  8.000000  9.000000  0.000000
* 2020 01 01 00 45 00.000000
PG01  142.000000  27.000000  14.000000  0.000000
* 2020 01 01 01 00 00.000000
PG01  313.000000  64.000000  21.000000  0.000000
";
    let sp3 = sp3_data.parse().expect("parse SP3");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_sp3(sp3);
    let mut diag = ProductDiagnostics::default();
    let state =
        products.sat_state(eph.sat, 1_350.0, &mut diag).expect("precise orbit state from SP3");

    assert!(diag.fallbacks.is_empty());
    assert!((state.x_m - 24_250.0).abs() < 1e-6);

    let summary = diag
        .sp3_interpolation_summary
        .expect("SP3 interpolation summary in precise orbit diagnostics");
    assert_eq!(summary.sample_count, 3);
    assert!(summary.max_position_error_m.abs() < 1e-6);
    assert!(summary.rms_position_error_m.abs() < 1e-6);
}

#[test]
fn precise_orbit_products_report_unusable_sp3_gap_before_broadcast_fallback() {
    let eph = make_eph(1);
    let sp3_data = "\
* 2020 01 01 00 00 00.000000
PG01  1.000000  0.000000  5.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10.000000  1.000000  6.000000  0.000000
* 2020 01 01 01 00 00.000000
PG01  313.000000  64.000000  21.000000  0.000000
";
    let sp3 = sp3_data.parse().expect("parse SP3");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_sp3(sp3);
    let mut diag = ProductDiagnostics::default();

    let state = products.sat_state(eph.sat, 2_700.0, &mut diag).expect("broadcast fallback state");

    assert!(!diag.fallbacks.is_empty());
    assert!(diag.fallbacks.iter().any(|message| message.contains("SP3 unusable")));
    assert!(state.x_m.is_finite());
}
