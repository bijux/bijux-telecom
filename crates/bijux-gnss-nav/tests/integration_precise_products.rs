#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    gps_satellite_clock_correction, BroadcastProductsProvider, GpsEphemeris, ProductDiagnostics,
    Products, ProductsProvider,
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
fn broadcast_products_expose_full_broadcast_clock_terms() {
    let eph = make_eph(1);
    let provider = BroadcastProductsProvider::new(vec![eph.clone()]);
    let mut diag = ProductDiagnostics::default();
    let correction =
        provider.clock_correction(eph.sat, 1_600.0, &mut diag).expect("broadcast clock correction");
    let expected = gps_satellite_clock_correction(&eph, 1_600.0);

    assert!(diag.fallbacks.is_empty());
    assert!((correction.bias_s - expected.bias_s).abs() < 1e-18);
    assert!((correction.base_bias_s - expected.base_bias_s).abs() < 1e-18);
    assert!((correction.drift_s_per_s - expected.drift_s_per_s).abs() < 1e-24);
    assert!((correction.drift_rate_s_per_s2 - expected.drift_rate_s_per_s2).abs() < 1e-30);
    assert!((correction.relativistic_s - expected.relativistic_s).abs() < 1e-18);
    assert!((correction.group_delay_s - expected.group_delay_s).abs() < 1e-18);
}

#[test]
fn precise_clk_bias_overrides_broadcast_clock_terms() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000003
";
    let clk = clk_data.parse().expect("clk parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();
    let correction =
        products.clock_correction(eph.sat, 900.0, &mut diag).expect("precise clock correction");

    assert!(diag.fallbacks.is_empty());
    assert!((correction.bias_s - 3.0e-9).abs() < 1e-18);
    assert!((correction.base_bias_s - 3.0e-9).abs() < 1e-18);
    assert_eq!(correction.drift_s_per_s, 0.0);
    assert_eq!(correction.drift_rate_s_per_s2, 0.0);
    assert_eq!(correction.relativistic_s, 0.0);
    assert_eq!(correction.group_delay_s, 0.0);
}

#[test]
fn precise_clk_falls_back_to_broadcast_clock_terms_out_of_coverage() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000003
";
    let clk = clk_data.parse().expect("clk parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();
    let correction = products
        .clock_correction(eph.sat, 1_800.0, &mut diag)
        .expect("broadcast fallback clock correction");
    let expected = gps_satellite_clock_correction(&eph, 1_800.0);

    assert!(!diag.fallbacks.is_empty());
    assert!((correction.bias_s - expected.bias_s).abs() < 1e-18);
    assert!((correction.base_bias_s - expected.base_bias_s).abs() < 1e-18);
    assert!((correction.relativistic_s - expected.relativistic_s).abs() < 1e-18);
    assert!((correction.group_delay_s - expected.group_delay_s).abs() < 1e-18);
}

#[test]
fn stale_broadcast_ephemeris_is_rejected_for_satellite_state() {
    let eph = make_eph(1);
    let provider = BroadcastProductsProvider::new(vec![eph.clone()]);
    let mut diag = ProductDiagnostics::default();

    let state = provider.sat_state(eph.sat, 7_201.0, &mut diag);

    assert!(state.is_none());
    assert_eq!(diag.fallbacks.len(), 1);
    assert!(diag.fallbacks[0].contains("broadcast ephemeris stale"));
    assert!(diag.fallbacks[0].contains("toe_age_s=7201.000"));
}

#[test]
fn stale_broadcast_ephemeris_is_rejected_for_clock_correction() {
    let eph = make_eph(1);
    let provider = BroadcastProductsProvider::new(vec![eph.clone()]);
    let mut diag = ProductDiagnostics::default();

    let correction = provider.clock_correction(eph.sat, 7_201.0, &mut diag);

    assert!(correction.is_none());
    assert_eq!(diag.fallbacks.len(), 1);
    assert!(diag.fallbacks[0].contains("broadcast ephemeris stale"));
    assert!(diag.fallbacks[0].contains("toc_age_s=7201.000"));
}
