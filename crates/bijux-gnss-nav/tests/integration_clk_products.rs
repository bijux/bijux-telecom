#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    BroadcastProductsProvider, ClkInterpolationEdgePolicy, ClkInterpolationWindowPolicy,
    GpsEphemeris, PreciseProductDiscontinuityKind, PreciseProductSurface, ProductDiagnostics,
    Products, ProductsProvider,
};

fn make_eph(prn: u8) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
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
fn precise_clock_products_report_clk_interpolation_summary() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000001
AS G01 2020 01 01 00 15 00.000000  2  0.000000010  0.000000002
AS G01 2020 01 01 00 30 00.000000  2  0.000000049  0.000000009
AS G01 2020 01 01 00 45 00.000000  2  0.000000142  0.000000028
AS G01 2020 01 01 01 00 00.000000  2  0.000000313  0.000000065
";
    let clk = clk_data.parse().expect("parse CLK");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();
    let correction = products
        .clock_correction(eph.sat, 1_350.0, &mut diag)
        .expect("precise clock correction from CLK");

    assert!(diag.fallbacks.is_empty());
    assert!((correction.bias_s - 2.425e-8).abs() < 1e-18);

    let summary = diag
        .clk_interpolation_summary
        .expect("CLK interpolation summary in precise clock diagnostics");
    assert_eq!(summary.policy.support_point_count, 4);
    assert_eq!(summary.policy.polynomial_order, 3);
    assert_eq!(summary.policy.window_policy, ClkInterpolationWindowPolicy::NearestCentered);
    assert_eq!(summary.policy.edge_policy, ClkInterpolationEdgePolicy::InsideCoverageOnly);
    assert_eq!(summary.policy.max_bias_step_s, 1.0e-6);
    assert_eq!(summary.sample_count, 3);
    assert_eq!(summary.withheld_sample_count, 3);
    assert_eq!(summary.withheld_edge_sample_count, 2);
    assert!(summary.max_bias_error_s.abs() < 1e-18);
    assert!(summary.rms_bias_error_s.abs() < 1e-18);
    assert!(summary.max_sigma_error_s.expect("sigma summary").abs() < 1e-18);
    assert!(summary.rms_sigma_error_s.expect("sigma RMS summary").abs() < 1e-18);
}

#[test]
fn precise_clock_products_preserve_clk_clock_rate() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  6  0.000000001  0.000000001 -0.000000003  0.000000001  0.000000005  0.000000001
AS G01 2020 01 01 00 15 00.000000  6  0.000000010  0.000000002 -0.000000012  0.000000002  0.000000014  0.000000002
";
    let clk = clk_data.parse().expect("parse CLK");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();

    let correction =
        products.clock_correction(eph.sat, 900.0, &mut diag).expect("precise clock correction");

    assert!(diag.fallbacks.is_empty());
    assert_eq!(correction.bias_s, 1.0e-8);
    assert_eq!(correction.drift_s_per_s, -12.0e-9);
    assert_eq!(correction.drift_rate_s_per_s2, 14.0e-9);
}

#[test]
fn precise_clock_products_report_unusable_clk_jump_before_broadcast_fallback() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000002
AS G01 2020 01 01 00 30 00.000000  1  0.000003500
";
    let clk = clk_data.parse().expect("parse CLK");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();

    let correction =
        products.clock_correction(eph.sat, 1_350.0, &mut diag).expect("broadcast fallback clock");

    assert!(!diag.fallbacks.is_empty());
    assert!(diag.fallbacks.iter().any(|message| message.contains("CLK clock_jump")));
    assert!(diag.discontinuities.iter().any(|discontinuity| {
        discontinuity.surface == PreciseProductSurface::Clock
            && discontinuity.kind == PreciseProductDiscontinuityKind::ClockJump
            && discontinuity.sat == eph.sat
    }));
    assert_ne!(correction.bias_s, 3.5e-6);
}
