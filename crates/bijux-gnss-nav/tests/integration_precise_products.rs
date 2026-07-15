#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    gps_satellite_clock_correction, BiasSinexProvider, BroadcastProductsProvider, CodeBiasProvider,
    GpsEphemeris, ProductDiagnostics, Products, ProductsProvider, SatelliteClockUncertaintySource,
    SatelliteOrbitUncertaintySource,
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
fn precise_products_attach_clk_sigma_to_satellite_state_uncertainty() {
    let eph = make_eph(1);
    let sp3_data = "\
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000  0.000000  4  5  6  7
* 2020 01 01 00 15 00.000000
PG01  10001.000000  20001.000000  30001.000000  0.000000  4  5  6  7
";
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000010
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
";
    let sp3 = sp3_data.parse().expect("sp3 parse");
    let clk = clk_data.parse().expect("clk parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()]))
        .with_sp3(sp3)
        .with_clk(clk);
    let mut diag = ProductDiagnostics::default();

    let state = products.sat_state(eph.sat, 900.0, &mut diag).expect("precise satellite state");

    assert_eq!(state.uncertainty.orbit_sigma_m, Some(0.064));
    assert_eq!(state.uncertainty.orbit_source, SatelliteOrbitUncertaintySource::Sp3Accuracy);
    assert_eq!(state.uncertainty.clock_sigma_s, Some(20.0e-9));
    assert_eq!(state.uncertainty.clock_source, SatelliteClockUncertaintySource::ClkSigma);
}

#[test]
fn precise_products_attach_clk_sigma_to_broadcast_orbit_fallback_state() {
    let eph = make_eph(1);
    let clk_data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000010
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
";
    let clk = clk_data.parse().expect("clk parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph.clone()])).with_clk(clk);
    let mut diag = ProductDiagnostics::default();

    let state = products.sat_state(eph.sat, 900.0, &mut diag).expect("broadcast orbit state");

    assert_eq!(state.uncertainty.orbit_source, SatelliteOrbitUncertaintySource::GpsUra);
    assert_eq!(state.uncertainty.orbit_sigma_m, Some(4.85));
    assert_eq!(state.uncertainty.clock_sigma_s, Some(20.0e-9));
    assert_eq!(state.uncertainty.clock_source, SatelliteClockUncertaintySource::ClkSigma);
    assert!(diag.fallbacks.iter().any(|fallback| fallback.contains("SP3 missing")));
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

#[test]
fn products_expose_external_bias_sinex_code_biases() {
    let eph = make_eph(1);
    let dcb = "\
%=BIA 1.00 COD 2016:327:06748 IGS 2016:323:00000 2016:324:00000 A 00000003
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
OSB G063 G01 C1C 2016:323:00000 2016:324:00000 ns 10.2669 0.0257
OSB G063 G01 C1W 2016:323:00000 2016:324:00000 ns 11.7118 0.0174
OSB G063 G01 C2W 2016:323:00000 2016:324:00000 ns 19.2886 0.0281
-BIAS/SOLUTION
%=ENDBIA
"
    .parse::<BiasSinexProvider>()
    .expect("bias sinex parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph])).with_dcb(dcb);
    let l1 = bijux_gnss_core::api::SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        band: bijux_gnss_core::api::SignalBand::L1,
        code: bijux_gnss_core::api::SignalCode::Ca,
    };
    let l2 = bijux_gnss_core::api::SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        band: bijux_gnss_core::api::SignalBand::L2,
        code: bijux_gnss_core::api::SignalCode::Py,
    };

    assert!(products.code_bias_m(l1).is_some());
    assert!(products.differential_code_bias_m(l1, l2).is_some());
}

#[test]
fn broadcast_products_expose_gps_group_delay_code_biases_by_signal() {
    let eph = make_eph(1);
    let products = BroadcastProductsProvider::new(vec![eph.clone()]);
    let l1 = SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca };
    let l5 = SigId { sat: eph.sat, band: SignalBand::L5, code: SignalCode::Unknown };

    let l1_bias_m = products.code_bias_m(l1).expect("L1 code bias");
    let l5_bias_m = products.code_bias_m(l5).expect("L5 code bias");

    assert!(l1_bias_m.abs() < 1.0e-12);
    assert!(l5_bias_m.abs() > 1.0);
}

#[test]
fn products_sum_broadcast_group_delay_with_external_code_biases() {
    let eph = make_eph(1);
    let dcb = "\
%=BIA 1.00 COD 2016:327:06748 IGS 2016:323:00000 2016:324:00000 A 00000003
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
OSB G063 G01 C1C 2016:323:00000 2016:324:00000 ns 10.2669 0.0257
OSB G063 G01 C5Q 2016:323:00000 2016:324:00000 ns 14.0000 0.0200
-BIAS/SOLUTION
%=ENDBIA
"
    .parse::<BiasSinexProvider>()
    .expect("bias sinex parse");
    let products = Products::new(BroadcastProductsProvider::new(vec![eph])).with_dcb(dcb);
    let l1 = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let l5 = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        band: SignalBand::L5,
        code: SignalCode::Unknown,
    };

    let l1_bias_m = products.code_bias_m(l1).expect("L1 bias");
    let l5_bias_m = products.code_bias_m(l5).expect("L5 bias");

    assert!(l1_bias_m.abs() > 1.0);
    assert!(l5_bias_m > l1_bias_m);
}
