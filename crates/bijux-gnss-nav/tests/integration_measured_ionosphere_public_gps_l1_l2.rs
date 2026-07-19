#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_nav::api::{
    measured_ionosphere_from_obs_epochs, parse_rinex_gps_observation_dataset,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

fn approx_eq(left: f64, right: f64, relative_tolerance: f64) {
    let scale = left.abs().max(right.abs()).max(1.0);
    let error = (left - right).abs();
    assert!(
        error <= relative_tolerance * scale,
        "left={left} right={right} error={error} scale={scale} tolerance={relative_tolerance}",
    );
}

#[test]
fn public_gps_l1_l2_measured_ionosphere_emits_many_valid_code_observations() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public GPS dual-frequency observations");

    let measured =
        measured_ionosphere_from_obs_epochs(&observations.epochs, SignalBand::L1, SignalBand::L2);
    let valid_code =
        measured.iter().filter(|observation| observation.code_status == "ok").collect::<Vec<_>>();

    assert!(
        valid_code.len() > 50,
        "expected many valid L1/L2 measured ionosphere code observations, got {}",
        valid_code.len()
    );
    assert!(
        valid_code.iter().all(|observation| {
            observation.code_geometry_free_m.is_some()
                && observation.code_delay_band_1_m.is_some()
                && observation.code_delay_band_2_m.is_some()
                && observation.code_delay_var_band_1_m2.is_some()
                && observation.code_delay_var_band_2_m2.is_some()
        }),
        "expected valid code observations to carry geometry-free and band-specific delays",
    );
    assert!(
        valid_code.iter().all(|observation| {
            observation.code_geometry_free_m.expect("geometry-free code").is_finite()
                && observation.code_delay_band_1_m.expect("band 1 code delay").is_finite()
                && observation.code_delay_band_2_m.expect("band 2 code delay").is_finite()
                && observation.code_delay_var_band_1_m2.expect("band 1 code variance").is_finite()
                && observation.code_delay_var_band_2_m2.expect("band 2 code variance").is_finite()
        }),
        "expected valid code observations to stay finite",
    );
}

#[test]
fn public_gps_l1_l2_measured_ionosphere_preserves_code_dual_frequency_relations() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public GPS dual-frequency observations");

    let measured =
        measured_ionosphere_from_obs_epochs(&observations.epochs, SignalBand::L1, SignalBand::L2);
    let valid_code =
        measured.iter().filter(|observation| observation.code_status == "ok").collect::<Vec<_>>();

    assert!(
        valid_code.len() > 50,
        "expected many valid L1/L2 measured ionosphere code observations, got {}",
        valid_code.len()
    );

    for observation in valid_code {
        let code_geometry_free_m = observation.code_geometry_free_m.expect("geometry-free code");
        let code_delay_band_1_m = observation.code_delay_band_1_m.expect("band 1 code delay");
        let code_delay_band_2_m = observation.code_delay_band_2_m.expect("band 2 code delay");
        let f1_2 = observation.f1_hz * observation.f1_hz;
        let f2_2 = observation.f2_hz * observation.f2_hz;
        let denom = f1_2 - f2_2;

        approx_eq(code_delay_band_2_m - code_delay_band_1_m, code_geometry_free_m, 1.0e-12);
        approx_eq(code_delay_band_1_m, code_geometry_free_m * (f2_2 / denom), 1.0e-12);
        approx_eq(code_delay_band_2_m, code_geometry_free_m * (f1_2 / denom), 1.0e-12);
    }
}
