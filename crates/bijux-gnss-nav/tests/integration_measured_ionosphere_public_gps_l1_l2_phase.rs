#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_nav::api::{measured_ionosphere_from_obs_epochs, parse_rinex_gps_observation_dataset};

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
fn public_gps_l1_l2_measured_ionosphere_emits_many_valid_phase_observations() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public GPS dual-frequency observations");

    let measured =
        measured_ionosphere_from_obs_epochs(&observations.epochs, SignalBand::L1, SignalBand::L2);
    let valid_phase = measured
        .iter()
        .filter(|observation| observation.phase_status == "ok")
        .collect::<Vec<_>>();

    assert!(
        valid_phase.len() > 50,
        "expected many valid L1/L2 measured ionosphere phase observations, got {}",
        valid_phase.len()
    );
    assert!(
        valid_phase.iter().all(|observation| {
            observation.phase_geometry_free_m.is_some()
                && observation.leveled_phase_geometry_free_m.is_some()
                && observation.phase_delay_band_1_m.is_some()
                && observation.phase_delay_band_2_m.is_some()
                && observation.phase_level_bias_m.is_some()
        }),
        "expected valid phase observations to carry leveled geometry-free and band-specific delays",
    );
    assert!(
        valid_phase.iter().all(|observation| {
            observation.phase_geometry_free_m.expect("geometry-free phase").is_finite()
                && observation
                    .leveled_phase_geometry_free_m
                    .expect("leveled geometry-free phase")
                    .is_finite()
                && observation.phase_delay_band_1_m.expect("band 1 phase delay").is_finite()
                && observation.phase_delay_band_2_m.expect("band 2 phase delay").is_finite()
                && observation.phase_level_bias_m.expect("phase level bias").is_finite()
        }),
        "expected valid phase observations to stay finite",
    );
}

#[test]
fn public_gps_l1_l2_measured_ionosphere_preserves_phase_dual_frequency_relations() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public GPS dual-frequency observations");

    let measured =
        measured_ionosphere_from_obs_epochs(&observations.epochs, SignalBand::L1, SignalBand::L2);
    let valid_phase = measured
        .iter()
        .filter(|observation| observation.phase_status == "ok")
        .collect::<Vec<_>>();

    assert!(
        valid_phase.len() > 50,
        "expected many valid L1/L2 measured ionosphere phase observations, got {}",
        valid_phase.len()
    );

    for observation in valid_phase {
        let leveled_phase_geometry_free_m = observation
            .leveled_phase_geometry_free_m
            .expect("leveled geometry-free phase");
        let phase_delay_band_1_m = observation.phase_delay_band_1_m.expect("band 1 phase delay");
        let phase_delay_band_2_m = observation.phase_delay_band_2_m.expect("band 2 phase delay");
        let f1_2 = observation.f1_hz * observation.f1_hz;
        let f2_2 = observation.f2_hz * observation.f2_hz;
        let denom = f1_2 - f2_2;

        approx_eq(
            phase_delay_band_2_m - phase_delay_band_1_m,
            leveled_phase_geometry_free_m,
            1.0e-12,
        );
        approx_eq(
            phase_delay_band_1_m,
            leveled_phase_geometry_free_m * (f2_2 / denom),
            1.0e-12,
        );
        approx_eq(
            phase_delay_band_2_m,
            leveled_phase_geometry_free_m * (f1_2 / denom),
            1.0e-12,
        );
    }
}
