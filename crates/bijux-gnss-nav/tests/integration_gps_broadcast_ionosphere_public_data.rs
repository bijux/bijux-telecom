#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_nav::api::BroadcastIonosphereResidualObservation;
use bijux_gnss_nav::api::{
    gps_broadcast_ionosphere_residuals_from_obs_epochs, parse_rinex_broadcast_navigation,
    parse_rinex_gps_observation_dataset,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

fn valid_broadcast_delays(
    observations: &[BroadcastIonosphereResidualObservation],
) -> Vec<(f64, f64)> {
    observations
        .iter()
        .filter(|observation| observation.broadcast_status == "ok")
        .filter_map(|observation| {
            Some((observation.broadcast_delay_band_1_m?, observation.broadcast_delay_band_2_m?))
        })
        .collect()
}

#[test]
fn public_gps_nav_payload_emits_physical_l1_and_l2_broadcast_delays() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public dual-frequency observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public broadcast navigation");
    let receiver_ecef = observations
        .approx_position_ecef_m
        .expect("public RINEX header should provide approximate receiver coordinates");

    let residuals = gps_broadcast_ionosphere_residuals_from_obs_epochs(
        &observations.epochs,
        receiver_ecef,
        &navigation,
        SignalBand::L1,
        SignalBand::L2,
    );
    let valid_delays = valid_broadcast_delays(&residuals);

    assert!(
        valid_delays.len() > 50,
        "expected many valid broadcast delay estimates, got {}",
        valid_delays.len()
    );
    assert!(
        valid_delays.iter().all(|(l1_delay_m, _)| l1_delay_m.is_finite() && *l1_delay_m > 0.0),
        "expected all valid L1 broadcast delays to be finite and positive",
    );
    assert!(
        valid_delays.iter().all(|(l1_delay_m, l2_delay_m)| {
            l2_delay_m.is_finite() && *l2_delay_m > *l1_delay_m
        }),
        "expected all valid L2 broadcast delays to exceed L1 on dispersive GPS paths",
    );
}

#[test]
fn public_gps_nav_payload_keeps_broadcast_delays_in_a_realistic_range() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public dual-frequency observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public broadcast navigation");
    let receiver_ecef = observations
        .approx_position_ecef_m
        .expect("public RINEX header should provide approximate receiver coordinates");

    let residuals = gps_broadcast_ionosphere_residuals_from_obs_epochs(
        &observations.epochs,
        receiver_ecef,
        &navigation,
        SignalBand::L1,
        SignalBand::L2,
    );
    let valid_delays = valid_broadcast_delays(&residuals);
    let mean_l1_delay_m = valid_delays.iter().map(|(l1_delay_m, _)| *l1_delay_m).sum::<f64>()
        / valid_delays.len() as f64;
    let max_l1_delay_m =
        valid_delays.iter().map(|(l1_delay_m, _)| *l1_delay_m).fold(0.0_f64, f64::max);

    assert!(mean_l1_delay_m > 1.0, "expected nontrivial mean L1 delay, got {mean_l1_delay_m}");
    assert!(mean_l1_delay_m < 20.0, "unexpected mean L1 delay, got {mean_l1_delay_m}");
    assert!(max_l1_delay_m > mean_l1_delay_m);
    assert!(max_l1_delay_m < 40.0, "unexpected L1 slant delay envelope, got {max_l1_delay_m}");
}
