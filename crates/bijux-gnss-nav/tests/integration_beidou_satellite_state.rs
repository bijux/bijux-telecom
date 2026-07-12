mod support;

use bijux_gnss_core::api::{GpsTime, ObsSignalTiming, Seconds};
use bijux_gnss_nav::api::{
    beidou_navigation_age, beidou_satellite_clock_correction_b1i, sat_state_beidou_b1i,
    sat_state_beidou_b1i_from_observation, select_best_beidou_navigation,
};
use support::beidou_d1_fixture::sample_beidou_navigation;

#[test]
fn decoded_beidou_navigation_keeps_clock_and_observation_paths_consistent() {
    let navigation = sample_beidou_navigation();
    let signal_timing = ObsSignalTiming {
        signal_travel_time_s: Seconds(0.081_4),
        transmit_gps_time: GpsTime { week: navigation.bdt.week as u32, tow_s: 65_432.123 },
    };

    let direct_state = sat_state_beidou_b1i(
        &navigation,
        signal_timing.transmit_gps_time.tow_s,
        signal_timing.signal_travel_time_s.0,
    );
    let observation_state = sat_state_beidou_b1i_from_observation(
        &navigation,
        65_432.205,
        24_000_000.0,
        Some(signal_timing),
    );
    let clock =
        beidou_satellite_clock_correction_b1i(&navigation, signal_timing.transmit_gps_time.tow_s);

    assert!((direct_state.x_m - observation_state.x_m).abs() < 1.0e-9);
    assert!((direct_state.y_m - observation_state.y_m).abs() < 1.0e-9);
    assert!((direct_state.z_m - observation_state.z_m).abs() < 1.0e-9);
    assert!((direct_state.clock_correction.bias_s - clock.bias_s).abs() < 1.0e-18);
}

#[test]
fn decoded_beidou_navigation_selection_prefers_fresh_navigation() {
    let fresh = sample_beidou_navigation();
    let mut stale = sample_beidou_navigation();
    stale.clock.toc_s -= 18_000.0;
    stale.ephemeris.toe_s -= 18_000.0;

    let age = beidou_navigation_age(&fresh, 65_000.0);
    let candidates = [stale, fresh.clone()];
    let selected =
        select_best_beidou_navigation(&candidates, fresh.sat, 65_000.0).expect("selected nav");

    assert!(age.is_valid());
    assert_eq!(selected.clock.toc_s, fresh.clock.toc_s);
    assert_eq!(selected.ephemeris.toe_s, fresh.ephemeris.toe_s);
}
