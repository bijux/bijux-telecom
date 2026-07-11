#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::Seconds;
use bijux_gnss_nav::api::PositionSolver;
use bijux_gnss_nav::PositionSolveRefusalKind;
use support::position_truth::{
    four_satellite_position_scenario, sample_ephemeris, timed_position_observation_from_truth,
};

#[test]
fn position_solver_refuses_when_fewer_than_four_observations_remain() {
    let scenario = four_satellite_position_scenario(0.0);
    let observations = scenario.observations[..3].to_vec();

    let refusal = PositionSolver::new()
        .try_solve_wls(&observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect_err("three observations must be refused");

    assert_eq!(refusal.kind, PositionSolveRefusalKind::InsufficientObservations);
    assert_eq!(refusal.sat_count, 3);
    assert_eq!(refusal.used_sat_count, 3);
    assert_eq!(refusal.rejected_sat_count(), 0);
}

#[test]
fn position_solver_refuses_when_outlier_rejection_leaves_three_usable_satellites() {
    let mut scenario = four_satellite_position_scenario(0.0);
    let extra_ephemeris = sample_ephemeris(5, 3.2, 3.6);
    scenario.observations.push(timed_position_observation_from_truth(
        &extra_ephemeris,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        scenario.receiver_clock_bias_s,
    ));
    scenario.ephemerides.push(extra_ephemeris);
    let mut observations = scenario.observations.clone();
    observations[0].pseudorange_m += 1_000.0;
    if let Some(signal_timing) = &mut observations[0].signal_timing {
        let delay_s = 1_000.0 / 299_792_458.0;
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 + delay_s);
        signal_timing.transmit_gps_time = signal_timing.transmit_gps_time.offset_seconds(-delay_s);
    }
    observations[1].pseudorange_m -= 1_000.0;
    if let Some(signal_timing) = &mut observations[1].signal_timing {
        let delay_s = 1_000.0 / 299_792_458.0;
        signal_timing.signal_travel_time_s =
            Seconds(signal_timing.signal_travel_time_s.0 - delay_s);
        signal_timing.transmit_gps_time = signal_timing.transmit_gps_time.offset_seconds(delay_s);
    }

    let refusal = PositionSolver::new()
        .try_solve_wls(&observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect_err("outlier rejection should refuse a three-satellite remainder");

    assert_eq!(refusal.kind, PositionSolveRefusalKind::InsufficientUsableSatellites);
    assert_eq!(refusal.sat_count, 5);
    assert!(refusal.used_sat_count < 4);
}
