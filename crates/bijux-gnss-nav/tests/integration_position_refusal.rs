#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::Seconds;
use bijux_gnss_core::api::{Constellation, MeasurementRejectReason, SatId};
use bijux_gnss_nav::api::{
    position_broadcast_navigation_from_gps_ephemerides, GlonassAlmanacTimeData,
    GlonassBroadcastNavigationFrame, GlonassFrameTime, GlonassImmediateHealth,
    GlonassImmediateNavigationData, GlonassSatelliteType, GlonassStateVector, GlonassSystemTime,
    PositionBroadcastNavigation, PositionSolveRefusalKind, PositionSolver,
};
use support::position_truth::{
    four_satellite_position_scenario, sample_ephemeris, timed_position_observation,
    timed_position_observation_from_truth,
};

fn sample_glonass_navigation(sat: SatId) -> GlonassBroadcastNavigationFrame {
    GlonassBroadcastNavigationFrame {
        sat,
        immediate: GlonassImmediateNavigationData {
            sat,
            frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
            ephemeris_reference_time_s: 83_700,
            tb_update_interval_min: 30,
            tb_is_odd: Some(true),
            state_vector: GlonassStateVector {
                x_m: -7_557_760.253_906_25,
                y_m: -23_962_225.585_937_5,
                z_m: -4_337_567.871_093_75,
                vx_mps: 101.318_359_375,
                vy_mps: 602.112_770_080_566_4,
                vz_mps: -3_495.733_261_108_398_4,
                ax_mps2: -3.725_290_298_461_914e-6,
                ay_mps2: 0.0,
                az_mps2: 1.862_645_149_230_957e-6,
            },
            relative_frequency_bias: 0.0,
            clock_bias_s: -2.572_406_083_345_413_2e-5,
            l2_l1_delay_s: Some(5.587_935_448e-9),
            health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
            immediate_data_age_days: 28,
            satellite_type: GlonassSatelliteType::GlonassM,
            reported_slot: None,
            system_time: Some(GlonassSystemTime { day_number: 864, four_year_interval: Some(8) }),
            resolved_day_index: Some(864),
            accuracy_code: Some(2),
        },
        system_time: Some(GlonassAlmanacTimeData {
            system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
            utc_offset_s: 0.0,
            gps_minus_glonass_s: -10_782.0,
        }),
        almanac_entries: Vec::new(),
    }
}

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
fn position_solver_refuses_when_invalid_satellite_time_leaves_three_usable_satellites() {
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
    for index in [0usize, 1usize] {
        if let Some(signal_timing) = &mut observations[index].signal_timing {
            signal_timing.signal_travel_time_s =
                Seconds(signal_timing.signal_travel_time_s.0 + 0.01);
        }
    }

    let refusal = PositionSolver::new()
        .try_solve_wls(&observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect_err("invalid timing should leave too few usable satellites");

    assert_eq!(refusal.kind, PositionSolveRefusalKind::InvalidSatelliteTime);
    assert_eq!(refusal.sat_count, 5);
    assert!(refusal.used_sat_count < 4);
}

#[test]
fn position_solver_refuses_mixed_glonass_without_known_time_offset() {
    let scenario = four_satellite_position_scenario(0.0);
    let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let mut observations = scenario.observations.clone();
    observations.push(timed_position_observation(glonass_sat, 24_000_000.0, scenario.t_rx_s));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&scenario.ephemerides);
    let mut glonass_navigation = sample_glonass_navigation(glonass_sat);
    glonass_navigation.system_time = None;
    navigation.push(PositionBroadcastNavigation::Glonass(glonass_navigation));

    let refusal = PositionSolver::new()
        .try_solve_wls_with_navigation_data(&observations, &navigation, scenario.t_rx_s)
        .expect_err("mixed solve should refuse unknown GLONASS time offset");

    assert_eq!(refusal.kind, PositionSolveRefusalKind::UnknownInterSystemTimeOffset);
    assert_eq!(refusal.sat_count, 5);
    assert_eq!(refusal.used_sat_count, 4);
    assert!(refusal.rejected.contains(&(glonass_sat, MeasurementRejectReason::TimeInconsistency)));
}

#[test]
fn position_solver_keeps_gps_only_solution_when_unresolved_glonass_is_unobserved() {
    let scenario = four_satellite_position_scenario(0.0);
    let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&scenario.ephemerides);
    let mut glonass_navigation = sample_glonass_navigation(glonass_sat);
    glonass_navigation.system_time = None;
    navigation.push(PositionBroadcastNavigation::Glonass(glonass_navigation));

    let solution = PositionSolver::new()
        .try_solve_wls_with_navigation_data(&scenario.observations, &navigation, scenario.t_rx_s)
        .expect("gps-only observations should ignore unresolved unobserved GLONASS navigation");

    assert_eq!(solution.used_sat_count, 4);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
}
