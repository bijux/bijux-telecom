#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::{Constellation, GpsTime, SatId, Seconds};
use bijux_gnss_nav::api::{
    ephemerides_from_decoded_gps_l1ca_lnav, geodetic_to_ecef, parse_rinex_nav,
    position_broadcast_navigation_from_glonass_frames,
    position_broadcast_navigation_from_gps_ephemerides, sat_state_galileo_e1,
    sat_state_glonass_l1, sat_state_gps_l1ca, write_rinex_nav, Ephemeris,
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime, GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame,
    GlonassFrameTime, GlonassImmediateHealth, GlonassImmediateNavigationData,
    GlonassSatelliteType, GlonassStateVector, GlonassSystemTime, GpsEphemeris, GpsL1CaHowWord,
    GpsL1CaLnavDecodedSubframe, GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit,
    GpsL1CaLnavSubframe3Orbit, GpsL1CaLnavSubframeAlignment, GpsL1CaTlmWord,
    GpsL1CaWordParitySummary, PositionBroadcastNavigation, PositionObservation, PositionSolver,
};
use support::position_truth::{
    add_klobuchar_delay_to_observations, add_saastamoinen_delay_to_observations,
    clear_broadcast_clock_parameters, four_satellite_position_scenario,
    four_satellite_position_scenario_with_ephemerides, iterative_pseudorange_residual_m,
    iterative_pseudorange_residual_without_earth_rotation_m, sample_ephemerides,
    sample_ephemerides_with_clock_parameters, sample_ephemeris, sample_klobuchar_coefficients,
    timed_position_observation, timed_position_observation_from_truth, BroadcastClockParameters,
    SyntheticPositionScenario,
};

fn broadcast_clock_fixture_parameters() -> [(u8, BroadcastClockParameters); 4] {
    [
        (
            1,
            BroadcastClockParameters {
                af0_s: 240.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: -8.0e-9,
            },
        ),
        (
            2,
            BroadcastClockParameters {
                af0_s: -170.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: 6.0e-9,
            },
        ),
        (
            3,
            BroadcastClockParameters {
                af0_s: 330.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: -4.0e-9,
            },
        ),
        (
            4,
            BroadcastClockParameters {
                af0_s: -95.0e-9,
                af1_s_per_s: 0.0,
                af2_s_per_s2: 0.0,
                tgd_s: 10.0e-9,
            },
        ),
    ]
}

fn broadcast_clock_position_scenario(receiver_clock_bias_s: f64) -> SyntheticPositionScenario {
    four_satellite_position_scenario_with_ephemerides(
        receiver_clock_bias_s,
        sample_ephemerides_with_clock_parameters(&broadcast_clock_fixture_parameters()),
    )
}

fn position_error_3d_m(
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = ecef_x_m - truth_ecef_m.0;
    let dy = ecef_y_m - truth_ecef_m.1;
    let dz = ecef_z_m - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn sample_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn },
        iodnav: prn as u16,
        gst: GalileoSystemTime { week: 2222, tow_s: 504_018 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: 504_018.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            toe_s: 504_000.0,
            sqrt_a: 5_440.612_319,
            e: 0.001_23,
            i0: 0.953,
            idot: -2.1e-10,
            omega0,
            omegadot: -5.8e-9,
            w: -0.37,
            m0,
            delta_n: 4.7e-9,
            cuc: -3.2e-6,
            cus: 4.1e-6,
            crc: 178.0,
            crs: -91.0,
            cic: 1.9e-7,
            cis: -2.4e-7,
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 0.0,
            ai1: 0.0,
            ai2: 0.0,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    }
}

fn galileo_pseudorange_from_truth(
    navigation: &GalileoBroadcastNavigationData,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    galileo_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + galileo_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn sample_glonass_navigation(
    prn: u8,
    state_vector: GlonassStateVector,
    clock_bias_s: f64,
    l2_l1_delay_s: f64,
) -> GlonassBroadcastNavigationFrame {
    let sat = SatId { constellation: Constellation::Glonass, prn };
    GlonassBroadcastNavigationFrame {
        sat,
        immediate: GlonassImmediateNavigationData {
            sat,
            frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
            ephemeris_reference_time_s: 83_700,
            tb_update_interval_min: 30,
            tb_is_odd: Some(true),
            state_vector,
            relative_frequency_bias: 0.0,
            clock_bias_s,
            l2_l1_delay_s: Some(l2_l1_delay_s),
            health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
            immediate_data_age_days: 28,
            satellite_type: GlonassSatelliteType::GlonassM,
            reported_slot: None,
            system_time: Some(GlonassSystemTime { day_number: 864, four_year_interval: Some(8) }),
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

fn sample_glonass_navigation_slot14() -> GlonassBroadcastNavigationFrame {
    sample_glonass_navigation(
        14,
        GlonassStateVector {
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
        -2.572_406_083_345_413_2e-5,
        5.587_935_448e-9,
    )
}

fn sample_glonass_navigation_slot8() -> GlonassBroadcastNavigationFrame {
    sample_glonass_navigation(
        8,
        GlonassStateVector {
            x_m: 9_639_804.687_5,
            y_m: 5_433_780.761_718_75,
            z_m: 23_045_452.148_437_5,
            vx_mps: -1_955.772_399_902_343_8,
            vy_mps: 2_454.154_968_261_718_8,
            vz_mps: 238.015_174_865_722_66,
            ax_mps2: 9.313_225_746_154_785e-7,
            ay_mps2: 0.0,
            az_mps2: -2.793_967_723_846_435_5e-6,
        },
        6.508_920_341_730_117_7e-5,
        -3.725_290_298e-9,
    )
}

fn glonass_pseudorange_from_truth(
    navigation: &GlonassBroadcastNavigationFrame,
    truth_ecef_m: (f64, f64, f64),
    t_rx_s: f64,
    receiver_clock_bias_s: f64,
    glonass_bias_s: f64,
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_glonass_l1(navigation, t_rx_s - tau, tau)
            .expect("GLONASS broadcast state for pseudorange generation");
        let dx = truth_ecef_m.0 - state.x_m;
        let dy = truth_ecef_m.1 - state.y_m;
        let dz = truth_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m + (receiver_clock_bias_s + glonass_bias_s) * 299_792_458.0
            - state.clock_correction.bias_s * 299_792_458.0;
        let next_tau = pseudorange_m / 299_792_458.0;
        if (next_tau - tau).abs() < 1.0e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

#[test]
fn position_solver_returns_solution() {
    let solver = PositionSolver::new();
    let solution = solver.solve_wls(&[], &[], 0.0);
    assert!(solution.is_none());
}

#[test]
fn ephemeris_is_constructible() {
    let eph = Ephemeris { sat: SatId { constellation: Constellation::Gps, prn: 1 }, toe_s: 0.0 };
    assert_eq!(eph.sat.prn, 1);
}

#[test]
fn position_observation_constructible() {
    let obs = PositionObservation {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        pseudorange_m: 20_000_000.0,
        cn0_dbhz: 40.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
    };
    assert_eq!(obs.sat.prn, 3);
}

#[test]
fn position_solver_solves_observations_without_signal_timing_when_pseudoranges_are_finite() {
    let t_rx_s = 504_018.07;
    let ephs = sample_ephemerides();
    let observations = ephs
        .iter()
        .map(|eph| PositionObservation {
            sat: eph.sat,
            pseudorange_m: 21_000_000.0,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
            signal_timing: None,
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &ephs, t_rx_s)
        .expect("finite pseudoranges without signal timing should still solve");

    assert!(solution.ecef_x_m.is_finite());
    assert!(solution.ecef_y_m.is_finite());
    assert!(solution.ecef_z_m.is_finite());
}

#[test]
fn position_solver_refuses_inconsistent_signal_timing() {
    let t_rx_s = 504_018.07;
    let ephs = sample_ephemerides();
    let observations = ephs
        .iter()
        .map(|eph| {
            let mut obs = timed_position_observation(eph.sat, 21_000_000.0, t_rx_s);
            obs.signal_timing = obs.signal_timing.map(|mut timing| {
                timing.signal_travel_time_s = Seconds(timing.signal_travel_time_s.0 + 0.01);
                timing
            });
            obs
        })
        .collect::<Vec<_>>();

    assert!(PositionSolver::new().solve_wls(&observations, &ephs, t_rx_s).is_none());
}

#[test]
fn single_point_solver_recovers_four_satellite_fix() {
    let scenario = four_satellite_position_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("timed four-satellite observations should solve");

    assert_eq!(solution.sat_count, 4);
    assert_eq!(solution.used_sat_count, 4);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!((solution.ecef_x_m - scenario.truth_ecef_m.0).abs() < 5.0);
    assert!((solution.ecef_y_m - scenario.truth_ecef_m.1).abs() < 5.0);
    assert!((solution.ecef_z_m - scenario.truth_ecef_m.2).abs() < 5.0);
    assert!(solution.clock_bias_s.abs() < 1.0e-9);
    assert!(solution.hdop.is_some());
    assert!(solution.vdop.is_some());
    assert!(solution.gdop.is_some());
    assert!(solution.tdop.is_some());
    assert!(
        (solution.pdop.powi(2)
            - (solution.hdop.expect("hdop").powi(2) + solution.vdop.expect("vdop").powi(2)))
        .abs()
            < 1.0e-9
    );
    assert!(
        (solution.gdop.expect("gdop").powi(2)
            - (solution.pdop.powi(2) + solution.tdop.expect("tdop").powi(2)))
        .abs()
            < 1.0e-9
    );
}

#[test]
fn single_point_solver_recovers_receiver_clock_bias() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("biased timed pseudoranges should solve");

    assert_eq!(solution.used_sat_count, 4);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!((solution.ecef_x_m - scenario.truth_ecef_m.0).abs() < 5.0);
    assert!((solution.ecef_y_m - scenario.truth_ecef_m.1).abs() < 5.0);
    assert!((solution.ecef_z_m - scenario.truth_ecef_m.2).abs() < 5.0);
    assert!((solution.clock_bias_s - scenario.receiver_clock_bias_s).abs() < 1.0e-9);
}

#[test]
fn mixed_gps_galileo_solver_recovers_position_and_clock_split() {
    let gps_ephemerides = sample_ephemerides();
    let galileo_navigation =
        vec![sample_galileo_navigation(19, 1.17, 0.84), sample_galileo_navigation(24, -0.83, 1.52)];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let galileo_bias_s = -1.15e-6;
    let t_rx_s = 504_018.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(galileo_navigation.iter().map(|navigation| {
        let pseudorange_m = galileo_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            galileo_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+galileo observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-9);
    assert_eq!(solution.inter_system_biases.len(), 1);
    assert_eq!(solution.inter_system_biases[0].constellation, Constellation::Galileo);
    assert!((solution.inter_system_biases[0].bias_s.0 - galileo_bias_s).abs() < 1.0e-9);
}

#[test]
fn mixed_gps_glonass_solver_recovers_position_and_clock_split() {
    let gps_ephemerides = sample_ephemerides();
    let glonass_navigation =
        vec![sample_glonass_navigation_slot14(), sample_glonass_navigation_slot8()];
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let glonass_bias_s = 8.5e-7;
    let t_rx_s = 504_918.07 + receiver_clock_bias_s;

    let mut observations = gps_ephemerides
        .iter()
        .map(|ephemeris| {
            timed_position_observation_from_truth(
                ephemeris,
                truth_ecef_m,
                t_rx_s,
                receiver_clock_bias_s,
            )
        })
        .collect::<Vec<_>>();
    observations.extend(glonass_navigation.iter().map(|navigation| {
        let pseudorange_m = glonass_pseudorange_from_truth(
            navigation,
            truth_ecef_m,
            t_rx_s,
            receiver_clock_bias_s,
            glonass_bias_s,
        );
        timed_position_observation(navigation.sat, pseudorange_m, t_rx_s)
    }));

    let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
    navigation.extend(position_broadcast_navigation_from_glonass_frames(&glonass_navigation));

    let solution = PositionSolver::new()
        .solve_wls_with_navigation_data(&observations, &navigation, t_rx_s)
        .expect("mixed gps+glonass observations should solve");

    assert_eq!(solution.clock_reference_constellation, Constellation::Gps);
    assert_eq!(solution.used_sat_count, 6);
    assert_eq!(solution.rejected_sat_count, 0);
    assert!(solution.rejected.is_empty(), "unexpected rejections: {:?}", solution.rejected);
    assert!(
        position_error_3d_m(solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m, truth_ecef_m,)
            < 5.0
    );
    assert!((solution.clock_bias_s - receiver_clock_bias_s).abs() < 1.0e-8);
    assert_eq!(solution.inter_system_biases.len(), 1);
    assert_eq!(solution.inter_system_biases[0].constellation, Constellation::Glonass);
    assert!((solution.inter_system_biases[0].bias_s.0 - glonass_bias_s).abs() < 1.0e-8);
}

#[test]
fn single_point_solver_applies_broadcast_ionosphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let klobuchar = sample_klobuchar_coefficients();
    let ionosphere_biased_observations = add_klobuchar_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        klobuchar,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_broadcast_ionosphere(
            &ionosphere_biased_observations,
            &scenario.ephemerides,
            scenario.t_rx_s,
            Some(&klobuchar),
        )
        .expect("ionosphere-corrected observations should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&ionosphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 3.0);
}

#[test]
fn single_point_solver_applies_saastamoinen_troposphere_correction() {
    let scenario = four_satellite_position_scenario(0.0);
    let troposphere_biased_observations = add_saastamoinen_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
    );

    let corrected_solution = PositionSolver { apply_troposphere: true, ..PositionSolver::new() }
        .solve_wls(&troposphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("troposphere-corrected observations should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&troposphere_biased_observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        uncorrected_solution.ecef_x_m,
        uncorrected_solution.ecef_y_m,
        uncorrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 2.0);
}

#[test]
fn single_point_solver_refreshes_unbiased_geometry_after_state_update() {
    let scenario = four_satellite_position_scenario(0.0);
    let solver = PositionSolver {
        max_iterations: 1,
        residual_gate_m: 1.0e9,
        chi_square_gate: 1.0e18,
        robust: false,
        raim: false,
        ..PositionSolver::new()
    };
    let solution = solver
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("single-iteration geometry should still produce a solution");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let expected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        assert!((residual_m - expected_residual_m).abs() < 1.0e-6);
    }
}

#[test]
fn single_point_solver_refreshes_biased_geometry_after_state_update() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solver = PositionSolver {
        max_iterations: 1,
        residual_gate_m: 1.0e9,
        chi_square_gate: 1.0e18,
        robust: false,
        raim: false,
        ..PositionSolver::new()
    };
    let solution = solver
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("single-iteration biased geometry should still produce a solution");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let expected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        assert!((residual_m - expected_residual_m).abs() < 1.0e-6);
    }
}

#[test]
fn single_point_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("timed four-satellite observations should solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris = scenario
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("scenario ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}

#[test]
fn single_point_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let zero_clock_ephemerides = clear_broadcast_clock_parameters(&scenario.ephemerides);

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("broadcast clock corrected position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock_ephemerides, scenario.t_rx_s)
        .expect("zero-clock comparison position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
    assert!((corrected_solution.clock_bias_s - scenario.receiver_clock_bias_s).abs() < 1.0e-9);
}

fn decoded_lnav_subframes_from_ephemeris(eph: &GpsEphemeris) -> Vec<GpsL1CaLnavDecodedSubframe> {
    let alignment = |subframe_index: usize| GpsL1CaLnavSubframeAlignment {
        start_bit_index: subframe_index * 300,
        end_bit_index_exclusive: (subframe_index + 1) * 300,
        start_prompt_index: subframe_index * 6_000,
        end_prompt_index_exclusive: (subframe_index + 1) * 6_000,
        inverted: false,
        word_count: 10,
        parity_ok_count: 10,
    };
    let tlm = GpsL1CaTlmWord { preamble: 0x8B, parity_ok: true };
    let parity = GpsL1CaWordParitySummary {
        word_count: 10,
        passed_word_count: 10,
        failed_word_indexes: Vec::new(),
    };
    let word_parity_ok = vec![true; 10];

    vec![
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(0),
            tlm: tlm.clone(),
            how: GpsL1CaHowWord {
                tow_count: (eph.toc_s / 6.0) as u32,
                tow_start_s: (eph.toc_s / 6.0) as u32 * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 1,
                parity_ok: true,
            },
            clock: Some(GpsL1CaLnavSubframe1Clock {
                week: (eph.week % 1024) as u16,
                iodc: eph.iodc,
                sv_health: eph.sv_health,
                toc_s: eph.toc_s,
                af0: eph.af0,
                af1: eph.af1,
                af2: eph.af2,
                tgd: eph.tgd,
            }),
            orbit_subframe_2: None,
            orbit_subframe_3: None,
            parity: parity.clone(),
            word_parity_ok: word_parity_ok.clone(),
        },
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(1),
            tlm: tlm.clone(),
            how: GpsL1CaHowWord {
                tow_count: (eph.toe_s / 6.0) as u32,
                tow_start_s: (eph.toe_s / 6.0) as u32 * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 2,
                parity_ok: true,
            },
            clock: None,
            orbit_subframe_2: Some(GpsL1CaLnavSubframe2Orbit {
                iode: eph.iode,
                crs: eph.crs,
                delta_n: eph.delta_n,
                m0: eph.m0,
                cuc: eph.cuc,
                e: eph.e,
                cus: eph.cus,
                sqrt_a: eph.sqrt_a,
                toe_s: eph.toe_s,
            }),
            orbit_subframe_3: None,
            parity: parity.clone(),
            word_parity_ok: word_parity_ok.clone(),
        },
        GpsL1CaLnavDecodedSubframe {
            alignment: alignment(2),
            tlm,
            how: GpsL1CaHowWord {
                tow_count: (eph.toe_s / 6.0) as u32 + 1,
                tow_start_s: ((eph.toe_s / 6.0) as u32 + 1) * 6,
                alert: false,
                anti_spoof: false,
                subframe_id: 3,
                parity_ok: true,
            },
            clock: None,
            orbit_subframe_2: None,
            orbit_subframe_3: Some(GpsL1CaLnavSubframe3Orbit {
                iode: eph.iode,
                cic: eph.cic,
                omega0: eph.omega0,
                cis: eph.cis,
                i0: eph.i0,
                crc: eph.crc,
                w: eph.w,
                omegadot: eph.omegadot,
                idot: eph.idot,
            }),
            parity,
            word_parity_ok,
        },
    ]
}

#[test]
fn rinex_nav_ephemeris_feeds_position_solver() {
    let source = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ];
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-position-{}-{}.rnx",
        std::process::id(),
        source.len()
    ));
    write_rinex_nav(&path, &source, true).expect("write rinex nav");
    let data = std::fs::read_to_string(&path).expect("read rinex nav");
    let parsed = parse_rinex_nav(&data).expect("parse rinex nav");
    std::fs::remove_file(&path).expect("remove rinex nav");

    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let observations = parsed
        .iter()
        .map(|eph| {
            let mut tau = 0.07;
            let mut pseudorange_m = 0.0;
            for _ in 0..10 {
                let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                let dx = rx_x - state.x_m;
                let dy = rx_y - state.y_m;
                let dz = rx_z - state.z_m;
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                pseudorange_m = range - state.clock_correction.bias_s * 299_792_458.0;
                let next_tau = pseudorange_m / 299_792_458.0;
                if (next_tau - tau).abs() < 1.0e-12 {
                    break;
                }
                tau = next_tau;
            }
            timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &parsed, t_rx_s)
        .expect("parsed rinex nav should support a position solve");

    assert!((solution.ecef_x_m - rx_x).abs() < 5.0);
    assert!((solution.ecef_y_m - rx_y).abs() < 5.0);
    assert!((solution.ecef_z_m - rx_z).abs() < 5.0);
}

#[test]
fn rinex_nav_position_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let corrected_path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-clock-corrected-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    let zero_clock_path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-zero-clock-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    write_rinex_nav(&corrected_path, &scenario.ephemerides, true)
        .expect("write corrected rinex nav");
    write_rinex_nav(
        &zero_clock_path,
        &clear_broadcast_clock_parameters(&scenario.ephemerides),
        true,
    )
    .expect("write zero-clock rinex nav");
    let corrected = parse_rinex_nav(
        &std::fs::read_to_string(&corrected_path).expect("read corrected rinex nav"),
    )
    .expect("parse corrected rinex nav");
    let zero_clock = parse_rinex_nav(
        &std::fs::read_to_string(&zero_clock_path).expect("read zero-clock rinex nav"),
    )
    .expect("parse zero-clock rinex nav");
    std::fs::remove_file(&corrected_path).expect("remove corrected rinex nav");
    std::fs::remove_file(&zero_clock_path).expect("remove zero-clock rinex nav");

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &corrected, scenario.t_rx_s)
        .expect("corrected rinex position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock, scenario.t_rx_s)
        .expect("zero-clock rinex position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
}

#[test]
fn rinex_nav_position_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-earth-rotation-{}-{}.rnx",
        std::process::id(),
        scenario.ephemerides.len()
    ));
    write_rinex_nav(&path, &scenario.ephemerides, true).expect("write rinex nav");
    let parsed = parse_rinex_nav(&std::fs::read_to_string(&path).expect("read rinex nav"))
        .expect("parse rinex nav");
    std::fs::remove_file(&path).expect("remove rinex nav");

    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &parsed, scenario.t_rx_s)
        .expect("parsed rinex nav should support a position solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris =
            parsed.iter().find(|ephemeris| ephemeris.sat == *sat).expect("parsed ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}

#[test]
fn decoded_lnav_ephemeris_feeds_position_solver() {
    let source = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
    ];
    let parsed = source
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded ephemeris")
        })
        .collect::<Vec<_>>();

    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 504_018.07;
    let observations = parsed
        .iter()
        .map(|eph| {
            let mut tau = 0.07;
            let mut pseudorange_m = 0.0;
            for _ in 0..10 {
                let state = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
                let dx = rx_x - state.x_m;
                let dy = rx_y - state.y_m;
                let dz = rx_z - state.z_m;
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                pseudorange_m = range - state.clock_correction.bias_s * 299_792_458.0;
                let next_tau = pseudorange_m / 299_792_458.0;
                if (next_tau - tau).abs() < 1.0e-12 {
                    break;
                }
                tau = next_tau;
            }
            timed_position_observation(eph.sat, pseudorange_m, t_rx_s)
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&observations, &parsed, t_rx_s)
        .expect("decoded LNAV ephemerides should support a position solve");

    assert!((solution.ecef_x_m - rx_x).abs() < 5.0);
    assert!((solution.ecef_y_m - rx_y).abs() < 5.0);
    assert!((solution.ecef_z_m - rx_z).abs() < 5.0);
}

#[test]
fn decoded_lnav_position_solver_uses_broadcast_satellite_clock_correction() {
    let scenario = broadcast_clock_position_scenario(2.75e-4);
    let zero_clock_ephemerides = clear_broadcast_clock_parameters(&scenario.ephemerides);
    let corrected = scenario
        .ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded corrected ephemeris")
        })
        .collect::<Vec<_>>();
    let zero_clock = zero_clock_ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded zero-clock ephemeris")
        })
        .collect::<Vec<_>>();

    let corrected_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &corrected, scenario.t_rx_s)
        .expect("corrected decoded LNAV position should solve");
    let zero_clock_solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &zero_clock, scenario.t_rx_s)
        .expect("zero-clock decoded LNAV position should still solve");

    let corrected_error_m = position_error_3d_m(
        corrected_solution.ecef_x_m,
        corrected_solution.ecef_y_m,
        corrected_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );
    let zero_clock_error_m = position_error_3d_m(
        zero_clock_solution.ecef_x_m,
        zero_clock_solution.ecef_y_m,
        zero_clock_solution.ecef_z_m,
        scenario.truth_ecef_m,
    );

    assert!(corrected_error_m < 5.0);
    assert!(zero_clock_error_m > corrected_error_m + 20.0);
}

#[test]
fn decoded_lnav_position_solver_residuals_include_earth_rotation_correction() {
    let scenario = four_satellite_position_scenario(2.75e-4);
    let parsed = scenario
        .ephemerides
        .iter()
        .map(|eph| {
            let (ephemerides, rejections) = ephemerides_from_decoded_gps_l1ca_lnav(
                eph.sat.prn,
                &decoded_lnav_subframes_from_ephemeris(eph),
                Some(eph.week),
            );
            assert!(rejections.is_empty(), "rejections={rejections:?}");
            assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
            ephemerides.into_iter().next().expect("decoded ephemeris")
        })
        .collect::<Vec<_>>();

    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &parsed, scenario.t_rx_s)
        .expect("decoded LNAV ephemerides should support a position solve");

    assert_eq!(solution.used_sat_count, 4);
    for (sat, residual_m, _weight) in &solution.residuals {
        let observation = scenario
            .observations
            .iter()
            .find(|observation| observation.sat == *sat)
            .expect("scenario observation");
        let ephemeris =
            parsed.iter().find(|ephemeris| ephemeris.sat == *sat).expect("decoded ephemeris");
        let corrected_residual_m = iterative_pseudorange_residual_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );
        let uncorrected_residual_m = iterative_pseudorange_residual_without_earth_rotation_m(
            ephemeris,
            observation,
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            solution.clock_bias_s,
            scenario.t_rx_s,
        );

        assert!((residual_m - corrected_residual_m).abs() < 1.0e-6);
        assert!(uncorrected_residual_m.abs() > residual_m.abs() + 1.0);
    }
}
