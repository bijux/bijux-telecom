#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::{Constellation, GpsTime, SatId, Seconds};
use bijux_gnss_nav::api::{
    ephemerides_from_decoded_gps_l1ca_lnav, geodetic_to_ecef, parse_rinex_nav, sat_state_gps_l1ca,
    write_rinex_nav, Ephemeris, GpsEphemeris, GpsL1CaHowWord, GpsL1CaLnavDecodedSubframe,
    GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit,
    GpsL1CaLnavSubframeAlignment, GpsL1CaTlmWord, GpsL1CaWordParitySummary, PositionObservation,
    PositionSolver,
};
use support::position_truth::{
    add_klobuchar_delay_to_observations, clear_broadcast_clock_parameters,
    four_satellite_position_scenario,
    four_satellite_position_scenario_with_ephemerides, iterative_pseudorange_residual_m,
    iterative_pseudorange_residual_without_earth_rotation_m,
    sample_ephemerides, sample_ephemerides_with_clock_parameters, sample_ephemeris,
    sample_klobuchar_coefficients,
    timed_position_observation, BroadcastClockParameters, SyntheticPositionScenario,
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

fn broadcast_clock_position_scenario(
    receiver_clock_bias_s: f64,
) -> SyntheticPositionScenario {
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
fn position_solver_refuses_observations_without_signal_timing() {
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

    assert!(PositionSolver::new().solve_wls(&observations, &ephs, t_rx_s).is_none());
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
    write_rinex_nav(&corrected_path, &scenario.ephemerides, true).expect("write corrected rinex nav");
    write_rinex_nav(&zero_clock_path, &clear_broadcast_clock_parameters(&scenario.ephemerides), true)
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
        let ephemeris = parsed
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("parsed ephemeris");
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
        let ephemeris = parsed
            .iter()
            .find(|ephemeris| ephemeris.sat == *sat)
            .expect("decoded ephemeris");
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
