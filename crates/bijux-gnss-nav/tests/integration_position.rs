#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    ephemerides_from_decoded_gps_l1ca_lnav, geodetic_to_ecef, parse_rinex_nav, sat_state_gps_l1ca,
    write_rinex_nav, Ephemeris, GpsEphemeris, GpsL1CaHowWord, GpsL1CaLnavDecodedSubframe,
    GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit,
    GpsL1CaLnavSubframeAlignment, GpsL1CaTlmWord, GpsL1CaWordParitySummary, PositionObservation,
    PositionSolver,
};
use support::position_truth::{
    sample_ephemerides, sample_ephemeris, timed_position_observation,
};

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
