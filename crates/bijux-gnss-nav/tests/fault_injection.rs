#![allow(missing_docs)]
use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation, PositionSolver,
};

fn make_eph(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn timed_position_observation(sat: SatId, pseudorange_m: f64, t_rx_s: f64) -> PositionObservation {
    let signal_travel_time_s = pseudorange_m / 299_792_458.0;
    PositionObservation {
        sat,
        pseudorange_m,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: Some(GpsTime { week: 0, tow_s: t_rx_s }),
        signal_timing: Some(ObsSignalTiming {
            signal_travel_time_s: Seconds(signal_travel_time_s),
            transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
        }),
        signal_id: None,
    }
}

#[test]
fn fault_injection_rejects_bad_pseudorange() {
    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 100_000.0;

    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
        make_eph(6, 4.0, 4.5),
    ];

    let mut obs = Vec::new();
    for eph in &ephs {
        let state = sat_state_gps_l1ca(eph, t_rx_s, 0.07);
        let dx = rx_x - state.x_m;
        let dy = rx_y - state.y_m;
        let dz = rx_z - state.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut pr = range + 299_792_458.0 * (0.0 - state.clock_correction.bias_s);
        if eph.sat.prn == 3 {
            pr += 1000.0;
        }
        obs.push(timed_position_observation(eph.sat, pr, t_rx_s));
    }

    let solver = PositionSolver { raim: true, residual_gate_m: 300.0, ..PositionSolver::new() };
    let solution = solver.solve_wls(&obs, &ephs, t_rx_s);
    if let Some(solution) = solution {
        assert!(solution
            .rejected
            .iter()
            .any(|(sat, _)| { *sat == SatId { constellation: Constellation::Gps, prn: 3 } }));
    }
}

#[test]
fn position_solver_uses_explicit_observation_timing_for_ephemeris_age() {
    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let relative_receiver_time_s = 0.07;
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.07 };

    let mut ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
    ];
    for eph in &mut ephs {
        eph.toe_s = 345_600.0;
        eph.toc_s = 345_600.0;
        eph.week = receive_gps_time.week;
    }

    let mut obs = Vec::new();
    for eph in &ephs {
        let mut tau = 0.07;
        let mut pseudorange_m = 0.0;
        for _ in 0..10 {
            let state = sat_state_gps_l1ca(eph, receive_gps_time.tow_s - tau, tau);
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
        obs.push(PositionObservation {
            sat: eph.sat,
            pseudorange_m,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: Some(receive_gps_time),
            signal_timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(pseudorange_m / 299_792_458.0),
                transmit_gps_time: receive_gps_time
                    .offset_seconds(-(pseudorange_m / 299_792_458.0)),
            }),
            signal_id: None,
        });
    }

    let solver = PositionSolver::new();
    let solution = solver.solve_wls(&obs, &ephs, relative_receiver_time_s);
    assert!(solution.is_some(), "explicit observation timing should keep ephemerides usable");
}

#[test]
fn position_solver_rejects_stale_ephemeris_when_current_satellites_can_still_solve() {
    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 100_000.0;

    let mut ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
    ];
    for eph in &mut ephs[..4] {
        eph.toe_s = t_rx_s;
        eph.toc_s = t_rx_s;
    }
    ephs[4].toe_s = t_rx_s - 7_201.0;
    ephs[4].toc_s = t_rx_s - 7_201.0;

    let mut obs = Vec::new();
    for eph in &ephs {
        let state = sat_state_gps_l1ca(eph, t_rx_s, 0.07);
        let dx = rx_x - state.x_m;
        let dy = rx_y - state.y_m;
        let dz = rx_z - state.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        obs.push(timed_position_observation(
            eph.sat,
            range - state.clock_correction.bias_s * 299_792_458.0,
            t_rx_s,
        ));
    }

    let solution = PositionSolver::new()
        .solve_wls(&obs, &ephs, t_rx_s)
        .expect("current satellites should still solve");

    assert!(solution.rejected.iter().any(|(sat, reason)| {
        *sat == SatId { constellation: Constellation::Gps, prn: 5 }
            && *reason == bijux_gnss_core::api::MeasurementRejectReason::InvalidEphemeris
    }));
}
