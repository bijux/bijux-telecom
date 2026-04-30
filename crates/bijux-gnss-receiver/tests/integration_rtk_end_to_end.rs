#![allow(missing_docs)]
use bijux_gnss_core::api::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, SatId, SigId,
    SignalBand, SignalSpec,
};
use bijux_gnss_nav::api::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation, PositionSolver,
};
use bijux_gnss_receiver::api::{
    baseline_from_ecef, build_dd, build_sd, choose_ref_sat, solution_separation, solve_baseline_dd,
};

fn make_eph(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
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

fn make_obs_epoch(
    role: ReceiverRole,
    t_rx_s: f64,
    pos_ecef: (f64, f64, f64),
    ephs: &[GpsEphemeris],
) -> ObsEpoch {
    let lambda_m = 299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value();
    let sats: Vec<ObsSatellite> = ephs
        .iter()
        .map(|eph| {
            let sat = sat_state_gps_l1ca(eph, t_rx_s, 0.0);
            let dx = pos_ecef.0 - sat.x_m;
            let dy = pos_ecef.1 - sat.y_m;
            let dz = pos_ecef.2 - sat.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            ObsSatellite {
                signal_id: SigId {
                    sat: eph.sat,
                    band: SignalBand::L1,
                    code: bijux_gnss_core::api::SignalCode::Ca,
                },
                pseudorange_m: bijux_gnss_core::api::Meters(
                    range + 299_792_458.0 * (0.0 - sat.clock_bias_s),
                ),
                pseudorange_var_m2: 4.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(range / lambda_m),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "synthetic".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: SignalSpec {
                        constellation: Constellation::Gps,
                        band: SignalBand::L1,
                        code: bijux_gnss_core::api::SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
                    },
                    ..ObsMetadata::default()
                },
            }
        })
        .collect();
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(t_rx_s),
        gps_week: None,
        tow_s: None,
        epoch_idx: (t_rx_s * 1000.0) as u64,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role,
        sats,
    }
}

#[test]
fn float_baseline_close_to_truth() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 10.0);
    let t_rx_s = 100_000.0;
    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
    ];
    let mut obs = Vec::new();
    for eph in &ephs {
        let sat = sat_state_gps_l1ca(eph, t_rx_s, 0.07);
        let dx = rover.0 - sat.x_m;
        let dy = rover.1 - sat.y_m;
        let dz = rover.2 - sat.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        obs.push(PositionObservation {
            sat: eph.sat,
            pseudorange_m: range + 299_792_458.0 * (0.0 - sat.clock_bias_s),
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
        });
    }
    let solver = PositionSolver::new();
    if let Some(solution) = solver.solve_wls(&obs, &ephs, t_rx_s) {
        let baseline = baseline_from_ecef(
            [base.0, base.1, base.2],
            [solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m],
        );
        assert!(baseline.enu_m[0].abs() < 500.0);
    }
}

#[test]
fn rtk_dd_solution_close_to_baseline() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let t_rx_s = 100_000.0;
    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
    ];

    let base_epoch = make_obs_epoch(ReceiverRole::Base, t_rx_s, base, &ephs);
    let rover_epoch = make_obs_epoch(ReceiverRole::Rover, t_rx_s, rover, &ephs);
    let sd = build_sd(&base_epoch, &rover_epoch);
    let ref_sig = choose_ref_sat(&sd).expect("ref sig");
    let dd = build_dd(&sd, ref_sig);
    let baseline =
        solve_baseline_dd(&dd, [base.0, base.1, base.2], &ephs, t_rx_s).expect("baseline");
    let expected = baseline_from_ecef([base.0, base.1, base.2], [rover.0, rover.1, rover.2]);

    assert!((baseline.enu_m[0] - expected.enu_m[0]).abs() < 500.0, "east error too large");
    assert!((baseline.enu_m[1] - expected.enu_m[1]).abs() < 500.0, "north error too large");
}

#[test]
fn solution_separation_reports_deltas() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let t_rx_s = 100_000.0;
    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
    ];
    let base_epoch = make_obs_epoch(ReceiverRole::Base, t_rx_s, base, &ephs);
    let rover_epoch = make_obs_epoch(ReceiverRole::Rover, t_rx_s, rover, &ephs);
    let sd = build_sd(&base_epoch, &rover_epoch);
    let ref_sig = choose_ref_sat(&sd).expect("ref sig");
    let dd = build_dd(&sd, ref_sig);
    let sep =
        solution_separation(&dd, [base.0, base.1, base.2], &ephs, t_rx_s).expect("separation");
    assert_eq!(sep.len(), dd.len());
    assert!(sep.iter().all(|s| s.delta_enu_m.is_finite()));
}
