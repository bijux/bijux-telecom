#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ObservationSupportClass,
    ObservationUncertaintyClass, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId,
    SignalBand, SignalCode, SignalSpec, SolutionStatus,
};
use bijux_gnss_nav::api::{geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris};
use bijux_gnss_receiver::api::{
    Navigation, NavigationMotionClass, ReceiverPipelineConfig, ReceiverRuntime,
};

fn make_eph(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: t_ref_s,
        toc_s: t_ref_s,
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

fn synthetic_pseudorange_m(eph: &GpsEphemeris, t_rx_s: f64, position_ecef: (f64, f64, f64)) -> f64 {
    let c = 299_792_458.0;
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
        let dx = position_ecef.0 - sat.x_m;
        let dy = position_ecef.1 - sat.y_m;
        let dz = position_ecef.2 - sat.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range - sat.clock_correction.bias_s * c;
        let next_tau = pseudorange_m / c;
        if (next_tau - tau).abs() < 1e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

fn make_obs_epoch(
    epoch_idx: u64,
    t_rx_s: f64,
    position_ecef: (f64, f64, f64),
    ephs: &[GpsEphemeris],
) -> ObsEpoch {
    let sats = ephs
        .iter()
        .map(|eph| {
            let pseudorange_m = synthetic_pseudorange_m(eph, t_rx_s, position_ecef);
            ObsSatellite {
                signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: Meters(pseudorange_m),
                pseudorange_var_m2: 4.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: Some(45.0),
                azimuth_deg: Some(0.0),
                weight: Some(1.0),
                timing: Some(ObsSignalTiming {
                    signal_travel_time_s: Seconds(pseudorange_m / 299_792_458.0),
                    transmit_gps_time: GpsTime {
                        week: 0,
                        tow_s: t_rx_s - (pseudorange_m / 299_792_458.0),
                    },
                }),
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "synthetic".to_string(),
                    integration_ms: 1,
                    lock_quality: 1.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: SignalSpec {
                        constellation: Constellation::Gps,
                        band: SignalBand::L1,
                        code: SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
                    },
                    observation_status: "accepted".to_string(),
                    observation_support_class: match ObservationSupportClass::Supported {
                        ObservationSupportClass::Supported => "supported",
                        ObservationSupportClass::Degraded => "degraded",
                        ObservationSupportClass::Unsupported => "unsupported",
                    }
                    .to_string(),
                    observation_uncertainty_class: match ObservationUncertaintyClass::Unknown {
                        ObservationUncertaintyClass::Low => "low",
                        ObservationUncertaintyClass::Medium => "medium",
                        ObservationUncertaintyClass::High => "high",
                        ObservationUncertaintyClass::Unknown => "unknown",
                    }
                    .to_string(),
                    ..ObsMetadata::default()
                },
            }
        })
        .collect();

    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: Some(0),
        tow_s: Some(Seconds(t_rx_s)),
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn deterministic_pseudorange_offset_m(epoch_idx: u64, prn: u8) -> f64 {
    let pattern = ((epoch_idx as usize * 17 + prn as usize * 13) % 9) as f64 - 4.0;
    pattern * 0.85
}

fn apply_deterministic_pseudorange_offsets(obs: &mut ObsEpoch) {
    for sat in &mut obs.sats {
        let offset_m = deterministic_pseudorange_offset_m(obs.epoch_idx, sat.signal_id.sat.prn);
        sat.pseudorange_m.0 += offset_m;
        if let Some(timing) = sat.timing.as_mut() {
            timing.signal_travel_time_s = Seconds(sat.pseudorange_m.0 / 299_792_458.0);
            timing.transmit_gps_time.tow_s = obs.t_rx_s.0 - timing.signal_travel_time_s.0;
        }
    }
}

fn translate_truth_ecef_m(
    truth_ecef_m: (f64, f64, f64),
    truth_velocity_mps: (f64, f64, f64),
    dt_s: f64,
) -> (f64, f64, f64) {
    (
        truth_ecef_m.0 + truth_velocity_mps.0 * dt_s,
        truth_ecef_m.1 + truth_velocity_mps.1 * dt_s,
        truth_ecef_m.2 + truth_velocity_mps.2 * dt_s,
    )
}

fn position_error_3d_m(
    solution: &bijux_gnss_core::api::NavSolutionEpoch,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
    let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
    let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn root_mean_square(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn path_length_m(solutions: &[bijux_gnss_core::api::NavSolutionEpoch]) -> f64 {
    solutions
        .windows(2)
        .map(|window| {
            let dx = window[1].ecef_x_m.0 - window[0].ecef_x_m.0;
            let dy = window[1].ecef_y_m.0 - window[0].ecef_y_m.0;
            let dz = window[1].ecef_z_m.0 - window[0].ecef_z_m.0;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

fn path_length_from_positions_m(positions_ecef_m: &[(f64, f64, f64)]) -> f64 {
    positions_ecef_m
        .windows(2)
        .map(|window| {
            let dx = window[1].0 - window[0].0;
            let dy = window[1].1 - window[0].1;
            let dz = window[1].2 - window[0].2;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

fn navigation_test_config(
    position_solution_smoothing: bool,
    position_solution_motion_class: NavigationMotionClass,
) -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig::default();
    config.position_solution_smoothing = position_solution_smoothing;
    config.position_solution_motion_class = position_solution_motion_class;
    config.science_thresholds.min_mean_cn0_dbhz = 1.0;
    config.science_thresholds.max_pdop = 100.0;
    config.science_thresholds.max_gdop = 100.0;
    config.science_thresholds.max_residual_rms_m = 1_000.0;
    config
}

#[test]
fn public_navigation_api_smooths_static_and_moving_solution_sequences() {
    let static_truth = geodetic_to_ecef(37.0, -122.0, 25.0);
    let moving_velocity_mps = (8.0, -3.0, 1.5);
    let t0_rx_s = 100_000.0;
    let ephs = vec![
        make_eph(1, 0.0, 0.0, t0_rx_s),
        make_eph(2, 0.8, 0.9, t0_rx_s),
        make_eph(3, 1.6, 1.8, t0_rx_s),
        make_eph(4, 2.4, 2.7, t0_rx_s),
        make_eph(5, 3.2, 3.6, t0_rx_s),
        make_eph(6, 4.0, 4.5, t0_rx_s),
    ];

    let mut baseline_nav = Navigation::new(
        navigation_test_config(false, NavigationMotionClass::Vehicle),
        ReceiverRuntime::default(),
    );
    let mut static_nav = Navigation::new(
        navigation_test_config(true, NavigationMotionClass::Static),
        ReceiverRuntime::default(),
    );
    let mut vehicle_nav = Navigation::new(
        navigation_test_config(true, NavigationMotionClass::Vehicle),
        ReceiverRuntime::default(),
    );

    let mut raw_static_errors_m = Vec::new();
    let mut smoothed_static_errors_m = Vec::new();
    let mut raw_static_solutions = Vec::new();
    let mut smoothed_static_solutions = Vec::new();
    let mut smoothed_moving_solutions = Vec::new();
    let mut raw_moving_errors_m = Vec::new();
    let mut smoothed_moving_errors_m = Vec::new();

    for epoch_idx in 0..50u64 {
        let static_t_rx_s = t0_rx_s + epoch_idx as f64;
        let mut static_obs = make_obs_epoch(epoch_idx, static_t_rx_s, static_truth, &ephs);
        apply_deterministic_pseudorange_offsets(&mut static_obs);
        let raw_static = baseline_nav.solve_epoch(&static_obs, &ephs).expect("raw static");
        let smoothed_static = static_nav.solve_epoch(&static_obs, &ephs).expect("smoothed static");
        assert_ne!(raw_static.status, SolutionStatus::Unavailable);
        assert_ne!(smoothed_static.status, SolutionStatus::Unavailable);
        raw_static_errors_m.push(position_error_3d_m(&raw_static, static_truth));
        smoothed_static_errors_m.push(position_error_3d_m(&smoothed_static, static_truth));
        raw_static_solutions.push(raw_static);
        smoothed_static_solutions.push(smoothed_static);

        let moving_truth =
            translate_truth_ecef_m(static_truth, moving_velocity_mps, epoch_idx as f64);
        let moving_t_rx_s = t0_rx_s + 200.0 + epoch_idx as f64;
        let mut moving_obs = make_obs_epoch(epoch_idx, moving_t_rx_s, moving_truth, &ephs);
        apply_deterministic_pseudorange_offsets(&mut moving_obs);
        let raw_moving = baseline_nav.solve_epoch(&moving_obs, &ephs).expect("raw moving");
        let smoothed_moving = vehicle_nav.solve_epoch(&moving_obs, &ephs).expect("smoothed moving");
        raw_moving_errors_m.push(position_error_3d_m(&raw_moving, moving_truth));
        smoothed_moving_errors_m.push(position_error_3d_m(&smoothed_moving, moving_truth));
        smoothed_moving_solutions.push(smoothed_moving);
    }

    let moving_truth_trace = (0..50u64)
        .map(|epoch_idx| {
            translate_truth_ecef_m(static_truth, moving_velocity_mps, epoch_idx as f64)
        })
        .collect::<Vec<_>>();

    assert!(
        root_mean_square(&smoothed_static_errors_m) < root_mean_square(&raw_static_errors_m) * 0.75
    );
    assert!(path_length_m(&smoothed_static_solutions) < path_length_m(&raw_static_solutions) * 0.4);
    assert!(root_mean_square(&smoothed_moving_errors_m) <= root_mean_square(&raw_moving_errors_m));
    assert!(
        path_length_m(&smoothed_moving_solutions)
            > path_length_from_positions_m(&moving_truth_trace) * 0.9
    );
}
