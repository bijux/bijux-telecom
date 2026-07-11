#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
    SignalSpec,
};
use bijux_gnss_nav::api::{geodetic_to_ecef, sat_state_gps_l1ca_at_receive_time, GpsEphemeris};
use bijux_gnss_receiver::api::{
    build_dd, build_sd, choose_ref_sat, solve_baseline_dd, solve_float_baseline_dd,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn make_eph(prn: u8, omega0: f64, m0: f64, toe_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 2200,
        sv_health: 0,
        toe_s,
        toc_s: toe_s,
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

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) =
        bijux_gnss_nav::api::ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn make_obs_epoch(
    role: ReceiverRole,
    receive_gps_time: GpsTime,
    receiver_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    ambiguities_cycles: &BTreeMap<SatId, f64>,
) -> ObsEpoch {
    let wavelength_m = SPEED_OF_LIGHT_MPS / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value();
    let sats = ephemerides
        .iter()
        .map(|ephemeris| {
            let mut travel_time_s = 0.07;
            let sat = loop {
                let state = sat_state_gps_l1ca_at_receive_time(
                    ephemeris,
                    receive_gps_time.tow_s,
                    travel_time_s,
                );
                let range_m = geometric_range_m(receiver_ecef_m, [state.x_m, state.y_m, state.z_m]);
                let next_travel_time_s = range_m / SPEED_OF_LIGHT_MPS;
                if (next_travel_time_s - travel_time_s).abs() < 1.0e-12 {
                    break state;
                }
                travel_time_s = next_travel_time_s;
            };
            let range_m = geometric_range_m(receiver_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
            let pseudorange_m = range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
            let ambiguity_cycles = ambiguities_cycles.get(&ephemeris.sat).copied().unwrap_or(0.0);
            let timing = ObsSignalTiming {
                signal_travel_time_s: Seconds(travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-travel_time_s),
            };
            ObsSatellite {
                signal_id: SigId {
                    sat: ephemeris.sat,
                    band: SignalBand::L1,
                    code: bijux_gnss_core::api::SignalCode::Ca,
                },
                pseudorange_m: bijux_gnss_core::api::Meters(pseudorange_m),
                pseudorange_var_m2: 4.0e-4,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(
                    range_m / wavelength_m + ambiguity_cycles,
                ),
                carrier_phase_var_cycles2: 1.0e-4,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 48.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: Some(timing),
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "synthetic".to_string(),
                    integration_ms: 1,
                    lock_quality: 48.0,
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
        t_rx_s: Seconds(receive_gps_time.tow_s),
        source_time: ReceiverSampleTrace::from_sample_index(
            (receive_gps_time.tow_s * 1000.0) as u64,
            1_000.0,
        ),
        gps_week: Some(receive_gps_time.week),
        tow_s: Some(Seconds(receive_gps_time.tow_s)),
        epoch_idx: (receive_gps_time.tow_s * 1000.0) as u64,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role,
        sats,
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn receiver_float_baseline_solver_projects_nav_solution() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let truth_enu_m = [8.5, -4.25, 1.75];
    let rover_ecef_m = enu_to_ecef(base_ecef_m, truth_enu_m);
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
    let ephemerides = vec![
        make_eph(3, 0.0, 0.0, receive_gps_time.tow_s - 900.0),
        make_eph(7, 0.8, 0.9, receive_gps_time.tow_s - 900.0),
        make_eph(11, 1.6, 1.8, receive_gps_time.tow_s - 900.0),
        make_eph(14, 2.4, 2.7, receive_gps_time.tow_s - 900.0),
        make_eph(19, 3.2, 3.6, receive_gps_time.tow_s - 900.0),
    ];
    let base_ambiguities_cycles = BTreeMap::new();
    let rover_ambiguities_cycles = BTreeMap::from([
        (ephemerides[0].sat, 17.25),
        (ephemerides[1].sat, 24.75),
        (ephemerides[2].sat, 31.5),
        (ephemerides[3].sat, 42.125),
        (ephemerides[4].sat, 53.625),
    ]);
    let base_epoch = make_obs_epoch(
        ReceiverRole::Base,
        receive_gps_time,
        base_ecef_m,
        &ephemerides,
        &base_ambiguities_cycles,
    );
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        rover_ecef_m,
        &ephemerides,
        &rover_ambiguities_cycles,
    );

    let single_differences = build_sd(&base_epoch, &rover_epoch);
    let reference = choose_ref_sat(&single_differences).expect("reference");
    let double_differences = build_dd(&single_differences, reference);
    let float_solution = solve_float_baseline_dd(
        &double_differences,
        base_ecef_m,
        &ephemerides,
        receive_gps_time.tow_s,
    )
    .expect("float solution");
    let projected_solution =
        solve_baseline_dd(&double_differences, base_ecef_m, &ephemerides, receive_gps_time.tow_s)
            .expect("projected solution");

    assert!((float_solution.enu_m[0] - truth_enu_m[0]).abs() < 0.05);
    assert!((float_solution.enu_m[1] - truth_enu_m[1]).abs() < 0.05);
    assert!((float_solution.enu_m[2] - truth_enu_m[2]).abs() < 0.10);
    assert_eq!(float_solution.float_ambiguities.len(), double_differences.len());
    assert_eq!(projected_solution.enu_m, float_solution.enu_m);
    assert_eq!(
        projected_solution.covariance_m2.expect("projected covariance"),
        float_solution.covariance_enu_m2
    );
    assert!(!projected_solution.fixed);
}
