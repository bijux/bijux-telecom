#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalSpec,
};
use bijux_gnss_nav::api::{geodetic_to_ecef, sat_state_gps_l1ca_at_receive_time, GpsEphemeris};
use bijux_gnss_receiver::api::{
    baseline_from_ecef, build_dd, build_sd, choose_ref_sat, dd_residual_metrics,
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

fn make_obs_epoch(
    role: ReceiverRole,
    receive_gps_time: GpsTime,
    receiver_ecef_m: [f64; 3],
    ephs: &[GpsEphemeris],
) -> ObsEpoch {
    let lambda_m = SPEED_OF_LIGHT_MPS / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value();
    let sats = ephs
        .iter()
        .map(|eph| {
            let mut tau_s = 0.07;
            let sat = loop {
                let state = sat_state_gps_l1ca_at_receive_time(eph, receive_gps_time.tow_s, tau_s);
                let range_m = geometric_range_m(receiver_ecef_m, [state.x_m, state.y_m, state.z_m]);
                let next_tau_s = range_m / SPEED_OF_LIGHT_MPS;
                if (next_tau_s - tau_s).abs() < 1.0e-12 {
                    break state;
                }
                tau_s = next_tau_s;
            };
            let range_m = geometric_range_m(receiver_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
            let pseudorange_m = range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
            let timing = ObsSignalTiming {
                signal_travel_time_s: Seconds(tau_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-tau_s),
            };
            ObsSatellite {
                signal_id: SigId {
                    sat: eph.sat,
                    band: SignalBand::L1,
                    code: bijux_gnss_core::api::SignalCode::Ca,
                },
                pseudorange_m: bijux_gnss_core::api::Meters(pseudorange_m),
                pseudorange_var_m2: 4.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(range_m / lambda_m),
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
                observation_status: bijux_gnss_core::api::ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: Some(timing),
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
fn rtk_residual_metrics_use_carried_satellite_timing() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let truth_baseline = baseline_from_ecef(base_ecef_m, [rover.0, rover.1, rover.2]);
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.08 };
    let ephs = vec![
        make_eph(1, 0.0, 0.0, 345_600.0),
        make_eph(2, 0.8, 0.9, 345_600.0),
        make_eph(3, 1.6, 1.8, 345_600.0),
        make_eph(4, 2.4, 2.7, 345_600.0),
        make_eph(5, 3.2, 3.6, 345_600.0),
    ];

    let base_epoch = make_obs_epoch(ReceiverRole::Base, receive_gps_time, base_ecef_m, &ephs);
    let rover_epoch =
        make_obs_epoch(ReceiverRole::Rover, receive_gps_time, [rover.0, rover.1, rover.2], &ephs);
    let sd = build_sd(&base_epoch, &rover_epoch);
    let ref_sig = choose_ref_sat(&sd).expect("reference satellite");
    let dd = build_dd(&sd, ref_sig);
    assert!(!dd.is_empty());
    assert!(dd.iter().all(|obs| obs.signal_timing.is_some() && obs.ref_signal_timing.is_some()));

    let (timed_rms_m, _timed_predicted_m, _) =
        dd_residual_metrics(&dd, base_ecef_m, truth_baseline.enu_m, &ephs, receive_gps_time.tow_s)
            .expect("timed residual metrics");

    let mut uncorrected_dd = dd.clone();
    for obs in &mut uncorrected_dd {
        obs.signal_timing = None;
        obs.ref_signal_timing = None;
        obs.signal_pseudorange_m = 1.0;
        obs.ref_signal_pseudorange_m = 1.0;
    }
    let (uncorrected_rms_m, _uncorrected_predicted_m, _) = dd_residual_metrics(
        &uncorrected_dd,
        base_ecef_m,
        truth_baseline.enu_m,
        &ephs,
        receive_gps_time.tow_s,
    )
    .expect("uncorrected residual metrics");

    assert!(
        uncorrected_rms_m > timed_rms_m + 1.0e-4,
        "dropping carried timing should measurably worsen DD residuals (timed_rms_m={timed_rms_m:.6}, uncorrected_rms_m={uncorrected_rms_m:.6})"
    );
}
