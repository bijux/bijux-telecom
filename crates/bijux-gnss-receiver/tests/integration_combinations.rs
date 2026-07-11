#![allow(missing_docs)]
use bijux_gnss_core::api::{
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5, Constellation, LockFlags,
    ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, ReceiverSampleTrace, SatId, SigId,
    SignalBand, SignalCode,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, geometry_free_diagnostics_from_obs_epochs, GeometryFreeEvent,
    GeometryFreeThresholds,
};

fn make_dual_freq_epoch(p1: f64, p2: f64, phi1: f64, phi2: f64) -> ObsEpoch {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let mut s1 = signal_spec_gps_l1_ca();
    let mut s2 = signal_spec_gps_l2_py();
    s1.code_rate_hz = 1_023_000.0;
    s2.code_rate_hz = 1_023_000.0;
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: bijux_gnss_core::api::Meters(p1),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(phi1),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s1,
                    ..ObsMetadata::default()
                },
            },
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
                pseudorange_m: bijux_gnss_core::api::Meters(p2),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(phi2),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s2,
                    ..ObsMetadata::default()
                },
            },
        ],
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn make_l1_l5_epoch(p1: f64, p5: f64, phi1: f64, phi5: f64) -> ObsEpoch {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let mut s1 = signal_spec_gps_l1_ca();
    let s5 = signal_spec_gps_l5();
    s1.code_rate_hz = 1_023_000.0;
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: bijux_gnss_core::api::Meters(p1),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(phi1),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s1,
                    ..ObsMetadata::default()
                },
            },
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L5, code: SignalCode::Unknown },
                pseudorange_m: bijux_gnss_core::api::Meters(p5),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(phi5),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s5,
                    ..ObsMetadata::default()
                },
            },
        ],
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn iono_free_reduces_iono_term() {
    let base_range = 20_200_000.0;
    let iono_l1 = 5.0;
    let iono_l2 = 8.0;
    let epoch = make_dual_freq_epoch(
        base_range + iono_l1,
        base_range + iono_l2,
        (base_range - iono_l1) / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l2) / (299_792_458.0 / bijux_gnss_core::api::GPS_L2_PY_CARRIER_HZ.value()),
    );
    let combos = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);
    assert_eq!(combos.len(), 1);
    let combo = &combos[0];
    let if_code = combo.if_code_m.expect("if code");
    assert!((if_code - base_range).abs() < 10.0);
    let geometry_free = combo.geometry_free_phase_m.expect("geometry-free phase");
    assert!((geometry_free - 3.0).abs() < 1.0e-6, "{geometry_free}");
}

#[test]
fn melbourne_wubbena_detects_slip() {
    let epoch1 = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0);
    let epoch2 = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1100.0, 1001.0);
    let combos = combinations_from_obs_epochs(&[epoch1, epoch2], SignalBand::L1, SignalBand::L2);
    let mw1 = combos[0].melbourne_wubbena_m.unwrap();
    let mw2 = combos[1].melbourne_wubbena_m.unwrap();
    assert!((mw2 - mw1).abs() > 1.0);
}

#[test]
fn geometry_free_detects_ionosphere_drift() {
    let diagnostics = geometry_free_diagnostics_from_obs_epochs(
        &[
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 999.75),
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 999.55),
        ],
        SignalBand::L1,
        SignalBand::L2,
        GeometryFreeThresholds { ionosphere_delta_m: 0.01, cycle_slip_jump_m: 0.05 },
    );

    assert_eq!(diagnostics[0].event, GeometryFreeEvent::InsufficientHistory);
    assert_eq!(diagnostics[1].event, GeometryFreeEvent::IonosphereDrift);
    assert!(diagnostics[1].delta_from_previous_m.expect("delta") > 0.01);
}

#[test]
fn geometry_free_detects_cycle_slip() {
    let diagnostics = geometry_free_diagnostics_from_obs_epochs(
        &[
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0),
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1001.0, 1001.0),
        ],
        SignalBand::L1,
        SignalBand::L2,
        GeometryFreeThresholds { ionosphere_delta_m: 0.01, cycle_slip_jump_m: 0.05 },
    );

    assert_eq!(diagnostics[1].event, GeometryFreeEvent::CycleSlipSuspect);
    assert!(diagnostics[1].delta_from_previous_m.expect("delta") > 0.05);
}

#[test]
fn missing_frequency_marks_invalid() {
    let epoch = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0);
    let mut single = epoch.clone();
    single.sats.pop();
    let combos = combinations_from_obs_epochs(&[single], SignalBand::L1, SignalBand::L2);
    assert_eq!(combos.len(), 1);
    assert_eq!(combos[0].status, "invalid");
}

#[test]
fn iono_free_supports_l1_l5_pairs() {
    let base_range = 20_200_000.0;
    let iono_l1 = 5.0;
    let iono_l5 = 9.0;
    let epoch = make_l1_l5_epoch(
        base_range + iono_l1,
        base_range + iono_l5,
        (base_range - iono_l1) / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l5) / (299_792_458.0 / bijux_gnss_core::api::GPS_L5_CARRIER_HZ.value()),
    );

    let combos = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);
    assert_eq!(combos.len(), 1);
    assert_eq!(combos[0].status, "ok");
    let if_code = combos[0].if_code_m.expect("if code");
    assert!((if_code - base_range).abs() < 10.0);
}
