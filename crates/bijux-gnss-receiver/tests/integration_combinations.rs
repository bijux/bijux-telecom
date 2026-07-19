#![allow(missing_docs)]
use bijux_gnss_core::api::{
    Constellation, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole,
    ReceiverSampleTrace, SatId, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, geometry_free_diagnostics_from_obs_epochs,
    iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    melbourne_wubbena_diagnostics_from_obs_epochs, GeometryFreeEvent, GeometryFreeThresholds,
    MelbourneWubbenaEvent, MelbourneWubbenaThresholds,
};
use bijux_gnss_signal::api::{
    signal_meters_to_cycles, signal_spec_beidou_b1i, signal_spec_beidou_b2i,
    signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py,
    signal_spec_gps_l5,
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
                signal_id: SigId { sat, band: SignalBand::L5, code: SignalCode::L5I },
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

struct SignalObservationMeters {
    signal: SignalSpec,
    band: SignalBand,
    code: SignalCode,
    pseudorange_m: f64,
    carrier_phase_m: f64,
}

struct DualSignalObsEpochRequest {
    sat: SatId,
    reference_observation: SignalObservationMeters,
    comparison_observation: SignalObservationMeters,
}

fn make_dual_signal_epoch_from_phase_meters(request: DualSignalObsEpochRequest) -> ObsEpoch {
    let DualSignalObsEpochRequest { sat, reference_observation, comparison_observation } = request;
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
                signal_id: SigId {
                    sat,
                    band: reference_observation.band,
                    code: reference_observation.code,
                },
                pseudorange_m: Meters(reference_observation.pseudorange_m),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: signal_meters_to_cycles(
                    Meters(reference_observation.carrier_phase_m),
                    reference_observation.signal,
                ),
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
                    signal: reference_observation.signal,
                    ..ObsMetadata::default()
                },
            },
            ObsSatellite {
                signal_id: SigId {
                    sat,
                    band: comparison_observation.band,
                    code: comparison_observation.code,
                },
                pseudorange_m: Meters(comparison_observation.pseudorange_m),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: signal_meters_to_cycles(
                    Meters(comparison_observation.carrier_phase_m),
                    comparison_observation.signal,
                ),
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
                    signal: comparison_observation.signal,
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
        (base_range - iono_l1)
            / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l2)
            / (299_792_458.0 / bijux_gnss_core::api::GPS_L2_PY_CARRIER_HZ.value()),
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
fn melbourne_wubbena_marks_nominal_wide_lane_behavior() {
    let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
        &[
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0),
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.01, 1001.0),
        ],
        SignalBand::L1,
        SignalBand::L2,
        MelbourneWubbenaThresholds { wide_lane_slip_jump_cycles: 0.5 },
    );

    assert_eq!(diagnostics[0].event, MelbourneWubbenaEvent::InsufficientHistory);
    assert_eq!(diagnostics[1].event, MelbourneWubbenaEvent::Nominal);
    assert!(
        diagnostics[1].delta_from_previous_wide_lane_cycles.expect("wide-lane delta").abs() < 0.5
    );
}

#[test]
fn melbourne_wubbena_marks_wide_lane_slip() {
    let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
        &[
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0),
            make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1100.0, 1001.0),
        ],
        SignalBand::L1,
        SignalBand::L2,
        MelbourneWubbenaThresholds { wide_lane_slip_jump_cycles: 0.5 },
    );

    assert_eq!(diagnostics[1].event, MelbourneWubbenaEvent::WideLaneSlipSuspect);
    assert!(
        diagnostics[1].delta_from_previous_wide_lane_cycles.expect("wide-lane delta").abs() >= 0.5
    );
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
        (base_range - iono_l1)
            / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l5) / (299_792_458.0 / bijux_gnss_core::api::GPS_L5_CARRIER_HZ.value()),
    );

    let combos = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);
    assert_eq!(combos.len(), 1);
    assert_eq!(combos[0].status, "ok");
    let if_code = combos[0].if_code_m.expect("if code");
    assert!((if_code - base_range).abs() < 10.0);
}

#[test]
fn iono_free_code_api_supports_l1_l5_pairs() {
    let base_range = 20_200_000.0;
    let iono_l1 = 5.0;
    let iono_l5 = 9.0;
    let epoch = make_l1_l5_epoch(
        base_range + iono_l1,
        base_range + iono_l5,
        (base_range - iono_l1)
            / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l5) / (299_792_458.0 / bijux_gnss_core::api::GPS_L5_CARRIER_HZ.value()),
    );

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_m.expect("iono-free code") - base_range).abs() < 10.0);
}

#[test]
fn iono_free_code_api_keeps_code_available_without_carrier_lock() {
    let mut epoch = make_dual_freq_epoch(20_200_005.0, 20_200_008.235_308_18, 1000.0, 1001.0);
    for satellite in &mut epoch.sats {
        satellite.lock_flags.carrier_lock = false;
        satellite.carrier_phase_var_cycles2 = f64::NAN;
    }

    let iono_free =
        iono_free_code_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(iono_free.len(), 1);
    assert_eq!(iono_free[0].status, "ok");
    assert!(iono_free[0].code_m.is_some());
    assert!(iono_free[0].variance_m2.is_some());

    assert_eq!(combinations.len(), 1);
    assert_eq!(combinations[0].status, "invalid");
    assert_eq!(combinations[0].if_code_status, "ok");
    assert!(combinations[0].if_code_m.is_some());
    assert!(combinations[0].if_phase_m.is_none());
}

#[test]
fn iono_free_phase_api_supports_l1_l5_pairs() {
    let base_range = 20_200_000.0;
    let iono_l1 = 5.0;
    let iono_l5 = 9.0;
    let epoch = make_l1_l5_epoch(
        base_range + iono_l1,
        base_range + iono_l5,
        (base_range - iono_l1)
            / (299_792_458.0 / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        (base_range - iono_l5) / (299_792_458.0 / bijux_gnss_core::api::GPS_L5_CARRIER_HZ.value()),
    );

    let observations = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].phase_m.expect("iono-free phase") - base_range).abs() < 10.0);
    assert!(observations[0].phase_cycles.is_some());
    assert!(observations[0].narrow_lane_wavelength_m.is_some());
}

#[test]
fn iono_free_phase_api_keeps_phase_available_without_code_lock() {
    let mut epoch = make_dual_freq_epoch(20_200_005.0, 20_200_008.235_308_18, 1000.0, 1001.0);
    for satellite in &mut epoch.sats {
        satellite.lock_flags.code_lock = false;
        satellite.pseudorange_var_m2 = f64::NAN;
    }

    let phase = iono_free_phase_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(phase.len(), 1);
    assert_eq!(phase[0].status, "ok");
    assert!(phase[0].phase_m.is_some());
    assert!(phase[0].variance_m2.is_some());

    assert_eq!(combinations.len(), 1);
    assert_eq!(combinations[0].status, "invalid");
    assert_eq!(combinations[0].if_phase_status, "ok");
    assert!(combinations[0].if_phase_m.is_some());
    assert!(combinations[0].if_code_m.is_none());
}

#[test]
fn galileo_geometry_free_and_iono_free_phase_use_e1_e5_wavelengths() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let e1 = signal_spec_galileo_e1b();
    let e5 = signal_spec_galileo_e5a();
    let narrow_lane_wavelength_m = 299_792_458.0 / (e1.carrier_hz.value() + e5.carrier_hz.value());
    let wide_lane_wavelength_m =
        299_792_458.0 / (e1.carrier_hz.value() - e5.carrier_hz.value()).abs();
    let epoch = make_dual_signal_epoch_from_phase_meters(DualSignalObsEpochRequest {
        sat,
        reference_observation: SignalObservationMeters {
            signal: e1,
            band: SignalBand::E1,
            code: SignalCode::E1B,
            pseudorange_m: 24_345_678.125,
            carrier_phase_m: 24_345_677.0,
        },
        comparison_observation: SignalObservationMeters {
            signal: e5,
            band: SignalBand::E5,
            code: SignalCode::E5a,
            pseudorange_m: 24_345_679.875,
            carrier_phase_m: 24_345_674.5,
        },
    });

    let combinations =
        combinations_from_obs_epochs(&[epoch.clone()], SignalBand::E1, SignalBand::E5);
    let iono_free_phase = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::E1, SignalBand::E5);

    let f1_2 = e1.carrier_hz.value().powi(2);
    let f2_2 = e5.carrier_hz.value().powi(2);
    let expected_if_phase_m = (f1_2 * 24_345_677.0 - f2_2 * 24_345_674.5) / (f1_2 - f2_2);
    let expected_narrow_lane_cycles = (24_345_677.0 + 24_345_674.5) / narrow_lane_wavelength_m;
    let expected_wide_lane_cycles = (24_345_677.0 - 24_345_674.5) / wide_lane_wavelength_m;

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert!(
        (combinations[0].geometry_free_phase_m.expect("geometry-free phase") - 2.5).abs() < 1.0e-6
    );
    assert!(
        (combinations[0].if_phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-6
    );
    assert!(
        (combinations[0].narrow_lane_wavelength_m.expect("narrow-lane wavelength")
            - narrow_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!(
        (combinations[0].narrow_lane_cycles.expect("narrow-lane cycles")
            - expected_narrow_lane_cycles)
            .abs()
            < 1.0e-6
    );
    assert!(
        (combinations[0].wide_lane_wavelength_m.expect("wide-lane wavelength")
            - wide_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!(
        (combinations[0].wide_lane_cycles.expect("wide-lane cycles") - expected_wide_lane_cycles)
            .abs()
            < 1.0e-6
    );
    assert!(
        (iono_free_phase[0].phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-6
    );
}

#[test]
fn beidou_geometry_free_and_iono_free_phase_use_b1_b2_wavelengths() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let b1 = signal_spec_beidou_b1i();
    let b2 = signal_spec_beidou_b2i();
    let narrow_lane_wavelength_m = 299_792_458.0 / (b1.carrier_hz.value() + b2.carrier_hz.value());
    let wide_lane_wavelength_m =
        299_792_458.0 / (b1.carrier_hz.value() - b2.carrier_hz.value()).abs();
    let epoch = make_dual_signal_epoch_from_phase_meters(DualSignalObsEpochRequest {
        sat,
        reference_observation: SignalObservationMeters {
            signal: b1,
            band: SignalBand::B1,
            code: SignalCode::B1I,
            pseudorange_m: 24_345_678.125,
            carrier_phase_m: 24_345_677.5,
        },
        comparison_observation: SignalObservationMeters {
            signal: b2,
            band: SignalBand::B2,
            code: SignalCode::B2I,
            pseudorange_m: 24_345_679.875,
            carrier_phase_m: 24_345_674.25,
        },
    });

    let combinations =
        combinations_from_obs_epochs(&[epoch.clone()], SignalBand::B1, SignalBand::B2);
    let iono_free_phase = iono_free_phase_from_obs_epochs(&[epoch], SignalBand::B1, SignalBand::B2);

    let f1_2 = b1.carrier_hz.value().powi(2);
    let f2_2 = b2.carrier_hz.value().powi(2);
    let expected_if_phase_m = (f1_2 * 24_345_677.5 - f2_2 * 24_345_674.25) / (f1_2 - f2_2);
    let expected_narrow_lane_cycles = (24_345_677.5 + 24_345_674.25) / narrow_lane_wavelength_m;
    let expected_wide_lane_cycles = (24_345_677.5 - 24_345_674.25) / wide_lane_wavelength_m;

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert!(
        (combinations[0].geometry_free_phase_m.expect("geometry-free phase") - 3.25).abs() < 1.0e-6
    );
    assert!(
        (combinations[0].if_phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-6
    );
    assert!(
        (combinations[0].narrow_lane_wavelength_m.expect("narrow-lane wavelength")
            - narrow_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!(
        (combinations[0].narrow_lane_cycles.expect("narrow-lane cycles")
            - expected_narrow_lane_cycles)
            .abs()
            < 1.0e-6
    );
    assert!(
        (combinations[0].wide_lane_wavelength_m.expect("wide-lane wavelength")
            - wide_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!(
        (combinations[0].wide_lane_cycles.expect("wide-lane cycles") - expected_wide_lane_cycles)
            .abs()
            < 1.0e-6
    );
    assert!(
        (iono_free_phase[0].phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-6
    );
}

#[test]
fn galileo_melbourne_wubbena_diagnostics_publish_wide_lane_wavelength() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 19 };
    let e1 = signal_spec_galileo_e1b();
    let e5 = signal_spec_galileo_e5a();
    let expected_wide_lane_wavelength_m =
        299_792_458.0 / (e1.carrier_hz.value() - e5.carrier_hz.value()).abs();
    let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
        &[make_dual_signal_epoch_from_phase_meters(DualSignalObsEpochRequest {
            sat,
            reference_observation: SignalObservationMeters {
                signal: e1,
                band: SignalBand::E1,
                code: SignalCode::E1B,
                pseudorange_m: 24_345_678.125,
                carrier_phase_m: 24_345_677.0,
            },
            comparison_observation: SignalObservationMeters {
                signal: e5,
                band: SignalBand::E5,
                code: SignalCode::E5a,
                pseudorange_m: 24_345_679.875,
                carrier_phase_m: 24_345_674.5,
            },
        })],
        SignalBand::E1,
        SignalBand::E5,
        MelbourneWubbenaThresholds::default(),
    );

    assert_eq!(diagnostics.len(), 1);
    assert_eq!(diagnostics[0].event, MelbourneWubbenaEvent::InsufficientHistory);
    assert!(
        (diagnostics[0].wide_lane_wavelength_m.expect("wide-lane wavelength")
            - expected_wide_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
}

#[test]
fn beidou_melbourne_wubbena_diagnostics_publish_wide_lane_wavelength() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 7 };
    let b1 = signal_spec_beidou_b1i();
    let b2 = signal_spec_beidou_b2i();
    let expected_wide_lane_wavelength_m =
        299_792_458.0 / (b1.carrier_hz.value() - b2.carrier_hz.value()).abs();
    let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
        &[make_dual_signal_epoch_from_phase_meters(DualSignalObsEpochRequest {
            sat,
            reference_observation: SignalObservationMeters {
                signal: b1,
                band: SignalBand::B1,
                code: SignalCode::B1I,
                pseudorange_m: 24_345_678.125,
                carrier_phase_m: 24_345_677.5,
            },
            comparison_observation: SignalObservationMeters {
                signal: b2,
                band: SignalBand::B2,
                code: SignalCode::B2I,
                pseudorange_m: 24_345_679.875,
                carrier_phase_m: 24_345_674.25,
            },
        })],
        SignalBand::B1,
        SignalBand::B2,
        MelbourneWubbenaThresholds::default(),
    );

    assert_eq!(diagnostics.len(), 1);
    assert_eq!(diagnostics[0].event, MelbourneWubbenaEvent::InsufficientHistory);
    assert!(
        (diagnostics[0].wide_lane_wavelength_m.expect("wide-lane wavelength")
            - expected_wide_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
}
