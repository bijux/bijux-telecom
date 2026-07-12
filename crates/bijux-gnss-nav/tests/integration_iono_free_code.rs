#![allow(missing_docs)]

use bijux_gnss_core::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
    Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
    Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{combinations_from_obs_epochs, iono_free_code_from_obs_epochs};

fn dual_frequency_epoch(
    sat: SatId,
    first_band: SignalBand,
    first_code: SignalCode,
    first_signal: SignalSpec,
    second_band: SignalBand,
    second_code: SignalCode,
    second_signal: SignalSpec,
    pseudorange_1_m: f64,
    pseudorange_2_m: f64,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            satellite(
                sat,
                first_band,
                first_code,
                first_signal,
                pseudorange_1_m,
                code_lock,
                carrier_lock,
            ),
            satellite(
                sat,
                second_band,
                second_code,
                second_signal,
                pseudorange_2_m,
                code_lock,
                carrier_lock,
            ),
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    pseudorange_m: f64,
    code_lock: bool,
    carrier_lock: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(0.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: false, cycle_slip: false },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
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
            signal,
            ..ObsMetadata::default()
        },
    }
}

fn dispersive_delay_at_band(
    iono_l1_m: f64,
    l1_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    iono_l1_m * (l1_signal.carrier_hz.value() * l1_signal.carrier_hz.value())
        / (target_signal.carrier_hz.value() * target_signal.carrier_hz.value())
}

#[test]
fn iono_free_code_recovers_geometric_range_for_l1_l2() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let epoch = dual_frequency_epoch(
        SatId { constellation: Constellation::Gps, prn: 11 },
        SignalBand::L1,
        SignalCode::Ca,
        l1,
        SignalBand::L2,
        SignalCode::Py,
        l2,
        base_range_m + iono_l1_m,
        base_range_m + dispersive_delay_at_band(iono_l1_m, l1, l2),
        true,
        true,
    );

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_code_recovers_geometric_range_for_l1_l5() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let l1 = signal_spec_gps_l1_ca();
    let l5 = signal_spec_gps_l5();
    let epoch = dual_frequency_epoch(
        SatId { constellation: Constellation::Gps, prn: 11 },
        SignalBand::L1,
        SignalCode::Ca,
        l1,
        SignalBand::L5,
        SignalCode::Unknown,
        l5,
        base_range_m + iono_l1_m,
        base_range_m + dispersive_delay_at_band(iono_l1_m, l1, l5),
        true,
        true,
    );

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L5);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_code_recovers_geometric_range_for_e1_e5() {
    let base_range_m = 24_000_000.0;
    let iono_e1_m = 4.5;
    let e1 = signal_spec_galileo_e1b();
    let e5 = signal_spec_galileo_e5a();
    let epoch = dual_frequency_epoch(
        SatId { constellation: Constellation::Galileo, prn: 19 },
        SignalBand::E1,
        SignalCode::E1B,
        e1,
        SignalBand::E5,
        SignalCode::E5a,
        e5,
        base_range_m + iono_e1_m,
        base_range_m + dispersive_delay_at_band(iono_e1_m, e1, e5),
        true,
        true,
    );

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::E1, SignalBand::E5);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_code_recovers_geometric_range_for_b1_b2() {
    let base_range_m = 24_100_000.0;
    let iono_b1_m = 3.75;
    let b1 = signal_spec_beidou_b1i();
    let b2 = signal_spec_beidou_b2i();
    let epoch = dual_frequency_epoch(
        SatId { constellation: Constellation::Beidou, prn: 7 },
        SignalBand::B1,
        SignalCode::B1I,
        b1,
        SignalBand::B2,
        SignalCode::B2I,
        b2,
        base_range_m + iono_b1_m,
        base_range_m + dispersive_delay_at_band(iono_b1_m, b1, b2),
        true,
        true,
    );

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::B1, SignalBand::B2);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_m.expect("iono-free code") - base_range_m).abs() < 1.0e-6);
}

#[test]
fn iono_free_code_remains_available_when_phase_combinations_are_not() {
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 5.0;
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let epoch = dual_frequency_epoch(
        SatId { constellation: Constellation::Gps, prn: 11 },
        SignalBand::L1,
        SignalCode::Ca,
        l1,
        SignalBand::L2,
        SignalCode::Py,
        l2,
        base_range_m + iono_l1_m,
        base_range_m + dispersive_delay_at_band(iono_l1_m, l1, l2),
        true,
        false,
    );

    let iono_free =
        iono_free_code_from_obs_epochs(&[epoch.clone()], SignalBand::L1, SignalBand::L2);
    let combinations = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    assert_eq!(iono_free.len(), 1);
    assert_eq!(iono_free[0].status, "ok");
    assert!(iono_free[0].code_m.is_some());

    assert_eq!(combinations.len(), 1);
    assert_eq!(combinations[0].status, "invalid");
    assert_eq!(combinations[0].if_code_status, "ok");
    assert!(combinations[0].if_code_m.is_some());
    assert!(combinations[0].if_phase_m.is_none());
}

#[test]
fn iono_free_code_variance_matches_linear_error_propagation() {
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let variance_1_m2 = 1.5;
    let variance_2_m2 = 4.0;
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let epoch = ObsEpoch {
        t_rx_s: Seconds(0.0),
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
                pseudorange_var_m2: variance_1_m2,
                ..satellite(sat, SignalBand::L1, SignalCode::Ca, l1, 20_200_005.0, true, true)
            },
            ObsSatellite {
                pseudorange_var_m2: variance_2_m2,
                ..satellite(
                    sat,
                    SignalBand::L2,
                    SignalCode::Py,
                    l2,
                    20_200_008.235_308_182,
                    true,
                    true,
                )
            },
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let observations = iono_free_code_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);

    let f1_2 = l1.carrier_hz.value() * l1.carrier_hz.value();
    let f2_2 = l2.carrier_hz.value() * l2.carrier_hz.value();
    let denom = f1_2 - f2_2;
    let weight_1 = f1_2 / denom;
    let weight_2 = -f2_2 / denom;
    let expected_variance = weight_1.powi(2) * variance_1_m2 + weight_2.powi(2) * variance_2_m2;

    assert!((observations[0].variance_m2.expect("variance") - expected_variance).abs() < 1.0e-9);
}
