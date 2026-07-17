use super::*;
use bijux_gnss_core::api::ObsEpoch as ObservationEpoch;

#[test]
fn code_carrier_divergence_decomposition_exempts_expected_ionosphere_from_multipath() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 3.0;
    let l1_delay_after_m = 9.0;
    let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
    assert_ne!(before.sats[0].signal_id.band, before.sats[1].signal_id.band);
    assert_eq!(before.sats[0].signal_id.sat, before.sats[1].signal_id.sat);
    assert!(before.sats[0].lock_flags.code_lock);
    assert!(before.sats[1].lock_flags.code_lock);
    assert!(before.sats[0].metadata.signal.carrier_hz.value() > 0.0);
    assert!(before.sats[1].metadata.signal.carrier_hz.value() > 0.0);
    assert_ne!(
        before.sats[0].metadata.signal.carrier_hz.value(),
        before.sats[1].metadata.signal.carrier_hz.value()
    );
    assert!(before.sats[0].pseudorange_m.0.is_finite());
    assert!(before.sats[1].pseudorange_m.0.is_finite());
    assert!(dual_frequency_code_delay_evidence(&before.sats[0], &before.sats[1]).is_some());
    assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        106.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(!sat.multipath_suspect, "{sat:?}");
    assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
    assert!(divergence.multipath_m.abs() < 1.0e-9);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
    assert_eq!(sat.error_model.as_ref().expect("error model").multipath_proxy_m, Meters(0.0));
}

#[test]
fn code_carrier_divergence_decomposition_preserves_ionosphere_sign() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 9.0;
    let l1_delay_after_m = 3.0;
    let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 118.0, 0.0, l2_delay_before_m, 0.0);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        118.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(!sat.multipath_suspect, "{sat:?}");
    assert!((divergence.jump_m - l1_jump_m).abs() < 1.0e-6);
    assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
    assert!(divergence.multipath_m.abs() < 1.0e-9);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
}

#[test]
fn code_carrier_divergence_decomposition_keeps_residual_multipath_separate() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 3.0;
    let l1_delay_after_m = 9.0;
    let residual_multipath_m = 8.0;
    let l1_ionosphere_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l1_jump_m = l1_ionosphere_jump_m + residual_multipath_m;
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
    assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        106.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(sat.multipath_suspect, "{sat:?}");
    assert!((divergence.expected_ionosphere_m - l1_ionosphere_jump_m).abs() < 1.0e-6);
    assert!((divergence.multipath_m - residual_multipath_m).abs() < 1.0e-6);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
    assert!(
        (sat.error_model.as_ref().expect("error model").multipath_proxy_m.0 - residual_multipath_m)
            .abs()
            < 1.0e-6
    );
}

#[test]
fn dual_frequency_cycle_slip_fusion_records_geometry_free_detector() {
    let l1_delay_m = 4.0;
    let l2_delay_m =
        scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    current.epoch_idx = 71;
    let signal = current.sats[0].metadata.signal;
    current.sats[0].carrier_phase_cycles = Cycles(
        current.sats[0].carrier_phase_cycles.0 + signal_meters_to_cycles(Meters(0.25), signal).0,
    );
    current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 + 10.0);
    let raw_pseudorange_m = current.sats[0].pseudorange_m.0 - 10.0;
    let mut raw_snapshots = std::collections::HashMap::new();
    raw_snapshots.insert(
        observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
        raw_observation_snapshot(&current.sats[0], raw_pseudorange_m),
    );
    let mut hatch = std::collections::HashMap::new();

    apply_dual_frequency_cycle_slip_fusion(
        &mut current,
        Some(&previous),
        &raw_snapshots,
        &mut hatch,
    );

    let sat = current
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L1)
        .expect("L1 observation");
    let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(sat.lock_flags.cycle_slip, "{sat:?}");
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::GeometryFreePhase));
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("geometry_free_phase"));
    assert!((sat.pseudorange_m.0 - raw_pseudorange_m).abs() < 1.0e-9);
    assert_eq!(sat.metadata.smoothing_age, 1);
}

#[test]
fn dual_frequency_cycle_slip_fusion_records_melbourne_wubbena_detector() {
    let l1_delay_m = 4.0;
    let l2_delay_m =
        scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    current.epoch_idx = 72;
    current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 - 2.0);
    let mut raw_snapshots = std::collections::HashMap::new();
    raw_snapshots.insert(
        observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
        raw_observation_snapshot(&current.sats[0], current.sats[0].pseudorange_m.0),
    );
    let mut hatch = std::collections::HashMap::new();

    apply_dual_frequency_cycle_slip_fusion(
        &mut current,
        Some(&previous),
        &raw_snapshots,
        &mut hatch,
    );

    let sat = current
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L1)
        .expect("L1 observation");
    let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(sat.lock_flags.cycle_slip, "{sat:?}");
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::MelbourneWubbena));
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("melbourne_wubbena"));
}

fn scaled_ionosphere_delay_m(
    reference_delay_m: f64,
    reference_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    reference_delay_m * reference_signal.carrier_hz.value().powi(2)
        / target_signal.carrier_hz.value().powi(2)
}

fn divergence_epoch(
    l1_ionosphere_delay_m: f64,
    l1_raw_divergence_m: f64,
    l1_divergence_jump_m: f64,
    l2_ionosphere_delay_m: f64,
    l2_divergence_jump_m: f64,
) -> ObservationEpoch {
    let base_range_m = 20_200_000.0;
    accepted_rover_observation_epoch(AcceptedRoverObservationEpochRequest {
        t_rx_s: Seconds(70.0),
        source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 70,
        discontinuity: false,
        sats: vec![
            divergence_satellite(
                signal_spec_gps_l1_ca(),
                l1_ionosphere_delay_m,
                l1_raw_divergence_m,
                l1_divergence_jump_m,
                base_range_m,
            ),
            divergence_satellite(
                signal_spec_gps_l2c(),
                l2_ionosphere_delay_m,
                100.0 + l2_divergence_jump_m,
                l2_divergence_jump_m,
                base_range_m,
            ),
        ],
        decision_reason: None,
        manifest: None,
    })
}

fn divergence_satellite(
    signal: SignalSpec,
    ionosphere_delay_m: f64,
    raw_divergence_m: f64,
    divergence_jump_m: f64,
    base_range_m: f64,
) -> ObsSatellite {
    let pseudorange_m = base_range_m + ionosphere_delay_m;
    let carrier_phase_m = pseudorange_m - raw_divergence_m;
    ObsSatellite {
        signal_id: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: signal.band,
            code: signal.code,
        },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: signal_meters_to_cycles(Meters(carrier_phase_m), signal),
        carrier_phase_var_cycles2: 1.0,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
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
        weight: None,
        timing: None,
        error_model: Some(bijux_gnss_core::api::MeasurementErrorModel {
            thermal_noise_m: Meters(0.0),
            tracking_jitter_m: Meters(1.0),
            multipath_proxy_m: Meters(0.0),
            clock_error_m: Meters(0.0),
        }),
        metadata: ObsMetadata {
            signal,
            integration_ms: 20,
            code_carrier_divergence: Some(CodeCarrierDivergence::from_terms(
                raw_divergence_m,
                divergence_jump_m,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )),
            ..ObsMetadata::default()
        },
    }
}
