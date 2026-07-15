use super::*;

#[test]
fn iono_free_mode_keeps_one_representative_per_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
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
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let sats = vec![&l1, &l2];

    let representatives = iono_free_satellite_representatives(&sats);

    assert_eq!(representatives.len(), 1);
    assert_eq!(representatives[0].signal_id.band, SignalBand::L1);
}

#[test]
fn ppp_filter_resolves_iono_free_code_bias_from_both_signals() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_200_010.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
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
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        pseudorange_m: Meters(20_200_005.0),
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let obs = ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![l1, l2],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };
    let measurement = iono_free_code_observation_from_obs(&obs, sat).expect("iono-free code");
    let bias_table = SignalCodeBiases::from_biases([
        CodeBias { sig: measurement.signal_1, bias_m: 2.0 },
        CodeBias { sig: measurement.signal_2, bias_m: -0.5 },
    ]);

    let resolved_bias_m = resolved_iono_free_code_bias_m(&bias_table, measurement);

    assert!((resolved_bias_m - 2.0).abs() > 1.0e-3);
    assert!(resolved_bias_m.is_finite());
}

#[test]
fn ppp_filter_resolves_single_frequency_code_bias_at_epoch_time() {
    struct TimeAwareBiasProvider;

    impl CodeBiasProvider for TimeAwareBiasProvider {
        fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
            None
        }

        fn code_bias_m_at(
            &self,
            _sig: SigId,
            time: Option<bijux_gnss_core::api::GpsTime>,
        ) -> Option<f64> {
            Some(time?.tow_s)
        }
    }

    let signal = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };

    let resolved_bias_m = resolved_code_bias_m(
        &TimeAwareBiasProvider,
        signal,
        Some(bijux_gnss_core::api::GpsTime { week: 0, tow_s: 123.5 }),
    );

    assert!((resolved_bias_m - 123.5).abs() < f64::EPSILON);
}

#[test]
fn ppp_filter_tracks_phase_windup_by_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let (receiver_x_m, receiver_y_m, receiver_z_m) = geodetic_to_ecef(45.0, 12.0, 80.0);
    let receiver_pos_m = [receiver_x_m, receiver_y_m, receiver_z_m];
    let sat_pos_m = [15_600_000.0, -10_200_000.0, 21_700_000.0];
    let mut states = BTreeMap::new();

    let first = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_400.0 }),
    );
    let second = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_430.0 }),
    );

    assert!(first.is_finite());
    assert!(second.is_finite());
    assert!(states.contains_key(&sat));
    assert!((second - first).abs() < 0.5);
}

#[test]
fn ppp_filter_skips_phase_windup_without_epoch_time() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut states = BTreeMap::new();

    let windup_cycles = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        [1.0, 0.0, 0.0],
        [20_200_000.0, 14_000_000.0, 21_700_000.0],
        None,
    );

    assert_eq!(windup_cycles, 0.0);
    assert!(states.is_empty());
}
