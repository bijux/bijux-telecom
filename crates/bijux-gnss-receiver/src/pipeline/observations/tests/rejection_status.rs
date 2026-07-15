use super::*;

#[test]
fn apply_epoch_decision_refuses_malformed_duplicate_signal_set() {
    let mut epoch = nav_observation_epoch_fixture(0);
    let duplicate = epoch.sats[0].clone();
    epoch.sats.push(duplicate);
    apply_epoch_decision(&mut epoch);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    let reason = epoch.decision_reason.expect("decision reason");
    assert!(reason.contains("malformed_observation_set"));
    assert!(reason.contains("duplicate signal_id"));
}

#[test]
fn observations_reject_sample_rate_mismatch_tracking_reason() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        sat,
        lock: true,
        cn0_dbhz: 45.0,
        dll_lock: false,
        pll_lock: false,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let track = TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![mismatch_epoch],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat.observation_reject_reasons.iter().any(|reason| reason == "sample_rate_mismatch"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
}

#[test]
fn observations_preserve_tracking_cn0_on_unlock_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let expected_cn0_dbhz = 31.25;
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        cn0_dbhz: expected_cn0_dbhz,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Missing);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_preserve_tracking_cn0_on_inconsistent_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let expected_cn0_dbhz = 36.5;
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        lock: true,
        pll_lock: false,
        dll_lock: false,
        cn0_dbhz: expected_cn0_dbhz,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_keep_doppler_on_tracking_unlock_epoch() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let expected_doppler_hz = 125.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        carrier_hz: Hertz(carrier_hz),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        cn0_dbhz: 45.0,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Missing);
    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
    assert!(sat.doppler_var_hz2.is_finite());
}

#[test]
fn observations_keep_doppler_on_sample_rate_mismatch_epoch() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let expected_doppler_hz = -80.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        carrier_hz: Hertz(carrier_hz),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        lock: true,
        pll_lock: false,
        dll_lock: false,
        cn0_dbhz: 45.0,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
    assert!(sat.doppler_var_hz2.is_finite());
}

#[test]
fn observations_reject_non_positive_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(12, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "non_positive_pseudorange"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}

#[test]
fn observations_reject_out_of_bounds_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(13, &config, 70, carrier_hz, 0.0, 200_000, 0.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "pseudorange_out_of_bounds"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}

#[test]
fn observations_reject_non_finite_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(14, &config, 70, carrier_hz, 0.0, 68, f64::NAN);
    let (epochs, _) = observations_from_tracking(&config, &[epoch]);
    let epoch = epochs.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat.observation_reject_reasons.iter().any(|reason| reason == "non_finite_pseudorange"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}
