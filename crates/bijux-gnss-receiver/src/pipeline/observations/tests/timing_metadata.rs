use super::*;

#[test]
fn tracking_time_tag_prefers_last_locked_sample() {
    let locked = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 100,
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        lock_state: "tracking".to_string(),
        lock: true,
        ..TrackEpoch::default()
    };
    let unlocked_after = TrackEpoch {
        epoch: Epoch { index: 1 },
        sample_index: 101,
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        lock_state: "lost".to_string(),
        lock: false,
        ..TrackEpoch::default()
    };
    let mut last_locked = None;
    let (source_locked, sample_locked) = tracking_time_tag(&locked, &mut last_locked);
    let (source_unlocked, sample_unlocked) = tracking_time_tag(&unlocked_after, &mut last_locked);

    assert_eq!(source_locked, "tracking");
    assert_eq!(sample_locked, 100);
    assert_eq!(source_unlocked, "tracking_last_locked");
    assert_eq!(sample_unlocked, 100);
}

#[test]
fn observation_manifest_uses_stable_model_version_and_epoch_key() {
    let config = ReceiverPipelineConfig::default();
    let tracks = vec![make_track(1, &config), make_track(2, &config)];
    let report = observations_from_tracking_results(&config, &tracks, 10);
    assert!(!report.output.is_empty());
    for epoch in &report.output {
        let manifest = epoch.manifest.as_ref().expect("manifest");
        assert_eq!(manifest.version, bijux_gnss_core::api::OBSERVATION_MODEL_VERSION);
        assert_eq!(manifest.epoch_id, bijux_gnss_core::api::obs_epoch_stability_key(epoch));
    }
}

#[test]
fn grouped_observation_epochs_preserve_common_receiver_sample_trace() {
    let config = ReceiverPipelineConfig::default();
    let epoch_idx = 70;
    let expected_sample_index = epoch_sample_index(&config, epoch_idx);
    let tracks = vec![make_track(3, &config), make_track(8, &config)];

    let report = observations_from_tracking_results(&config, &tracks, 10);
    let epoch = report.output.iter().find(|epoch| epoch.epoch_idx == epoch_idx).expect("epoch");
    let manifest = epoch.manifest.as_ref().expect("manifest");

    assert_eq!(epoch.sats.len(), 2);
    assert_eq!(epoch.source_time.sample_index, expected_sample_index);
    assert_eq!(epoch.source_time.sample_rate_hz, config.sampling_freq_hz);
    assert_eq!(epoch.t_rx_s, epoch.source_time.receiver_time_s);
    assert_eq!(manifest.source_sample_index, expected_sample_index);
    assert_eq!(manifest.source_time, epoch.source_time);
}

#[test]
fn grouped_observation_epochs_reject_mixed_receiver_times() {
    let config = ReceiverPipelineConfig::default();
    let expected_sample_index = epoch_sample_index(&config, 70);
    let shifted_sample_index = epoch_sample_index(&config, 71);
    let mut shifted_epoch = make_tracking_epoch(8, &config, 70, 0.0);
    shifted_epoch.sample_index = shifted_sample_index;
    shifted_epoch.source_time =
        ReceiverSampleTrace::from_sample_index(shifted_sample_index, config.sampling_freq_hz);

    let report = observations_from_tracking_results(
        &config,
        &[make_track(3, &config), track_from_epoch(shifted_epoch)],
        10,
    );
    let epoch = report.output.iter().find(|epoch| epoch.epoch_idx == 70).expect("epoch");
    let shifted_sat = epoch.sats.iter().find(|sat| sat.signal_id.sat.prn == 8).expect("sat");

    assert_eq!(epoch.source_time.sample_index, expected_sample_index);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert_eq!(shifted_sat.observation_status, ObservationStatus::Inconsistent);
    assert!(shifted_sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "receiver_time_mismatch"));
    assert!(shifted_sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "receiver_sample_trace_mismatch"));
    assert!(report.events.iter().any(|event| event.code == "OBS_GROUPED_RECEIVER_TIME_MISMATCH"));
}

#[test]
fn observations_accept_expected_epoch_timing_interval() {
    let config =
        ReceiverPipelineConfig { tracking_integration_ms: 10, ..ReceiverPipelineConfig::default() };
    let base_epoch_idx = 70;
    let base_sample_index = epoch_sample_index(&config, base_epoch_idx);
    let next_epoch_idx = base_epoch_idx + 1;
    let mut next_epoch = make_tracking_epoch(3, &config, next_epoch_idx, 0.0);
    next_epoch.sample_index = observation_sample_index(&config, base_sample_index, 1);
    next_epoch.source_time =
        ReceiverSampleTrace::from_sample_index(next_epoch.sample_index, config.sampling_freq_hz);

    let report = observations_from_tracking_results(
        &config,
        &[
            track_from_epoch(make_tracking_epoch(3, &config, base_epoch_idx, 0.0)),
            track_from_epoch(next_epoch),
        ],
        10,
    );

    assert_eq!(report.output.len(), 2);
    assert!(report.events.iter().all(|event| event.code != "GNSS_OBS_TIME_INTERVAL_INVALID"));
    assert!(report.output.iter().all(|epoch| epoch.valid));
}

#[test]
fn observations_reject_invalid_epoch_timing_interval() {
    let config =
        ReceiverPipelineConfig { tracking_integration_ms: 10, ..ReceiverPipelineConfig::default() };
    let base_epoch_idx = 70;
    let base_sample_index = epoch_sample_index(&config, base_epoch_idx);
    let next_epoch_idx = base_epoch_idx + 1;
    let mut next_epoch = make_tracking_epoch(3, &config, next_epoch_idx, 0.0);
    next_epoch.sample_index = observation_sample_index(&config, base_sample_index, 1)
        + observation_interval_samples(&config) / 10;
    next_epoch.source_time =
        ReceiverSampleTrace::from_sample_index(next_epoch.sample_index, config.sampling_freq_hz);

    let report = observations_from_tracking_results(
        &config,
        &[
            track_from_epoch(make_tracking_epoch(3, &config, base_epoch_idx, 0.0)),
            track_from_epoch(next_epoch),
        ],
        10,
    );
    let epoch =
        report.output.iter().find(|epoch| epoch.epoch_idx == next_epoch_idx).expect("epoch");

    assert!(!epoch.valid);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("invalid_observation_timing"));
    assert!(report.events.iter().any(|event| event.code == "GNSS_OBS_TIME_INTERVAL_INVALID"));
}

#[test]
fn observations_stamp_receive_and_transmit_gps_times_from_anchor() {
    let config = ReceiverPipelineConfig::default();
    let mut track = make_track(4, &config);
    track.epochs[0].signal_delay_alignment = Some(SignalDelayAlignment {
        whole_code_periods: 70,
        sample_delay_samples: 0,
        source: "synthetic_truth".to_string(),
    });
    let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(capture_start_gps_time),
        &[track],
        10,
    );
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");
    let timing = sat.timing.expect("signal timing");

    assert_eq!(epoch.gps_week, Some(2200));
    let tow_s = epoch.tow_s.expect("epoch tow");
    assert!((tow_s.0 - 345_600.07).abs() <= 1.0e-6);
    assert!(
        (timing.signal_travel_time_s.0 - sat.pseudorange_m.0 / SPEED_OF_LIGHT_MPS).abs() <= 1.0e-12
    );
    assert_eq!(timing.transmit_gps_time.week, 2200);
    assert!((timing.transmit_gps_time.tow_s - 345_600.0).abs() <= 1.0e-9);
}

#[test]
fn observations_apply_receiver_clock_model_to_observables() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        receiver_clock_bias_s: 2.0e-6,
        receiver_clock_frequency_bias_hz: 17.5,
        receiver_clock_bias_sigma_s: 3.0e-8,
        receiver_clock_source: "synthetic_receiver_clock".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let raw_carrier_phase_cycles = 125.25;
    let raw_doppler_hz = 100.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, raw_doppler_hz);
    let track = track_from_epoch(make_tracking_epoch_with_alignment(
        6,
        &config,
        70,
        carrier_hz,
        raw_carrier_phase_cycles,
        68,
        128.0,
    ));

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");
    let error_model = sat.error_model.as_ref().expect("error model");
    let expected_pseudorange_m =
        aligned_pseudorange_m(&config, 68, 128.0) + 2.0e-6 * SPEED_OF_LIGHT_MPS;
    let expected_carrier_phase_cycles =
        raw_carrier_phase_cycles + 2.0e-6 * GPS_L1_CA_CARRIER_HZ.value();

    assert!((epoch.t_rx_s.0 - (70.0e-3 + 2.0e-6)).abs() <= 1.0e-12);
    assert!((sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6);
    assert!((sat.carrier_phase_cycles.0 - expected_carrier_phase_cycles).abs() <= 1.0e-6);
    assert!((sat.doppler_hz.0 - (raw_doppler_hz + 17.5)).abs() <= f64::EPSILON);
    assert!((error_model.clock_error_m.0 - 3.0e-8 * SPEED_OF_LIGHT_MPS).abs() <= 1.0e-12);
    assert_eq!(sat.metadata.receiver_clock_bias_s, Seconds(2.0e-6));
    assert_eq!(sat.metadata.receiver_clock_frequency_bias_hz, 17.5);
    assert_eq!(sat.metadata.receiver_clock_bias_sigma_s, Seconds(3.0e-8));
    assert_eq!(sat.metadata.receiver_clock_source, "synthetic_receiver_clock");
}

#[test]
fn hatch_smoothed_observations_preserve_receiver_clock_uncertainty() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        receiver_clock_bias_sigma_s: 4.0e-8,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
    let first = make_tracking_epoch_with_alignment(6, &config, 70, carrier_hz, 125.25, 68, 128.0);
    let second = make_tracking_epoch_with_alignment(6, &config, 71, carrier_hz, 126.25, 68, 128.0);

    let report = observations_from_tracking_results(
        &config,
        &[TrackingResult {
            sat: first.sat,
            carrier_hz,
            code_phase_samples: first.code_phase_samples.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: carrier_hz,
            acq_to_track_state: "accepted".to_string(),
            epochs: vec![first, second],
            transitions: Vec::new(),
        }],
        10,
    );
    let smoothed_sat = report.output[1].sats.first().expect("smoothed observation satellite");
    let error_model = smoothed_sat.error_model.as_ref().expect("error model");

    assert!(smoothed_sat.metadata.smoothing_age > 0);
    assert!((error_model.clock_error_m.0 - 4.0e-8 * SPEED_OF_LIGHT_MPS).abs() <= 1.0e-12);
}

#[test]
fn observations_without_alignment_keep_fallback_model_and_omit_signal_timing() {
    let config = ReceiverPipelineConfig::default();
    let track = make_track(5, &config);
    let capture_start_gps_time = GpsTime { week: 2200, tow_s: 345_600.0 };

    let report = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(capture_start_gps_time),
        &[track],
        10,
    );
    let sat = report.output.first().expect("observation epoch").sats.first().expect("sat");

    assert_eq!(sat.metadata.pseudorange_model, "receiver_epoch_fallback");
    assert_eq!(sat.metadata.observation_support_class, "degraded");
    assert!(sat.metadata.signal_delay_alignment_source.is_empty());
    assert!(sat.timing.is_none());
}
