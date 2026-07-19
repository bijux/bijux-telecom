use super::*;

#[test]
fn observations_mark_carrier_phase_arc_start_and_continuity() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let epochs = vec![
        make_tracking_epoch_with_phase(6, &config, 70, carrier_hz, 10.00),
        make_tracking_epoch_with_phase(6, &config, 71, carrier_hz, 10.125),
        make_tracking_epoch_with_phase(6, &config, 72, carrier_hz, 10.250),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    assert_eq!(observations.len(), 3);
    assert_eq!(observations[0].sats[0].metadata.carrier_phase_continuity, "arc_start");
    assert_eq!(observations[0].sats[0].metadata.carrier_phase_arc_start_epoch_idx, 70);
    assert_eq!(
        observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index,
        observations[0].sats[0].metadata.time_tag_sample_index
    );
    assert_eq!(
        observations[1].sats[0]
            .metadata
            .cycle_slip_evidence
            .as_ref()
            .expect("cycle-slip evidence")
            .triggered_detectors(),
        Vec::<CycleSlipDetector>::new()
    );
    assert_eq!(observations[1].sats[0].metadata.carrier_phase_continuity, "continuous");
    assert_eq!(observations[2].sats[0].metadata.carrier_phase_continuity, "continuous");
    assert!((observations[1].sats[0].carrier_phase_cycles.0 - 10.125).abs() <= f64::EPSILON);
    assert_eq!(
        observations[2].sats[0].metadata.carrier_phase_arc_start_sample_index,
        observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index
    );
    let first_arc = observations[0].sats[0]
        .metadata
        .carrier_phase_arc
        .as_ref()
        .expect("first carrier-phase arc");
    assert_eq!(first_arc.signal_id, observations[0].sats[0].signal_id);
    assert_eq!(first_arc.start_reason, "arc_start");
    assert!(first_arc.valid_for_smoothing);
    assert!(first_arc.valid_for_ambiguity);
    assert_eq!(
        observations[1].sats[0]
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("continuous carrier-phase arc")
            .id,
        first_arc.id
    );
    assert_eq!(
        observations[2].sats[0]
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("second continuous carrier-phase arc")
            .id,
        first_arc.id
    );
}

#[test]
fn observations_reset_carrier_phase_arc_after_unlock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 71 },
        sample_index: epoch_sample_index(&config, 71),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 71),
            config.sampling_freq_hz,
        ),
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        carrier_hz: Hertz(carrier_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        ..TrackEpoch::default()
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(10, &config, 70, carrier_hz, 8.0),
            unlocked_epoch,
            make_tracking_epoch_with_phase(10, &config, 72, carrier_hz, 0.5),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let unlocked = &report.output[1].sats[0];
    let relocked = &report.output[2].sats[0];

    assert_eq!(unlocked.metadata.carrier_phase_continuity, "unusable");
    assert_eq!(unlocked.metadata.carrier_phase_arc_start_epoch_idx, 0);
    let unusable_arc =
        unlocked.metadata.carrier_phase_arc.as_ref().expect("unusable carrier-phase boundary");
    assert_eq!(unusable_arc.start_reason, "loss_of_lock");
    assert!(!unusable_arc.valid_for_smoothing);
    assert!(!unusable_arc.valid_for_ambiguity);
    assert_eq!(relocked.metadata.carrier_phase_continuity, "reset_after_unlock");
    assert_eq!(relocked.metadata.carrier_phase_arc_start_epoch_idx, 72);
    assert_eq!(
        relocked.metadata.carrier_phase_arc_start_sample_index,
        epoch_sample_index(&config, 72)
    );
    let relocked_arc =
        relocked.metadata.carrier_phase_arc.as_ref().expect("relocked carrier-phase arc");
    assert_eq!(relocked_arc.start_reason, "loss_of_lock");
    assert!(relocked_arc.valid_for_smoothing);
    assert_ne!(unusable_arc.id, relocked_arc.id);
    assert!(relocked.lock_flags.cycle_slip);
    let evidence = relocked.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(evidence.detected);
    assert_eq!(evidence.primary_reason.as_deref(), Some("loss_of_lock"));
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::TrackingLock));
    assert!((relocked.carrier_phase_cycles.0 - 0.5).abs() <= f64::EPSILON);
}

#[test]
fn observations_record_phase_innovation_cycle_slip_evidence() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let epochs = vec![
        make_tracking_epoch_with_phase(11, &config, 70, carrier_hz, 10.0),
        make_tracking_epoch_with_phase(11, &config, 71, carrier_hz, 10.125),
        make_tracking_epoch_with_phase(11, &config, 72, carrier_hz, 10.750),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    let slipped = &observations[2].sats[0];
    let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(slipped.lock_flags.cycle_slip);
    assert!(evidence.detected);
    assert_eq!(evidence.primary_reason.as_deref(), Some("carrier_phase_discontinuity"));
    assert!(
        evidence.triggered_detectors().contains(&CycleSlipDetector::DopplerPredictedPhase),
        "{evidence:?}"
    );
    assert!(
        evidence.triggered_detectors().contains(&CycleSlipDetector::PhaseInnovation),
        "{evidence:?}"
    );
    assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
    assert_eq!(evidence.false_alarm_probability_budget, CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET);
}

#[test]
fn observations_coast_carrier_phase_arc_through_recoverable_fade() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let faded_epoch = TrackEpoch {
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "degraded".to_string(),
        lock_state_reason: Some("signal_fade".to_string()),
        ..make_tracking_epoch_with_phase(13, &config, 71, carrier_hz, 10.125)
    };
    let recovered_epoch = TrackEpoch {
        lock_state_reason: Some("fade_recovered".to_string()),
        ..make_tracking_epoch_with_phase(13, &config, 72, carrier_hz, 10.250)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 13 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(13, &config, 70, carrier_hz, 10.0),
            faded_epoch,
            recovered_epoch,
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let first = &report.output[0].sats[0];
    let faded = &report.output[1].sats[0];
    let recovered = &report.output[2].sats[0];

    assert_eq!(first.metadata.carrier_phase_continuity, "arc_start");
    assert_eq!(faded.metadata.carrier_phase_continuity, "coasted");
    assert!(!faded.lock_flags.cycle_slip);
    let first_arc = first.metadata.carrier_phase_arc.as_ref().expect("first arc");
    assert_eq!(
        faded.metadata.carrier_phase_arc_start_sample_index,
        first.metadata.carrier_phase_arc_start_sample_index
    );
    assert_eq!(
        faded.metadata.carrier_phase_arc.as_ref().expect("coasted carrier-phase arc").id,
        first_arc.id
    );
    assert_eq!(recovered.metadata.carrier_phase_continuity, "continuous");
    assert!(!recovered.lock_flags.cycle_slip);
    assert_eq!(
        recovered.metadata.carrier_phase_arc_start_sample_index,
        first.metadata.carrier_phase_arc_start_sample_index
    );
    assert_eq!(
        recovered.metadata.carrier_phase_arc.as_ref().expect("recovered carrier-phase arc").id,
        first_arc.id
    );
}

#[test]
fn observations_start_new_carrier_phase_arc_after_reacquisition() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
    let lost_epoch = TrackEpoch {
        epoch: Epoch { index: 71 },
        sample_index: epoch_sample_index(&config, 71),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 71),
            config.sampling_freq_hz,
        ),
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        carrier_hz: Hertz(carrier_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let reacquired_epoch = TrackEpoch {
        lock_state_reason: Some("reacquired".to_string()),
        ..make_tracking_epoch_with_phase(14, &config, 72, carrier_hz, 0.5)
    };
    let settled_epoch = make_tracking_epoch_with_phase(14, &config, 73, carrier_hz, 0.6);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(14, &config, 70, carrier_hz, 8.0),
            lost_epoch,
            reacquired_epoch,
            settled_epoch,
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let lost = &report.output[1].sats[0];
    let reacquired = &report.output[2].sats[0];
    let settled = &report.output[3].sats[0];

    assert_eq!(lost.metadata.carrier_phase_continuity, "unusable");
    assert_eq!(reacquired.metadata.carrier_phase_continuity, "reset_after_reacquisition");
    assert_eq!(reacquired.metadata.carrier_phase_arc_start_epoch_idx, 72);
    let lost_arc = lost.metadata.carrier_phase_arc.as_ref().expect("lost carrier-phase boundary");
    let reacquired_arc =
        reacquired.metadata.carrier_phase_arc.as_ref().expect("reacquired carrier-phase arc");
    assert_eq!(lost_arc.start_reason, "loss_of_lock");
    assert!(!lost_arc.valid_for_smoothing);
    assert_eq!(reacquired_arc.start_reason, "reacquired");
    assert!(reacquired_arc.valid_for_smoothing);
    assert_ne!(lost_arc.id, reacquired_arc.id);
    assert!(reacquired.lock_flags.cycle_slip);
    assert_eq!(settled.metadata.carrier_phase_continuity, "continuous");
    assert_eq!(settled.metadata.carrier_phase_arc_start_epoch_idx, 72);
    assert_eq!(
        settled.metadata.carrier_phase_arc.as_ref().expect("settled carrier-phase arc").id,
        reacquired_arc.id
    );
}

#[test]
fn observations_reset_carrier_phase_arc_after_cycle_slip() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let slip_epoch = TrackEpoch {
        cycle_slip: true,
        cycle_slip_reason: Some("phase_jump".to_string()),
        ..make_tracking_epoch_with_phase(12, &config, 71, carrier_hz, 21.0)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(12, &config, 70, carrier_hz, 10.0),
            slip_epoch,
            make_tracking_epoch_with_phase(12, &config, 72, carrier_hz, 21.125),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let slipped = &report.output[1].sats[0];
    let post_slip = &report.output[2].sats[0];

    assert_eq!(slipped.metadata.carrier_phase_continuity, "reset_after_cycle_slip");
    assert_eq!(slipped.metadata.carrier_phase_arc_start_epoch_idx, 71);
    let slipped_arc =
        slipped.metadata.carrier_phase_arc.as_ref().expect("slipped carrier-phase arc");
    assert_eq!(slipped_arc.start_reason, "phase_jump");
    assert!(slipped_arc.valid_for_smoothing);
    assert!(slipped.lock_flags.cycle_slip);
    assert_eq!(post_slip.metadata.carrier_phase_continuity, "continuous");
    assert_eq!(post_slip.metadata.carrier_phase_arc_start_epoch_idx, 71);
    assert_eq!(
        post_slip.metadata.carrier_phase_arc_start_sample_index,
        epoch_sample_index(&config, 71)
    );
    assert_eq!(
        post_slip.metadata.carrier_phase_arc.as_ref().expect("post-slip carrier-phase arc").id,
        slipped_arc.id
    );
}
