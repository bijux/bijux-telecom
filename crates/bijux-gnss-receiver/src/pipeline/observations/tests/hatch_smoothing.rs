use super::*;

#[test]
fn observations_advance_hatch_metadata_during_continuous_lock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(18, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(18, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(18, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 3);
    let sats = report
        .output
        .iter()
        .map(|epoch| epoch.sats.first().expect("observation satellite"))
        .collect::<Vec<_>>();

    assert_eq!(sats.len(), 3);
    assert!(sats.iter().all(|sat| sat.metadata.smoothing_window == 3));
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 2, 3]
    );
    assert!(sats.iter().all(|sat| sat.metadata.smoothing_resets == 0));
}

#[test]
fn observations_leave_hatch_smoothing_uninitialized_during_unlock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let unlocked_epoch = TrackEpoch {
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..make_tracking_epoch_with_alignment(19, &config, 71, carrier_hz, 10.125, 68, 0.0)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 19 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(19, &config, 70, carrier_hz, 10.0, 68, 0.0),
            unlocked_epoch,
            make_tracking_epoch_with_alignment(19, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let locked = &report.output[0].sats[0];
    let unlocked = &report.output[1].sats[0];
    let relocked = &report.output[2].sats[0];
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);

    assert_eq!(locked.metadata.smoothing_age, 1);
    assert_eq!(locked.metadata.smoothing_resets, 0);
    assert_eq!(unlocked.metadata.smoothing_age, 0);
    assert_eq!(unlocked.metadata.smoothing_resets, 1);
    assert!((unlocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
    assert_eq!(relocked.metadata.smoothing_age, 1);
    assert_eq!(relocked.metadata.smoothing_resets, 1);
    assert!((relocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
}

#[test]
fn observations_restart_hatch_smoothing_on_detected_cycle_slip() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 20 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(20, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(20, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(20, &config, 72, carrier_hz, 10.250, 68, 0.02),
            make_tracking_epoch_with_alignment(20, &config, 73, carrier_hz, 10.375, 68, 0.02),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
    let slipped = sats[2];
    let post_slip = sats[3];
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.02);

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 2, 1, 2]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 0, 1, 1]
    );
    assert!(slipped.lock_flags.cycle_slip);
    let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    let detectors = evidence.triggered_detectors();
    let divergence = evidence
        .contributors
        .iter()
        .find(|contributor| contributor.detector == CycleSlipDetector::CodeCarrierDivergence)
        .expect("code-carrier divergence contributor");
    assert!(detectors.contains(&CycleSlipDetector::CodeCarrierDivergence));
    assert!(divergence.triggered);
    assert!(
        divergence.value.expect("divergence value")
            > divergence.threshold.expect("divergence threshold")
    );
    assert_eq!(divergence.units, "m");
    assert_eq!(evidence.primary_reason.as_deref(), Some("code_carrier_divergence"));
    assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
    assert_eq!(evidence.false_alarm_probability_budget, CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET);
    assert!((slipped.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
    assert_eq!(post_slip.metadata.smoothing_age, 2);
}

#[test]
fn observations_refuse_hatch_smoothing_across_invalid_carrier_phase_arc() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let carrier_invalid = TrackEpoch {
        pll_lock: false,
        lock_state: "degraded".to_string(),
        lock_state_reason: Some("carrier_loop_unstable".to_string()),
        ..make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.01)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(21, &config, 70, carrier_hz, 10.0, 68, 0.0),
            carrier_invalid,
            make_tracking_epoch_with_alignment(21, &config, 72, carrier_hz, 0.5, 68, 0.02),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
    let first_arc = sats[0].metadata.carrier_phase_arc.as_ref().expect("first carrier-phase arc");
    let invalid_arc =
        sats[1].metadata.carrier_phase_arc.as_ref().expect("invalid carrier-phase boundary");
    let reset_arc = sats[2].metadata.carrier_phase_arc.as_ref().expect("reset carrier-phase arc");

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 0, 1]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 1, 1]
    );
    assert_eq!(sats[1].metadata.carrier_phase_continuity, "unusable");
    assert_eq!(invalid_arc.start_reason, "loss_of_lock");
    assert!(!invalid_arc.valid_for_smoothing);
    assert_ne!(first_arc.id, invalid_arc.id);
    assert_eq!(sats[2].metadata.carrier_phase_continuity, "reset_after_unlock");
    assert!(reset_arc.valid_for_smoothing);
    assert_ne!(invalid_arc.id, reset_arc.id);
}
