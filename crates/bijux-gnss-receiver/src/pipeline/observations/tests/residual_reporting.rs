use super::*;

fn residual_epoch(
    report: &StepReport<Vec<ObservationResidualEpochReport>>,
    epoch_idx: u64,
) -> &ObservationResidualEpochReport {
    report.output.iter().find(|epoch| epoch.epoch_idx == epoch_idx).expect("residual epoch")
}

#[test]
fn observation_residuals_report_raw_corrected_and_expected_pseudorange() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
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
            make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observation_residuals_from_tracking_results_with_gps_anchor(
        &config,
        Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
        &[track],
        10,
    );
    let epoch = residual_epoch(&report, 71);
    let sat = epoch.sats.first().expect("residual satellite");
    let lambda_m = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ.value();
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);
    let expected_corrected = expected_raw + 0.5 * 0.125 * lambda_m;

    assert!((sat.pseudorange_m.raw - expected_raw).abs() <= 1.0e-9, "{sat:?}");
    assert!((sat.pseudorange_m.corrected - expected_corrected).abs() <= 1.0e-9, "{sat:?}");
    assert_eq!(sat.pseudorange_m.expected, Some(expected_raw));
    assert_eq!(
        sat.pseudorange_m.reference_model.as_deref(),
        Some("signal_travel_time_from_gps_anchor")
    );
    assert!(
        (sat.pseudorange_m.residual.expect("pseudorange residual")
            - (expected_corrected - expected_raw))
            .abs()
            <= 1.0e-9,
        "{sat:?}"
    );
    assert!(sat.pseudorange_m.sigma.expect("pseudorange sigma") > 0.0);
}

#[test]
fn observation_residuals_predict_carrier_phase_for_continuous_arcs() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 22 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(22, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(22, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(22, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observation_residuals_from_tracking_results(&config, &[track], 10);
    let epoch = residual_epoch(&report, 72);
    let sat = epoch.sats.first().expect("residual satellite");

    assert_eq!(
        sat.carrier_phase_cycles.reference_model.as_deref(),
        Some("previous_continuous_phase_prediction")
    );
    assert_eq!(sat.carrier_phase_cycles.expected, Some(10.250));
    assert!(sat.carrier_phase_cycles.residual.expect("carrier residual").abs() <= 1.0e-12);
    assert_eq!(sat.carrier_phase_cycles.raw, 10.250);
    assert_eq!(sat.carrier_phase_cycles.corrected, 10.250);
}

#[test]
fn observation_residuals_surface_epoch_and_satellite_rejections() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(23, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report =
        observation_residuals_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let epoch = residual_epoch(&report, 70);
    let sat = epoch.sats.first().expect("residual satellite");

    assert!(!epoch.accepted);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!sat.accepted);
    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "non_positive_pseudorange"));
}

#[test]
fn observation_decisions_surface_impossible_pseudorange_reason() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(15, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let decisions = observation_decisions_from_epochs(&report.output);
    let decision = decisions.first().expect("observation decision");

    assert_eq!(decision.decision, ObservationEpochDecision::Rejected);
    assert!(decision.reasons.iter().any(|reason| reason == "non_positive_pseudorange"));
}
