use super::*;

#[test]
fn acquisition_decision_rejects_weak_primary_peak() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let decision = acquisition_decision(2.0, 2.0, 2.0, 2.0, &thresholds);
    assert_eq!(decision.hypothesis.to_string(), "rejected");
    assert_eq!(decision.reason, AcquisitionDecisionReason::LowPeakMetric);
    assert_eq!(decision.score, 0.0);
}

#[test]
fn acquisition_decision_marks_ambiguous_on_low_peak_separation() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let decision = acquisition_decision(3.0, 1.0, 1.4, 2.0, &thresholds);
    assert_eq!(decision.hypothesis.to_string(), "ambiguous");
    assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
    assert!((decision.score - 1.2).abs() < 1e-6);
}

#[test]
fn acquisition_decision_marks_ambiguous_on_comparable_competing_candidate() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let decision = acquisition_decision(3.5, 2.0, 2.0, 1.4, &thresholds);
    assert_eq!(decision.hypothesis.to_string(), "ambiguous");
    assert_eq!(decision.reason, AcquisitionDecisionReason::AmbiguousRatioThresholds);
    assert!((decision.score - 1.435).abs() < 1e-6);
}

#[test]
fn acquisition_decision_accepts_clean_peak() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let decision = acquisition_decision(3.5, 2.0, 2.5, 2.1, &thresholds);
    assert_eq!(decision.hypothesis.to_string(), "accepted");
    assert_eq!(decision.reason, AcquisitionDecisionReason::AcceptedByRatioThresholds);
    assert!((decision.score - 2.75).abs() < f32::EPSILON);
}

#[test]
fn glonass_request_requires_frequency_channel() {
    let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 511_000.0,
        intermediate_freq_hz: 125_000.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let error = acquisition_signal_model_for_request(
        &config,
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
    )
    .expect_err("GLONASS requests must declare one FDMA channel");

    assert!(
        matches!(error, SignalError::MissingGlonassFrequencyChannel(error_sat) if error_sat == sat)
    );
}

#[test]
fn unsupported_registry_signal_request_returns_explicit_error() {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 511_500.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 511_500.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let error = acquisition_signal_model_for_request(
        &config,
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L2,
            signal_code: SignalCode::L2C,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
    )
    .expect_err("GPS L2C acquisition must report unsupported search implementation");

    assert_eq!(
        error,
        SignalError::UnsupportedSignalDefinition {
            constellation: Constellation::Gps,
            signal_band: SignalBand::L2,
            signal_code: SignalCode::L2C,
        }
    );
}

#[test]
fn glonass_request_uses_glonass_l1_search_model() {
    let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 511_000.0,
        intermediate_freq_hz: 125_000.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let model = acquisition_signal_model_for_request(
        &config,
        AcqRequest {
            sat,
            glonass_frequency_channel: Some(channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
    )
    .expect("GLONASS request model");
    let expected_center_hz = config.intermediate_freq_hz
        + (glonass_l1_carrier_hz(channel).value() - GPS_L1_CA_CARRIER_HZ.value());
    let sampled_code =
        model.sampled_local_code_period(config.sampling_freq_hz, 511).expect("GLONASS local code");
    let expected_code =
        sample_glonass_l1_st_code(config.sampling_freq_hz, 0.0, 511).expect("GLONASS code");

    assert_eq!(model.signal_band, SignalBand::L1);
    assert_eq!(model.code_rate_hz, 511_000.0);
    assert_eq!(model.code_length, 511);
    assert_eq!(model.code_period_ms, 1);
    assert!(
        (model.search_center_hz(config.intermediate_freq_hz) - expected_center_hz).abs()
            <= f64::EPSILON
    );
    assert_eq!(sampled_code, expected_code);
}

#[test]
fn glonass_request_skips_secondary_peak_multipath_screening() {
    let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 511_000.0,
        intermediate_freq_hz: 125_000.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let model = acquisition_signal_model_for_request(
        &config,
        AcqRequest {
            sat,
            glonass_frequency_channel: Some(channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 2_000,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
    )
    .expect("GLONASS request model");

    assert!(!model.supports_secondary_peak_multipath_screening());
}

#[test]
fn zero_signal_run_preserves_unsupported_request_error() {
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 511_500.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 511_500.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L2,
        signal_code: SignalCode::L2C,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = zero_signal_run(
        &config,
        &[request],
        ReceiverSampleTrace::from_sample_time(SampleTime {
            sample_index: 0,
            sample_rate_hz: config.sampling_freq_hz,
        }),
        4092,
        Some("zeroed_fixture"),
        true,
    );

    assert_eq!(run.results.len(), 1);
    assert_eq!(run.results[0].len(), 1);
    let candidate = &run.results[0][0];
    assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(
        candidate.explain_selection_reason.as_deref(),
        Some("invalid_acquisition_signal_model: unsupported signal definition for Gps L2 L2C")
    );
    assert_eq!(candidate.carrier_hz.0, config.intermediate_freq_hz);
    assert_eq!(run.explains.len(), 1);
    assert_eq!(run.explains[0].selected_reason, "invalid_acquisition_signal_model");
}

#[test]
fn zero_signal_run_preserves_explicit_gps_l5q_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = zero_signal_run(
        &config,
        &[request],
        ReceiverSampleTrace::from_sample_time(SampleTime {
            sample_index: 0,
            sample_rate_hz: config.sampling_freq_hz,
        }),
        10_230,
        Some("zeroed_fixture"),
        true,
    );

    let candidate = &run.results[0][0];
    assert_eq!(candidate.signal_band, SignalBand::L5);
    assert_eq!(candidate.signal_code, SignalCode::L5Q);
    assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Rejected.to_string());
    assert_eq!(run.explains[0].selected_reason, "zero_signal_input");
}

#[test]
fn zero_signal_run_preserves_explicit_galileo_e5b_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5b,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = zero_signal_run(
        &config,
        &[request],
        ReceiverSampleTrace::from_sample_time(SampleTime {
            sample_index: 0,
            sample_rate_hz: config.sampling_freq_hz,
        }),
        10_230,
        Some("zeroed_fixture"),
        true,
    );

    let candidate = &run.results[0][0];
    assert_eq!(candidate.signal_band, SignalBand::E5);
    assert_eq!(candidate.signal_code, SignalCode::E5b);
    assert_eq!(candidate.hypothesis.to_string(), AcqHypothesis::Rejected.to_string());
    assert_eq!(run.explains[0].selected_reason, "zero_signal_input");
}

#[test]
fn competing_candidate_ratio_uses_top_two_peak_mean_ratios() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let ratio = competing_candidate_ratio(&[
        candidate_for_search_window_test(sat, 0.0, 4.0),
        candidate_for_search_window_test(sat, 250.0, 3.0),
        candidate_for_search_window_test(sat, 500.0, 1.5),
    ]);

    assert!((ratio - (4.0 / 3.0)).abs() < 1.0e-6);
}

#[test]
fn acquisition_decision_accepts_absent_competing_candidate_with_strong_local_peak() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let decision = acquisition_decision(12.0, 1.8, 1.8, f32::INFINITY, &thresholds);

    assert_eq!(decision.hypothesis.to_string(), "accepted");
    assert_eq!(decision.reason, AcquisitionDecisionReason::AcceptedByRatioThresholds);
}

#[test]
fn selected_candidate_reason_reports_low_peak_metric_threshold() {
    let config = ReceiverPipelineConfig::default();
    let thresholds = resolved_thresholds(&config);
    let reason = selected_candidate_reason(
        AcquisitionDecision {
            hypothesis: AcqHypothesis::Rejected,
            reason: AcquisitionDecisionReason::LowPeakMetric,
            score: 0.0,
        },
        2.0,
        2.0,
        2.0,
        &thresholds,
    );

    assert_eq!(reason, "low_peak_metric: peak_mean_ratio=2.000000 below threshold 2.500000");
}

#[test]
fn calibrated_thresholds_record_false_alarm_provenance() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    config.acquisition_threshold_policy.mode = AcquisitionThresholdMode::CalibratedFalseAlarm;
    config.acquisition_threshold_policy.false_alarm_probability = 0.05;
    config.acquisition_threshold_policy.calibration_trial_count = 12;
    config.acquisition_threshold_policy.confidence_level = 0.95;

    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 217.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        0x6A11_E177,
        0.005,
    );
    let acquisition = Acquisition::new(config.clone(), ReceiverRuntime::default());
    let result = acquisition.run_fft(&frame, &[sat]).remove(0);
    let provenance = result
        .threshold_provenance
        .as_ref()
        .expect("acquisition result should carry threshold provenance");

    assert_eq!(provenance.mode, "calibrated_false_alarm");
    assert_eq!(
        provenance.false_alarm_probability,
        Some(config.acquisition_threshold_policy.false_alarm_probability)
    );
    assert_eq!(
        provenance.calibration_trial_count,
        Some(config.acquisition_threshold_policy.calibration_trial_count)
    );
    assert_eq!(
        provenance.calibration_confidence_level,
        Some(config.acquisition_threshold_policy.confidence_level)
    );
    assert!(provenance.peak_mean_threshold > 1.0);
    assert_eq!(provenance.peak_second_threshold, config.acquisition_peak_second_threshold);
    let measured_rate = provenance
        .calibration_false_alarm_rate
        .expect("calibrated threshold provenance should record the measured rate");
    let interval_low = provenance
        .calibration_false_alarm_interval_low
        .expect("calibrated threshold provenance should record the confidence interval low");
    let interval_high = provenance
        .calibration_false_alarm_interval_high
        .expect("calibrated threshold provenance should record the confidence interval high");
    assert!((0.0..=1.0).contains(&measured_rate));
    assert!(interval_low <= measured_rate);
    assert!(measured_rate <= interval_high);
}

#[test]
fn selected_reason_for_candidate_reports_low_peak_metric() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let mut candidate = candidate_for_search_window_test(sat, 0.0, 2.0);
    candidate.hypothesis = AcqHypothesis::Rejected;

    assert_eq!(selected_reason_for_candidate(&candidate), "low_peak_metric");
}

#[test]
fn selected_reason_for_candidate_reports_wrong_prn_correlation() {
    let sat = SatId { constellation: Constellation::Gps, prn: 8 };
    let mut candidate = candidate_for_search_window_test(sat, 0.0, 3.0);
    candidate.hypothesis = AcqHypothesis::Rejected;
    candidate.explain_selection_reason =
        Some("wrong_prn_correlation: prn 8 suppressed by stronger prn 7".to_string());

    assert_eq!(selected_reason_for_candidate(&candidate), "wrong_prn_correlation");
}

#[test]
fn selected_reason_for_candidate_reports_multipath_suspect() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut candidate = candidate_for_search_window_test(sat, 0.0, 3.0);
    candidate.hypothesis = AcqHypothesis::Ambiguous;
    candidate.explain_selection_reason = Some(
        "multipath_suspect: delayed secondary peak at sample 1440 (+240 samples, 60.000 chips)"
            .to_string(),
    );

    assert_eq!(selected_reason_for_candidate(&candidate), "multipath_suspect");
}

#[test]
fn suppress_wrong_prn_correlations_rejects_ambiguous_cross_prn_candidates() {
    let dominant_sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let wrong_sat = SatId { constellation: Constellation::Gps, prn: 8 };
    let mut sat_evaluations = vec![
        AcquisitionSatEvaluation {
            sat: dominant_sat,
            candidates: vec![AcqResult {
                hypothesis: AcqHypothesis::Ambiguous,
                peak_mean_ratio: 18.6,
                peak_second_ratio: 1.25,
                explain_selection_reason: Some("ambiguous_ratio_thresholds".to_string()),
                ..candidate_for_search_window_test(dominant_sat, 0.0, 18.6)
            }],
            search_window_diagnostic: None,
        },
        AcquisitionSatEvaluation {
            sat: wrong_sat,
            candidates: vec![AcqResult {
                hypothesis: AcqHypothesis::Ambiguous,
                peak_mean_ratio: 3.1,
                peak_second_ratio: 1.02,
                explain_selection_reason: Some("ambiguous_ratio_thresholds".to_string()),
                ..candidate_for_search_window_test(wrong_sat, 0.0, 3.1)
            }],
            search_window_diagnostic: None,
        },
    ];

    suppress_wrong_prn_correlations(&mut sat_evaluations);

    let suppressed = sat_evaluations[1].candidates.first().expect("suppressed candidate");
    assert_eq!(suppressed.hypothesis.to_string(), "rejected");
    assert_eq!(selected_reason_for_candidate(suppressed), "wrong_prn_correlation");
    assert_eq!(suppressed.score, 0.0);
}
