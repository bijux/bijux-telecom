use super::*;
use crate::engine::receiver_config::AcquisitionThresholdMode;
use crate::engine::runtime::ReceiverRuntime;
use crate::sim::synthetic::{
    generate_l1_ca, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
};
use bijux_gnss_core::api::{
    AcqAssistanceBounds, AcqComponentCombinationMode, AcqComponentProvenance,
    AcqComponentStatistic, Constellation, GlonassFrequencyChannel, ReceiverSampleTrace, SampleTime,
    SamplesFrame, SatId, Seconds, SignalBand, SignalComponentRole, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_signal::api::{
    glonass_l1_carrier_hz, sample_glonass_l1_st_code, samples_per_code, shared_path_doppler_hz,
    signal_spec_gps_l1_ca, signal_spec_gps_l5_i, SignalSource,
};

#[path = "tests/coherent_accumulation.rs"]
mod coherent_accumulation;

fn acquisition_component_plan_for_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    coherent_ms: u32,
) -> AcquisitionComponentPlan {
    acquisition_strategies_for_signal(sat, signal_band, signal_code, None, coherent_ms)
        .expect("acquisition strategies")
        .into_iter()
        .next()
        .and_then(|strategy| strategy.components.into_iter().next())
        .expect("primary acquisition component")
}

fn alternating_frame(sample_rate_hz: f64, sample_count: usize) -> SamplesFrame {
    SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz },
        Seconds(1.0 / sample_rate_hz),
        (0..sample_count)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    )
}

fn signal_only_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    frame_len: usize,
) -> SamplesFrame {
    let mut source = SyntheticSignalSource::new_signal_only(config, scenario);
    source.next_frame(frame_len).expect("signal-only frame").expect("acquisition frame")
}

fn resolved_thresholds(config: &ReceiverPipelineConfig) -> ResolvedAcquisitionThresholds {
    ResolvedAcquisitionThresholds {
        peak_mean_threshold: config.acquisition_peak_mean_threshold,
        peak_second_threshold: config.acquisition_peak_second_threshold,
        provenance: AcqThresholdProvenance {
            mode: "fixed_ratio".to_string(),
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
            doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
            peak_mean_threshold: config.acquisition_peak_mean_threshold,
            peak_second_threshold: config.acquisition_peak_second_threshold,
            false_alarm_probability: None,
            calibration_trial_count: None,
            calibration_confidence_level: None,
            calibration_false_alarm_rate: None,
            calibration_false_alarm_interval_low: None,
            calibration_false_alarm_interval_high: None,
        },
    }
}

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

#[test]
fn parabolic_refinement_estimates_sub_bin_offset_from_neighbor_peaks() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        candidate_for_search_window_test(sat, -250.0, 9.0),
        candidate_for_search_window_test(sat, 0.0, 16.0),
        candidate_for_search_window_test(sat, 250.0, 12.0),
    ];

    let refinement =
        estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250).expect("refinement");

    assert_eq!(refinement.method, "parabolic_peak");
    assert_eq!(refinement.coarse_carrier_hz.0, 0.0);
    assert!((refinement.offset_bins - 0.136_363_636_4).abs() < 1.0e-6);
    assert!((refinement.offset_hz - 34.090_909_1).abs() < 1.0e-6);
}

#[test]
fn parabolic_refinement_skips_search_edge_candidates() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        candidate_for_search_window_test(sat, 0.0, 16.0),
        candidate_for_search_window_test(sat, 250.0, 12.0),
    ];

    let refinement = estimate_acquisition_doppler_refinement(0.0, 0.0, &candidates, 250);

    assert!(refinement.is_none());
}

#[test]
fn quadratic_surface_refinement_estimates_joint_peak_offsets() {
    let surface = LocalAcquisitionLikelihoodSurface {
        doppler_cross_section: [7.84, 10.0, 9.16],
        code_phase_cross_section: [9.38, 10.0, 8.62],
        values: [[7.62, 7.84, 6.06], [9.38, 10.0, 8.62], [8.14, 9.16, 8.18]],
    };

    let (doppler_offset_bins, code_phase_offset_samples) =
        estimate_quadratic_surface_peak_offsets(&surface).expect("joint refinement");

    assert!((doppler_offset_bins - 0.2).abs() < 1.0e-6);
    assert!((code_phase_offset_samples + 0.15).abs() < 1.0e-6);
}

#[test]
fn quadratic_surface_refinement_rejects_non_maximum_center() {
    let surface = LocalAcquisitionLikelihoodSurface {
        doppler_cross_section: [11.0, 10.0, 8.0],
        code_phase_cross_section: [8.0, 10.0, 7.0],
        values: [[7.0, 11.0, 6.5], [8.0, 10.0, 7.0], [6.0, 8.0, 5.5]],
    };

    assert!(estimate_quadratic_surface_peak_offsets(&surface).is_none());
}

#[test]
fn log_likelihood_covariance_tightens_for_sharper_surfaces() {
    let broad = estimate_log_likelihood_covariance_2x2(
        &LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [14.0, 16.0, 15.0],
            code_phase_cross_section: [14.0, 16.0, 15.0],
            values: [[13.0, 14.0, 13.5], [14.0, 16.0, 15.0], [13.5, 15.0, 14.0]],
        },
        1.0,
        250.0,
        1.0,
    )
    .expect("broad covariance");
    let sharp = estimate_log_likelihood_covariance_2x2(
        &LocalAcquisitionLikelihoodSurface {
            doppler_cross_section: [9.0, 16.0, 12.0],
            code_phase_cross_section: [9.0, 16.0, 12.0],
            values: [[8.0, 9.0, 8.5], [9.0, 16.0, 12.0], [8.5, 12.0, 10.0]],
        },
        1.0,
        250.0,
        1.0,
    )
    .expect("sharp covariance");

    assert!(sharp.doppler_variance_hz2 < broad.doppler_variance_hz2, "{sharp:?} {broad:?}");
    assert!(
        sharp.code_phase_variance_samples2 < broad.code_phase_variance_samples2,
        "{sharp:?} {broad:?}"
    );
}

#[test]
fn log_likelihood_covariance_reports_rate_variance_when_rate_axis_is_available() {
    let covariance = estimate_log_likelihood_covariance_3x3(
        &LocalAcquisitionLikelihoodVolume {
            values: [
                [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
                [[4.0, 6.0, 4.0], [8.0, 16.0, 8.0], [4.0, 6.0, 4.0]],
                [[3.0, 4.0, 3.0], [5.0, 7.0, 5.0], [3.0, 4.0, 3.0]],
            ],
        },
        1.0,
        250.0,
        1.0,
        5_000.0,
    )
    .expect("rate-aware covariance");

    assert!(covariance.doppler_variance_hz2 > 0.0);
    assert!(covariance.code_phase_variance_samples2 > 0.0);
    assert!(
        covariance.doppler_rate_variance_hz2_per_s2.is_some_and(|variance| variance > 0.0),
        "{covariance:?}"
    );
}

#[test]
fn estimate_acquisition_uncertainty_skips_ambiguous_candidate() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let config = ReceiverPipelineConfig::default();
    let signal_model =
        AcquisitionSignalModel::for_sat_signal(sat, Some(SignalBand::L1), SignalCode::Ca, None)
            .expect("signal model lookup")
            .expect("gps l1ca signal model");
    let frame = noise_only_frame(
        config.sampling_freq_hz,
        signal_model.samples_per_code(config.sampling_freq_hz),
        0xA11CE,
    );
    let uncertainty = estimate_acquisition_uncertainty(
        &config,
        &frame,
        &signal_model,
        &AcqResult {
            hypothesis: AcqHypothesis::Ambiguous,
            ..candidate_for_search_window_test(sat, 0.0, 4.0)
        },
        1,
        1,
        250,
        0,
        250,
    );

    assert!(uncertainty.is_none());
}

#[test]
fn code_phase_refinement_estimates_sub_sample_offset_from_neighbor_bins() {
    let profile = [12.0, 16.0, 15.0];
    let (offset_samples, left, center, right) =
        estimate_parabolic_code_phase_offset_samples(&profile, 1).expect("refinement");

    assert!((offset_samples - 0.3).abs() < 1.0e-6);
    assert_eq!(left, 12.0);
    assert_eq!(center, 16.0);
    assert_eq!(right, 15.0);
}

#[test]
fn code_phase_refinement_wraps_negative_offsets_at_period_edges() {
    let offset_samples = -0.2;
    let refined_code_phase_samples = wrap_acquisition_code_phase_samples(0.0 + offset_samples, 4);

    assert!(offset_samples < 0.0);
    assert!((refined_code_phase_samples - 3.8).abs() < 1.0e-6);
}

#[test]
fn correlation_metrics_report_primary_and_secondary_peak_indices() {
    let metrics = correlation_metrics(&[1.0, 4.0, 2.0, 3.5, 1.5]);

    assert_eq!(metrics.peak_idx, 1);
    assert_eq!(metrics.second_idx, 3);
    assert_eq!(metrics.peak, 4.0);
    assert_eq!(metrics.second, 3.5);
    assert!((metrics.mean - 2.4).abs() < 1.0e-6);
}

#[test]
fn delayed_secondary_peak_diagnostic_ignores_main_lobe_neighbors() {
    let diagnostic =
        delayed_secondary_peak_diagnostic(&[1.0, 8.0, 7.6, 7.2, 1.4, 5.5, 1.2, 0.8], 1, 8, 4)
            .expect("delayed secondary peak");

    assert_eq!(diagnostic.secondary_code_phase_samples, 5);
    assert_eq!(diagnostic.delay_samples, 4);
}

#[test]
fn delayed_secondary_peak_diagnostic_skips_short_wraparound_alias() {
    let diagnostic =
        delayed_secondary_peak_diagnostic(&[6.0, 1.0, 0.8, 0.9, 1.1, 1.3, 1.2, 5.4], 7, 8, 4);

    assert!(diagnostic.is_none());
}

#[test]
fn multipath_candidate_reason_reports_delay_context() {
    let reason = multipath_candidate_reason(
        5.2,
        1.3,
        3.4,
        &DelayedSecondaryPeakDiagnostic { secondary_code_phase_samples: 1440, delay_samples: 240 },
        4092,
        1023,
        1.92,
    );

    assert!(reason.starts_with("multipath_suspect: delayed secondary peak"));
    assert!(reason.contains("+240 samples, 60.000 chips"));
    assert!(reason.contains("peak_second_ratio=1.300000"));
}

#[test]
fn acquisition_rejects_unsupported_coherent_integration_lengths() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..16)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_with_explain(&frame, &[sat], 1, 3, 1);

    let result = run.results.first().and_then(|rows| rows.first()).expect("result");
    let explain = run.explains.first().expect("explain");
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(explain.selected_reason, "unsupported_coherent_integration_ms");
    assert_eq!(
        result.explain_selection_reason.as_deref(),
        Some(
            "unsupported_coherent_integration_ms: acquisition coherent integration must be one of [1, 2, 5, 10, 20] ms but received 3 ms"
        )
    );
}

#[test]
fn unsupported_coherent_integration_preserves_explicit_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..32)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat,
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
        coherent_ms: 3,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("deferred result");
    assert_eq!(result.signal_band, SignalBand::L5);
    assert_eq!(result.signal_code, SignalCode::L5Q);
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(run.explains[0].selected_reason, "unsupported_coherent_integration_ms");
}

#[test]
fn code_fft_cache_reuses_local_code_across_integration_profiles() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let signal_model = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L1,
        SignalCode::Unknown,
        None,
    );
    let component = acquisition_component_plan_for_signal(sat, SignalBand::L1, SignalCode::Ca, 1);

    acquisition.code_fft(
        &signal_model,
        &component,
        sat,
        SignalCode::Ca,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_first_profile = acquisition.stats_snapshot();
    assert_eq!(after_first_profile.cache_misses, 1);
    assert_eq!(after_first_profile.cache_hits, 0);

    acquisition.code_fft(
        &signal_model,
        &component,
        sat,
        SignalCode::Ca,
        samples_per_code,
        1,
        4,
        fft.as_ref(),
    );
    let after_second_profile = acquisition.stats_snapshot();
    assert_eq!(after_second_profile.cache_misses, 1);
    assert_eq!(after_second_profile.cache_hits, 1);
    assert_eq!(after_second_profile.cache_miss_incompatible, 0);
}

#[test]
fn code_fft_cache_separates_gps_l5_signal_codes() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let samples_per_code = 10_230;
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(samples_per_code);
    let gps_l5_i = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let gps_l5_q = acquisition_signal_model_for_sat(
        &acquisition.config,
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        None,
    );
    let gps_l5_i_component =
        acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5I, 1);
    let gps_l5_q_component =
        acquisition_component_plan_for_signal(sat, SignalBand::L5, SignalCode::L5Q, 1);

    acquisition.code_fft(
        &gps_l5_i,
        &gps_l5_i_component,
        sat,
        SignalCode::L5I,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_i = acquisition.stats_snapshot();
    assert_eq!(after_l5_i.cache_misses, 1);
    assert_eq!(after_l5_i.cache_hits, 0);

    acquisition.code_fft(
        &gps_l5_q,
        &gps_l5_q_component,
        sat,
        SignalCode::L5Q,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_q = acquisition.stats_snapshot();
    assert_eq!(after_l5_q.cache_misses, 2);
    assert_eq!(after_l5_q.cache_hits, 0);
    assert_eq!(after_l5_q.cache_miss_incompatible, 1);

    acquisition.code_fft(
        &gps_l5_q,
        &gps_l5_q_component,
        sat,
        SignalCode::L5Q,
        samples_per_code,
        1,
        1,
        fft.as_ref(),
    );
    let after_l5_q_reuse = acquisition.stats_snapshot();
    assert_eq!(after_l5_q_reuse.cache_misses, 2);
    assert_eq!(after_l5_q_reuse.cache_hits, 1);
}

#[test]
fn run_fft_for_requests_preserves_explicit_gps_l5q_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..10_230)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
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
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("acquisition result");
    assert_eq!(result.signal_band, SignalBand::L5);
    assert_eq!(result.signal_code, SignalCode::L5Q);
}

#[test]
fn centered_doppler_requests_report_absolute_doppler_and_assumptions() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 1,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let true_doppler_hz = 750.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 147.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        },
        0xC3E1_7EAD,
        0.020,
    );
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_center_hz: true_doppler_hz,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: Some(true_doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let result = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_for_requests(&frame, &[request])
        .remove(0);

    assert!(
        matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{result:?}"
    );
    assert!((result.doppler_hz.0 - true_doppler_hz).abs() <= 1.0e-6, "{result:?}");
    let assumptions = result
        .assumptions
        .as_ref()
        .expect("centered acquisition result should preserve assumptions");
    assert_eq!(assumptions.doppler_center_hz, true_doppler_hz);
    assert_eq!(assumptions.expected_line_of_sight_doppler_hz, Some(true_doppler_hz));
}

#[test]
fn assisted_bounds_reduce_reported_search_domain() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let true_doppler_hz = 0.0;
    let code_phase_chips = 147.25;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: true_doppler_hz,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        },
        0xB01D_5E47,
        0.020,
    );
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_center_hz: true_doppler_hz,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: Some(true_doppler_hz),
        assistance_bounds: Some(AcqAssistanceBounds {
            expected_code_phase_samples: code_phase_chips * config.sampling_freq_hz
                / config.code_freq_basis_hz,
            time_uncertainty_s: 0.5e-6,
            position_uncertainty_m: 75.0,
            oscillator_uncertainty_hz: 120.0,
            approximate_velocity_uncertainty_mps: 6.0,
        }),
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let result = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_for_requests(&frame, &[request])
        .remove(0);

    assert!(
        matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{result:?}"
    );
    let assumptions = result
        .assumptions
        .as_ref()
        .expect("assisted acquisition result should preserve assumptions");
    assert_eq!(assumptions.assistance_bounds, request.assistance_bounds);
    assert!(assumptions.doppler_search_hz < request.doppler_search_hz, "{assumptions:?}");
    assert!(assumptions.code_phase_search_bins < assumptions.samples_per_code, "{assumptions:?}");
    assert_eq!(assumptions.code_phase_search_mode, "assisted_bounds");
}

#[test]
fn wrong_assisted_code_phase_bounds_retry_full_search() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 10 };
    let code_phase_chips = 147.25;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        },
        0xFA11_BAC4,
        0.020,
    );
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: Some(0.0),
        assistance_bounds: Some(AcqAssistanceBounds {
            expected_code_phase_samples: 32.0,
            time_uncertainty_s: 0.0,
            position_uncertainty_m: 0.0,
            oscillator_uncertainty_hz: 120.0,
            approximate_velocity_uncertainty_mps: 6.0,
        }),
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let result = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_for_requests(&frame, &[request])
        .remove(0);

    assert!(
        matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{result:?}"
    );
    let assumptions = result
        .assumptions
        .as_ref()
        .expect("fallback acquisition result should preserve assumptions");
    assert_eq!(assumptions.code_phase_search_mode, "full_code");
    assert_eq!(assumptions.assistance_bounds, None);
    assert!(
        result
            .explain_selection_reason
            .as_deref()
            .is_some_and(|reason| reason.contains("assistance_bounds_fallback")),
        "{result:?}"
    );
}

#[test]
fn run_fft_for_requests_preserves_explicit_galileo_e5b_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..10_230)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
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
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("acquisition result");
    assert_eq!(result.signal_band, SignalBand::E5);
    assert_eq!(result.signal_code, SignalCode::E5b);
}

#[test]
fn run_fft_for_requests_keeps_galileo_e5_cache_entries_separate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..10_230)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let e5a_request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5a,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };
    let e5b_request = AcqRequest { signal_code: SignalCode::E5b, ..e5a_request };

    let e5a_run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[e5a_request], 1);
    let after_e5a = acquisition.stats_snapshot();
    assert_eq!(e5a_run.results[0][0].signal_code, SignalCode::E5a);
    assert_eq!(after_e5a.cache_misses, 2);
    assert_eq!(after_e5a.cache_hits, 0);
    assert_eq!(after_e5a.cache_miss_incompatible, 1);

    let e5b_run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[e5b_request], 1);
    let after_e5b = acquisition.stats_snapshot();
    assert_eq!(e5b_run.results[0][0].signal_code, SignalCode::E5b);
    assert_eq!(after_e5b.cache_misses, 4);
    assert_eq!(after_e5b.cache_hits, 0);
    assert_eq!(after_e5b.cache_miss_incompatible, 3);
}

#[test]
fn run_fft_for_galileo_e5a_records_component_combination_provenance() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = alternating_frame(config.sampling_freq_hz, 10_230);
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5a,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
    let candidates = &run.results[0];

    assert_eq!(candidates.len(), 4);
    assert!(candidates.iter().all(|candidate| candidate.signal_code == SignalCode::E5a));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Data]
        })
    }));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::SingleComponent
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Pilot]
        })
    }));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::NoncoherentComponentSum
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        })
    }));
    assert!(candidates.iter().any(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode == AcqComponentCombinationMode::CoherentComponentSum
                && provenance.components.iter().map(|component| component.role).collect::<Vec<_>>()
                    == vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        })
    }));
}

#[test]
fn run_fft_for_galileo_e5a_skips_coherent_component_sum_beyond_one_millisecond() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = alternating_frame(config.sampling_freq_hz, 20_460);
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5a,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 2,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
    let candidates = &run.results[0];

    assert_eq!(candidates.len(), 3);
    assert!(candidates.iter().all(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode != AcqComponentCombinationMode::CoherentComponentSum
        })
    }));
}

#[test]
fn run_fft_for_gps_l5q_records_single_pilot_component_provenance() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = alternating_frame(config.sampling_freq_hz, 10_230);
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
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
        doppler_search_hz: 0,
        doppler_step_hz: 1,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);
    let provenance = run.results[0][0].component_provenance().expect("component provenance");

    assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
    assert_eq!(
        provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
}

#[test]
fn acquisition_stability_keys_are_sorted() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let mut rows = vec![
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(100.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(100.0),
            code_phase_samples: 10,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(50.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(50.0),
            code_phase_samples: 20,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    ];
    rows.sort_by(|a, b| {
        let primary =
            b.peak_mean_ratio.partial_cmp(&a.peak_mean_ratio).unwrap_or(std::cmp::Ordering::Equal);
        if primary == std::cmp::Ordering::Equal {
            return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
        }
        primary
    });
    let keys = stable_acq_result_keys(&rows);
    assert!(keys.windows(2).all(|window| window[0] <= window[1]));
}

#[test]
fn search_window_diagnostic_detects_rising_upper_edge_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.5),
            candidate_for_search_window_test(sat, -1_250.0, 1.7),
            candidate_for_search_window_test(sat, 1_250.0, 2.4),
            candidate_for_search_window_test(sat, 1_500.0, 2.9),
        ],
        0.0,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(diagnostic.best_axis_value, 1_500.0);
    assert_eq!(diagnostic.interior_axis_value, 1_250.0);
}

#[test]
fn search_window_diagnostic_ignores_interior_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.4),
            candidate_for_search_window_test(sat, -1_250.0, 1.8),
            candidate_for_search_window_test(sat, 0.0, 3.2),
            candidate_for_search_window_test(sat, 1_500.0, 2.7),
        ],
        0.0,
        1_500,
        250,
        2.5,
    );

    assert!(diagnostic.is_none());
}

#[test]
fn search_window_diagnostic_prefers_edge_signal_over_interior_ambiguity() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 3.5),
            candidate_for_search_window_test(sat, -1_250.0, 1.6),
            candidate_for_search_window_test(sat, 0.0, 3.6),
            candidate_for_search_window_test(sat, 1_500.0, 1.9),
        ],
        0.0,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Lower);
    assert_eq!(diagnostic.best_axis_value, -1_500.0);
    assert_eq!(diagnostic.interior_axis_value, -1_250.0);
}

#[test]
fn search_window_diagnostic_ignores_weak_edge_peak() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(sat, -1_500.0, 1.1),
            candidate_for_search_window_test(sat, -1_250.0, 1.0),
            candidate_for_search_window_test(sat, 1_250.0, 1.8),
            candidate_for_search_window_test(sat, 1_500.0, 2.0),
        ],
        0.0,
        1_500,
        250,
        2.5,
    );

    assert!(diagnostic.is_none());
}

#[test]
fn search_window_diagnostic_respects_nonzero_intermediate_frequency() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let intermediate_freq_hz = 1_250_000.0;
    let diagnostic = signal_outside_search_range(
        &[
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_500.0),
                1.5,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, -1_250.0),
                1.7,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0),
                2.4,
            ),
            candidate_for_search_window_test(
                sat,
                carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0),
                2.9,
            ),
        ],
        intermediate_freq_hz,
        1_500,
        250,
        2.5,
    )
    .expect("search-window diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::Doppler);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(
        diagnostic.best_axis_value,
        carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_500.0)
    );
    assert_eq!(
        diagnostic.interior_axis_value,
        carrier_hz_from_doppler_hz(intermediate_freq_hz, 1_250.0)
    );
}

#[test]
fn run_fft_returns_deferred_result_for_each_satellite_when_frame_is_too_short() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_integration_ms: 2,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..samples_per_code)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let sats = vec![
        SatId { constellation: Constellation::Gps, prn: 3 },
        SatId { constellation: Constellation::Gps, prn: 7 },
    ];
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());

    let run = acquisition.run_fft_topn_with_explain(&frame, &sats, 1, 2, 1);

    assert_eq!(run.results.len(), sats.len());
    assert_eq!(run.explains.len(), sats.len());
    for (idx, sat) in sats.iter().enumerate() {
        let result = run.results[idx].first().expect("placeholder result");
        assert_eq!(result.sat, *sat);
        assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
        assert_eq!(result.code_phase_samples, 0);
        assert_eq!(result.peak_mean_ratio, 0.0);
        assert_eq!(
            result.explain_selection_reason.as_deref(),
            Some("insufficient_frame: acquisition requires 8184 samples but received 4092"),
        );

        let explain = &run.explains[idx];
        assert_eq!(explain.sat, *sat);
        assert_eq!(explain.selected_reason, "insufficient_frame");
        assert_eq!(explain.candidate_count, 1);
    }
}

#[test]
fn insufficient_frame_preserves_explicit_signal_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        (0..1023)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
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

    let run = acquisition.run_fft_topn_for_requests_with_explain(&frame, &[request], 1);

    let result = run.results[0].first().expect("deferred result");
    assert_eq!(result.signal_band, SignalBand::E5);
    assert_eq!(result.signal_code, SignalCode::E5b);
    assert_eq!(result.hypothesis.to_string(), AcqHypothesis::Deferred.to_string());
    assert_eq!(run.explains[0].selected_reason, "insufficient_frame");
}

#[test]
fn galileo_e1_cboc_profile_reports_side_peak_geometry() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::E1,
            signal_code: SignalCode::E1B,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0x6A11_E175,
        0.020,
    );
    let signal_model =
        acquisition_signal_model_for_sat(&config, sat, SignalBand::E1, SignalCode::E1B, None);
    let profile = measure_code_phase_profile(
        &config,
        &signal_model,
        &frame,
        sat,
        signal_model.search_center_hz(config.intermediate_freq_hz),
        0.0,
        config.acquisition_integration_ms,
        config.acquisition_noncoherent,
    )
    .expect("Galileo E1 correlation profile");
    let metrics = correlation_metrics(&profile);
    let diagnostic = delayed_secondary_peak_diagnostic(
        &profile,
        metrics.peak_idx,
        signal_model.samples_per_code(config.sampling_freq_hz),
        signal_model.code_length,
    );
    let delayed_peak = diagnostic
        .as_ref()
        .map(|value| profile[value.secondary_code_phase_samples])
        .unwrap_or_default();
    let delayed_peak_mean_ratio = delayed_peak / (metrics.mean + 1.0e-6);

    assert!(metrics.peak > delayed_peak);
    assert!(diagnostic.is_some(), "{metrics:?}");
    assert!(
        delayed_peak_mean_ratio > config.acquisition_peak_mean_threshold,
        "delayed local peak should still look acquisition-worthy: {delayed_peak_mean_ratio}"
    );
}

#[test]
fn galileo_e1_strategy_candidates_prefer_pilot_cboc_acquisition() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::E1,
            signal_code: SignalCode::E1B,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0x6A11_E176,
        0.020,
    );
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1B,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    };
    let run = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
    let selected = run.results[0].first().expect("selected Galileo E1 candidate");
    let provenance =
        selected.component_provenance().expect("selected Galileo E1 component provenance");

    assert_eq!(selected.hypothesis.to_string(), "accepted", "{run:?}");
    assert_eq!(run.explains[0].selected_reason, "accepted_by_ratio_thresholds", "{run:?}");
    assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
    assert_eq!(
        provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
    assert_eq!(provenance.components[0].secondary_code_phase_periods, Some(0), "{run:?}");
    let code_phase_uncertainty_samples = selected
        .uncertainty
        .as_ref()
        .expect("Galileo E1 accepted candidate uncertainty")
        .code_phase_samples;
    assert!(
        code_phase_uncertainty_samples > 0.0 && code_phase_uncertainty_samples < 1.0,
        "expected sub-chip Galileo E1 code-phase uncertainty, got {code_phase_uncertainty_samples}: {run:?}",
    );
}

#[test]
fn competing_candidate_ratio_ignores_strategy_variants_of_same_hypothesis() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let best = candidate_for_search_window_test(sat, 0.0, 12.0);
    let same_hypothesis_variant = candidate_for_search_window_test(sat, 0.0, 10.0);
    let competing = candidate_for_search_window_test(sat, 500.0, 4.0);

    assert!(
        (competing_candidate_ratio(&[best, same_hypothesis_variant, competing]) - 3.0).abs()
            <= f32::EPSILON
    );
}

#[test]
fn galileo_e1_signal_only_streaming_frame_returns_explicit_ambiguity() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x6A11_E100,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "galileo-e1-streaming-candidate-shape".to_string(),
    };
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let frame = source
        .next_frame(81_840)
        .expect("signal-only frame")
        .expect("signal-only acquisition frame");
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1B,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    };
    let run = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_topn_for_requests_with_explain(&frame, &[request], 4);
    let selected = run.results[0].first().expect("selected Galileo E1 signal-only candidate");
    let provenance = selected
        .component_provenance()
        .expect("selected Galileo E1 signal-only component provenance");

    assert_eq!(selected.hypothesis.to_string(), "ambiguous", "{run:?}");
    assert_eq!(run.explains[0].selected_reason, "ambiguous_ratio_thresholds", "{run:?}");
    assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::SingleComponent);
    assert_eq!(
        provenance.components.iter().map(|component| component.role).collect::<Vec<_>>(),
        vec![SignalComponentRole::Pilot]
    );
    assert!(selected.uncertainty.is_none(), "{run:?}");
}

#[test]
fn related_signal_follow_up_reruns_same_satellite_cross_band_request() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 17 };
    let l1_doppler_hz = -750.0;
    let l5_doppler_hz =
        shared_path_doppler_hz(l1_doppler_hz, signal_spec_gps_l1_ca(), signal_spec_gps_l5_i())
            .expect("same-satellite carrier scaling");
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.030,
        seed: 0x2810_0001,
        satellites: vec![
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: l1_doppler_hz,
                code_phase_chips: 32.1,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 56.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_hz: l5_doppler_hz,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 31.5,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "related-signal-follow-up-same-satellite".to_string(),
    };
    let frame = signal_only_frame(&config, &scenario, 30_690);
    let requests = [
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        },
        AcqRequest {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        },
    ];

    let run = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_topn_for_requests_with_explain(&frame, &requests, 1);

    let l1_result = run.results[0].first().expect("L1 acquisition result");
    let l5_result = run.results[1].first().expect("L5 acquisition result");
    let assumptions =
        l5_result.assumptions.as_ref().expect("cross-band follow-up should preserve assumptions");
    let expected_l5_center_hz = shared_path_doppler_hz(
        l1_result.doppler_hz.0,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l5_i(),
    )
    .expect("measured L1 Doppler should scale onto L5");

    assert!(
        matches!(l5_result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{run:#?}"
    );
    assert!(assumptions.assistance_bounds.is_some(), "{l5_result:#?}");
    assert!(
        (assumptions.doppler_center_hz - expected_l5_center_hz).abs() <= 1.0e-6,
        "{l5_result:#?}"
    );
    assert!(
        l5_result
            .explain_selection_reason
            .as_deref()
            .is_some_and(|reason| reason.contains("same_satellite_cross_band_assistance")),
        "{l5_result:#?}"
    );
    assert!(
        run.explains[1].selected_reason.contains("same_satellite_cross_band_assistance"),
        "{run:#?}"
    );
}

#[test]
fn related_signal_follow_up_does_not_cross_satellite_boundaries() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        acquisition_doppler_search_hz: 2_000,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        ..ReceiverPipelineConfig::default()
    };
    let l1_sat = SatId { constellation: Constellation::Gps, prn: 17 };
    let l5_sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let l5_doppler_hz =
        shared_path_doppler_hz(-750.0, signal_spec_gps_l1_ca(), signal_spec_gps_l5_i())
            .expect("same-carrier-ratio scaling");
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.030,
        seed: 0x2810_0002,
        satellites: vec![
            SyntheticSignalParams {
                sat: l1_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: -750.0,
                code_phase_chips: 32.1,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 56.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: l5_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_hz: l5_doppler_hz,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 31.5,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "related-signal-follow-up-cross-satellite".to_string(),
    };
    let frame = signal_only_frame(&config, &scenario, 30_690);
    let requests = [
        AcqRequest {
            sat: l1_sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        },
        AcqRequest {
            sat: l5_sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
        },
    ];

    let run = Acquisition::new(config, ReceiverRuntime::default())
        .run_fft_topn_for_requests_with_explain(&frame, &requests, 1);

    let l5_result = run.results[1].first().expect("L5 acquisition result");
    let assumptions = l5_result.assumptions.as_ref().expect("L5 assumptions");

    assert!(assumptions.assistance_bounds.is_none(), "{l5_result:#?}");
    assert!(
        !run.explains[1].selected_reason.contains("same_satellite_cross_band_assistance"),
        "{run:#?}"
    );
}

#[test]
fn search_window_diagnostic_detects_doppler_rate_edge_rise() {
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let candidates = vec![
        search_window_rate_candidate(sat, 250.0, -20_000.0, 6.0),
        search_window_rate_candidate(sat, 250.0, -15_000.0, 5.0),
        search_window_rate_candidate(sat, 250.0, 15_000.0, 7.0),
        search_window_rate_candidate(sat, 250.0, 20_000.0, 9.0),
    ];

    let diagnostic =
        signal_outside_doppler_rate_search_range(&candidates, 250.0, 0.0, 20_000, 5_000, 4.0)
            .expect("doppler-rate edge diagnostic");

    assert_eq!(diagnostic.dimension, SearchWindowDimension::DopplerRate);
    assert_eq!(diagnostic.edge, SearchWindowEdge::Upper);
    assert_eq!(diagnostic.best_axis_value, 20_000.0);
    assert_eq!(diagnostic.interior_axis_value, 15_000.0);
}

fn candidate_for_search_window_test(
    sat: SatId,
    carrier_hz: f64,
    peak_mean_ratio: f32,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(carrier_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz),
        code_phase_samples: 0,
        peak: peak_mean_ratio,
        second_peak: 1.0,
        mean: 1.0,
        peak_mean_ratio,
        peak_second_ratio: peak_mean_ratio,
        cn0_proxy: peak_mean_ratio,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn search_window_rate_candidate(
    sat: SatId,
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
    peak_mean_ratio: f32,
) -> AcqResult {
    let mut candidate = candidate_for_search_window_test(sat, carrier_hz, peak_mean_ratio);
    candidate.doppler_rate_hz_per_s = doppler_rate_hz_per_s;
    candidate
}
