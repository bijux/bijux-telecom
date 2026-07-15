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

#[path = "tests/code_fft_cache.rs"]
mod code_fft_cache;
#[path = "tests/coherent_accumulation.rs"]
mod coherent_accumulation;
#[path = "tests/decision_policy.rs"]
mod decision_policy;
#[path = "tests/integration_guards.rs"]
mod integration_guards;
#[path = "tests/refinement_diagnostics.rs"]
mod refinement_diagnostics;
#[path = "tests/request_execution.rs"]
mod request_execution;
#[path = "tests/search_window_diagnostics.rs"]
mod search_window_diagnostics;

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
