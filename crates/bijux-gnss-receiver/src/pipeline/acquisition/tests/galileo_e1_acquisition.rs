use super::*;

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
