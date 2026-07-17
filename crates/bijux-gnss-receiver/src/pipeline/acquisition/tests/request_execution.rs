use super::*;

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

    assert_eq!(candidates.len(), 3);
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
    assert!(candidates.iter().all(|candidate| {
        candidate.component_provenance().is_some_and(|provenance| {
            provenance.combination_mode != AcqComponentCombinationMode::CoherentComponentSum
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
fn run_fft_for_galileo_e5a_prefers_expected_doppler_candidate_under_truth_guidance() {
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
    let sat = SatId { constellation: Constellation::Galileo, prn: 18 };
    let true_doppler_hz = 750.0;
    let frame = crate::api::sim::generate_l1_ca_multi(
        &config,
        &crate::api::sim::SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.060,
            seed: 0x6AE5_A000,
            satellites: vec![crate::api::sim::SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::E5,
                signal_code: SignalCode::E5a,
                doppler_hz: true_doppler_hz,
                code_phase_chips: 2_048.25,
                carrier_phase_rad: 0.4,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "galileo-e5a-truth-guided-acquisition".to_string(),
        },
    );
    let acquisition = Acquisition::new(config, ReceiverRuntime::default());
    let request = AcqRequest {
        sat,
        glonass_frequency_channel: None,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5a,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        doppler_rate_search_hz_per_s: 0,
        doppler_rate_step_hz_per_s: 250,
        expected_line_of_sight_doppler_hz: Some(true_doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: 2_000,
        doppler_step_hz: 250,
        coherent_ms: 1,
        noncoherent: 1,
    };

    let result = acquisition.run_fft_for_requests(&frame, &[request]).remove(0);

    assert!(
        matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
        "{result:?}"
    );
    assert!((result.doppler_hz.0 - true_doppler_hz).abs() <= 1.0e-6, "{result:?}");
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
