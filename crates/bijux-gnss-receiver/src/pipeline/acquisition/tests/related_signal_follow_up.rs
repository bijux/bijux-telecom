use super::*;

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
