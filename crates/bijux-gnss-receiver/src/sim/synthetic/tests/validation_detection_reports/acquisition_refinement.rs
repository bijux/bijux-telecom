#[test]
fn truth_guided_code_phase_refinement_improves_fractional_pseudorange_initialization() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let mut best_improvement_samples = 0.0_f64;

    for code_phase_chips in [200.125, 200.25, 200.375, 200.5, 200.625, 200.75, 200.875] {
        let scenario = SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: 0.04,
            seed: 24071985,
            satellites: vec![SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 65.0,
                navigation_data: false.into(),
            }],
            ephemerides: Vec::new(),
            id: "acquisition_code_phase_refinement_truth".to_string(),
        };
        let frame = generate_l1_ca_multi(&config, &scenario);
        let bundle = build_iq16_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            "2026-07-09T00:00:00Z",
            Some("unit code-phase refinement validation".to_string()),
        );
        let scaled_frame = SamplesFrame::new(
            frame.t0,
            frame.dt_s,
            frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
        );

        let report = validate_truth_guided_acquisition_code_phase_refinement(
            &config,
            &scaled_frame,
            &bundle.truth,
        );

        assert!(report.pass, "{report:?}");
        assert_eq!(report.satellites.len(), 1);
        let row = &report.satellites[0];
        assert!(row.pass, "{row:?}");
        best_improvement_samples = best_improvement_samples.max(row.improvement_samples);
    }

    assert!(
        best_improvement_samples > 0.0,
        "expected at least one fractional synthetic fixture to improve"
    );
}

#[test]
fn truth_guided_acquisition_coherent_integration_report_combines_code_phase_and_doppler() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 10_000,
        acquisition_doppler_step_hz: 500,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.04,
        seed: 24_071_985,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 750.0,
            code_phase_chips: 200.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "acquisition-coherent-integration-profile".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-09T00:00:00Z",
        Some("unit acquisition coherent integration validation".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    let report = validate_truth_guided_acquisition_coherent_integration(
        &config,
        &scaled_frame,
        &bundle.truth,
        1,
        1,
        2,
        1,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.coherent_ms, 1);
    assert_eq!(report.noncoherent, 1);
    assert_eq!(report.satellites.len(), 1);
    assert!(report.satellites[0].pass, "{:?}", report.satellites[0]);
}
