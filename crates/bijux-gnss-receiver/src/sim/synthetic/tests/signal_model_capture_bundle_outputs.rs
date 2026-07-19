#[test]
fn iq16_capture_bundle_scales_without_clipping_and_preserves_truth() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.01,
        seed: 91,
        satellites: vec![
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 500.0,
                code_phase_chips: 200.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: true.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: -1000.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 56.0,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "iq16-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);

    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-09T00:00:00Z",
        Some("synthetic iq bundle".to_string()),
    );

    assert_eq!(bundle.metadata.format, IqSampleFormat::Iq16Le);
    assert_eq!(bundle.metadata.quantization_bits, Some(16));
    assert_eq!(bundle.truth.quantization, IqQuantization::Signed16Bit);
    assert_eq!(bundle.truth.scenario_id, "iq16-bundle");
    assert_eq!(bundle.truth.sample_count, frame.len());
    assert_eq!(bundle.truth.sample_rate_hz, 4_092_000.0);
    assert_eq!(bundle.truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
    assert!(
        (bundle.truth.noise_power_per_complex_sample - SYNTHETIC_COMPLEX_NOISE_POWER as f32).abs()
            < 1e-6
    );
    assert_eq!(bundle.raw_iq_bytes.len(), frame.len() * 4);
    assert!(bundle.truth.peak_component_before_scaling > 0.0);
    assert!(bundle.truth.output_scale_applied > 0.0);
    assert!(bundle.truth.output_scale_applied <= 1.0);
    assert!(
        bundle.truth.satellites[0].signal_amplitude > bundle.truth.satellites[1].signal_amplitude
    );

    assert!(
        bundle
            .raw_iq_bytes
            .chunks_exact(2)
            .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]))
            .any(|sample| sample != 0),
        "encoded synthetic capture should contain non-zero samples"
    );
}

#[test]
fn quantized_capture_bundle_tracks_effective_bits_and_storage_format() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.002,
        seed: 73,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: -250.0,
            code_phase_chips: 123.0,
            carrier_phase_rad: 0.1,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "signed4-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);

    let bundle = build_quantized_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        IqQuantization::Signed4Bit,
        "2026-07-13T00:00:00Z",
        Some("synthetic signed4 bundle".to_string()),
    );

    assert_eq!(bundle.metadata.format, IqSampleFormat::Iq8);
    assert_eq!(bundle.metadata.quantization_bits, Some(4));
    assert_eq!(bundle.truth.quantization, IqQuantization::Signed4Bit);
    assert_eq!(bundle.truth.sample_format, IqSampleFormat::Iq8);
    assert_eq!(bundle.truth.quantization_bits, 4);
    assert_eq!(bundle.raw_iq_bytes.len(), frame.len() * 2);
}
