#[test]
fn truth_bundle_records_constant_and_alternating_nav_bit_truth() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 250.0,
        duration_s: 0.05,
        seed: 44,
        satellites: vec![
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 500.0,
                code_phase_chips: 200.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 50.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: -750.0,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 45.0,
                navigation_data: true.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("synthetic truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.25,
        0.8,
    );

    assert_eq!(truth.schema_version, 8);
    assert_eq!(truth.scenario_id, "truth-bundle");
    assert_eq!(truth.seed, 44);
    assert_eq!(truth.sample_format, IqSampleFormat::Iq16Le);
    assert_eq!(truth.quantization, IqQuantization::Signed16Bit);
    assert_eq!(truth.sample_rate_hz, 4_000_000.0);
    assert_eq!(truth.receiver_clock_frequency_bias_hz, 250.0);
    assert_eq!(truth.receiver_oscillator_model.carrier_frequency_bias_hz, 250.0);
    assert_eq!(truth.receiver_oscillator_model.carrier_frequency_drift_hz_per_s, 0.0);
    assert_eq!(truth.receiver_oscillator_model.sampling_clock_fractional_error, 0.0);
    assert_eq!(truth.quantization_bits, 16);
    assert_eq!(truth.noise_std_per_component, SYNTHETIC_NOISE_STD_PER_COMPONENT);
    assert!(
        (truth.noise_power_per_complex_sample - SYNTHETIC_COMPLEX_NOISE_POWER as f32).abs() < 1e-6
    );
    assert_eq!(truth.peak_component_before_scaling, 1.25);
    assert_eq!(truth.output_scale_applied, 0.8);
    assert!(
        truth.receiver_oscillator_truth.carrier_frequency_bias_hz.len() >= 2,
        "{:?}",
        truth.receiver_oscillator_truth.carrier_frequency_bias_hz
    );
    assert!(truth.receiver_oscillator_truth.carrier_frequency_bias_hz.iter().all(|point| (point
        .value
        - 250.0)
        .abs()
        <= f64::EPSILON));
    assert!(truth
        .receiver_oscillator_truth
        .phase_noise_rad
        .iter()
        .all(|point| point.value.abs() <= f64::EPSILON));
    assert!(truth
        .receiver_oscillator_truth
        .frequency_noise_hz
        .iter()
        .all(|point| point.value.abs() <= f64::EPSILON));
    assert_eq!(truth.satellites.len(), 2);

    let constant = &truth.satellites[0];
    assert_eq!(
        constant.signal_amplitude,
        signal_amplitude_from_cn0(constant.cn0_db_hz, truth.sample_rate_hz)
    );
    assert_eq!(constant.glonass_frequency_channel, None);
    assert_eq!(constant.navigation_data, super::SyntheticNavigationData::ConstantPositive);
    assert_eq!(constant.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
    assert_eq!(constant.nav_bit_segments.len(), 1);
    assert_eq!(constant.nav_bit_segments[0].start_sample, 0);
    assert_eq!(constant.nav_bit_segments[0].end_sample, frame.len() as u64);
    assert_eq!(constant.nav_bit_segments[0].bit, 1);

    let alternating = &truth.satellites[1];
    assert_eq!(
        alternating.signal_amplitude,
        signal_amplitude_from_cn0(alternating.cn0_db_hz, truth.sample_rate_hz)
    );
    assert!(constant.signal_amplitude > alternating.signal_amplitude);
    assert_eq!(alternating.glonass_frequency_channel, None);
    assert_eq!(
        alternating.navigation_data,
        super::SyntheticNavigationData::AlternatingStartPositive
    );
    assert_eq!(alternating.nav_bit_mode, SyntheticNavBitMode::AlternatingGpsLnav20ms);
    assert_eq!(alternating.nav_bit_segments.len(), 3);
    assert_eq!(alternating.nav_bit_segments[0].start_sample, 0);
    assert_eq!(alternating.nav_bit_segments[0].end_sample, 80_000);
    assert_eq!(alternating.nav_bit_segments[0].bit, 1);
    assert_eq!(alternating.nav_bit_segments[1].start_sample, 80_000);
    assert_eq!(alternating.nav_bit_segments[1].end_sample, 160_000);
    assert_eq!(alternating.nav_bit_segments[1].bit, -1);
    assert_eq!(alternating.nav_bit_segments[2].start_sample, 160_000);
    assert_eq!(alternating.nav_bit_segments[2].end_sample, frame.len() as u64);
    assert_eq!(alternating.nav_bit_segments[2].bit, 1);
}

#[test]
fn truth_bundle_preserves_glonass_frequency_channel_metadata() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.01,
        seed: 7,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            glonass_frequency_channel: Some(channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 250.0,
            code_phase_chips: 10.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "glonass-truth".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("glonass truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        0.75,
    );

    assert_eq!(truth.schema_version, 8);
    assert_eq!(truth.satellites.len(), 1);
    assert_eq!(truth.satellites[0].sat, scenario.satellites[0].sat);
    assert_eq!(truth.satellites[0].glonass_frequency_channel, Some(channel));
    assert_eq!(truth.receiver_oscillator_model, SyntheticReceiverOscillatorModel::default());
}

#[test]
fn truth_bundle_records_separate_receiver_oscillator_effect_series() {
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
        duration_s: 0.004,
        seed: 17,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 600.0,
            code_phase_chips: 80.5,
            carrier_phase_rad: 0.1,
            cn0_db_hz: 50.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-oscillator-truth".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: 125.0,
        carrier_frequency_drift_hz_per_s: 18.0,
        sampling_clock_fractional_error: 75.0e-6,
        sampling_clock_fractional_drift_per_s: 12.0e-6,
        phase_noise: SyntheticReceiverPhaseNoiseModel {
            seed: 77,
            knot_interval_samples: 2_046,
            step_std_rad: 0.05,
        },
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 79,
            update_interval_samples: 2_046,
            white_phase_std_rad: 0.02,
            white_frequency_std_hz: 3.0,
            random_walk_frequency_step_std_hz: 1.5,
        },
    };
    let frame =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let bundle = build_quantized_capture_bundle_with_receiver_oscillator(
        super::super::SyntheticQuantizedCaptureRequest {
            scenario_id: &scenario.id,
            scenario: &scenario,
            frame: &frame,
            quantization: IqQuantization::Float32,
            capture_start_utc: "2026-07-13T00:00:00Z",
            notes: Some("receiver oscillator truth bundle".to_string()),
            receiver_oscillator_model: &receiver_oscillator,
            source_front_end_filter: None,
        },
    );

    assert_eq!(bundle.truth.receiver_oscillator_model, receiver_oscillator);
    assert!(bundle
        .truth
        .receiver_oscillator_truth
        .carrier_frequency_bias_hz
        .iter()
        .all(|point| (point.value - 125.0).abs() <= f64::EPSILON));
    assert!(bundle
        .truth
        .receiver_oscillator_truth
        .carrier_frequency_drift_hz_per_s
        .iter()
        .all(|point| (point.value - 18.0).abs() <= f64::EPSILON));
    assert!(bundle
        .truth
        .receiver_oscillator_truth
        .phase_noise_rad
        .iter()
        .any(|point| point.value.abs() > 1.0e-6));
    assert!(bundle
        .truth
        .receiver_oscillator_truth
        .frequency_noise_hz
        .iter()
        .any(|point| point.value.abs() > 1.0e-6));
    let last_sampling_clock_point = bundle
        .truth
        .receiver_oscillator_truth
        .sampling_clock_time_error_s
        .last()
        .expect("sampling clock truth point");
    let expected_time_error_s = last_sampling_clock_point.time_s
        * receiver_oscillator.sampling_clock_fractional_error
        + 0.5
            * receiver_oscillator.sampling_clock_fractional_drift_per_s
            * last_sampling_clock_point.time_s
            * last_sampling_clock_point.time_s;
    assert!(
        (last_sampling_clock_point.value - expected_time_error_s).abs() <= 1.0e-12,
        "{last_sampling_clock_point:?}"
    );
}
