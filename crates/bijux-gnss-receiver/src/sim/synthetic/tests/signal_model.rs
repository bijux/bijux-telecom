include!("signal_model/reference_samples.rs");


include!("signal_model/truth_bundle_metadata.rs");


#[test]
fn truth_bundle_records_gps_l5q_nh20_segments() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.023,
        seed: 255,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 24 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L5,
            signal_code: bijux_gnss_core::api::SignalCode::L5Q,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "gps-l5q-truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("gps l5q truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::GpsL5QNh20);
    assert_eq!(satellite.nav_bit_segments.len(), 23);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 10_230);
    assert_eq!(satellite.nav_bit_segments[19].start_sample, 194_370);
    assert_eq!(satellite.nav_bit_segments[19].end_sample, 204_600);
    assert_eq!(satellite.nav_bit_segments[20].start_sample, 204_600);
    assert_eq!(satellite.nav_bit_segments[20].end_sample, 214_830);
    assert_eq!(satellite.nav_bit_segments[22].end_sample, frame.len() as u64);

    for (epoch_index, segment) in satellite.nav_bit_segments.iter().enumerate() {
        assert_eq!(
            segment.bit,
            bijux_gnss_signal::api::gps_l5_q_epoch_symbol(epoch_index),
            "unexpected NH20 chip at epoch {epoch_index}"
        );
    }
}

#[test]
fn truth_bundle_records_gps_l5i_native_epoch_segments_with_fixed_data() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.012,
        seed: 301,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L5,
            signal_code: bijux_gnss_core::api::SignalCode::L5I,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "gps-l5i-truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("gps l5i truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
    assert_eq!(satellite.nav_bit_segments.len(), 12);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 10_230);
    assert_eq!(satellite.nav_bit_segments[11].end_sample, frame.len() as u64);

    for (epoch_index, segment) in satellite.nav_bit_segments.iter().enumerate() {
        assert_eq!(
            segment.bit,
            bijux_gnss_signal::api::gps_l5_i_epoch_symbol(&[1], epoch_index)
                .expect("GPS L5I epoch symbol"),
            "unexpected GPS L5I epoch symbol at {epoch_index}"
        );
    }
}

#[test]
fn truth_bundle_records_galileo_e5a_native_epoch_segments_with_fixed_data() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.021,
        seed: 303,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::E5,
            signal_code: bijux_gnss_core::api::SignalCode::E5a,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "galileo-e5a-truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("galileo e5a truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
    assert_eq!(satellite.nav_bit_segments.len(), 21);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 10_230);
    assert_eq!(satellite.nav_bit_segments[20].end_sample, frame.len() as u64);

    for (epoch_index, segment) in satellite.nav_bit_segments.iter().enumerate() {
        assert_eq!(
            segment.bit,
            bijux_gnss_signal::api::galileo_e5a_i_epoch_symbol(&[1], epoch_index)
                .expect("Galileo E5a epoch symbol"),
            "unexpected Galileo E5a epoch symbol at {epoch_index}"
        );
    }
}

#[test]
fn truth_bundle_records_beidou_b1i_d1_epoch_segments() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.021,
        seed: 305,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::B1,
            signal_code: bijux_gnss_core::api::SignalCode::B1I,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "beidou-b1i-truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("beidou b1i truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
    assert_eq!(satellite.nav_bit_segments.len(), 21);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 2046);
    assert_eq!(satellite.nav_bit_segments[20].end_sample, frame.len() as u64);

    for (epoch_index, segment) in satellite.nav_bit_segments.iter().enumerate() {
        assert_eq!(
            segment.bit,
            bijux_gnss_signal::api::beidou_d1_epoch_symbol(&[1], epoch_index)
                .expect("BeiDou D1 epoch symbol"),
            "unexpected BeiDou B1I epoch symbol at {epoch_index}"
        );
    }
}

#[test]
fn truth_bundle_records_beidou_b2i_d1_epoch_segments() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
            - bijux_gnss_core::api::BEIDOU_B2_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.021,
        seed: 307,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::B2,
            signal_code: bijux_gnss_core::api::SignalCode::B2I,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "beidou-b2i-truth-bundle".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("beidou b2i truth bundle".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::ConstantPositive);
    assert_eq!(satellite.nav_bit_segments.len(), 21);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 2046);
    assert_eq!(satellite.nav_bit_segments[20].end_sample, frame.len() as u64);

    for (epoch_index, segment) in satellite.nav_bit_segments.iter().enumerate() {
        assert_eq!(
            segment.bit,
            bijux_gnss_signal::api::beidou_d1_epoch_symbol(&[1], epoch_index)
                .expect("BeiDou D1 epoch symbol"),
            "unexpected BeiDou B2I epoch symbol at {epoch_index}"
        );
    }
}

#[test]
fn synthetic_scenario_parses_legacy_data_bit_flip_field() {
    let scenario: SyntheticScenario = toml::from_str(
        r#"
id = "legacy-data-bit-flip"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.02
seed = 17

[[satellites]]
sat = { constellation = "Gps", prn = 3 }
doppler_hz = 0.0
code_phase_chips = 0.0
carrier_phase_rad = 0.0
cn0_db_hz = 45.0
data_bit_flip = true
"#,
    )
    .expect("legacy scenario must deserialize");

    assert_eq!(scenario.satellites.len(), 1);
    assert_eq!(
        scenario.satellites[0].navigation_data,
        super::SyntheticNavigationData::AlternatingStartPositive
    );
}

#[test]
fn truth_bundle_records_gps_l2c_symbol_sequence_boundaries() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let navigation_data = super::SyntheticNavigationData::SymbolSequence(vec![-1, 1, 1]);
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.050,
        seed: 912,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 5 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L2,
            signal_code: bijux_gnss_core::api::SignalCode::L2C,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: navigation_data.clone(),
        }],
        ephemerides: Vec::new(),
        id: "gps-l2c-symbol-sequence".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("gps l2c symbol sequence".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.navigation_data, navigation_data);
    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::NativeSymbolSequence);
    assert_eq!(satellite.nav_bit_segments.len(), 3);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 81_840);
    assert_eq!(satellite.nav_bit_segments[0].bit, -1);
    assert_eq!(satellite.nav_bit_segments[1].start_sample, 81_840);
    assert_eq!(satellite.nav_bit_segments[1].end_sample, 163_680);
    assert_eq!(satellite.nav_bit_segments[1].bit, 1);
    assert_eq!(satellite.nav_bit_segments[2].start_sample, 163_680);
    assert_eq!(satellite.nav_bit_segments[2].end_sample, frame.len() as u64);
    assert_eq!(satellite.nav_bit_segments[2].bit, 1);
}

#[test]
fn truth_bundle_records_galileo_e1_symbol_sequence_boundaries() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        ..ReceiverPipelineConfig::default()
    };
    let navigation_data = super::SyntheticNavigationData::SymbolSequence(vec![-1, 1, 1]);
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.010,
        seed: 913,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::E1,
            signal_code: bijux_gnss_core::api::SignalCode::E1B,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: navigation_data.clone(),
        }],
        ephemerides: Vec::new(),
        id: "galileo-e1-symbol-sequence".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-13T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("galileo e1 symbol sequence".to_string()),
    };

    let truth = build_truth_bundle(
        &scenario.id,
        &scenario,
        &frame,
        &metadata,
        IqQuantization::Signed16Bit,
        1.0,
        1.0,
    );
    let satellite = &truth.satellites[0];

    assert_eq!(satellite.navigation_data, navigation_data);
    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::NativeSymbolSequence);
    assert_eq!(satellite.nav_bit_segments.len(), 3);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 16_368);
    assert_eq!(satellite.nav_bit_segments[0].bit, -1);
    assert_eq!(satellite.nav_bit_segments[1].start_sample, 16_368);
    assert_eq!(satellite.nav_bit_segments[1].end_sample, 32_736);
    assert_eq!(satellite.nav_bit_segments[1].bit, 1);
    assert_eq!(satellite.nav_bit_segments[2].start_sample, 32_736);
    assert_eq!(satellite.nav_bit_segments[2].end_sample, frame.len() as u64);
    assert_eq!(satellite.nav_bit_segments[2].bit, 1);
}

#[test]
fn glonass_frequency_channel_offsets_synthetic_intermediate_frequency() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 2_500_000.0,
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let lower_channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-7).expect("channel -7 must be valid");
    let upper_channel =
        bijux_gnss_core::api::GlonassFrequencyChannel::new(6).expect("channel 6 must be valid");
    let lower = SatState::new_with_receiver_clock_frequency_bias_hz(
        &config,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 5 },
            glonass_frequency_channel: Some(lower_channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        0.0,
    );
    let upper = SatState::new_with_receiver_clock_frequency_bias_hz(
        &config,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 12 },
            glonass_frequency_channel: Some(upper_channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        0.0,
    );

    let expected_spacing_hz = 13.0 * bijux_gnss_core::api::GLONASS_L1_CHANNEL_SPACING_HZ.value();

    assert!((upper.if_hz - lower.if_hz - expected_spacing_hz).abs() <= 1e-6);
}

#[test]
fn receiver_clock_frequency_bias_shifts_synthetic_carrier_phase_increment() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_000.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let unbiased =
        SatState::new_with_receiver_clock_frequency_bias_hz(&config, params.clone(), 0.0);
    let biased =
        SatState::new_with_receiver_clock_frequency_bias_hz(&config, params.clone(), 500.0);
    let sample_dt_s = 1.0 / config.sampling_freq_hz;
    let unbiased_phase_step =
        phase_step_rad(unbiased.sample_at(0.0), unbiased.sample_at(sample_dt_s));
    let biased_phase_step = phase_step_rad(biased.sample_at(0.0), biased.sample_at(sample_dt_s));
    let expected_extra_phase_step = std::f64::consts::TAU * 500.0 / config.sampling_freq_hz;
    let actual_extra_phase_step = wrap_phase_rad((biased_phase_step - unbiased_phase_step) as f64);

    assert!(
                (actual_extra_phase_step - expected_extra_phase_step).abs() <= 1e-6,
                "receiver clock bias phase step mismatch: actual={actual_extra_phase_step}, expected={expected_extra_phase_step}"
            );
}

#[test]
fn synthetic_fade_windows_zero_signal_inside_the_requested_interval() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let mut signal_only = super::generate_l1_ca_signal_only(&config, params.clone(), 0.010);
    super::apply_synthetic_fade_windows(
        &mut signal_only,
        &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
    );

    let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert!(
        signal_only.iq[..fade_start].iter().any(|sample| sample.norm_sqr() > 0.0),
        "samples before the fade must preserve signal energy"
    );
    assert!(
        signal_only.iq[fade_start..fade_end].iter().all(|sample| sample.norm_sqr() <= f32::EPSILON),
        "samples inside the fade window must be fully attenuated"
    );
    assert!(
        signal_only.iq[fade_end..].iter().any(|sample| sample.norm_sqr() > 0.0),
        "samples after the fade must preserve signal energy"
    );
}

#[test]
fn synthetic_fade_generator_preserves_noise_outside_the_attenuated_signal_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xFADE_0001, 0.010);
    let faded = generate_l1_ca_with_fades(
        &config,
        params,
        &[SyntheticFadeWindow { start_s: 0.002, end_s: 0.004, signal_scale: 0.0 }],
        0xFADE_0001,
        0.010,
    );

    let fade_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let fade_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert_eq!(baseline.iq[..fade_start], faded.iq[..fade_start]);
    assert_eq!(baseline.iq[fade_end..], faded.iq[fade_end..]);
    assert!(
        baseline.iq[fade_start..fade_end] != faded.iq[fade_start..fade_end],
        "the attenuated window must differ from the unfaded baseline"
    );
}

#[test]
fn synthetic_phase_window_rotates_signal_only_inside_configured_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let mut signal_only = super::generate_l1_ca_signal_only(&config, params.clone(), 0.010);
    super::apply_synthetic_phase_windows(
        &mut signal_only,
        &[SyntheticPhaseWindow {
            start_s: 0.002,
            end_s: 0.004,
            phase_offset_rad: std::f64::consts::FRAC_PI_2,
        }],
    );

    let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert!(signal_only.iq[..phase_start].iter().all(|sample| sample.im.abs() <= 1.0e-6));
    assert!(
        signal_only.iq[phase_start..phase_end].iter().any(|sample| sample.im.abs() > 1.0e-3),
        "windowed samples must carry a rotated quadrature component"
    );
    assert!(signal_only.iq[phase_end..].iter().all(|sample| sample.im.abs() <= 1.0e-6));
}

#[test]
fn synthetic_phase_generator_preserves_noise_outside_the_rotated_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 5 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xFACE_0001, 0.010);
    let rotated = generate_l1_ca_with_phase_windows(
        &config,
        params,
        &[SyntheticPhaseWindow { start_s: 0.002, end_s: 0.004, phase_offset_rad: 0.8 }],
        0xFACE_0001,
        0.010,
    );

    let phase_start = (0.002 * config.sampling_freq_hz).round() as usize;
    let phase_end = (0.004 * config.sampling_freq_hz).round() as usize;
    assert_eq!(baseline.iq[..phase_start], rotated.iq[..phase_start]);
    assert_eq!(baseline.iq[phase_end..], rotated.iq[phase_end..]);
    assert_ne!(baseline.iq[phase_start..phase_end], rotated.iq[phase_start..phase_end]);
}

#[test]
fn zero_doppler_rate_matches_constant_synthetic_generator() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 9 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 750.0,
        code_phase_chips: 144.375,
        carrier_phase_rad: 0.15,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let baseline = generate_l1_ca(&config, params.clone(), 0xD0A0_0001, 0.020);
    let ramped = generate_l1_ca_with_doppler_ramp(
        &config,
        SyntheticDopplerRampParams { signal: params.clone(), doppler_rate_hz_per_s: 0.0 },
        0xD0A0_0001,
        0.020,
    );

    assert_eq!(ramped.t0.sample_index, baseline.t0.sample_index);
    assert_eq!(ramped.t0.sample_rate_hz, baseline.t0.sample_rate_hz);
    assert_eq!(ramped.dt_s.0, baseline.dt_s.0);
    assert_eq!(ramped.iq, baseline.iq);
}

#[test]
fn zero_jerk_carrier_dynamics_matches_doppler_ramp_generator() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 650.0,
        code_phase_chips: 211.25,
        carrier_phase_rad: 0.30,
        cn0_db_hz: 58.0,
        navigation_data: false.into(),
    };
    let ramped = generate_l1_ca_with_doppler_ramp(
        &config,
        SyntheticDopplerRampParams { signal: params.clone(), doppler_rate_hz_per_s: 45.0 },
        0xCA44_1000,
        0.020,
    );
    let dynamics = generate_l1_ca_with_carrier_dynamics(
        &config,
        SyntheticCarrierDynamicsParams {
            signal: params,
            doppler_rate_hz_per_s: 45.0,
            doppler_jerk_hz_per_s2: 0.0,
        },
        0xCA44_1000,
        0.020,
    );

    assert_eq!(dynamics.t0.sample_index, ramped.t0.sample_index);
    assert_eq!(dynamics.t0.sample_rate_hz, ramped.t0.sample_rate_hz);
    assert_eq!(dynamics.dt_s.0, ramped.dt_s.0);
    assert_eq!(dynamics.iq, ramped.iq);
}

#[test]
fn doppler_ramp_updates_instantaneous_carrier_frequency_linearly() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        &config, params, 150.0, 40.0,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1e-12);
    assert!((sat_state.carrier_hz_at(0.25) - 1_360.0).abs() <= 1e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_370.0).abs() <= 1e-12);
}

#[test]
fn carrier_dynamics_updates_instantaneous_carrier_frequency_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 15 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_carrier_dynamics_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 150.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        40.0,
        120.0,
        4_092,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.25) - 1_363.75).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_385.0).abs() <= 1.0e-12);
}

#[test]
fn receiver_oscillator_drift_accumulates_separately_from_signal_doppler_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 1_200.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 150.0,
            carrier_frequency_drift_hz_per_s: 25.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        40.0,
        4_092,
    );

    assert!((sat_state.carrier_hz_at(0.0) - 1_350.0).abs() <= 1.0e-12);
    assert!((sat_state.carrier_hz_at(0.50) - 1_382.5).abs() <= 1.0e-12);
}

#[test]
fn doppler_ramp_integrates_into_carrier_phase_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 900.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.35,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        &config, params, 100.0, -25.0,
    );
    let t_s: f64 = 0.40;
    let expected_phase_rad =
        0.35 + std::f64::consts::TAU * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s);

    assert!(
        (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
        "carrier phase ramp mismatch: actual={}, expected={expected_phase_rad}",
        sat_state.carrier_phase_rad_at(t_s)
    );
}

#[test]
fn carrier_dynamics_integrates_into_carrier_phase_cubically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 900.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.35,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let sat_state = SatState::new_with_carrier_dynamics_and_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 100.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 0.0,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        -25.0,
        90.0,
        4_092,
    );
    let t_s: f64 = 0.40;
    let expected_phase_rad = 0.35
        + std::f64::consts::TAU
            * ((1_000.0 * t_s) + 0.5 * (-25.0) * t_s * t_s + 90.0 * t_s.powi(3) / 6.0);

    assert!(
        (sat_state.carrier_phase_rad_at(t_s) - expected_phase_rad).abs() <= 1e-9,
        "carrier dynamics phase mismatch: actual={}, expected={expected_phase_rad}",
        sat_state.carrier_phase_rad_at(t_s)
    );
}

#[test]
fn receiver_oscillator_phase_noise_is_deterministic_and_distinct_from_nominal_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
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
        seed: 11,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 6 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-phase-noise".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        carrier_frequency_bias_hz: 0.0,
        carrier_frequency_drift_hz_per_s: 0.0,
        sampling_clock_fractional_error: 0.0,
        sampling_clock_fractional_drift_per_s: 0.0,
        phase_noise: SyntheticReceiverPhaseNoiseModel {
            seed: 91,
            knot_interval_samples: 1_023,
            step_std_rad: 0.08,
        },
        noise: SyntheticReceiverOscillatorNoiseModel::default(),
    };

    let phase_noisy_a =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let phase_noisy_b =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let nominal = generate_l1_ca_multi(&config, &scenario);

    assert_eq!(phase_noisy_a.iq, phase_noisy_b.iq);
    assert_ne!(phase_noisy_a.iq[1_023], nominal.iq[1_023]);
}

#[test]
fn receiver_oscillator_white_phase_noise_is_deterministic_and_distinct_from_nominal_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
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
        seed: 11,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 6 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 55.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-white-phase-noise".to_string(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 173,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.06,
            white_frequency_std_hz: 0.0,
            random_walk_frequency_step_std_hz: 0.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };

    let phase_noisy_a =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let phase_noisy_b =
        generate_l1_ca_multi_with_receiver_oscillator(&config, &scenario, &receiver_oscillator);
    let nominal = generate_l1_ca_multi(&config, &scenario);

    assert_eq!(phase_noisy_a.iq, phase_noisy_b.iq);
    assert_ne!(phase_noisy_a.iq[1_023], nominal.iq[1_023]);
}

#[test]
fn receiver_oscillator_white_frequency_noise_changes_instantaneous_carrier_truth() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 4 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 20.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 211,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.0,
            white_frequency_std_hz: 8.0,
            random_walk_frequency_step_std_hz: 0.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };
    let noisy = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        receiver_oscillator,
        4_092,
    );
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );

    assert_ne!(noisy.carrier_hz_at(0.001), nominal.carrier_hz_at(0.001));
    assert!(noisy.carrier_phase_rad_at(0.003).is_finite());
}

#[test]
fn receiver_oscillator_random_walk_frequency_noise_accumulates_carrier_phase() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 30.0,
        code_phase_chips: 0.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let receiver_oscillator = SyntheticReceiverOscillatorModel {
        noise: SyntheticReceiverOscillatorNoiseModel {
            seed: 313,
            update_interval_samples: 1_023,
            white_phase_std_rad: 0.0,
            white_frequency_std_hz: 0.0,
            random_walk_frequency_step_std_hz: 12.0,
        },
        ..SyntheticReceiverOscillatorModel::default()
    };
    let noisy = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        receiver_oscillator,
        6_138,
    );
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel::default(),
        6_138,
    );

    let first_phase_delta = noisy.carrier_phase_rad_at(0.001) - nominal.carrier_phase_rad_at(0.001);
    let later_phase_delta = noisy.carrier_phase_rad_at(0.005) - nominal.carrier_phase_rad_at(0.005);

    assert!(first_phase_delta.abs() <= f64::EPSILON);
    assert!(later_phase_delta.abs() > 1.0e-6);
}

#[test]
fn receiver_oscillator_sampling_clock_error_advances_code_phase_at_expected_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 4 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 100.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );
    let skewed = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 0.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 100.0e-6,
            sampling_clock_fractional_drift_per_s: 0.0,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        4_092,
    );
    let t_s = 0.001;
    let actual_extra_chips = skewed.total_chip_phase_at(t_s) - nominal.total_chip_phase_at(t_s);
    let expected_extra_chips = config.code_freq_basis_hz * t_s * 100.0e-6;

    assert!((actual_extra_chips - expected_extra_chips).abs() <= 1.0e-9);
}

#[test]
fn receiver_oscillator_sampling_clock_drift_accelerates_code_phase_quadratically() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let params = SyntheticSignalParams {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        glonass_frequency_channel: None,
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 100.0,
        carrier_phase_rad: 0.0,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    };
    let nominal = SatState::new_with_receiver_oscillator(
        &config,
        params.clone(),
        SyntheticReceiverOscillatorModel::default(),
        4_092,
    );
    let drifted = SatState::new_with_receiver_oscillator(
        &config,
        params,
        SyntheticReceiverOscillatorModel {
            carrier_frequency_bias_hz: 0.0,
            carrier_frequency_drift_hz_per_s: 0.0,
            sampling_clock_fractional_error: 25.0e-6,
            sampling_clock_fractional_drift_per_s: 60.0e-6,
            phase_noise: SyntheticReceiverPhaseNoiseModel::default(),
            noise: SyntheticReceiverOscillatorNoiseModel::default(),
        },
        4_092,
    );
    let t_s = 0.010;
    let actual_extra_chips = drifted.total_chip_phase_at(t_s) - nominal.total_chip_phase_at(t_s);
    let expected_extra_chips = config.code_freq_basis_hz
        * (t_s * 25.0e-6 + 0.5 * 60.0e-6 * t_s * t_s);

    assert!((actual_extra_chips - expected_extra_chips).abs() <= 1.0e-9);
}

#[test]
fn alternating_nav_bit_sign_flips_on_twenty_millisecond_boundaries() {
    assert_eq!(nav_bit_index_at_time_s(-1.0), 0);
    assert_eq!(nav_bit_index_at_time_s(0.0), 0);
    assert_eq!(nav_bit_index_at_time_s(0.019_999_999), 0);
    assert_eq!(nav_bit_index_at_time_s(0.020_000_000), 1);
    assert_eq!(nav_bit_index_at_time_s(0.039_999_999), 1);
    assert_eq!(nav_bit_index_at_time_s(0.040_000_000), 2);

    assert_eq!(nav_bit_sign_at_time_s(false, 0.0), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.0), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.019_999_999), 1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.020_000_000), -1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.039_999_999), -1);
    assert_eq!(nav_bit_sign_at_time_s(true, 0.040_000_000), 1);
}

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
