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
