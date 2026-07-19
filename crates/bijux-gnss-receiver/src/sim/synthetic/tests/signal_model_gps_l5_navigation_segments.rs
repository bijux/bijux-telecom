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
