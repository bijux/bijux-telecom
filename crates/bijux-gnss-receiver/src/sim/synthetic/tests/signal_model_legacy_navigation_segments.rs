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
