#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GlonassFrequencyChannel, SatId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::sim::{
    build_iq16_capture_bundle, generate_l1_ca_multi, SyntheticNavBitMode, SyntheticScenario,
    SyntheticSignalParams,
};
use bijux_gnss_receiver::api::ReceiverPipelineConfig;
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, GLONASS_L1_TIME_MARK};

fn glonass_truth_config(channel: GlonassFrequencyChannel) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 511_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    }
}

#[test]
fn glonass_truth_bundle_records_ten_millisecond_symbol_segments() {
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = glonass_truth_config(channel);
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.041,
        seed: 0x6100_2591,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            glonass_frequency_channel: Some(channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: true,
        }],
        ephemerides: Vec::new(),
        id: "glonass-l1-truth-segments".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("integration glonass-l1-truth-segments".to_string()),
    );
    let satellite = &bundle.truth.satellites[0];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::GlonassL1AlternatingDataString);
    assert_eq!(satellite.nav_bit_segments.len(), 5);
    assert_eq!(satellite.nav_bit_segments[0].start_sample, 0);
    assert_eq!(satellite.nav_bit_segments[0].end_sample, 5_110);
    assert_eq!(satellite.nav_bit_segments[1].start_sample, 5_110);
    assert_eq!(satellite.nav_bit_segments[1].end_sample, 10_220);
    assert_eq!(satellite.nav_bit_segments[4].start_sample, 20_440);
    assert_eq!(satellite.nav_bit_segments[4].end_sample, 20_951);
    assert_eq!(satellite.nav_bit_segments[0].bit, 1);
    assert_eq!(satellite.nav_bit_segments[1].bit, -1);
    assert_eq!(satellite.nav_bit_segments[2].bit, -1);
    assert_eq!(satellite.nav_bit_segments[3].bit, 1);
    assert_eq!(satellite.nav_bit_segments[4].bit, -1);
}

#[test]
fn glonass_truth_bundle_switches_to_time_mark_after_1p7_seconds() {
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = glonass_truth_config(channel);
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 1.710,
        seed: 0x6100_2592,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            glonass_frequency_channel: Some(channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "glonass-l1-time-mark-transition".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("integration glonass-l1-time-mark-transition".to_string()),
    );
    let satellite = &bundle.truth.satellites[0];
    let first_time_mark_segment = &satellite.nav_bit_segments[170];

    assert_eq!(satellite.nav_bit_mode, SyntheticNavBitMode::GlonassL1FixedDataString);
    assert_eq!(first_time_mark_segment.start_sample, 868_700);
    assert_eq!(first_time_mark_segment.end_sample, 873_810);
    assert!((first_time_mark_segment.start_s - 1.700).abs() <= f64::EPSILON);
    assert!((first_time_mark_segment.end_s - 1.710).abs() <= f64::EPSILON);
    assert_eq!(first_time_mark_segment.bit, GLONASS_L1_TIME_MARK[0]);
}
