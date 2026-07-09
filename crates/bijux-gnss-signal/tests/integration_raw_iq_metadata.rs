#![allow(missing_docs)]

use bijux_gnss_signal::api::{IqSampleFormat, RawIqMetadata};

#[test]
fn iq_sample_formats_report_encoded_width() {
    assert_eq!(IqSampleFormat::Iq8.bytes_per_complex_sample(), 2);
    assert_eq!(IqSampleFormat::Iq16Le.bytes_per_complex_sample(), 4);
    assert_eq!(IqSampleFormat::Cf32Le.bytes_per_complex_sample(), 8);
}

#[test]
fn raw_iq_metadata_round_trips_via_toml() {
    let spec = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: 5_000_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 512,
        quantization_bits: Some(16),
        notes: Some("fixture".to_string()),
    };

    let encoded = toml::to_string(&spec).expect("encode toml");
    let decoded: RawIqMetadata = toml::from_str(&encoded).expect("decode toml");

    assert_eq!(decoded, spec);
}
