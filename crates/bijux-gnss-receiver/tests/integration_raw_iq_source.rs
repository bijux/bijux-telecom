#![allow(missing_docs)]

use std::fs;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_receiver::api::signal::{IqSampleFormat, RawIqMetadata};
use bijux_gnss_receiver::api::{FileSamples, SignalSource};

fn temp_file_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!("bijux_{}_{}_{}.bin", name, std::process::id(), nanos))
}

#[test]
fn file_samples_open_raw_iq_uses_declared_metadata() {
    let path = temp_file_path("raw_iq_source");
    fs::write(&path, [0u8, 0u8, 0u8, 0u8, 255u8, 127u8, 0u8, 128u8]).expect("write iq file");

    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: 5_000_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: None,
    };
    let mut source = FileSamples::open_raw_iq(&path, metadata.clone()).expect("open raw iq");

    let frame = source.next_frame(2).expect("read frame").expect("frame");
    assert_eq!(source.metadata(), &metadata);
    assert_eq!(frame.t0.sample_rate_hz, metadata.sample_rate_hz);
    assert_eq!(frame.t0.sample_index, 0);
    assert_eq!(frame.len(), 2);

    fs::remove_file(&path).expect("remove iq file");
}

#[test]
fn file_samples_rejects_unsupported_raw_iq_format() {
    let path = temp_file_path("raw_iq_unsupported");
    fs::write(&path, [0u8; 8]).expect("write iq file");

    let metadata = RawIqMetadata {
        format: IqSampleFormat::Cf32Le,
        sample_rate_hz: 5_000_000.0,
        intermediate_freq_hz: 0.0,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(32),
        notes: None,
    };
    let mut source = FileSamples::open_raw_iq(&path, metadata).expect("open raw iq");
    let err = source.next_frame(1).expect_err("format must be rejected");

    assert!(err.to_string().contains("not supported"));

    fs::remove_file(&path).expect("remove iq file");
}
