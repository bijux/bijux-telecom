#![allow(missing_docs)]

use std::fs;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::signal::{samples_per_code, IqSampleFormat, RawIqMetadata};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, FileSamples, ReceiverPipelineConfig, SignalSource,
};

fn temp_file_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!("bijux_{}_{}_{}.cf32", name, std::process::id(), nanos))
}

#[test]
fn complex_float32_iq_matches_float_fixture_acquisition() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 500.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.5,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0xCF32BEEF,
        samples_per_code as f64 / config.sampling_freq_hz,
    );

    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acquisition = AcquisitionEngine::new(config.clone(), runtime).with_doppler(1000, 500);
    let float_result = acquisition.run_fft(&frame, &[sat]).remove(0);

    let path = temp_file_path("acquisition_parity");
    fs::write(&path, encode_frame_to_cf32_le_bytes(&frame.iq)).expect("write cf32 fixture");

    let metadata = RawIqMetadata {
        format: IqSampleFormat::Cf32Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(32),
        notes: Some("synthetic acquisition parity fixture".to_string()),
    };
    let mut source = FileSamples::open_raw_iq(&path, metadata).expect("open cf32 fixture");
    let cf32_frame = source.next_frame(samples_per_code).expect("read frame").expect("frame");
    let cf32_result = acquisition.run_fft(&cf32_frame, &[sat]).remove(0);
    let float_coarse_carrier_hz = float_result
        .doppler_refinement
        .as_ref()
        .map(|refinement| refinement.coarse_carrier_hz)
        .unwrap_or(float_result.carrier_hz);
    let cf32_coarse_carrier_hz = cf32_result
        .doppler_refinement
        .as_ref()
        .map(|refinement| refinement.coarse_carrier_hz)
        .unwrap_or(cf32_result.carrier_hz);

    assert_eq!(cf32_result.sat, float_result.sat);
    assert_eq!(cf32_coarse_carrier_hz, float_coarse_carrier_hz);
    assert!((cf32_result.carrier_hz.0 - float_result.carrier_hz.0).abs() < 1e-6);
    assert_eq!(cf32_result.code_phase_samples, float_result.code_phase_samples);
    assert!((cf32_result.peak_mean_ratio - float_result.peak_mean_ratio).abs() < 1e-6);
    assert!((cf32_result.peak_second_ratio - float_result.peak_second_ratio).abs() < 1e-6);

    fs::remove_file(&path).expect("remove cf32 fixture");
}

fn encode_frame_to_cf32_le_bytes(samples: &[num_complex::Complex<f32>]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 8);
    for sample in samples {
        encoded.extend_from_slice(&sample.re.to_le_bytes());
        encoded.extend_from_slice(&sample.im.to_le_bytes());
    }
    encoded
}
