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
    std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq16", name, std::process::id(), nanos))
}

#[test]
fn signed_16bit_iq_matches_float_fixture_acquisition() {
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
    let mut frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 500.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.5,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0x16BEEF01,
        samples_per_code as f64 / config.sampling_freq_hz,
    );
    normalize_iq(&mut frame.iq);

    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acquisition = AcquisitionEngine::new(config.clone(), runtime).with_doppler(1000, 500);
    let float_result = acquisition.run_fft(&frame, &[sat]).remove(0);

    let path = temp_file_path("acquisition_parity");
    fs::write(&path, quantize_frame_to_i16_le_bytes(&frame.iq)).expect("write iq16 fixture");

    let metadata = RawIqMetadata {
        format: IqSampleFormat::Iq16Le,
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        capture_start_utc: "2026-07-09T00:00:00Z".to_string(),
        offset_bytes: 0,
        quantization_bits: Some(16),
        notes: Some("synthetic acquisition parity fixture".to_string()),
    };
    let mut source = FileSamples::open_raw_iq(&path, metadata).expect("open iq16 fixture");
    let iq16_frame = source.next_frame(samples_per_code).expect("read frame").expect("frame");
    let iq16_result = acquisition.run_fft(&iq16_frame, &[sat]).remove(0);
    let float_coarse_carrier_hz = float_result
        .doppler_refinement
        .as_ref()
        .map(|refinement| refinement.coarse_carrier_hz)
        .unwrap_or(float_result.carrier_hz);
    let iq16_coarse_carrier_hz = iq16_result
        .doppler_refinement
        .as_ref()
        .map(|refinement| refinement.coarse_carrier_hz)
        .unwrap_or(iq16_result.carrier_hz);

    assert_eq!(iq16_result.sat, float_result.sat);
    assert_eq!(iq16_coarse_carrier_hz, float_coarse_carrier_hz);
    assert!(
        (iq16_result.carrier_hz.0 - float_result.carrier_hz.0).abs() <= 1.0,
        "refined carrier drifted too far: {} vs {}",
        iq16_result.carrier_hz.0,
        float_result.carrier_hz.0
    );
    assert_eq!(iq16_result.code_phase_samples, float_result.code_phase_samples);
    assert!(
        iq16_result.peak_mean_ratio > 20.0,
        "peak mean ratio fell below a strong acquisition margin: {}",
        iq16_result.peak_mean_ratio,
    );
    assert!(
        iq16_result.peak_mean_ratio / float_result.peak_mean_ratio > 0.98,
        "peak mean ratio degraded too far: {} vs {}",
        iq16_result.peak_mean_ratio,
        float_result.peak_mean_ratio
    );
    assert!(
        iq16_result.peak_second_ratio > 1.2,
        "peak second ratio fell below a usable separation margin: {}",
        iq16_result.peak_second_ratio,
    );

    fs::remove_file(&path).expect("remove iq16 fixture");
}

fn quantize_frame_to_i16_le_bytes(samples: &[num_complex::Complex<f32>]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 4);
    for sample in samples {
        encoded.extend_from_slice(&quantize_component(sample.re).to_le_bytes());
        encoded.extend_from_slice(&quantize_component(sample.im).to_le_bytes());
    }
    encoded
}

fn quantize_component(value: f32) -> i16 {
    let scaled = (value * 32768.0).round();
    scaled.clamp(-32768.0, 32767.0) as i16
}

fn normalize_iq(samples: &mut [num_complex::Complex<f32>]) {
    let peak =
        samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max);
    if peak <= 0.999 {
        return;
    }
    let scale = 0.999 / peak;
    for sample in samples {
        sample.re *= scale;
        sample.im *= scale;
    }
}
