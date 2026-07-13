#![allow(missing_docs)]

use std::fs;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use bijux_gnss_core::api::{Constellation, SatId, SamplesFrame, SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    sim::{build_quantized_capture_bundle, generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
    signal::{quantize_samples_for_storage, IqQuantization},
    FileSamples, ReceiverPipelineConfig, SignalSource,
};

fn temp_file_path(name: &str, extension: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!(
        "bijux_{}_{}_{}.{}",
        name,
        std::process::id(),
        nanos,
        extension
    ))
}

#[test]
fn quantized_capture_bundle_round_trips_through_raw_iq_ingest() {
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
        seed: 91,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: -500.0,
            code_phase_chips: 123.0,
            carrier_phase_rad: 0.3,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "quantized-round-trip".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);

    for quantization in [
        IqQuantization::Float32,
        IqQuantization::Signed16Bit,
        IqQuantization::Signed8Bit,
        IqQuantization::Signed4Bit,
        IqQuantization::Signed2Bit,
        IqQuantization::Bipolar1Bit,
    ] {
        let bundle = build_quantized_capture_bundle(
            &scenario.id,
            &scenario,
            &frame,
            quantization,
            "2026-07-13T00:00:00Z",
            Some("quantized raw-iq round trip".to_string()),
        );
        let expected = expected_quantized_frame(&frame, bundle.truth.output_scale_applied, quantization);
        let extension = match quantization {
            IqQuantization::Float32 => "cf32",
            IqQuantization::Signed16Bit => "iq16",
            _ => "iq8",
        };
        let path = temp_file_path("quantized_round_trip", extension);
        fs::write(&path, &bundle.raw_iq_bytes).expect("write quantized fixture");

        let mut source =
            FileSamples::open_raw_iq(&path, bundle.metadata.clone()).expect("open quantized fixture");
        let decoded = source
            .next_frame(frame.len())
            .expect("read quantized frame")
            .expect("decoded frame");

        assert_eq!(decoded.len(), expected.len(), "{quantization:?}");
        for (decoded_sample, expected_sample) in decoded.iq.iter().zip(expected.iq.iter()) {
            assert!(
                (decoded_sample.re - expected_sample.re).abs() < 1e-6,
                "{quantization:?}: I mismatch decoded={decoded_sample:?} expected={expected_sample:?}"
            );
            assert!(
                (decoded_sample.im - expected_sample.im).abs() < 1e-6,
                "{quantization:?}: Q mismatch decoded={decoded_sample:?} expected={expected_sample:?}"
            );
        }

        fs::remove_file(&path).expect("remove quantized fixture");
    }
}

fn expected_quantized_frame(
    frame: &SamplesFrame,
    output_scale_applied: f32,
    quantization: IqQuantization,
) -> SamplesFrame {
    let scaled = frame.iq.iter().map(|sample| *sample * output_scale_applied).collect::<Vec<_>>();
    SamplesFrame::new(frame.t0, frame.dt_s, quantize_samples_for_storage(&scaled, quantization))
}
