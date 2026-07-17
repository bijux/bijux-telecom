#![allow(missing_docs)]
use std::f64::consts::TAU;

use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds, SignalBand};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca,
        wrapped_code_phase_error_samples_f64, SyntheticSignalParams,
    },
    AcquisitionEngine, ReceiverPipelineConfig, TrackingEngine,
};
use bijux_gnss_signal::api::{advance_code_phase_seconds, sample_ca_code, samples_per_code, Prn};
use num_complex::Complex;

#[test]
fn synthetic_correlator_peak_ratio() {
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

    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let code_phase_chips = 200.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0xABCDEF01,
        samples_per_code as f64 / config.sampling_freq_hz,
    );

    let local_code = generate_local_code(sat.prn, &config, code_phase_chips, samples_per_code);

    let correct = correlate(&frame.iq, &local_code);
    let incorrect = correlate(&frame.iq, &shift(&local_code, 250));
    let ratio = correct / (incorrect + 1e-6);
    assert!(ratio > 50.0, "peak ratio too low: {ratio}");
}

#[test]
fn synthetic_correlator_peak_ratio_with_non_integer_samples_per_chip() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);

    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let code_phase_chips = 200.375;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0xA11CE123,
        samples_per_code as f64 / config.sampling_freq_hz,
    );

    let local_code = generate_local_code(sat.prn, &config, code_phase_chips, samples_per_code);

    let correct = correlate(&frame.iq, &local_code);
    let incorrect = correlate(&frame.iq, &shift(&local_code, 251));
    let ratio = correct / (incorrect + 1e-6);
    assert!(ratio > 30.0, "non-integer-rate peak ratio too low: {ratio}");
}

#[test]
fn golden_acquisition_run_is_stable() {
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
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 500.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.5,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0xBEEFBEEF,
        samples_per_code as f64 / config.sampling_freq_hz,
    );

    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acquisition = AcquisitionEngine::new(config, runtime).with_doppler(1000, 500);
    let results = acquisition.run_fft(&frame, &[sat]);
    let r = &results[0];

    assert_eq!(r.sat, sat);
    let peak_mean = r.peak_mean_ratio;
    let peak_second = r.peak_second_ratio;

    assert!(peak_mean > 20.0, "peak_mean_ratio degraded: {peak_mean}");
    assert!((peak_second - 1.28).abs() < 0.2, "peak_second_ratio drifted: {peak_second}");
}

#[test]
fn galileo_e1_acquisition_detects_synthetic_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 500.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        },
        0xA11C_E1B0,
        0.020,
    );

    let acquisition = AcquisitionEngine::new(
        config.clone(),
        bijux_gnss_receiver::api::ReceiverRuntime::default(),
    )
    .with_doppler(1_000, 500);
    let mut results = acquisition.run_fft(&frame, &[sat]);
    let result = results.remove(0);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, 321.0) as f64;
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
        result.resolved_code_phase_samples(),
        expected_code_phase_samples,
        period_samples,
    );
    let uncertainty = result.uncertainty.as_ref().expect("Galileo E1 acquisition uncertainty");

    assert_eq!(result.sat, sat);
    assert_eq!(result.signal_band, SignalBand::E1, "result={result:?}");
    assert_eq!(result.hypothesis.to_string(), "accepted", "result={result:?}");
    assert_eq!(result.doppler_hz.0, 500.0, "result={result:?}");
    assert!(
        (result.carrier_hz.0 - 500.0).abs() <= 50.0,
        "Galileo E1 carrier estimate drifted: {}",
        result.carrier_hz.0
    );
    assert!(
        code_phase_error_samples <= 0.5,
        "Galileo E1 code-phase error too high: {code_phase_error_samples} samples for {result:?}"
    );
    assert!(
        result.peak_mean_ratio > 10.0,
        "Galileo E1 peak_mean_ratio too low: {}",
        result.peak_mean_ratio
    );
    assert!(
        result.peak_second_ratio > 1.2,
        "Galileo E1 peak_second_ratio too low: {}",
        result.peak_second_ratio
    );
    assert!(
        uncertainty.doppler_hz.is_finite()
            && uncertainty.doppler_hz > 0.0
            && uncertainty.doppler_hz < 250.0,
        "Galileo E1 Doppler uncertainty out of range: {:?}",
        uncertainty
    );
    assert!(
        uncertainty.code_phase_samples.is_finite()
            && uncertainty.code_phase_samples > 0.0
            && uncertainty.code_phase_samples < 0.5,
        "Galileo E1 code-phase uncertainty out of range: {:?}",
        uncertainty
    );
}

#[test]
fn tracking_correlator_preserves_prompt_phase_across_sixty_second_offset() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let code_phase_chips = 200.375;
    let carrier_hz = 1_234.625;
    let carrier_phase_rad = 0.4;
    let tracking =
        TrackingEngine::new(config.clone(), bijux_gnss_receiver::api::ReceiverRuntime::default());

    let first_frame = synthetic_epoch_frame(
        &config,
        sat.prn,
        0,
        code_phase_chips,
        carrier_hz,
        carrier_phase_rad,
        samples_per_code,
    );
    let offset_sample_index = (60.0 * config.sampling_freq_hz) as u64;
    let offset_code_phase_chips = advance_code_phase_seconds(
        code_phase_chips,
        config.code_freq_basis_hz,
        60.0,
        config.code_length,
    )
    .expect("valid sixty-second code phase");
    let offset_frame = synthetic_epoch_frame(
        &config,
        sat.prn,
        offset_sample_index,
        offset_code_phase_chips,
        carrier_hz,
        carrier_phase_rad,
        samples_per_code,
    );
    let code_phase_samples = code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
    let offset_code_phase_samples =
        offset_code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;

    let first = tracking.correlate_epoch(bijux_gnss_receiver::api::TrackingCorrelationRequest {
        frame: &first_frame,
        sat,
        carrier_hz,
        carrier_phase_cycles: carrier_phase_rad / (std::f64::consts::PI * 2.0),
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples,
        early_late_spacing_chips: 0.5,
    });
    let offset = tracking.correlate_epoch(bijux_gnss_receiver::api::TrackingCorrelationRequest {
        frame: &offset_frame,
        sat,
        carrier_hz,
        carrier_phase_cycles: carrier_phase_rad / (std::f64::consts::PI * 2.0),
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples: offset_code_phase_samples,
        early_late_spacing_chips: 0.5,
    });

    let first_phase = first.prompt.arg();
    let offset_phase = offset.prompt.arg();
    let phase_delta = wrapped_phase_delta(first_phase, offset_phase);
    assert!(
        phase_delta.abs() <= 1e-3,
        "prompt phase drifted across sixty seconds: first={first_phase}, offset={offset_phase}, delta={phase_delta}"
    );
    let amplitude_delta = (first.prompt.norm() - offset.prompt.norm()).abs();
    assert!(
        amplitude_delta <= 1e-3,
        "prompt magnitude drifted across sixty seconds: first={}, offset={}, delta={amplitude_delta}",
        first.prompt.norm(),
        offset.prompt.norm()
    );
}

fn correlate(samples: &[num_complex::Complex<f32>], code: &[f32]) -> f32 {
    let mut acc = num_complex::Complex::new(0.0f32, 0.0f32);
    for (s, &c) in samples.iter().zip(code.iter()) {
        acc += *s * c;
    }
    acc.norm()
}

fn shift(code: &[f32], offset: usize) -> Vec<f32> {
    let n = code.len();
    let mut out = vec![0.0f32; n];
    for i in 0..n {
        out[i] = code[(i + offset) % n];
    }
    out
}

fn generate_local_code(
    prn: u8,
    config: &ReceiverPipelineConfig,
    code_phase_chips: f64,
    samples_per_code: usize,
) -> Vec<f32> {
    sample_ca_code(
        Prn(prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        code_phase_chips,
        samples_per_code,
    )
    .expect("valid local code sampling")
}

fn synthetic_epoch_frame(
    config: &ReceiverPipelineConfig,
    prn: u8,
    start_sample_index: u64,
    code_phase_chips: f64,
    carrier_hz: f64,
    carrier_phase_rad: f64,
    sample_count: usize,
) -> SamplesFrame {
    let local_code = sample_ca_code(
        Prn(prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        code_phase_chips,
        sample_count,
    )
    .expect("valid synthetic local code");
    let iq = local_code
        .into_iter()
        .enumerate()
        .map(|(offset, code)| {
            let sample_index = start_sample_index + offset as u64;
            let phase_rad = (carrier_phase_rad
                + TAU * carrier_hz * (sample_index as f64 / config.sampling_freq_hz))
                .rem_euclid(TAU);
            Complex::new(phase_rad.cos() as f32, phase_rad.sin() as f32) * code
        })
        .collect::<Vec<_>>();
    SamplesFrame::new(
        SampleTime { sample_index: start_sample_index, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        iq,
    )
}

fn wrapped_phase_delta(left: f32, right: f32) -> f32 {
    let mut delta = right - left;
    while delta > std::f32::consts::PI {
        delta -= std::f32::consts::TAU;
    }
    while delta < -std::f32::consts::PI {
        delta += std::f32::consts::TAU;
    }
    delta
}
