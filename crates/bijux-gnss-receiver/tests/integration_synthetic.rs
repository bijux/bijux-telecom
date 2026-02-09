#![allow(missing_docs)]
use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    AcquisitionEngine,
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::samples_per_code;

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
    let samples_per_code = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );

    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 3,
    };
    let code_phase_chips = 200.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
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
fn golden_acquisition_run_is_stable() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );

    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 7,
    };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 500.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.5,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
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

    assert!(
        (peak_mean - 69.3).abs() < 5.0,
        "peak_mean_ratio drifted: {peak_mean}"
    );
    assert!(
        (peak_second - 1.28).abs() < 0.2,
        "peak_second_ratio drifted: {peak_second}"
    );
}

#[test]
fn synthetic_supports_multi_constellation_mock() {
    let config = ReceiverPipelineConfig::default();
    let _frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat: SatId {
                constellation: Constellation::Galileo,
                prn: 11,
            },
            doppler_hz: 0.0,
            code_phase_chips: 10.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 45.0,
            data_bit_flip: false,
        },
        0x1234,
        0.001,
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
    let code = bijux_gnss_signal::api::generate_ca_code(bijux_gnss_signal::api::Prn(prn))
        .expect("valid PRN");
    let mut out = Vec::with_capacity(samples_per_code);
    let dt_s = 1.0 / config.sampling_freq_hz;
    for n in 0..samples_per_code {
        let t = n as f64 * dt_s;
        let code_phase = code_phase_chips + config.code_freq_basis_hz * t;
        let chip_index = (code_phase.floor() as usize) % code.len();
        out.push(code[chip_index] as f32);
    }
    out
}
