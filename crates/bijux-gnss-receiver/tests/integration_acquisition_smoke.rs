#![allow(missing_docs)]
use bijux_gnss_receiver::api::{
    AcquisitionEngine,
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::{generate_ca_code, samples_per_code, Prn};

#[test]
fn bench_code_gen_smoke() {
    let _ = generate_ca_code(Prn(1)).expect("valid PRN");
}

#[test]
fn bench_fft_acquisition_smoke() {
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
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat: bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: 1,
            },
            doppler_hz: 0.0,
            code_phase_chips: 100.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 55.0,
            data_bit_flip: false,
        },
        0x1234_5678,
        samples_per_code as f64 / config.sampling_freq_hz,
    );
    let runtime = bijux_gnss_receiver::api::ReceiverRuntimeConfig::default();
    let acquisition = Acquisition::new(config, runtime).with_doppler(0, 500);
    let sat = bijux_gnss_core::api::SatId {
        constellation: bijux_gnss_core::api::Constellation::Gps,
        prn: 1,
    };
    let _ = acquisition.run_fft(&frame, &[sat]);
}
