use bijux_gnss_receiver::{
    acquisition::Acquisition,
    ca_code::{generate_ca_code, Prn},
    signal::samples_per_code,
    synthetic::{generate_l1_ca, SyntheticSignalParams},
    types::ReceiverConfig,
};
use criterion::{criterion_group, criterion_main, Criterion};

fn bench_code_gen(c: &mut Criterion) {
    c.bench_function("ca_code_prn1", |b| {
        b.iter(|| {
            let _ = generate_ca_code(Prn(1));
        })
    });
}

fn bench_fft_acquisition(c: &mut Criterion) {
    let config = ReceiverConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverConfig::default()
    };
    let samples_per_code = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat: bijux_gnss_core::SatId {
                constellation: bijux_gnss_core::Constellation::Gps,
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
    let acquisition = Acquisition::new(config).with_doppler(0, 500);
    let sat = bijux_gnss_core::SatId {
        constellation: bijux_gnss_core::Constellation::Gps,
        prn: 1,
    };

    c.bench_function("acquisition_fft_prn1", |b| {
        b.iter(|| {
            let _ = acquisition.run_fft(&frame, &[sat]);
        })
    });
}

criterion_group!(benches, bench_code_gen, bench_fft_acquisition);
criterion_main!(benches);
