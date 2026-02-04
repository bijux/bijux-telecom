use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{TrackingEngine, ReceiverRuntimeConfig};
use criterion::{criterion_group, criterion_main, BatchSize, Criterion};
use num_complex::Complex;

fn bench_correlator(c: &mut Criterion) {
    let config = ReceiverRuntimeConfig::default();
    let tracking = Tracking::new(config.clone());
    let samples_per_code = bijux_gnss_signal::api::samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    let iq = vec![Complex::new(0.5f32, -0.2f32); samples_per_code];
    let frame = SamplesFrame::new(
        SampleTime {
            sample_index: 0,
            sample_rate_hz: config.sampling_freq_hz,
        },
        Seconds(1.0 / config.sampling_freq_hz),
        iq,
    );
    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 1,
    };
    c.bench_function("tracking_correlator_epoch", |b| {
        b.iter_batched(
            || frame.clone(),
            |frame| {
                let _ = tracking.correlate_epoch(&frame, sat, 0.0, 0.0, 0.5);
            },
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, bench_correlator);
criterion_main!(benches);
