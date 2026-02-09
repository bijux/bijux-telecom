use bijux_gnss_core::api::{SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{AcquisitionEngine, ReceiverPipelineConfig};
use criterion::{criterion_group, criterion_main, BatchSize, Criterion};
use num_complex::Complex;

fn bench_acquisition_fft(c: &mut Criterion) {
    let config = ReceiverPipelineConfig::default();
    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acq = AcquisitionEngine::new(config.clone(), runtime);
    let samples_per_code = bijux_gnss_signal::api::samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    let n = samples_per_code;
    let iq = vec![Complex::new(1.0f32, 0.0f32); n];
    let frame = SamplesFrame::new(
        SampleTime {
            sample_index: 0,
            sample_rate_hz: config.sampling_freq_hz,
        },
        Seconds(1.0 / config.sampling_freq_hz),
        iq,
    );
    let sat = SatId {
        constellation: bijux_gnss_core::api::Constellation::Gps,
        prn: 1,
    };
    c.bench_function("acquisition_fft_1ms", |b| {
        b.iter_batched(
            || frame.clone(),
            |frame| {
                let _ = acq.run_fft(&frame, &[sat]);
            },
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, bench_acquisition_fft);
criterion_main!(benches);
