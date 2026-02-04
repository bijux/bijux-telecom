use bijux_gnss_core::{Constellation, SatId, SigId, SignalBand, SignalCode};
use bijux_gnss_nav::{Ekf, EkfConfig, Matrix, PseudorangeMeasurement};
use criterion::{criterion_group, criterion_main, BatchSize, Criterion};

fn bench_ekf_update(c: &mut Criterion) {
    let config = EkfConfig {
        gating_chi2_code: Some(1e9),
        gating_chi2_phase: Some(1e9),
        gating_chi2_doppler: Some(1e9),
        huber_k: None,
        square_root: true,
        covariance_epsilon: 1e-9,
        divergence_max_variance: 1e12,
    };
    let meas = PseudorangeMeasurement {
        sig: SigId {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 3,
            },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        },
        z_m: 20_200_000.0,
        sat_pos_m: [15_000_000.0, 0.0, 21_000_000.0],
        sat_clock_s: 0.0,
        tropo_m: 0.0,
        iono_m: 0.0,
        sigma_m: 10.0,
        elevation_deg: None,
        ztd_index: None,
        isb_index: None,
    };
    c.bench_function("ekf_update_pseudorange", |b| {
        b.iter_batched(
            || Ekf::new(vec![0.0; 8], Matrix::identity(8), config.clone()),
            |mut ekf| {
                let _ = ekf.update(&meas);
            },
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, bench_ekf_update);
criterion_main!(benches);
