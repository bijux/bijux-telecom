use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{ClkProvider, Sp3Provider};
use criterion::{criterion_group, criterion_main, BatchSize, Criterion};
use std::str::FromStr;

fn sample_sp3() -> Sp3Provider {
    let data = r#"
* 2020 01 01 00 00 00.000000
PG01 10000.000000 20000.000000 21000.000000
* 2020 01 01 00 15 00.000000
PG01 10010.000000 20010.000000 21010.000000
"#;
    Sp3Provider::from_str(data).expect("sp3 parse")
}

fn sample_clk() -> ClkProvider {
    let data = r#"
AS G01 2020 01 01 00 00 00.000000 0 0 0 0.000001
AS G01 2020 01 01 00 15 00.000000 0 0 0 0.000002
"#;
    ClkProvider::from_str(data).expect("clk parse")
}

fn bench_precise_products(c: &mut Criterion) {
    let sp3 = sample_sp3();
    let clk = sample_clk();
    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 1,
    };
    c.bench_function("sp3_interpolation", |b| {
        b.iter_batched(
            || sp3.clone(),
            |provider| {
                let _ = provider.sat_state(sat, 30.0);
            },
            BatchSize::SmallInput,
        )
    });
    c.bench_function("clk_interpolation", |b| {
        b.iter_batched(
            || clk.clone(),
            |provider| {
                let _ = provider.bias_s(sat, 30.0);
            },
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, bench_precise_products);
criterion_main!(benches);
