#![allow(missing_docs)]
use bijux_gnss_signal::api::{generate_ca_code, Prn};
use proptest::prelude::*;
use rustfft::{num_complex::Complex, FftPlanner};

proptest! {
    #[test]
    fn ca_code_is_deterministic(prn in 1u8..=32) {
        let a = generate_ca_code(Prn(prn)).expect("valid PRN");
        let b = generate_ca_code(Prn(prn)).expect("valid PRN");
        prop_assert_eq!(a, b);
    }

    #[test]
    fn ca_code_has_only_pm_one(prn in 1u8..=32) {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        prop_assert!(code.iter().all(|&c| c == 1 || c == -1));
    }
}

#[test]
fn fft_ifft_round_trip_tolerance() {
    let n = 1024;
    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut data: Vec<Complex<f32>> = (0..n)
        .map(|i| Complex::new((i as f32).sin(), (i as f32).cos()))
        .collect();

    let original = data.clone();
    fft.process(&mut data);
    ifft.process(&mut data);

    let scale = n as f32;
    for (a, b) in original.iter().zip(data.iter()) {
        let diff = (*a - *b / scale).norm();
        assert!(diff < 1e-3, "fft round-trip error too large: {diff}");
    }
}
