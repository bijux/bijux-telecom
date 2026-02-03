#![allow(missing_docs)]
use bijux_gnss_signal::Nco;
use proptest::prelude::*;

proptest! {
    #[test]
    fn nco_outputs_unit_magnitude(freq in -1.0e6f64..1.0e6f64, sample_rate in 1.0e3f64..20.0e6f64) {
        let mut nco = Nco::new(freq, sample_rate);
        for _ in 0..1024 {
            let (s, c) = nco.next_sin_cos();
            let mag = (s * s + c * c).sqrt();
            prop_assert!((mag - 1.0).abs() < 1e-9);
        }
    }
}
