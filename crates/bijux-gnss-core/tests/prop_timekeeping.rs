#![allow(missing_docs)]
use bijux_gnss_core::api::{GpsTime, SampleClock};
use proptest::prelude::*;

proptest! {
    #[test]
    fn gps_time_roundtrip(seconds in 0.0f64..10_000_000.0f64) {
        let t = GpsTime::from_seconds(seconds);
        let back = t.to_seconds();
        let expected = seconds % (604_800.0 * 1024.0);
        prop_assert!((back - expected).abs() < 1e-6);
    }

    #[test]
    fn sample_clock_epochs_monotonic(rate in 1_000.0f64..10_000_000.0f64, idx in 0u64..10_000_000u64) {
        let clock = SampleClock::new(rate);
        let e1 = clock.epoch_from_samples(idx);
        let e2 = clock.epoch_from_samples(idx + 1);
        prop_assert!(e2.index >= e1.index);
    }
}
