#![allow(missing_docs)]

mod support;

use support::navigation_noise::{synthetic_pvt_noise_evidence, synthetic_pvt_noise_sweep};

#[test]
fn noisy_synthetic_navigation_increases_position_error_with_pseudorange_noise() {
    let points = synthetic_pvt_noise_sweep();

    assert!(
        points.len() >= 2,
        "expected multiple noise profiles, got {}",
        points.len()
    );

    for window in points.windows(2) {
        let current = &window[0];
        let next = &window[1];

        assert!(
            next.max_abs_pseudorange_noise_m > current.max_abs_pseudorange_noise_m,
            "noise sweep is not strictly increasing: {}",
            synthetic_pvt_noise_evidence(&points),
        );
        assert!(
            next.max_position_error_3d_m > current.max_position_error_3d_m,
            "position error did not increase with injected pseudorange noise: {}",
            synthetic_pvt_noise_evidence(&points),
        );
    }
}
