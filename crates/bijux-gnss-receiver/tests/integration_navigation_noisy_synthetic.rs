#![allow(missing_docs)]

mod support;

use support::navigation_noise::{synthetic_pvt_noise_evidence, synthetic_pvt_noise_sweep};

#[test]
fn noisy_synthetic_navigation_increases_position_error_with_pseudorange_noise() {
    let points = synthetic_pvt_noise_sweep();

    assert!(points.len() >= 2, "expected multiple noise profiles, got {}", points.len());
    let clean = points.first().expect("clean noise profile");
    assert_eq!(clean.max_abs_pseudorange_noise_m, 0.0, "{}", synthetic_pvt_noise_evidence(&points));
    assert!(
        clean.max_position_error_3d_m < 5.0,
        "clean synthetic navigation is not accurate: {}",
        synthetic_pvt_noise_evidence(&points)
    );

    for window in points.windows(2) {
        let current = &window[0];
        let next = &window[1];

        assert!(
            next.max_abs_pseudorange_noise_m > current.max_abs_pseudorange_noise_m,
            "noise sweep is not strictly increasing: {}",
            synthetic_pvt_noise_evidence(&points),
        );
    }

    for noisy in points.iter().skip(1) {
        assert!(
            noisy.max_position_error_3d_m > clean.max_position_error_3d_m,
            "injected pseudorange noise did not degrade navigation output: {}",
            synthetic_pvt_noise_evidence(&points),
        );
        assert!(
            noisy.valid_solution_count < noisy.solution_count
                || noisy.max_position_error_3d_m > noisy.max_abs_pseudorange_noise_m,
            "noisy navigation neither refused corrupted epochs nor reported amplified position error: {}",
            synthetic_pvt_noise_evidence(&points),
        );
    }
}
