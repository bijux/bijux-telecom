#![allow(missing_docs)]

mod support;

use support::navigation_protection_faults::{
    synthetic_protection_fault_evidence, synthetic_protection_fault_sweep,
};

#[test]
fn pseudorange_fault_sweep_raises_protection_response_or_invalidates_epochs() {
    let points = synthetic_protection_fault_sweep();

    assert!(points.len() >= 2, "expected multiple synthetic fault profiles");
    let clean = points.first().expect("clean protection profile");
    assert_eq!(
        clean.max_abs_pseudorange_noise_m,
        0.0,
        "{}",
        synthetic_protection_fault_evidence(&points)
    );
    assert_eq!(clean.invalid_epoch_count, 0, "{}", synthetic_protection_fault_evidence(&points));
    assert_eq!(
        clean.protected_epoch_count,
        clean.solution_count,
        "{}",
        synthetic_protection_fault_evidence(&points)
    );

    for window in points.windows(2) {
        let current = &window[0];
        let next = &window[1];

        assert!(
            next.max_abs_pseudorange_noise_m > current.max_abs_pseudorange_noise_m,
            "fault sweep is not strictly increasing: {}",
            synthetic_protection_fault_evidence(&points),
        );
    }

    for faulty in points.iter().skip(1) {
        assert!(
            faulty.max_hpl_m > clean.max_hpl_m
                || faulty.max_vpl_m > clean.max_vpl_m
                || faulty.invalid_epoch_count > 0
                || faulty.protected_epoch_count < faulty.solution_count
                || faulty.horizontal_breach_epoch_count > 0
                || faulty.vertical_breach_epoch_count > 0,
            "synthetic faults did not raise, breach, invalidate, or suppress protection output: {}",
            synthetic_protection_fault_evidence(&points),
        );
    }
}
