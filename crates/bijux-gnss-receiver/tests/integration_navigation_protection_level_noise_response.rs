#![allow(missing_docs)]

mod support;

use support::navigation_protection_faults::{
    synthetic_protection_fault_evidence, synthetic_protection_fault_sweep,
};

#[test]
fn pseudorange_fault_sweep_raises_protection_response_or_invalidates_epochs() {
    let points = synthetic_protection_fault_sweep();

    assert!(points.len() >= 2, "expected multiple synthetic fault profiles");

    for window in points.windows(2) {
        let current = &window[0];
        let next = &window[1];

        assert!(
            next.max_abs_pseudorange_noise_m > current.max_abs_pseudorange_noise_m,
            "fault sweep is not strictly increasing: {}",
            synthetic_protection_fault_evidence(&points),
        );
        assert!(
            next.max_hpl_m > current.max_hpl_m
                || next.max_vpl_m > current.max_vpl_m
                || next.invalid_epoch_count > current.invalid_epoch_count,
            "synthetic faults did not raise protection levels or invalid epochs: {}",
            synthetic_protection_fault_evidence(&points),
        );
    }
}
