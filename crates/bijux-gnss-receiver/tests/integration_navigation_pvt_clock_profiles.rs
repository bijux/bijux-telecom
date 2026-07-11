#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use navigation_clock_profile::{
    build_navigation_clock_case, synthetic_navigation_clock_profiles, truth_clock_bias_by_epoch,
};

const MAX_CLOCK_BIAS_ERROR_S: f64 = 1.0e-9;

#[test]
fn pvt_truth_table_tracks_receiver_clock_bias_across_navigation_profiles() {
    for profile in synthetic_navigation_clock_profiles() {
        let expected_clock_bias_s = truth_clock_bias_by_epoch(&profile);
        let case = build_navigation_clock_case(profile.clone());

        assert_eq!(case.truth_table.solution_count, case.solutions.len());
        assert_eq!(case.truth_table.matched_epoch_count, case.truth_table.epochs.len());
        assert!(case.truth_table.unmatched_solution_epochs.is_empty(), "{:?}", case.truth_table);
        assert!(case.truth_table.unused_reference_epochs.is_empty(), "{:?}", case.truth_table);

        for epoch in &case.truth_table.epochs {
            let truth_clock_bias_s = expected_clock_bias_s
                .get(&epoch.epoch_index)
                .copied()
                .expect("truth clock bias for matched epoch");

            assert_eq!(epoch.clock_bias.truth_s, truth_clock_bias_s);
            assert_eq!(epoch.clock_bias.truth_m, truth_clock_bias_s * 299_792_458.0);
            assert!(epoch.clock_bias.measured_s.is_finite(), "{epoch:?}");
            assert!(epoch.clock_bias.measured_m.is_finite(), "{epoch:?}");
            assert!(epoch.clock_bias.error_s.abs() <= MAX_CLOCK_BIAS_ERROR_S, "{epoch:?}");
            assert!(
                epoch.clock_bias.error_m.abs() <= MAX_CLOCK_BIAS_ERROR_S * 299_792_458.0,
                "{epoch:?}"
            );
            assert!(epoch.valid, "{epoch:?}");
        }
    }
}
