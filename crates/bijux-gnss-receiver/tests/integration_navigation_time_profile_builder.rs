#![allow(missing_docs)]

#[path = "support/navigation_time_profile.rs"]
mod navigation_time_profile;

use navigation_time_profile::{
    build_navigation_time_case, max_end_abs_pseudorange_bias_m, max_start_abs_pseudorange_bias_m,
    synthetic_navigation_time_profiles,
};

#[test]
fn navigation_time_profiles_build_truth_ready_long_run_cases() {
    let profiles = synthetic_navigation_time_profiles();
    assert_eq!(profiles.len(), 3);

    for profile in profiles {
        let expected_epoch_count = profile.truth_epochs.len();
        let start_bias_m = max_start_abs_pseudorange_bias_m(&profile);
        let end_bias_m = max_end_abs_pseudorange_bias_m(&profile);
        let case = build_navigation_time_case(profile.clone());

        assert_eq!(case.time_profile.profile_name, profile.profile_name);
        assert_eq!(case.time_profile.truth_epochs.len(), expected_epoch_count);
        assert_eq!(case.truth_table.solution_count, case.solutions.len());
        assert_eq!(case.truth_table.matched_epoch_count, case.truth_table.epochs.len());
        assert!(case.truth_table.unmatched_solution_epochs.is_empty(), "{:?}", case.truth_table);
        assert!(case.truth_table.unused_reference_epochs.is_empty(), "{:?}", case.truth_table);
        assert!(case.truth_table.epochs.len() >= 2, "{:?}", case.truth_table);
        assert_eq!(max_start_abs_pseudorange_bias_m(&case.time_profile), start_bias_m);
        assert_eq!(max_end_abs_pseudorange_bias_m(&case.time_profile), end_bias_m);
    }
}
