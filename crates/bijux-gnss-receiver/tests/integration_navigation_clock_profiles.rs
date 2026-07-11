#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use navigation_clock_profile::{
    build_navigation_clock_case, synthetic_navigation_clock_profiles, truth_clock_drift_s_per_s,
};

#[test]
fn synthetic_navigation_clock_profiles_produce_truth_ready_navigation_cases() {
    let profiles = synthetic_navigation_clock_profiles();
    assert_eq!(profiles.len(), 2);

    for profile in profiles {
        let expected_drift_s_per_s = truth_clock_drift_s_per_s(&profile);
        let expected_epoch_count = profile.truth_epochs.len();
        let case = build_navigation_clock_case(profile.clone());

        assert_eq!(case.clock_profile.profile_name, profile.profile_name);
        assert_eq!(case.truth_table.scenario_id, case.scenario_id);
        assert_eq!(case.truth_table.solution_count, case.solutions.len());
        assert_eq!(case.truth_table.matched_epoch_count, case.solutions.len());
        assert_eq!(case.truth_table.epochs.len(), case.solutions.len());
        assert!(case.truth_table.unmatched_solution_epochs.is_empty(), "{:?}", case.truth_table);
        assert!(case.truth_table.unused_reference_epochs.is_empty(), "{:?}", case.truth_table);
        assert_eq!(case.clock_profile.truth_epochs.len(), expected_epoch_count);
        assert_eq!(truth_clock_drift_s_per_s(&case.clock_profile), expected_drift_s_per_s);
        assert!(!case.observations.is_empty());
        assert!(!case.solutions.is_empty());
    }
}
