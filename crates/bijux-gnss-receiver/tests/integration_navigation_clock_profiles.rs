#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_clock_profile, SyntheticPvtClockProfileCase,
};
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

#[test]
fn navigation_clock_profile_report_tracks_injected_clock_drift() {
    let cases = synthetic_navigation_clock_profiles()
        .into_iter()
        .map(build_navigation_clock_case)
        .collect::<Vec<_>>();
    let report = summarize_truth_guided_pvt_clock_profile(
        &cases
            .iter()
            .map(|case| SyntheticPvtClockProfileCase {
                scenario_id: &case.scenario_id,
                injected_clock_drift_s_per_s: truth_clock_drift_s_per_s(&case.clock_profile),
                solutions: &case.solutions,
                accuracy: &case.pvt_accuracy,
            })
            .collect::<Vec<_>>(),
        "navigation_clock_profile",
    );

    assert_eq!(report.points.len(), cases.len());

    let stable = &report.points[0];
    let drifting = &report.points[1];

    assert!(stable.ready && drifting.ready, "{report:?}");
    assert_eq!(stable.injected_clock_drift_s_per_s, 0.0);
    assert!(
        stable.final_solved_clock_drift_s_per_s.expect("stable drift").abs() <= 1.0e-12,
        "{report:?}"
    );

    assert!(drifting.injected_clock_drift_s_per_s > stable.injected_clock_drift_s_per_s);
    assert_eq!(drifting.epoch_count, drifting.passing_epoch_count, "{report:?}");
    assert_eq!(drifting.pass_rate, 1.0, "{report:?}");
    assert!(
        (drifting.final_solved_clock_drift_s_per_s.expect("final solved drift")
            - drifting.injected_clock_drift_s_per_s)
            .abs()
            <= 1.0e-8,
        "{report:?}"
    );
    assert!(drifting.max_residual_rms_m.expect("residual rms") >= 0.0, "{report:?}");
}
