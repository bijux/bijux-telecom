#![allow(missing_docs)]

#[path = "support/navigation_clock_profile.rs"]
mod navigation_clock_profile;

use bijux_gnss_receiver::api::sim::truth_guided_receiver_accuracy_budgets;

use navigation_clock_profile::{
    build_navigation_clock_case, synthetic_navigation_clock_profile,
    synthetic_navigation_clock_profiles,
};

#[test]
fn navigation_clock_profiles_stay_within_truth_guided_accuracy_budget() {
    let budgets = truth_guided_receiver_accuracy_budgets();

    for profile in synthetic_navigation_clock_profiles() {
        let case = build_navigation_clock_case(profile.clone());
        let report = &case.pvt_accuracy;

        assert!(report.pass, "{report:?}");
        assert!(report.truth_coverage_ready, "{report:?}");
        assert!(report.truth_coverage_issues.is_empty(), "{report:?}");
        assert_eq!(report.scenario_id, case.truth_table.scenario_id);
        assert_eq!(report.max_position_error_3d_m, budgets.pvt.max_position_error_3d_m);
        assert_eq!(report.max_clock_bias_error_m, budgets.pvt.max_clock_bias_error_m);
        assert_eq!(report.max_residual_rms_m, budgets.pvt.max_residual_rms_m);
        assert_eq!(report.max_pdop, budgets.pvt.max_pdop);
        assert_eq!(report.epoch_count, case.truth_table.epochs.len());
        assert_eq!(report.passing_epoch_count, report.epoch_count);
        assert!(report.epochs.iter().all(|epoch| epoch.pass), "{report:?}");
    }
}

#[test]
fn oscillator_drift_profile_preserves_position_and_clock_accuracy_margin() {
    let case = build_navigation_clock_case(synthetic_navigation_clock_profile(
        "oscillator_drift_receiver_clock",
    ));
    let report = &case.pvt_accuracy;

    let worst_position_error_m = report
        .epochs
        .iter()
        .map(|epoch| epoch.position_error_3d_m)
        .reduce(f64::max)
        .expect("pvt accuracy epoch");
    let worst_clock_bias_error_m = report
        .epochs
        .iter()
        .map(|epoch| epoch.clock_bias_error_m)
        .reduce(f64::max)
        .expect("pvt accuracy epoch");

    assert!(worst_position_error_m < report.max_position_error_3d_m, "{report:?}");
    assert!(worst_clock_bias_error_m < report.max_clock_bias_error_m, "{report:?}");
}
