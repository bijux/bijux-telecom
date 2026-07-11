#![allow(missing_docs)]

#[path = "support/navigation_time_profile.rs"]
mod navigation_time_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_time_profile, SyntheticPvtTimeTrend,
};

use navigation_time_profile::{
    build_navigation_time_case, build_navigation_time_cases, navigation_time_profile,
    truth_guided_time_profile_cases,
};

fn navigation_time_profile_report() -> bijux_gnss_receiver::api::sim::SyntheticPvtTimeProfileReport
{
    let cases = build_navigation_time_cases();

    summarize_truth_guided_pvt_time_profile(
        &truth_guided_time_profile_cases(&cases),
        "navigation_time_profile",
    )
}

#[test]
fn navigation_time_profile_identifies_stabilizing_accuracy_runs() {
    let report = navigation_time_profile_report();
    let point = report
        .points
        .iter()
        .find(|point| {
            point.scenario_id == "navigation_time_profile_stabilizing_navigation_accuracy"
        })
        .expect("stabilizing time profile point");

    assert!(point.ready, "{report:?}");
    assert_eq!(point.trend, SyntheticPvtTimeTrend::Stabilizing, "{report:?}");
    assert!(point.analysis_window_epoch_count > 0, "{report:?}");
    assert!(
        point.last_window_mean_position_error_3d_m < point.first_window_mean_position_error_3d_m,
        "{report:?}"
    );
    assert!(point.position_error_growth_m.expect("position growth") < 0.0, "{report:?}");
    assert!(
        point.position_error_drift_m_per_s.expect("stabilizing drift slope") < 0.0,
        "{report:?}"
    );
    assert!(point.residual_rms_growth_m.expect("residual growth") < 0.0, "{report:?}");
}

#[test]
fn navigation_time_profile_identifies_drifting_accuracy_runs() {
    let report = navigation_time_profile_report();
    let point = report
        .points
        .iter()
        .find(|point| point.scenario_id == "navigation_time_profile_drifting_navigation_accuracy")
        .expect("drifting time profile point");

    assert!(point.ready, "{report:?}");
    assert_eq!(point.trend, SyntheticPvtTimeTrend::Drifting, "{report:?}");
    assert!(point.analysis_window_epoch_count > 0, "{report:?}");
    assert!(
        point.last_window_mean_position_error_3d_m > point.first_window_mean_position_error_3d_m,
        "{report:?}"
    );
    assert!(point.position_error_growth_m.expect("position growth") > 0.0, "{report:?}");
    assert!(point.position_error_drift_m_per_s.expect("drifting slope") > 0.0, "{report:?}");
    assert!(point.residual_rms_growth_m.expect("residual growth") > 0.0, "{report:?}");
}

#[test]
fn navigation_time_profile_identifies_diverging_accuracy_runs() {
    let report = navigation_time_profile_report();
    let point = report
        .points
        .iter()
        .find(|point| point.scenario_id == "navigation_time_profile_diverging_navigation_accuracy")
        .expect("diverging time profile point");

    assert!(point.ready, "{report:?}");
    assert_eq!(point.trend, SyntheticPvtTimeTrend::Diverging, "{report:?}");
    assert!(point.pass_rate < 0.5, "{report:?}");
    assert!(point.position_error_growth_m.expect("position growth") > 0.0, "{report:?}");
    assert!(point.position_error_drift_m_per_s.expect("diverging slope") > 1.0, "{report:?}");
    assert!(
        point.last_window_mean_position_error_3d_m > point.first_window_mean_position_error_3d_m,
        "{report:?}"
    );
}

#[test]
fn navigation_time_profile_requires_minimum_long_run_duration() {
    let short_duration_case = build_navigation_time_case(navigation_time_profile(
        "short_duration_navigation_accuracy",
        0.0,
        4.0,
        100_000.0,
        0.1,
        5,
    ));
    let report = summarize_truth_guided_pvt_time_profile(
        &truth_guided_time_profile_cases(&[short_duration_case]),
        "navigation_time_profile",
    );
    let point = report.points.first().expect("short duration time profile point");

    assert!(!point.truth_coverage_ready, "{report:?}");
    assert!(
        point
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "insufficient_time_profile_duration"),
        "{report:?}"
    );
    assert!(!point.ready, "{report:?}");
}

#[test]
fn navigation_time_profile_requires_minimum_epoch_count() {
    let short_epoch_case = build_navigation_time_case(navigation_time_profile(
        "short_epoch_navigation_accuracy",
        0.0,
        4.0,
        100_000.0,
        0.5,
        4,
    ));
    let report = summarize_truth_guided_pvt_time_profile(
        &truth_guided_time_profile_cases(&[short_epoch_case]),
        "navigation_time_profile",
    );
    let point = report.points.first().expect("short epoch time profile point");

    assert!(!point.truth_coverage_ready, "{report:?}");
    assert!(
        point
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "insufficient_time_profile_truth_epochs"),
        "{report:?}"
    );
    assert!(
        point
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "insufficient_time_profile_accuracy_epochs"),
        "{report:?}"
    );
    assert!(!point.ready, "{report:?}");
}
