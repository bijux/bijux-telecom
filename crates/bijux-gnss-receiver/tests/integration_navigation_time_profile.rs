#![allow(missing_docs)]

#[path = "support/navigation_time_profile.rs"]
mod navigation_time_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_time_profile, SyntheticPvtTimeProfileCase, SyntheticPvtTimeTrend,
};

use navigation_time_profile::{build_navigation_time_case, synthetic_navigation_time_profiles};

fn navigation_time_profile_report() -> bijux_gnss_receiver::api::sim::SyntheticPvtTimeProfileReport
{
    let cases = synthetic_navigation_time_profiles()
        .into_iter()
        .map(build_navigation_time_case)
        .collect::<Vec<_>>();

    summarize_truth_guided_pvt_time_profile(
        &cases
            .iter()
            .map(|case| SyntheticPvtTimeProfileCase {
                scenario_id: &case.scenario_id,
                truth_table: &case.truth_table,
                accuracy: &case.pvt_accuracy,
            })
            .collect::<Vec<_>>(),
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
    assert!(
        point.last_window_mean_position_error_3d_m < point.first_window_mean_position_error_3d_m,
        "{report:?}"
    );
    assert!(
        point.position_error_drift_m_per_s.expect("stabilizing drift slope") < 0.0,
        "{report:?}"
    );
}
