#![allow(dead_code, missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_motion_profile, SyntheticPvtMotionProfileCase,
};

use navigation_motion_profile::{
    build_navigation_motion_case, synthetic_navigation_motion_profiles,
};

#[test]
fn pvt_motion_profile_distinguishes_static_and_moving_receiver_paths() {
    let cases = synthetic_navigation_motion_profiles()
        .into_iter()
        .map(build_navigation_motion_case)
        .collect::<Vec<_>>();
    let report = summarize_truth_guided_pvt_motion_profile(
        &cases
            .iter()
            .map(|case| SyntheticPvtMotionProfileCase {
                scenario_id: &case.scenario_id,
                truth_table: &case.truth_table,
                accuracy: &case.pvt_accuracy,
            })
            .collect::<Vec<_>>(),
        "navigation_motion_profile",
    );

    assert_eq!(report.points.len(), cases.len());

    let static_point = &report.points[0];
    let moving_point = report.points.last().expect("moving motion point");
    assert!(static_point.ready && moving_point.ready, "{report:?}");
    assert!(!static_point.moving, "{report:?}");
    assert!(moving_point.moving, "{report:?}");
    assert!(static_point.path_length_m.abs() <= 1.0e-9, "{report:?}");
    assert!(moving_point.path_length_m > static_point.path_length_m, "{report:?}");
    assert!(moving_point.mean_speed_mps > static_point.mean_speed_mps, "{report:?}");
    assert!(moving_point.truth_epoch_count >= 2, "{report:?}");
    assert!(static_point.pass_rate > 0.0, "{report:?}");
    assert!(moving_point.pass_rate > 0.0, "{report:?}");
    assert!(moving_point.stable_epoch_rate > 0.0, "{report:?}");
}
