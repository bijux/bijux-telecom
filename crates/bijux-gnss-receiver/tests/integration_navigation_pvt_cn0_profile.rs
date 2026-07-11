#![allow(missing_docs)]

#[path = "support/navigation_truth.rs"]
mod navigation_truth;
#[path = "support/navigation_cn0_profile.rs"]
mod navigation_cn0_profile;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_cn0_profile, SyntheticPvtCn0ProfileCase,
};

use navigation_cn0_profile::build_truth_seeded_navigation_cn0_case;

#[test]
fn pvt_accuracy_profile_tracks_signal_strength_from_weak_to_strong() {
    let weak_case = build_truth_seeded_navigation_cn0_case(30.0, "navigation_pvt_cn0_profile");
    let medium_case = build_truth_seeded_navigation_cn0_case(40.0, "navigation_pvt_cn0_profile");
    let strong_case = build_truth_seeded_navigation_cn0_case(52.0, "navigation_pvt_cn0_profile");
    let report = summarize_truth_guided_pvt_cn0_profile(
        &[
            SyntheticPvtCn0ProfileCase {
                scenario_id: &strong_case.scenario_id,
                observations: &strong_case.observations,
                accuracy: &strong_case.pvt_accuracy,
            },
            SyntheticPvtCn0ProfileCase {
                scenario_id: &weak_case.scenario_id,
                observations: &weak_case.observations,
                accuracy: &weak_case.pvt_accuracy,
            },
            SyntheticPvtCn0ProfileCase {
                scenario_id: &medium_case.scenario_id,
                observations: &medium_case.observations,
                accuracy: &medium_case.pvt_accuracy,
            },
        ],
        "navigation_pvt_cn0_profile",
    );

    assert_eq!(report.points.len(), 3);
    assert!(
        report.points[0].mean_observation_cn0_dbhz < report.points[1].mean_observation_cn0_dbhz
            && report.points[1].mean_observation_cn0_dbhz
                < report.points[2].mean_observation_cn0_dbhz,
        "{report:?}"
    );

    let weak = &report.points[0];
    let strong = &report.points[2];
    assert!(strong.epoch_count > 0, "{report:?}");
    assert!(strong.pass_rate >= weak.pass_rate, "{report:?}");
    match (weak.max_position_error_3d_m, strong.max_position_error_3d_m) {
        (Some(weak_max), Some(strong_max)) => assert!(strong_max <= weak_max + 1.0e-9, "{report:?}"),
        (None, Some(_)) => {}
        (_, None) => panic!("strong signal must produce a PVT error measurement: {report:?}"),
    }
}
