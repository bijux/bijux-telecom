#![allow(missing_docs)]

#[path = "support/navigation_cn0_profile.rs"]
mod navigation_cn0_profile;
#[path = "support/navigation_truth.rs"]
mod navigation_truth;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_cn0_profile, SyntheticPvtCn0ProfileCase,
};

use navigation_cn0_profile::build_truth_seeded_navigation_cn0_case;

#[test]
fn truth_seeded_navigation_cn0_cases_preserve_signal_strength_ordering() {
    let weak_case = build_truth_seeded_navigation_cn0_case(30.0, "navigation_cn0_support");
    let strong_case = build_truth_seeded_navigation_cn0_case(52.0, "navigation_cn0_support");
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
        ],
        "navigation_cn0_support",
    );

    assert_eq!(report.points.len(), 2);
    assert!(
        report.points[1].mean_observation_cn0_dbhz
            > report.points[0].mean_observation_cn0_dbhz + 8.0,
        "{report:?}"
    );
    assert!(report.points[1].epoch_count >= report.points[0].epoch_count, "{report:?}");
}
