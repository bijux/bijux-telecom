#![allow(missing_docs)]

#[path = "support/navigation_geometry_profile.rs"]
mod navigation_geometry_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_geometry_profile, SyntheticPvtGeometryProfileCase,
};

use navigation_geometry_profile::build_truth_seeded_navigation_geometry_case;

#[test]
fn pvt_accuracy_profile_tracks_good_medium_and_poor_geometry() {
    let poor_case = build_truth_seeded_navigation_geometry_case(
        &[7, 11, 19, 23],
        "navigation_geometry_profile",
    );
    let medium_case =
        build_truth_seeded_navigation_geometry_case(&[3, 7, 11, 19], "navigation_geometry_profile");
    let good_case = build_truth_seeded_navigation_geometry_case(
        &[3, 7, 11, 19, 23],
        "navigation_geometry_profile",
    );
    assert_eq!(good_case.visible_satellite_prns, vec![3, 7, 11, 19, 23]);
    assert_eq!(medium_case.visible_satellite_prns, vec![3, 7, 11, 19]);
    assert_eq!(poor_case.visible_satellite_prns, vec![7, 11, 19, 23]);
    let report = summarize_truth_guided_pvt_geometry_profile(
        &[
            SyntheticPvtGeometryProfileCase {
                scenario_id: &medium_case.scenario_id,
                accuracy: &medium_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &poor_case.scenario_id,
                accuracy: &poor_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &good_case.scenario_id,
                accuracy: &good_case.pvt_accuracy,
            },
        ],
        "navigation_geometry_profile",
    );

    assert_eq!(report.points.len(), 3);
    assert!(
        report.points[0].mean_pdop < report.points[1].mean_pdop
            && report.points[1].mean_pdop < report.points[2].mean_pdop,
        "{report:?}"
    );

    let good = &report.points[0];
    let medium = &report.points[1];
    let poor = &report.points[2];
    assert!(good.ready && medium.ready && poor.ready, "{report:?}");
    assert!(good.epoch_count > 0 && medium.epoch_count > 0 && poor.epoch_count > 0, "{report:?}");
    assert!(good.pass_rate >= poor.pass_rate, "{report:?}");
    match (good.max_position_error_3d_m, poor.max_position_error_3d_m) {
        (Some(good_max), Some(poor_max)) => assert!(poor_max >= good_max, "{report:?}"),
        _ => panic!("geometry profile must produce position error measurements: {report:?}"),
    }
}
