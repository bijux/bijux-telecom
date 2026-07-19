#![allow(missing_docs)]

#[path = "support/navigation_geometry_profile.rs"]
mod navigation_geometry_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_geometry_profile, SyntheticPvtGeometryProfileCase,
};

use navigation_geometry_profile::build_truth_seeded_navigation_geometry_case;

const NOISE_FREE_POSITION_ERROR_CEILING_M: f64 = 1.0e-6;

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
    for point in &report.points {
        assert!(
            point
                .max_position_error_3d_m
                .is_some_and(|error_m| error_m <= NOISE_FREE_POSITION_ERROR_CEILING_M),
            "noise-free geometry error exceeds the numerical ceiling: {report:?}"
        );
    }
}

#[test]
fn pvt_accuracy_geometry_profile_improves_with_more_visible_satellites() {
    let four_satellite_case = build_truth_seeded_navigation_geometry_case(
        &[3, 7, 11, 19],
        "navigation_geometry_visibility",
    );
    let five_satellite_case = build_truth_seeded_navigation_geometry_case(
        &[3, 7, 11, 19, 23],
        "navigation_geometry_visibility",
    );
    let six_satellite_case = build_truth_seeded_navigation_geometry_case(
        &[3, 7, 11, 19, 23, 29],
        "navigation_geometry_visibility",
    );
    let report = summarize_truth_guided_pvt_geometry_profile(
        &[
            SyntheticPvtGeometryProfileCase {
                scenario_id: &four_satellite_case.scenario_id,
                accuracy: &four_satellite_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &six_satellite_case.scenario_id,
                accuracy: &six_satellite_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &five_satellite_case.scenario_id,
                accuracy: &five_satellite_case.pvt_accuracy,
            },
        ],
        "navigation_geometry_visibility",
    );

    assert_eq!(report.points.len(), 3);
    let most_visible = &report.points[0];
    let intermediate = &report.points[1];
    let least_visible = &report.points[2];
    for point in &report.points {
        assert!(point.ready, "{report:?}");
        assert!(point.mean_pdop.is_some_and(|pdop| pdop.is_finite() && pdop > 0.0), "{report:?}");
    }
    assert!(most_visible.mean_pdop < intermediate.mean_pdop, "{report:?}");
    assert!(intermediate.mean_pdop < least_visible.mean_pdop, "{report:?}");
    assert!(most_visible.pass_rate >= least_visible.pass_rate, "{report:?}");
    for point in &report.points {
        assert!(
            point
                .max_position_error_3d_m
                .is_some_and(|error_m| error_m <= NOISE_FREE_POSITION_ERROR_CEILING_M),
            "noise-free visibility error exceeds the numerical ceiling: {report:?}"
        );
    }
}

#[test]
fn pvt_accuracy_geometry_profile_reports_consistent_pdop_statistics() {
    let poor_case =
        build_truth_seeded_navigation_geometry_case(&[7, 11, 19, 23], "navigation_geometry_stats");
    let medium_case =
        build_truth_seeded_navigation_geometry_case(&[3, 7, 11, 19], "navigation_geometry_stats");
    let good_case = build_truth_seeded_navigation_geometry_case(
        &[3, 7, 11, 19, 23],
        "navigation_geometry_stats",
    );
    let report = summarize_truth_guided_pvt_geometry_profile(
        &[
            SyntheticPvtGeometryProfileCase {
                scenario_id: &poor_case.scenario_id,
                accuracy: &poor_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &good_case.scenario_id,
                accuracy: &good_case.pvt_accuracy,
            },
            SyntheticPvtGeometryProfileCase {
                scenario_id: &medium_case.scenario_id,
                accuracy: &medium_case.pvt_accuracy,
            },
        ],
        "navigation_geometry_stats",
    );

    assert_eq!(report.points.len(), 3);
    for point in &report.points {
        match (point.min_pdop, point.mean_pdop, point.max_pdop) {
            (Some(min_pdop), Some(mean_pdop), Some(max_pdop)) => {
                assert!(min_pdop > 0.0, "{report:?}");
                assert!(min_pdop <= mean_pdop, "{report:?}");
                assert!(mean_pdop <= max_pdop, "{report:?}");
            }
            _ => panic!("geometry profile must report finite PDOP statistics: {report:?}"),
        }
    }

    let good = &report.points[0];
    let medium = &report.points[1];
    let poor = &report.points[2];
    assert!(good.max_pdop < medium.max_pdop, "{report:?}");
    assert!(medium.max_pdop < poor.max_pdop, "{report:?}");
}
