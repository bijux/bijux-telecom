#![allow(missing_docs)]

#[path = "support/navigation_constellation_geometry_profile.rs"]
mod navigation_constellation_geometry_profile;

use bijux_gnss_core::api::Constellation;
use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_constellation_geometry_profile,
    SyntheticPvtConstellationGeometryProfileReport,
};

use navigation_constellation_geometry_profile::build_receiver_constellation_geometry_case;

#[test]
fn public_constellation_geometry_profile_reports_lower_dop_for_mixed_case() {
    let gps_only_case = build_receiver_constellation_geometry_case(
        "receiver_constellation_geometry_gps_only",
        4,
        0,
        0,
    );
    let mixed_case = build_receiver_constellation_geometry_case(
        "receiver_constellation_geometry_mixed",
        4,
        2,
        2,
    );

    let report: SyntheticPvtConstellationGeometryProfileReport =
        summarize_truth_guided_pvt_constellation_geometry_profile(
            &[gps_only_case.profile_case(), mixed_case.profile_case()],
            "receiver_constellation_geometry",
        );

    assert_eq!(report.points.len(), 2);
    let mixed = &report.points[0];
    let gps_only = &report.points[1];
    assert_eq!(mixed.scenario_id, "receiver_constellation_geometry_mixed");
    assert_eq!(
        mixed.constellations,
        vec![Constellation::Gps, Constellation::Galileo, Constellation::Beidou]
    );
    assert_eq!(mixed.availability_rate, 1.0);
    assert_eq!(gps_only.availability_rate, 1.0);
    assert!(mixed.ready && gps_only.ready, "{report:?}");
    assert!(
        mixed.mean_pdop.expect("mixed pdop") < gps_only.mean_pdop.expect("gps-only pdop"),
        "{report:?}"
    );
    assert!(
        mixed.mean_gdop.expect("mixed gdop") < gps_only.mean_gdop.expect("gps-only gdop"),
        "{report:?}"
    );
}

#[test]
fn public_constellation_geometry_profile_reports_higher_availability_for_mixed_case() {
    let gps_only_case = build_receiver_constellation_geometry_case(
        "receiver_constellation_availability_gps_only",
        3,
        0,
        0,
    );
    let mixed_case = build_receiver_constellation_geometry_case(
        "receiver_constellation_availability_mixed",
        3,
        2,
        2,
    );

    let report: SyntheticPvtConstellationGeometryProfileReport =
        summarize_truth_guided_pvt_constellation_geometry_profile(
            &[gps_only_case.profile_case(), mixed_case.profile_case()],
            "receiver_constellation_availability",
        );

    assert_eq!(report.points.len(), 2);
    let mixed = &report.points[0];
    let gps_only = &report.points[1];
    assert_eq!(mixed.scenario_id, "receiver_constellation_availability_mixed");
    assert_eq!(mixed.visible_satellite_count, 7);
    assert_eq!(mixed.solved_epoch_count, 1);
    assert_eq!(mixed.expected_epoch_count, 1);
    assert_eq!(mixed.availability_rate, 1.0);
    assert!(mixed.ready, "{report:?}");
    assert_eq!(gps_only.scenario_id, "receiver_constellation_availability_gps_only");
    assert_eq!(gps_only.constellations, vec![Constellation::Gps]);
    assert_eq!(gps_only.visible_satellite_count, 3);
    assert_eq!(gps_only.solved_epoch_count, 0);
    assert_eq!(gps_only.expected_epoch_count, 1);
    assert_eq!(gps_only.availability_rate, 0.0);
    assert!(!gps_only.ready, "{report:?}");
    assert!(!gps_only.truth_coverage_ready, "{report:?}");
    assert!(gps_only
        .truth_coverage_issues
        .iter()
        .any(|issue| issue.code == "no_navigation_solutions"));
}
