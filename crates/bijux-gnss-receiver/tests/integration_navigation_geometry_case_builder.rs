#![allow(missing_docs)]

#[path = "support/navigation_geometry_profile.rs"]
mod navigation_geometry_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use navigation_geometry_profile::build_truth_seeded_navigation_geometry_case;

#[test]
fn geometry_case_builder_rejects_duplicate_visible_satellites() {
    let result = std::panic::catch_unwind(|| {
        build_truth_seeded_navigation_geometry_case(
            &[3, 7, 11, 11],
            "navigation_geometry_builder_duplicates",
        )
    });

    assert!(result.is_err(), "duplicate geometry PRNs must panic");
}

#[test]
fn geometry_case_builder_rejects_insufficient_visible_satellites() {
    let result = std::panic::catch_unwind(|| {
        build_truth_seeded_navigation_geometry_case(
            &[3, 7, 11],
            "navigation_geometry_builder_insufficient",
        )
    });

    assert!(result.is_err(), "fewer than four geometry PRNs must panic");
}
