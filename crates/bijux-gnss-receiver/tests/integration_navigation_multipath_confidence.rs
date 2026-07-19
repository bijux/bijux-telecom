#![allow(missing_docs)]

#[path = "support/navigation_multipath_profile.rs"]
mod navigation_multipath_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_core::api::SolutionValidity;
use bijux_gnss_receiver::api::NavIntegrityClass;

use navigation_multipath_profile::{
    build_navigation_multipath_case, synthetic_navigation_multipath_profiles,
};

#[test]
fn synthetic_multipath_degrades_solution_confidence_and_integrity() {
    let cases = synthetic_navigation_multipath_profiles()
        .into_iter()
        .map(build_navigation_multipath_case)
        .collect::<Vec<_>>();
    let clean = cases.first().expect("clean multipath case");
    let severe = cases.last().expect("severe multipath case");

    assert!(
        clean
            .validation_report
            .integrity
            .iter()
            .all(|entry| entry.class == NavIntegrityClass::Nominal),
        "{:?}",
        clean.validation_report.integrity
    );
    assert!(
        severe
            .validation_report
            .integrity
            .iter()
            .any(|entry| entry.class != NavIntegrityClass::Nominal && !entry.reasons.is_empty()),
        "{:?}",
        severe.validation_report.integrity
    );
    assert!(
        clean
            .truth_table
            .epochs
            .iter()
            .all(|epoch| epoch.solution_validity == SolutionValidity::Stable),
        "{:?}",
        clean.truth_table.epochs
    );
    assert!(
        severe
            .truth_table
            .epochs
            .iter()
            .any(|epoch| epoch.solution_validity != SolutionValidity::Stable),
        "{:?}",
        severe.truth_table.epochs
    );
}
