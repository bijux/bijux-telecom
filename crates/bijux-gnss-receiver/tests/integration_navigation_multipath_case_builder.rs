#![allow(missing_docs)]

#[path = "support/navigation_multipath_profile.rs"]
mod navigation_multipath_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use navigation_multipath_profile::multipath_bias_profile;
use navigation_multipath_profile::{
    build_navigation_multipath_case, synthetic_navigation_multipath_profiles,
};

#[test]
fn generated_multipath_profiles_start_with_a_clean_reference() {
    let profiles = synthetic_navigation_multipath_profiles();
    let clean = profiles.first().expect("clean multipath profile");

    assert_eq!(clean.profile_name, "clean_reference");
    assert!(clean.satellites.is_empty(), "clean reference should not inject pseudorange bias");
}

#[test]
fn multipath_case_builder_names_cases_by_profile() {
    let clean_profile = synthetic_navigation_multipath_profiles()
        .into_iter()
        .next()
        .expect("clean multipath profile");
    let profile_name = clean_profile.profile_name;
    let case = build_navigation_multipath_case(clean_profile);

    assert_eq!(case.scenario_id, format!("navigation_multipath_profile_{profile_name}"));
}

#[test]
fn multipath_case_builder_rejects_duplicate_visible_satellites() {
    let result = std::panic::catch_unwind(|| {
        multipath_bias_profile("duplicate_visible_satellites", &[(7, 2.0), (11, 4.0), (11, 8.0)])
    });

    assert!(result.is_err(), "duplicate multipath PRNs must panic");
}

#[test]
fn multipath_case_builder_rejects_negative_pseudorange_bias() {
    let result = std::panic::catch_unwind(|| multipath_bias_profile("negative_bias", &[(7, -0.5)]));

    assert!(result.is_err(), "negative multipath bias must panic");
}
