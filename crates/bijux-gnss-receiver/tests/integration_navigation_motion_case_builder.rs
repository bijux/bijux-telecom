#![allow(dead_code, missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;
#[path = "support/navigation_truth.rs"]
mod navigation_truth;

use navigation_motion_profile::{
    build_navigation_motion_case, motion_profile_mean_speed_mps, motion_profile_path_length_m,
    synthetic_navigation_motion_profiles,
};

#[test]
fn generated_motion_profiles_include_static_and_moving_reference_paths() {
    let profiles = synthetic_navigation_motion_profiles();

    assert_eq!(profiles.len(), 2);
    assert_eq!(profiles[0].profile_name, "static_reference");
    assert_eq!(motion_profile_path_length_m(&profiles[0]), 0.0);
    assert_eq!(motion_profile_mean_speed_mps(&profiles[0]), 0.0);
    assert_eq!(profiles[1].profile_name, "linear_receiver_motion");
    assert!(motion_profile_path_length_m(&profiles[1]) > 0.0);
    assert!(motion_profile_mean_speed_mps(&profiles[1]) > 0.0);
}

#[test]
fn motion_case_builder_names_cases_by_profile() {
    let profile =
        synthetic_navigation_motion_profiles().into_iter().next().expect("static motion profile");
    let profile_name = profile.profile_name;
    let case = build_navigation_motion_case(profile);

    assert_eq!(case.scenario_id, format!("navigation_motion_profile_{profile_name}"));
}
