#![allow(dead_code, missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;

use navigation_motion_profile::{
    build_navigation_motion_case, motion_profile_mean_speed_mps, motion_profile_path_length_m,
    receiver_motion_profile, synthetic_navigation_motion_profiles, NavigationMotionProfile,
    NavigationMotionTruthEpoch,
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

#[test]
fn receiver_motion_profile_rejects_zero_truth_epochs() {
    let result = std::panic::catch_unwind(|| {
        receiver_motion_profile(
            "zero_truth_epochs",
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0),
            100_000.0,
            0.001,
            0,
        )
    });

    assert!(result.is_err(), "zero truth epochs must panic");
}

#[test]
fn receiver_motion_profile_rejects_nonpositive_epoch_spacing() {
    let result = std::panic::catch_unwind(|| {
        receiver_motion_profile(
            "nonpositive_epoch_spacing",
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0),
            100_000.0,
            0.0,
            2,
        )
    });

    assert!(result.is_err(), "nonpositive epoch spacing must panic");
}

#[test]
fn motion_case_builder_rejects_nonmonotonic_truth_times() {
    let result = std::panic::catch_unwind(|| {
        build_navigation_motion_case(NavigationMotionProfile {
            profile_name: "nonmonotonic_truth_times",
            truth_epochs: vec![
                NavigationMotionTruthEpoch {
                    epoch_idx: 0,
                    receive_time_s: 100_000.001,
                    truth_ecef_m: (1.0, 2.0, 3.0),
                    truth_velocity_ecef_mps: (0.0, 0.0, 0.0),
                },
                NavigationMotionTruthEpoch {
                    epoch_idx: 1,
                    receive_time_s: 100_000.001,
                    truth_ecef_m: (1.0, 2.0, 3.0),
                    truth_velocity_ecef_mps: (0.0, 0.0, 0.0),
                },
            ],
        })
    });

    assert!(result.is_err(), "nonmonotonic truth times must panic");
}
