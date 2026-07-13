#![allow(missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;

use navigation_motion_profile::{
    build_navigation_motion_case, synthetic_navigation_motion_profiles,
};

#[test]
fn navigation_pipeline_does_not_misclassify_receiver_motion_as_clock_anomaly() {
    let cases = synthetic_navigation_motion_profiles()
        .into_iter()
        .map(build_navigation_motion_case)
        .collect::<Vec<_>>();

    assert!(
        cases.iter().all(|case| case.solutions.iter().all(|solution| solution
            .explain_reasons
            .iter()
            .all(|reason| reason != "satellite_clock_anomaly"))),
        "receiver motion alone must not trigger satellite clock anomaly classification: {:?}",
        cases
            .iter()
            .map(|case| (
                case.motion_profile.profile_name,
                case.solutions
                    .iter()
                    .map(|solution| (solution.epoch.index, solution.explain_reasons.clone()))
                    .collect::<Vec<_>>()
            ))
            .collect::<Vec<_>>(),
    );
}
