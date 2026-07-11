#![allow(dead_code, missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;

use navigation_motion_profile::{
    build_navigation_motion_case, synthetic_navigation_motion_profiles,
};

#[test]
fn navigation_motion_truth_table_preserves_static_and_moving_receiver_paths() {
    let cases = synthetic_navigation_motion_profiles()
        .into_iter()
        .map(build_navigation_motion_case)
        .collect::<Vec<_>>();
    let static_case = &cases[0];
    let moving_case = &cases[1];

    assert_eq!(static_case.truth_table.epochs.len(), static_case.motion_profile.truth_epochs.len());
    assert_eq!(moving_case.truth_table.epochs.len(), moving_case.motion_profile.truth_epochs.len());

    let static_first = &static_case.truth_table.epochs[0].truth_ecef_m;
    let static_last =
        static_case.truth_table.epochs.last().expect("static truth epoch").truth_ecef_m;
    assert!(
        (static_first.x_m - static_last.x_m).abs() <= 1.0e-9
            && (static_first.y_m - static_last.y_m).abs() <= 1.0e-9
            && (static_first.z_m - static_last.z_m).abs() <= 1.0e-9,
        "{:?}",
        static_case.truth_table.epochs
    );

    let moving_first = &moving_case.truth_table.epochs[0];
    let moving_last = moving_case.truth_table.epochs.last().expect("moving truth epoch");
    assert!(
        moving_last.receive_time_s > moving_first.receive_time_s,
        "{:?}",
        moving_case.truth_table.epochs
    );

    let dx = moving_last.truth_ecef_m.x_m - moving_first.truth_ecef_m.x_m;
    let dy = moving_last.truth_ecef_m.y_m - moving_first.truth_ecef_m.y_m;
    let dz = moving_last.truth_ecef_m.z_m - moving_first.truth_ecef_m.z_m;
    let displacement_m = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!(displacement_m > 1.0, "{:?}", moving_case.truth_table.epochs);

    for window in moving_case.truth_table.epochs.windows(2) {
        assert!(
            window[1].receive_time_s > window[0].receive_time_s,
            "{:?}",
            moving_case.truth_table.epochs
        );
    }
}
