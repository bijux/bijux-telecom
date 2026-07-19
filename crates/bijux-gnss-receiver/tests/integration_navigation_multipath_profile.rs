#![allow(missing_docs)]

#[path = "support/navigation_multipath_profile.rs"]
mod navigation_multipath_profile;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::{
    summarize_truth_guided_pvt_multipath_profile, SyntheticPvtMultipathProfileCase,
};

use navigation_multipath_profile::{
    build_navigation_multipath_case, max_abs_pseudorange_bias_m, mean_abs_pseudorange_bias_m,
    synthetic_navigation_multipath_profiles,
};

#[test]
fn pvt_multipath_profile_tracks_residual_growth_from_clean_to_severe_reflections() {
    let cases = synthetic_navigation_multipath_profiles()
        .into_iter()
        .map(build_navigation_multipath_case)
        .collect::<Vec<_>>();
    let report = summarize_truth_guided_pvt_multipath_profile(
        &cases
            .iter()
            .map(|case| SyntheticPvtMultipathProfileCase {
                scenario_id: &case.scenario_id,
                affected_satellite_count: case.noise_profile.satellites.len(),
                mean_abs_pseudorange_bias_m: mean_abs_pseudorange_bias_m(&case.noise_profile),
                max_abs_pseudorange_bias_m: max_abs_pseudorange_bias_m(&case.noise_profile),
                truth_table: &case.truth_table,
                accuracy: &case.pvt_accuracy,
            })
            .collect::<Vec<_>>(),
        "navigation_multipath_profile",
    );

    assert_eq!(report.points.len(), cases.len());
    for window in report.points.windows(2) {
        let current = &window[0];
        let next = &window[1];
        assert!(next.max_abs_pseudorange_bias_m > current.max_abs_pseudorange_bias_m, "{report:?}");
    }

    let clean = &report.points[0];
    let moderate = &report.points[2];
    let severe = report.points.last().expect("severe multipath point");
    assert!(clean.ready && moderate.ready && severe.ready, "{report:?}");
    assert!(clean.stable_epoch_rate >= severe.stable_epoch_rate, "{report:?}");
    match (clean.max_residual_rms_m, moderate.max_residual_rms_m) {
        (Some(clean_residual_rms_m), Some(moderate_residual_rms_m)) => {
            assert!(moderate_residual_rms_m > clean_residual_rms_m, "{report:?}");
        }
        _ => panic!("multipath profile must produce residual RMS measurements: {report:?}"),
    }
    match (moderate.max_residual_rms_m, severe.max_residual_rms_m) {
        (Some(moderate_residual_rms_m), Some(severe_residual_rms_m)) => {
            assert!(severe_residual_rms_m > moderate_residual_rms_m, "{report:?}");
        }
        _ => panic!("multipath profile must produce residual RMS measurements: {report:?}"),
    }
    match (clean.max_position_error_3d_m, severe.max_position_error_3d_m) {
        (Some(clean_position_error_3d_m), Some(severe_position_error_3d_m)) => {
            assert!(severe_position_error_3d_m > clean_position_error_3d_m, "{report:?}");
        }
        _ => panic!("multipath profile must produce position error measurements: {report:?}"),
    }
}
