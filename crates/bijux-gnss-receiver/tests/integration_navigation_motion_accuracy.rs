#![allow(dead_code, missing_docs)]

#[path = "support/navigation_motion_profile.rs"]
mod navigation_motion_profile;

use bijux_gnss_core::api::SolutionValidity;

use navigation_motion_profile::{
    build_navigation_motion_case, synthetic_navigation_motion_profiles,
};

#[test]
fn static_and_moving_receiver_paths_satisfy_pvt_accuracy_budget() {
    let cases = synthetic_navigation_motion_profiles()
        .into_iter()
        .map(build_navigation_motion_case)
        .collect::<Vec<_>>();

    for case in cases {
        let refused_solutions = case
            .solutions
            .iter()
            .filter(|solution| !solution.valid)
            .map(|solution| {
                format!(
                    "epoch={} decision={} reasons={:?}",
                    solution.epoch.index, solution.explain_decision, solution.explain_reasons
                )
            })
            .collect::<Vec<_>>();
        assert!(
            case.pvt_accuracy.truth_coverage_ready,
            "{:?}",
            case.pvt_accuracy.truth_coverage_issues
        );
        assert_eq!(
            case.pvt_accuracy.epoch_count,
            case.motion_profile.truth_epochs.len(),
            "{:?}",
            case.pvt_accuracy
        );
        assert_eq!(
            case.pvt_accuracy.passing_epoch_count, case.pvt_accuracy.epoch_count,
            "accuracy={:?}\nrefused={:#?}",
            case.pvt_accuracy, refused_solutions
        );
        assert!(
            case.pvt_accuracy.pass,
            "accuracy={:?}\nrefused={:#?}",
            case.pvt_accuracy, refused_solutions
        );
        assert!(
            case.truth_table
                .epochs
                .iter()
                .all(|epoch| epoch.solution_validity == SolutionValidity::Stable),
            "truth_table={:?}\nrefused={:#?}",
            case.truth_table.epochs,
            refused_solutions
        );
    }
}
