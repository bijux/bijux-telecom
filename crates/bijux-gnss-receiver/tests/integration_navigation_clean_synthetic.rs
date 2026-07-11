#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::SolutionValidity;
use bijux_gnss_receiver::api::{build_validation_report_with_budgets, ValidationSciencePolicy};

use support::navigation_pipeline::{
    clean_synthetic_navigation_run, clean_synthetic_pvt_budgets, position_error_3d_m,
    CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M,
};

#[test]
fn clean_synthetic_navigation_emits_usable_code_only_pvt() {
    let run = clean_synthetic_navigation_run();

    assert!(
        run.tracking.iter().filter(|track| !track.epochs.is_empty()).count() >= 4,
        "expected at least four tracked channels",
    );
    assert!(!run.observations.is_empty(), "expected valid multisatellite observation epochs",);
    assert!(
        !run.solutions.is_empty(),
        "expected navigation solutions from clean synthetic observations",
    );
    assert!(
        run.solutions.iter().all(|solution| solution.valid),
        "expected all clean synthetic navigation solutions to stay valid: {:?}",
        run.solutions
            .iter()
            .map(|solution| (
                solution.epoch.index,
                solution.status,
                solution.explain_reasons.clone()
            ))
            .collect::<Vec<_>>()
    );
    assert!(
        run.solutions.iter().any(|solution| matches!(solution.validity, SolutionValidity::Stable)),
        "expected at least one stable clean synthetic navigation solution",
    );
}

#[test]
fn clean_synthetic_navigation_meets_reference_position_budget() {
    let run = clean_synthetic_navigation_run();
    let report = build_validation_report_with_budgets(
        &run.tracking,
        &run.observations,
        &run.solutions,
        &run.reference_epochs,
        run.config.sampling_freq_hz,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
        clean_synthetic_pvt_budgets(),
    )
    .expect("validation report");

    let matched_errors = run
        .solutions
        .iter()
        .map(|solution| {
            (solution.epoch.index, position_error_3d_m(solution, run.profile.truth_ecef_m))
        })
        .collect::<Vec<_>>();

    assert_eq!(
        report.reference_position_errors.len(),
        run.solutions.len(),
        "expected one truth match per clean synthetic navigation solution",
    );
    assert!(
        report.budget_violations.is_empty(),
        "clean synthetic navigation exceeded the reference position budget: {:?}",
        report.budget_violations
    );
    assert!(
        report.error_3d_m.max <= CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M,
        "clean synthetic navigation exceeded {:.3} m: {:?}",
        CLEAN_SYNTHETIC_PVT_POSITION_ERROR_MAX_M,
        matched_errors
    );
}
