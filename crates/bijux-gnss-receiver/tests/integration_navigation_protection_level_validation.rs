#![allow(missing_docs)]

mod support;

use bijux_gnss_receiver::api::{build_validation_report_with_budgets, ValidationSciencePolicy};

use support::navigation_pipeline::{
    clean_synthetic_navigation_run, clean_synthetic_pvt_budgets,
};

#[test]
fn clean_synthetic_validation_compares_protection_levels_with_truth() {
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

    assert!(
        !report.reference_position_errors.is_empty(),
        "expected reference-matched navigation epochs",
    );
    assert_eq!(
        report.protection_levels.matched_epoch_count,
        report.reference_position_errors.len(),
    );
    assert_eq!(
        report.protection_levels.horizontal_reported_epoch_count,
        report.reference_position_errors.len(),
    );
    assert_eq!(
        report.protection_levels.vertical_reported_epoch_count,
        report.reference_position_errors.len(),
    );
    assert_eq!(
        report.protection_levels.horizontal_contained_epoch_count,
        report.reference_position_errors.len(),
    );
    assert_eq!(
        report.protection_levels.vertical_contained_epoch_count,
        report.reference_position_errors.len(),
    );
    assert!(report.protection_levels.horizontal_breach_epochs.is_empty());
    assert!(report.protection_levels.vertical_breach_epochs.is_empty());

    for error in &report.reference_position_errors {
        assert!(error.hpl_m.is_some(), "missing HPL at epoch {}", error.epoch_idx);
        assert!(error.vpl_m.is_some(), "missing VPL at epoch {}", error.epoch_idx);
        assert_eq!(error.horizontal_within_hpl, Some(true));
        assert_eq!(error.vertical_within_vpl, Some(true));
        assert!(error.horizontal_margin_m.expect("horizontal margin") >= 0.0);
        assert!(error.vertical_margin_m.expect("vertical margin") >= 0.0);
    }
}
