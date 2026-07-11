#![allow(missing_docs)]

#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;
#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;

use bijux_gnss_receiver::api::sim::{
    truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
};

use navigation_pvt_truth_table::build_pvt_truth_table_fixture;

#[test]
fn pvt_accuracy_budget_enforces_hard_truth_thresholds() {
    let fixture = build_pvt_truth_table_fixture("clean_synthetic_navigation_pvt_truth", 0.0);
    let budgets = truth_guided_receiver_accuracy_budgets();
    let report = validate_pvt_accuracy_budget(&fixture.report, budgets.pvt);

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, fixture.report.scenario_id);
    assert_eq!(report.max_position_error_3d_m, budgets.pvt.max_position_error_3d_m);
    assert_eq!(report.max_clock_bias_error_m, budgets.pvt.max_clock_bias_error_m);
    assert_eq!(report.max_residual_rms_m, budgets.pvt.max_residual_rms_m);
    assert_eq!(report.max_pdop, budgets.pvt.max_pdop);
    assert_eq!(report.epoch_count, fixture.report.epochs.len());
    assert_eq!(report.passing_epoch_count, report.epoch_count);

    for epoch in &report.epochs {
        assert!(epoch.pass, "{epoch:?}");
        assert!(
            epoch.position_error_3d_m <= report.max_position_error_3d_m + f64::EPSILON,
            "{epoch:?}"
        );
        assert!(
            epoch.clock_bias_error_m <= report.max_clock_bias_error_m + f64::EPSILON,
            "{epoch:?}"
        );
        assert!(epoch.residual_rms_m <= report.max_residual_rms_m + f64::EPSILON, "{epoch:?}");
        assert!(epoch.pdop <= report.max_pdop + f64::EPSILON, "{epoch:?}");
    }
}
