#![allow(missing_docs)]

#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::{
    truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
    validate_truth_guided_pvt_table,
};

use navigation_pvt_truth_table::build_pvt_truth_table_fixture;

#[test]
fn pvt_accuracy_budget_requires_matched_truth_epochs() {
    let fixture = build_pvt_truth_table_fixture("clean_synthetic_navigation_pvt_truth", 0.0);
    assert_eq!(fixture.report.scenario_id, "clean_synthetic_navigation_pvt_truth");
    let truth_table =
        validate_truth_guided_pvt_table("missing_pvt_truth_epochs", &fixture.run.solutions, &[]);
    let budgets = truth_guided_receiver_accuracy_budgets();
    let accuracy = validate_pvt_accuracy_budget(&truth_table, budgets.pvt);

    assert_eq!(truth_table.matched_epoch_count, 0);
    assert_eq!(truth_table.unmatched_solution_epochs.len(), fixture.run.solutions.len());
    assert!(accuracy.epochs.is_empty(), "{accuracy:?}");
    assert!(!accuracy.truth_coverage_ready, "{accuracy:?}");
    assert!(!accuracy.pass, "{accuracy:?}");
    assert!(
        accuracy
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "no_matched_truth_epochs"),
        "{accuracy:?}"
    );
    assert!(
        accuracy
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "unmatched_solution_epoch"),
        "{accuracy:?}"
    );
}
