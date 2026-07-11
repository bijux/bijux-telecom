#![allow(missing_docs)]

#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::{
    truth_guided_receiver_accuracy_budgets, validate_pvt_accuracy_budget,
    validate_truth_guided_pvt_table,
};

use navigation_pipeline::pvt_truth_reference_epochs;
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

#[test]
fn pvt_accuracy_budget_requires_full_reference_consumption() {
    let fixture = build_pvt_truth_table_fixture("clean_synthetic_navigation_pvt_truth", 0.0);
    let mut reference_epochs = pvt_truth_reference_epochs(&fixture.run);
    let mut extra_reference = reference_epochs
        .last()
        .cloned()
        .expect("clean synthetic navigation reference epoch");
    extra_reference.position.epoch_idx += 1;
    reference_epochs.push(extra_reference);

    let truth_table = validate_truth_guided_pvt_table(
        "extra_pvt_truth_reference",
        &fixture.run.solutions,
        &reference_epochs,
    );
    let budgets = truth_guided_receiver_accuracy_budgets();
    let accuracy = validate_pvt_accuracy_budget(&truth_table, budgets.pvt);

    assert_eq!(truth_table.matched_epoch_count, fixture.run.solutions.len());
    assert_eq!(truth_table.unused_reference_epochs.len(), 1);
    assert_eq!(accuracy.passing_epoch_count, accuracy.epoch_count, "{accuracy:?}");
    assert!(!accuracy.truth_coverage_ready, "{accuracy:?}");
    assert!(!accuracy.pass, "{accuracy:?}");
    assert!(
        accuracy
            .truth_coverage_issues
            .iter()
            .any(|issue| issue.code == "unused_truth_reference_epoch"),
        "{accuracy:?}"
    );
}
