#![allow(missing_docs)]

#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;
mod support;

use navigation_pvt_truth_table::build_pvt_truth_table_fixture;

const SYNTHETIC_CLOCK_BIAS_S: f64 = 2.75e-4;
const MAX_CLOCK_BIAS_ERROR_S: f64 = 1.0e-9;

#[test]
fn pvt_truth_table_records_clock_biased_navigation_profile() {
    let fixture = build_pvt_truth_table_fixture(
        "clock_biased_synthetic_navigation_pvt_truth",
        SYNTHETIC_CLOCK_BIAS_S,
    );
    let report = &fixture.report;

    assert_eq!(report.solution_count, 1);
    assert_eq!(report.matched_epoch_count, 1);
    assert!(report.unmatched_solution_epochs.is_empty(), "{report:?}");
    assert!(report.unused_reference_epochs.is_empty(), "{report:?}");

    let epoch = report.epochs.first().expect("clock-biased pvt truth row");
    assert_eq!(epoch.clock_bias.truth_s, SYNTHETIC_CLOCK_BIAS_S);
    assert_eq!(epoch.clock_bias.truth_m, SYNTHETIC_CLOCK_BIAS_S * 299_792_458.0);
    assert!(epoch.clock_bias.measured_s > 0.0, "{epoch:?}");
    assert!(epoch.clock_bias.measured_m > 0.0, "{epoch:?}");
    assert!(epoch.clock_bias.error_s.abs() <= MAX_CLOCK_BIAS_ERROR_S, "{epoch:?}");
    assert!(epoch.clock_bias.error_m.abs() <= MAX_CLOCK_BIAS_ERROR_S * 299_792_458.0, "{epoch:?}");
    assert!(epoch.valid, "{epoch:?}");
    assert_eq!(epoch.solution_status, fixture.run.solutions[0].status);
}
