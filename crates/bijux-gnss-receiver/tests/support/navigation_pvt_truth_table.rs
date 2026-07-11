#![allow(missing_docs)]

use bijux_gnss_receiver::api::sim::{
    validate_truth_guided_pvt_table, SyntheticPvtTruthTableReport,
};

use crate::support::navigation_pipeline::{
    clean_synthetic_navigation_run, clean_synthetic_navigation_run_with_clock_bias,
    pvt_truth_reference_epochs, CleanSyntheticNavigationRun,
};

pub struct PvtTruthTableFixture {
    #[allow(dead_code)]
    pub run: CleanSyntheticNavigationRun,
    pub report: SyntheticPvtTruthTableReport,
}

pub fn build_pvt_truth_table_fixture(
    scenario_id: &str,
    truth_clock_bias_s: f64,
) -> PvtTruthTableFixture {
    let run = build_navigation_run(truth_clock_bias_s);
    let report = validate_truth_guided_pvt_table(
        scenario_id,
        &run.solutions,
        &pvt_truth_reference_epochs(&run),
    );
    PvtTruthTableFixture { run, report }
}

fn build_navigation_run(truth_clock_bias_s: f64) -> CleanSyntheticNavigationRun {
    if truth_clock_bias_s == 0.0 {
        clean_synthetic_navigation_run()
    } else {
        clean_synthetic_navigation_run_with_clock_bias(truth_clock_bias_s)
    }
}
