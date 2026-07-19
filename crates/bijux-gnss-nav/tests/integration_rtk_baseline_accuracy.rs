#![allow(missing_docs)]

use bijux_gnss_nav::api::rtk_float_baseline_from_double_differences;
use bijux_gnss_testkit::reference_data::rtk_baseline::{
    centimeter_level_rtk_baseline_budget, clean_gps_l1_short_baseline_case, rtk_baseline_accuracy,
};

#[test]
fn clean_short_baseline_float_solution_reaches_centimeter_accuracy() {
    let scenario = clean_gps_l1_short_baseline_case();

    let solution = rtk_float_baseline_from_double_differences(
        &scenario.double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float baseline");
    let accuracy = rtk_baseline_accuracy(solution.enu_m, scenario.truth_enu_m);

    assert!(
        accuracy.satisfies(centimeter_level_rtk_baseline_budget()),
        "accuracy={accuracy:?} solution={solution:?}"
    );
}
