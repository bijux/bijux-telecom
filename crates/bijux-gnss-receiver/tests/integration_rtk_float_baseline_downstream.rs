#![allow(missing_docs)]

use bijux_gnss_receiver::api::{
    build_dd, build_sd, choose_ref_sat, rtk_switch_double_difference_reference,
    rtk_transform_float_baseline_reference, solve_baseline_dd, solve_float_baseline_dd,
};
use bijux_gnss_testkit::rtk_baseline::{
    centimeter_level_rtk_baseline_budget, clean_gps_l1_short_baseline_case, rtk_baseline_accuracy,
};

#[test]
fn receiver_float_baseline_solver_projects_nav_solution() {
    let scenario = clean_gps_l1_short_baseline_case();

    let single_differences = build_sd(&scenario.base_epoch, &scenario.rover_epoch);
    let reference = choose_ref_sat(&single_differences).expect("reference");
    let double_differences = build_dd(&single_differences, reference);
    let float_solution = solve_float_baseline_dd(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float solution");
    let projected_solution = solve_baseline_dd(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("projected solution");

    assert!((float_solution.enu_m[0] - scenario.truth_enu_m[0]).abs() < 0.05);
    assert!((float_solution.enu_m[1] - scenario.truth_enu_m[1]).abs() < 0.05);
    assert!((float_solution.enu_m[2] - scenario.truth_enu_m[2]).abs() < 0.10);
    assert_eq!(float_solution.float_ambiguities.len(), double_differences.len());
    assert_eq!(float_solution.ambiguity_covariance_cycles2.len(), double_differences.len());
    assert_eq!(float_solution.enu_ambiguity_covariance_m_cycles.len(), 3);
    assert_eq!(projected_solution.enu_m, float_solution.enu_m);
    assert_eq!(
        projected_solution.covariance_m2.expect("projected covariance"),
        float_solution.covariance_enu_m2
    );
    assert!(!projected_solution.fixed);
}

#[test]
fn receiver_float_baseline_reference_switch_preserves_solution() {
    let scenario = clean_gps_l1_short_baseline_case();

    let single_differences = build_sd(&scenario.base_epoch, &scenario.rover_epoch);
    let reference = choose_ref_sat(&single_differences).expect("reference");
    let double_differences = build_dd(&single_differences, reference);
    let original_solution = solve_float_baseline_dd(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("original float solution");
    let new_reference = double_differences[0].sig;
    let transformed_solution =
        rtk_transform_float_baseline_reference(&original_solution, new_reference)
            .expect("transformed solution");
    let switched_double_differences =
        rtk_switch_double_difference_reference(&double_differences, new_reference)
            .expect("switched double differences");
    let switched_solution = solve_float_baseline_dd(
        &switched_double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("switched float solution");

    for axis in 0..3 {
        assert!((switched_solution.enu_m[axis] - transformed_solution.enu_m[axis]).abs() < 1.0e-6);
    }
    assert_eq!(
        switched_solution.float_ambiguities.len(),
        transformed_solution.float_ambiguities.len()
    );
    for (switched, transformed) in switched_solution
        .float_ambiguities
        .iter()
        .zip(transformed_solution.float_ambiguities.iter())
    {
        assert_eq!(switched.sig, transformed.sig);
        assert_eq!(switched.ref_sig, transformed.ref_sig);
        assert!((switched.float_cycles - transformed.float_cycles).abs() < 1.0e-6);
    }
}

#[test]
fn receiver_float_baseline_solver_reaches_centimeter_accuracy() {
    let scenario = clean_gps_l1_short_baseline_case();

    let single_differences = build_sd(&scenario.base_epoch, &scenario.rover_epoch);
    let reference = choose_ref_sat(&single_differences).expect("reference");
    let double_differences = build_dd(&single_differences, reference);
    let solution = solve_float_baseline_dd(
        &double_differences,
        scenario.base_ecef_m,
        &scenario.ephemerides,
        scenario.receive_gps_time.tow_s,
    )
    .expect("float solution");
    let accuracy = rtk_baseline_accuracy(solution.enu_m, scenario.truth_enu_m);

    assert!(
        accuracy.satisfies(centimeter_level_rtk_baseline_budget()),
        "accuracy={accuracy:?} solution={solution:?}"
    );
}
