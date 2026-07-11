#![allow(missing_docs)]

use bijux_gnss_receiver::api::{
    build_dd, build_sd, choose_ref_sat, solve_baseline_dd, solve_float_baseline_dd,
};
use bijux_gnss_testkit::rtk_baseline::clean_gps_l1_short_baseline_case;

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
    assert_eq!(projected_solution.enu_m, float_solution.enu_m);
    assert_eq!(
        projected_solution.covariance_m2.expect("projected covariance"),
        float_solution.covariance_enu_m2
    );
    assert!(!projected_solution.fixed);
}
