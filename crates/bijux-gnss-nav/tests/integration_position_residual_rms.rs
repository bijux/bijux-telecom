#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::PositionSolver;

use support::position_outlier::single_bad_pseudorange_scenario;

const BAD_PSEUDORANGE_BIAS_M: f64 = 1_000.0;

#[test]
fn clean_solution_reports_matching_pre_and_post_fit_rms() {
    let scenario = single_bad_pseudorange_scenario(0.0);
    let solution = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("clean six-satellite solution");

    assert!(
        (solution.pre_fit_residual_rms_m - solution.post_fit_residual_rms_m).abs() < 1.0e-12,
        "clean solution should not change residual RMS across fitting: {solution:?}",
    );
    assert!(
        (solution.rms_m - solution.post_fit_residual_rms_m).abs() < 1.0e-12,
        "compatibility RMS field should remain the post-fit RMS: {solution:?}",
    );
}

#[test]
fn bad_satellite_solution_reports_improved_post_fit_rms() {
    let scenario = single_bad_pseudorange_scenario(BAD_PSEUDORANGE_BIAS_M);
    let solution = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("guarded solution");

    assert!(
        solution.pre_fit_residual_rms_m > solution.post_fit_residual_rms_m + 100.0,
        "bad-satellite recovery should materially reduce residual RMS: {solution:?}",
    );
    assert!(
        (solution.rms_m - solution.post_fit_residual_rms_m).abs() < 1.0e-12,
        "compatibility RMS field should remain the post-fit RMS: {solution:?}",
    );
}
