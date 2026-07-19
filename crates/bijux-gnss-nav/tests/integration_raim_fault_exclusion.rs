#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::{PositionRobustWeighting, PositionSolver, RaimFaultDetectionStatus};

use support::position_outlier::{position_error_3d_m, single_bad_pseudorange_scenario};

const BAD_PSEUDORANGE_BIAS_M: f64 = 1_000.0;

#[test]
fn raim_fault_exclusion_reports_improved_solution_for_single_bad_pseudorange() {
    let scenario = single_bad_pseudorange_scenario(BAD_PSEUDORANGE_BIAS_M);
    let guarded_solver = PositionSolver::new();
    let mut unguarded_solver = PositionSolver::new();
    unguarded_solver.robust_weighting = PositionRobustWeighting::Disabled;
    unguarded_solver.raim = false;
    unguarded_solver.residual_gate_m = 1.0e9;
    unguarded_solver.chi_square_gate = 1.0e12;

    let guarded = guarded_solver
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("guarded solution");
    let unguarded = unguarded_solver
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("unguarded solution");
    let guarded_error_m = position_error_3d_m(
        guarded.ecef_x_m,
        guarded.ecef_y_m,
        guarded.ecef_z_m,
        scenario.baseline.truth_ecef_m,
    );
    let unguarded_error_m = position_error_3d_m(
        unguarded.ecef_x_m,
        unguarded.ecef_y_m,
        unguarded.ecef_z_m,
        scenario.baseline.truth_ecef_m,
    );
    let detection = guarded
        .raim_fault_detection
        .expect("bad-pseudorange scenario should evaluate RAIM detection");
    let exclusion = guarded
        .raim_fault_exclusion
        .expect("bad-pseudorange scenario should report RAIM exclusion");

    assert_eq!(detection.status, RaimFaultDetectionStatus::FaultDetected);
    assert_eq!(exclusion.excluded_sat, scenario.bad_sat);
    assert!(exclusion.improved(), "{exclusion:?}");
    assert!(
        exclusion.pre_exclusion_rms_m > exclusion.post_exclusion_rms_m + 100.0,
        "exclusion should materially reduce residual RMS: {exclusion:?}",
    );
    assert!(
        exclusion.solution_shift_m > 10.0,
        "excluding the faulty satellite should meaningfully move the solution: {exclusion:?}",
    );
    assert!(
        guarded_error_m < 10.0,
        "excluded solution should stay near truth: error={guarded_error_m}m solution={guarded:?}",
    );
    assert!(
        unguarded_error_m > guarded_error_m + 100.0,
        "explicit exclusion should materially improve the position: guarded={guarded_error_m}m unguarded={unguarded_error_m}m",
    );
}

#[test]
fn raim_fault_exclusion_stays_empty_for_clean_geometry() {
    let scenario = single_bad_pseudorange_scenario(0.0);
    let solution = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("clean six-satellite solution");

    assert!(solution.raim_fault_exclusion.is_none(), "{solution:?}");
}
