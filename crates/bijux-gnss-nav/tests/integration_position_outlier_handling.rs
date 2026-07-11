#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::MeasurementRejectReason;
use bijux_gnss_nav::api::PositionSolver;

use support::position_outlier::{position_error_3d_m, single_bad_pseudorange_scenario};

const BAD_PSEUDORANGE_BIAS_M: f64 = 1_000.0;

#[test]
fn position_solver_excludes_single_bad_pseudorange_and_recovers_truth() {
    let scenario = single_bad_pseudorange_scenario(BAD_PSEUDORANGE_BIAS_M);
    let guarded_solver = PositionSolver::new();
    let mut unguarded_solver = PositionSolver::new();
    unguarded_solver.robust = false;
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

    assert!(
        guarded
            .rejected
            .iter()
            .any(|(sat, reason)| *sat == scenario.bad_sat && *reason == MeasurementRejectReason::Outlier),
        "expected {:?} to be rejected as an outlier: {:?}",
        scenario.bad_sat,
        guarded.rejected,
    );
    assert_eq!(guarded.used_sat_count, 5, "{guarded:?}");
    assert_eq!(guarded.rejected_sat_count, 1, "{guarded:?}");
    assert!(
        guarded_error_m < 10.0,
        "guarded solution should stay near truth: error={}m solution={guarded:?}",
        guarded_error_m,
    );
    assert!(
        unguarded_error_m > guarded_error_m + 100.0,
        "outlier handling should materially improve the position: guarded={}m unguarded={}m",
        guarded_error_m,
        unguarded_error_m,
    );
}

#[test]
fn position_solver_reports_positive_effective_weights_for_retained_satellites() {
    let scenario = single_bad_pseudorange_scenario(BAD_PSEUDORANGE_BIAS_M);
    let solution = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("guarded solution");

    assert!(
        solution
            .residuals
            .iter()
            .all(|(sat, _residual_m, weight)| *sat != scenario.bad_sat && weight.is_finite() && *weight > 0.0),
        "retained satellites should carry positive effective weights: {:?}",
        solution.residuals,
    );
}
