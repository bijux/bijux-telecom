#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::MeasurementRejectReason;
use bijux_gnss_nav::api::{PositionSolver, RaimFaultDetectionStatus};
use support::position_outlier::single_bad_pseudorange_scenario;

#[test]
fn raim_fault_detection_identifies_single_bad_pseudorange() {
    let scenario = single_bad_pseudorange_scenario(1_000.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.baseline.ephemerides, scenario.baseline.t_rx_s)
        .expect("bad-pseudorange scenario should still solve after exclusion");

    let detection = solution
        .raim_fault_detection
        .expect("six-satellite bad-pseudorange scenario should evaluate RAIM");

    assert_eq!(detection.status, RaimFaultDetectionStatus::FaultDetected);
    assert_eq!(detection.suspect_sat, Some(scenario.bad_sat));
    assert!(detection.max_solution_separation_m > detection.threshold_m);
    assert!(solution.rejected.iter().any(
        |(sat, reason)| *sat == scenario.bad_sat && *reason == MeasurementRejectReason::Outlier
    ));
}

#[test]
fn raim_fault_detection_reports_consistent_clean_geometry() {
    let scenario = single_bad_pseudorange_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.baseline.ephemerides, scenario.baseline.t_rx_s)
        .expect("clean six-satellite scenario should solve");

    let detection =
        solution.raim_fault_detection.expect("clean six-satellite scenario should evaluate RAIM");

    assert_eq!(detection.status, RaimFaultDetectionStatus::Consistent);
    assert_eq!(detection.suspect_sat, None);
    assert!(detection.max_solution_separation_m <= detection.threshold_m);
    assert!(solution.rejected.is_empty());
}
