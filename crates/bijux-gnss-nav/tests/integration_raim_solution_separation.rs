#![allow(missing_docs)]

mod support;

use std::collections::BTreeSet;

use bijux_gnss_nav::api::{PositionRobustWeighting, PositionSolver, RaimFaultDetectionStatus};
use support::position_outlier::single_bad_pseudorange_scenario;

#[test]
fn raim_solution_separation_solves_each_clean_satellite_subset() {
    let scenario = single_bad_pseudorange_scenario(0.0);
    let solution = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("clean six-satellite scenario should solve");

    let separation = solution
        .raim_solution_separation
        .as_ref()
        .expect("clean six-satellite scenario should compare subset solutions");
    let detection = solution
        .raim_fault_detection
        .expect("clean six-satellite scenario should evaluate RAIM detection");

    assert_eq!(detection.status, RaimFaultDetectionStatus::Consistent);
    assert_eq!(separation.reference_sat_count, solution.used_sat_count);
    assert_eq!(separation.compared_subset_count(), solution.used_sat_count);
    assert_eq!(
        separation
            .compared_subsets
            .iter()
            .map(|subset| subset.excluded_sat)
            .collect::<BTreeSet<_>>()
            .len(),
        solution.used_sat_count,
    );
    assert!(separation
        .max_separation()
        .is_some_and(|subset| subset.separation_m <= detection.threshold_m));
}

#[test]
fn raim_solution_separation_identifies_bad_satellite_from_subset_solves() {
    let scenario = single_bad_pseudorange_scenario(1_000.0);
    let mut solver = PositionSolver::new();
    solver.robust_weighting = PositionRobustWeighting::Disabled;
    solver.residual_gate_m = 1.0e9;
    solver.chi_square_gate = 1.0e12;

    let solution = solver
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect("bad-pseudorange scenario should solve for subset-based fault detection");

    let separation = solution
        .raim_solution_separation
        .as_ref()
        .expect("bad-pseudorange scenario should compare subset solutions");
    let detection = solution
        .raim_fault_detection
        .expect("bad-pseudorange scenario should evaluate RAIM detection");
    let max_subset = separation.max_separation().expect("subset comparison should produce a maximum");

    assert_eq!(separation.reference_sat_count, solution.used_sat_count);
    assert_eq!(separation.compared_subset_count(), solution.used_sat_count);
    assert_eq!(max_subset.excluded_sat, scenario.bad_sat);
    assert_eq!(detection.status, RaimFaultDetectionStatus::FaultDetected);
    assert_eq!(detection.suspect_sat, Some(scenario.bad_sat));
    assert_eq!(detection.max_solution_separation_m, max_subset.separation_m);
    assert!(detection.max_solution_separation_m > detection.threshold_m);
    assert!(solution.raim_fault_exclusion.is_none(), "{solution:?}");
}
