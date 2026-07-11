#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::MeasurementRejectReason;
use bijux_gnss_nav::api::{PositionSolveRefusalKind, PositionSolver};

use support::position_outlier::five_satellite_bad_pseudorange_scenario;

#[test]
fn position_solver_refuses_five_satellite_raim_exclusion_case() {
    let scenario = five_satellite_bad_pseudorange_scenario(1_000.0);
    let refusal = PositionSolver::new()
        .try_solve_wls(
            &scenario.observations,
            &scenario.baseline.ephemerides,
            scenario.baseline.t_rx_s,
        )
        .expect_err("five-satellite fault exclusion should be refused as underdetermined");

    assert_eq!(refusal.kind, PositionSolveRefusalKind::UnderdeterminedRaimExclusion);
    assert_eq!(refusal.sat_count, 5);
    assert_eq!(refusal.used_sat_count, 4);
    assert_eq!(
        refusal
            .rejected
            .iter()
            .filter(|(_sat, reason)| *reason == MeasurementRejectReason::Outlier)
            .count(),
        1,
        "{refusal:?}",
    );
}
