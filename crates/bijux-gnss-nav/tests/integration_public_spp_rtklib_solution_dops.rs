#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::PositionSolver;

use support::public_spp_case::{ab43_public_spp_case, position_observations, public_ab43_epoch};
use support::rtklib_reference::ab43_rtklib_dop_reference;

fn assert_finite_nonnegative(label: &str, value: f64) {
    assert!(
        value.is_finite() && value >= 0.0,
        "{label} should be finite and nonnegative, got {value:.12}"
    );
}

#[test]
fn public_ab43_solution_reports_finite_dops_for_rtklib_epochs() {
    let case = ab43_public_spp_case();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    let mut checked_epochs = 0usize;
    for reference_epoch in ab43_rtklib_dop_reference() {
        let epoch = public_ab43_epoch(reference_epoch.gps_time).expect("find AB43 public epoch");
        let observations = position_observations(epoch)
            .into_iter()
            .filter(|observation| reference_epoch.satellites.contains(&observation.sat))
            .collect::<Vec<_>>();
        let Ok(solution) = solver.try_solve_wls_with_broadcast_ionosphere(
            &observations,
            &case.navigation.ephemerides,
            reference_epoch.gps_time.tow_s,
            case.navigation.klobuchar.as_ref(),
        ) else {
            continue;
        };

        let gdop = solution.gdop.expect("solution gdop");
        let hdop = solution.hdop.expect("solution hdop");
        let vdop = solution.vdop.expect("solution vdop");
        let tdop = solution.tdop.expect("solution tdop");

        assert_finite_nonnegative("gdop", gdop);
        assert_finite_nonnegative("pdop", solution.pdop);
        assert_finite_nonnegative("hdop", hdop);
        assert_finite_nonnegative("vdop", vdop);
        assert_finite_nonnegative("tdop", tdop);
        assert!(
            (gdop * gdop - (solution.pdop * solution.pdop + tdop * tdop)).abs() < 1.0e-9,
            "solution GDOP decomposition should remain internally consistent"
        );
        checked_epochs += 1;
    }

    assert!(checked_epochs > 0, "expected at least one public AB43 solution DOP check");
}
