#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::PositionSolver;

use support::public_spp_case::{ab43_public_spp_case, position_observations, public_ab43_epoch};
use support::rtklib_reference::ab43_rtklib_dop_reference;

fn assert_close(label: &str, actual: f64, expected: f64, tolerance: f64) {
    let error = (actual - expected).abs();
    assert!(
        error <= tolerance,
        "{label} mismatch: actual={actual:.12} expected={expected:.12} error={error:.12} tolerance={tolerance:.12}"
    );
}

#[test]
fn public_ab43_solution_dops_track_rtklib_reference() {
    let case = ab43_public_spp_case();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    for reference_epoch in ab43_rtklib_dop_reference() {
        let epoch = public_ab43_epoch(reference_epoch.gps_time).expect("find AB43 public epoch");
        let observations = position_observations(epoch)
            .into_iter()
            .filter(|observation| reference_epoch.satellites.contains(&observation.sat))
            .collect::<Vec<_>>();
        let solution = solver
            .try_solve_wls_with_broadcast_ionosphere(
                &observations,
                &case.navigation.ephemerides,
                reference_epoch.gps_time.tow_s,
                case.navigation.klobuchar.as_ref(),
            )
            .expect("solve AB43 public epoch with RTKLIB satellite set");

        assert_close("gdop", solution.gdop.expect("solution gdop"), reference_epoch.gdop, 0.05);
        assert_close("pdop", solution.pdop, reference_epoch.pdop, 0.05);
        assert_close("hdop", solution.hdop.expect("solution hdop"), reference_epoch.hdop, 0.05);
        assert_close("vdop", solution.vdop.expect("solution vdop"), reference_epoch.vdop, 0.05);
    }
}
