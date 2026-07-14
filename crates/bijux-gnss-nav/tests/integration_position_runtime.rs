#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::{PositionBroadcastNavigation, PositionRuntime, PositionRuntimeConfig};
use support::public_spp_case::ab43_public_spp_case;
use support::public_station_truth::station_enu_error_m;

fn position_error_3d_m(
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = ecef_x_m - truth_ecef_m.0;
    let dy = ecef_y_m - truth_ecef_m.1;
    let dz = ecef_z_m - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[test]
fn position_runtime_solves_public_broadcast_navigation_near_station_truth() {
    let case = ab43_public_spp_case();
    let truth_ecef_m = case.station_truth.truth_ecef_m();
    let mut runtime = PositionRuntime::new(PositionRuntimeConfig::default());

    let mut solved_epochs = 0usize;
    let mut best_error_m = f64::INFINITY;
    let mut best_enu_error = None;
    for epoch in &case.observations.epochs {
        let Some(solution) =
            runtime.solve_epoch_with_gps_broadcast_navigation(epoch, &case.navigation)
        else {
            continue;
        };

        let position_error_m = position_error_3d_m(
            solution.ecef_x_m.0,
            solution.ecef_y_m.0,
            solution.ecef_z_m.0,
            truth_ecef_m,
        );
        let enu_error = station_enu_error_m(
            (solution.ecef_x_m.0, solution.ecef_y_m.0, solution.ecef_z_m.0),
            &case.station_truth,
        );
        if position_error_m < best_error_m {
            best_error_m = position_error_m;
            best_enu_error = Some(enu_error);
        }
        solved_epochs += 1;
    }

    assert!(solved_epochs > 0, "public dataset should yield at least one runtime solution");
    assert!(best_error_m < 50.0, "best runtime error {best_error_m:.3} m exceeds tolerance");

    let best_enu_error = best_enu_error.expect("best runtime ENU error");
    assert!(
        best_enu_error.horizontal_m < 1.0,
        "best runtime horizontal error {0:.3} m exceeds tolerance",
        best_enu_error.horizontal_m,
    );
    assert!(
        best_enu_error.up_m.abs() < 0.5,
        "best runtime vertical error {0:.3} m exceeds tolerance",
        best_enu_error.up_m,
    );
}

#[test]
fn position_runtime_matches_equivalent_navigation_entry_route() {
    let case = ab43_public_spp_case();
    let navigation_entries = case
        .navigation
        .ephemerides
        .iter()
        .cloned()
        .map(PositionBroadcastNavigation::Gps)
        .collect::<Vec<_>>();

    let mut broadcast_runtime = PositionRuntime::new(PositionRuntimeConfig::default());
    let mut entry_runtime = PositionRuntime::new(PositionRuntimeConfig::default());

    let (broadcast_solution, entry_solution) = case
        .observations
        .epochs
        .iter()
        .find_map(|epoch| {
            let broadcast_solution = broadcast_runtime
                .solve_epoch_with_gps_broadcast_navigation(epoch, &case.navigation)?;
            let entry_solution = entry_runtime
                .solve_epoch_with_navigation_data_and_broadcast_ionosphere(
                    epoch,
                    &navigation_entries,
                    case.navigation.klobuchar.as_ref(),
                )?;
            Some((broadcast_solution, entry_solution))
        })
        .expect("public dataset should yield a comparable runtime solve");

    assert_eq!(broadcast_solution.status, entry_solution.status);
    assert_eq!(broadcast_solution.valid, entry_solution.valid);
    assert!((broadcast_solution.ecef_x_m.0 - entry_solution.ecef_x_m.0).abs() < 1.0e-6);
    assert!((broadcast_solution.ecef_y_m.0 - entry_solution.ecef_y_m.0).abs() < 1.0e-6);
    assert!((broadcast_solution.ecef_z_m.0 - entry_solution.ecef_z_m.0).abs() < 1.0e-6);
    assert!((broadcast_solution.clock_bias_s.0 - entry_solution.clock_bias_s.0).abs() < 1.0e-12);
    assert_eq!(broadcast_solution.residuals.len(), entry_solution.residuals.len());
    let broadcast_provenance = broadcast_solution.provenance.expect("broadcast runtime provenance");
    let entry_provenance = entry_solution.provenance.expect("entry runtime provenance");
    assert_eq!(broadcast_provenance.solver_family, entry_provenance.solver_family);
    assert_eq!(broadcast_provenance.weighting_mode, entry_provenance.weighting_mode);
    assert_eq!(broadcast_provenance.robust_solver, entry_provenance.robust_solver);
    assert_eq!(broadcast_provenance.raim_enabled, entry_provenance.raim_enabled);
    assert_eq!(broadcast_provenance.satellites_used, entry_provenance.satellites_used);
}
