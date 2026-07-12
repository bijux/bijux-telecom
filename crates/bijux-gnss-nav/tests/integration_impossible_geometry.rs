#![allow(missing_docs)]

mod support;

use bijux_gnss_nav::api::PositionSolver;
use support::position_truth::{
    four_satellite_position_scenario, impossible_geometry_position_scenario,
};

fn position_error_3d_m(solution_ecef_m: (f64, f64, f64), truth_ecef_m: (f64, f64, f64)) -> f64 {
    let dx = solution_ecef_m.0 - truth_ecef_m.0;
    let dy = solution_ecef_m.1 - truth_ecef_m.1;
    let dz = solution_ecef_m.2 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[test]
fn solver_flags_observations_that_imply_an_impossible_receiver_position() {
    let scenario = impossible_geometry_position_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("impossible-geometry observations should still produce a best-fit solution");

    let impossible_geometry =
        solution.impossible_geometry.expect("inside-earth truth must be flagged");

    assert!(
        position_error_3d_m(
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            scenario.truth_ecef_m,
        ) < 20.0
    );
    assert_eq!(impossible_geometry.used_satellite_count, solution.used_sat_count);
    assert!(impossible_geometry.receiver_radius_m < impossible_geometry.min_receiver_radius_m);
    assert!(impossible_geometry.altitude_m < impossible_geometry.min_altitude_m);
}

#[test]
fn solver_leaves_terrestrial_truth_unflagged() {
    let scenario = four_satellite_position_scenario(0.0);
    let solution = PositionSolver::new()
        .solve_wls(&scenario.observations, &scenario.ephemerides, scenario.t_rx_s)
        .expect("clean terrestrial geometry should solve");

    assert!(solution.impossible_geometry.is_none(), "{solution:?}");
}
