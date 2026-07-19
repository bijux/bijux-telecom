#![allow(missing_docs)]
mod support;

use bijux_gnss_nav::api::{GpsBroadcastNavigationData, PositionSolver};

use support::position_truth::{
    add_klobuchar_delay_to_observations, four_satellite_position_scenario,
    sample_klobuchar_coefficients,
};

fn position_error_3d_m(actual_ecef_m: (f64, f64, f64), truth_ecef_m: (f64, f64, f64)) -> f64 {
    let dx = actual_ecef_m.0 - truth_ecef_m.0;
    let dy = actual_ecef_m.1 - truth_ecef_m.1;
    let dz = actual_ecef_m.2 - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[test]
fn gps_broadcast_navigation_payload_recovers_single_point_position() {
    let scenario = four_satellite_position_scenario(150.0e-9);
    let klobuchar = sample_klobuchar_coefficients();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: scenario.ephemerides.clone(),
        klobuchar: Some(klobuchar),
    };
    let ionosphere_biased_observations = add_klobuchar_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        klobuchar,
    );

    let corrected_solution = PositionSolver::new()
        .solve_wls_with_gps_broadcast_navigation(
            &ionosphere_biased_observations,
            &navigation,
            scenario.t_rx_s,
        )
        .expect("broadcast navigation payload should solve");
    let uncorrected_solution = PositionSolver::new()
        .solve_wls(&ionosphere_biased_observations, &navigation.ephemerides, scenario.t_rx_s)
        .expect("uncorrected observations should still solve");

    let corrected_error_m = position_error_3d_m(
        (corrected_solution.ecef_x_m, corrected_solution.ecef_y_m, corrected_solution.ecef_z_m),
        scenario.truth_ecef_m,
    );
    let uncorrected_error_m = position_error_3d_m(
        (
            uncorrected_solution.ecef_x_m,
            uncorrected_solution.ecef_y_m,
            uncorrected_solution.ecef_z_m,
        ),
        scenario.truth_ecef_m,
    );

    assert!(corrected_solution.broadcast_ionosphere_applied);
    assert!(!uncorrected_solution.broadcast_ionosphere_applied);
    assert!(corrected_error_m < 5.0);
    assert!(uncorrected_error_m > corrected_error_m + 3.0);
}

#[test]
fn gps_broadcast_navigation_payload_matches_explicit_klobuchar_solution() {
    let scenario = four_satellite_position_scenario(-120.0e-9);
    let klobuchar = sample_klobuchar_coefficients();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: scenario.ephemerides.clone(),
        klobuchar: Some(klobuchar),
    };
    let ionosphere_biased_observations = add_klobuchar_delay_to_observations(
        &scenario.observations,
        &scenario.ephemerides,
        scenario.truth_ecef_m,
        scenario.t_rx_s,
        klobuchar,
    );

    let payload_solution = PositionSolver::new()
        .solve_wls_with_gps_broadcast_navigation(
            &ionosphere_biased_observations,
            &navigation,
            scenario.t_rx_s,
        )
        .expect("payload solution");
    let explicit_solution = PositionSolver::new()
        .solve_wls_with_broadcast_ionosphere(
            &ionosphere_biased_observations,
            &navigation.ephemerides,
            scenario.t_rx_s,
            Some(&klobuchar),
        )
        .expect("explicit solution");

    assert!((payload_solution.ecef_x_m - explicit_solution.ecef_x_m).abs() < 1.0e-9);
    assert!((payload_solution.ecef_y_m - explicit_solution.ecef_y_m).abs() < 1.0e-9);
    assert!((payload_solution.ecef_z_m - explicit_solution.ecef_z_m).abs() < 1.0e-9);
    assert_eq!(
        payload_solution.broadcast_ionosphere_applied,
        explicit_solution.broadcast_ionosphere_applied
    );
}
