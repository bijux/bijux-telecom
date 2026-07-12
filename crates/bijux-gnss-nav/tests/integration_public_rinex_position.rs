#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::path::PathBuf;

mod support;

use bijux_gnss_core::api::ObsEpoch;
use bijux_gnss_nav::api::{
    parse_rinex_broadcast_navigation, parse_rinex_gps_observation_dataset,
    position_observations_from_epoch, PositionObservation, PositionSolver,
};
use support::public_station_truth::{public_station_truth_by_fixture, station_enu_error_m};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

fn position_observations(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    position_observations_from_epoch(epoch)
}

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
fn public_rinex_obs_and_nav_resolve_near_station_position() {
    let station_fixture_name = "unavco_ab43_20180114.obs";
    let observations = parse_rinex_gps_observation_dataset(&fixture(station_fixture_name))
        .expect("parse public RINEX observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public RINEX navigation");
    let station_truth = public_station_truth_by_fixture(station_fixture_name);
    let truth_ecef_m = station_truth.truth_ecef_m();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    let mut solved_epochs = 0usize;
    let mut best_error_m = f64::INFINITY;
    let mut worst_error_m = 0.0_f64;
    let mut best_enu_error = None;
    let mut refusals = BTreeMap::new();
    for epoch in &observations.epochs {
        let position_observations = position_observations(epoch);
        let receive_tow_s = epoch.gps_time().expect("epoch GPS time").tow_s;
        let solution = solver.try_solve_wls_with_gps_broadcast_navigation(
            &position_observations,
            &navigation,
            receive_tow_s,
        );
        let Ok(solution) = solution else {
            let refusal = solution.expect_err("refusal");
            *refusals.entry(format!("{:?}", refusal.kind)).or_insert(0usize) += 1;
            continue;
        };
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            truth_ecef_m,
        );
        let enu_error = station_enu_error_m(
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            &station_truth,
        );
        if position_error_m < best_error_m {
            best_error_m = position_error_m;
            best_enu_error = Some(enu_error);
        }
        worst_error_m = worst_error_m.max(position_error_m);
        solved_epochs += 1;
    }

    assert!(
        solved_epochs > 0,
        "public dataset should produce at least one valid position solve; refusals={refusals:?}"
    );
    assert!(
        best_error_m < 50.0,
        "best public-dataset position error {best_error_m:.3} m exceeds tolerance"
    );
    assert!(
        worst_error_m < 150.0,
        "worst successful public-dataset position error {worst_error_m:.3} m exceeds tolerance"
    );
    let best_enu_error =
        best_enu_error.expect("successful public solution should record ENU error");
    assert!(
        best_enu_error.horizontal_m < 1.0,
        "best public-station horizontal error {0:.3} m exceeds tolerance; east={1:.3}m north={2:.3}m up={3:.3}m",
        best_enu_error.horizontal_m,
        best_enu_error.east_m,
        best_enu_error.north_m,
        best_enu_error.up_m,
    );
    assert!(
        best_enu_error.up_m.abs() < 0.5,
        "best public-station vertical error {0:.3} m exceeds tolerance; east={1:.3}m north={2:.3}m horizontal={3:.3}m",
        best_enu_error.up_m,
        best_enu_error.east_m,
        best_enu_error.north_m,
        best_enu_error.horizontal_m,
    );
}

#[test]
fn public_rinex_klobuchar_payload_improves_real_position_error() {
    let station_fixture_name = "unavco_ab43_20180114.obs";
    let observations = parse_rinex_gps_observation_dataset(&fixture(station_fixture_name))
        .expect("parse public RINEX observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public RINEX navigation");
    let mut uncorrected_navigation = navigation.clone();
    uncorrected_navigation.klobuchar = None;
    let station_truth = public_station_truth_by_fixture(station_fixture_name);
    let truth_ecef_m = station_truth.truth_ecef_m();
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    let mut compared_epochs = 0usize;
    let mut improved_epochs = 0usize;
    let mut corrected_best_error_m = f64::INFINITY;
    let mut uncorrected_best_error_m = f64::INFINITY;
    let mut total_improvement_m = 0.0;
    for epoch in &observations.epochs {
        let position_observations = position_observations(epoch);
        let receive_tow_s = epoch.gps_time().expect("epoch GPS time").tow_s;
        let corrected = solver.try_solve_wls_with_gps_broadcast_navigation(
            &position_observations,
            &navigation,
            receive_tow_s,
        );
        let uncorrected = solver.try_solve_wls_with_gps_broadcast_navigation(
            &position_observations,
            &uncorrected_navigation,
            receive_tow_s,
        );
        let (Ok(corrected), Ok(uncorrected)) = (corrected, uncorrected) else {
            continue;
        };

        let corrected_error_m = position_error_3d_m(
            corrected.ecef_x_m,
            corrected.ecef_y_m,
            corrected.ecef_z_m,
            truth_ecef_m,
        );
        let uncorrected_error_m = position_error_3d_m(
            uncorrected.ecef_x_m,
            uncorrected.ecef_y_m,
            uncorrected.ecef_z_m,
            truth_ecef_m,
        );
        if corrected_error_m + 0.1 < uncorrected_error_m {
            improved_epochs += 1;
        }
        corrected_best_error_m = corrected_best_error_m.min(corrected_error_m);
        uncorrected_best_error_m = uncorrected_best_error_m.min(uncorrected_error_m);
        total_improvement_m += uncorrected_error_m - corrected_error_m;
        compared_epochs += 1;
    }

    let mean_improvement_m =
        if compared_epochs > 0 { total_improvement_m / compared_epochs as f64 } else { 0.0 };

    assert!(compared_epochs > 0, "public dataset should yield comparable corrected epochs");
    assert!(
        improved_epochs * 2 > compared_epochs,
        "Klobuchar correction should improve most public epochs; improved={improved_epochs} compared={compared_epochs}"
    );
    assert!(
        mean_improvement_m > 0.5,
        "mean public position improvement {mean_improvement_m:.3} m is too small"
    );
    assert!(
        corrected_best_error_m + 0.5 < uncorrected_best_error_m,
        "best corrected public error {corrected_best_error_m:.3} m should beat uncorrected {uncorrected_best_error_m:.3} m"
    );
}
