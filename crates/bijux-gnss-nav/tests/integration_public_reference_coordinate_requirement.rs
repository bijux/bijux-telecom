#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::path::PathBuf;

mod support;

use bijux_gnss_core::api::ObsEpoch;
use bijux_gnss_nav::api::{parse_rinex_broadcast_navigation, PositionObservation, PositionSolver};
use bijux_gnss_nav::parse_rinex_gps_observation_dataset;
use bijux_gnss_testkit::reference_coordinate::TrustedReferenceCoordinateError;
use support::trusted_reference_coordinate::{
    require_trusted_reference_coordinate_by_fixture, trusted_reference_coordinate_enu_error_m,
    trusted_reference_coordinate_ecef_m,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

fn position_observations(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    let gps_receive_time = epoch.gps_time();
    epoch
        .sats
        .iter()
        .map(|sat| PositionObservation {
            sat: sat.signal_id.sat,
            pseudorange_m: sat.pseudorange_m.0,
            cn0_dbhz: sat.cn0_dbhz,
            elevation_deg: sat.elevation_deg,
            weight: 1.0,
            gps_receive_time,
            signal_timing: sat.timing,
        })
        .collect()
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
fn public_position_validation_rejects_missing_trusted_reference_coordinate() {
    let error = require_trusted_reference_coordinate_by_fixture("missing_fixture.obs")
        .expect_err("missing trusted reference coordinate must fail");

    assert_eq!(
        error,
        TrustedReferenceCoordinateError::MissingFixture("missing_fixture.obs".to_string())
    );
}

#[test]
fn public_rinex_position_validation_uses_trusted_reference_coordinate() {
    let station_fixture_name = "unavco_ab43_20180114.obs";
    let trusted_coordinate =
        require_trusted_reference_coordinate_by_fixture(station_fixture_name)
            .expect("trusted AB43 reference coordinate");
    let observations = parse_rinex_gps_observation_dataset(&fixture(station_fixture_name))
        .expect("parse public RINEX observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public RINEX navigation");
    let truth_ecef_m = trusted_reference_coordinate_ecef_m(&trusted_coordinate);
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    let mut solved_epochs = 0usize;
    let mut best_error_m = f64::INFINITY;
    let mut best_enu_error = None;
    let mut refusals = BTreeMap::new();
    for epoch in &observations.epochs {
        let observations = position_observations(epoch);
        let receive_tow_s = epoch.gps_time().expect("epoch GPS time").tow_s;
        let solution = solver.try_solve_wls_with_broadcast_ionosphere(
            &observations,
            &navigation.ephemerides,
            receive_tow_s,
            navigation.klobuchar.as_ref(),
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
        let enu_error = trusted_reference_coordinate_enu_error_m(
            (solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m),
            &trusted_coordinate,
        );
        if position_error_m < best_error_m {
            best_error_m = position_error_m;
            best_enu_error = Some(enu_error);
        }
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
