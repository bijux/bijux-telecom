#![allow(missing_docs)]

use std::path::PathBuf;
use std::collections::BTreeMap;

use bijux_gnss_core::api::ObsEpoch;
use bijux_gnss_nav::api::{
    parse_rinex_broadcast_navigation, PositionObservation, PositionSolver,
};
use bijux_gnss_nav::parse_rinex_gps_observation_dataset;

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
fn public_rinex_obs_and_nav_resolve_near_station_position() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public RINEX observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public RINEX navigation");
    let truth_ecef_m =
        observations.approx_position_ecef_m.expect("public fixture approximate station position");
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };

    let mut solved_epochs = 0usize;
    let mut best_error_m = f64::INFINITY;
    let mut worst_error_m = 0.0_f64;
    let mut refusals = BTreeMap::new();
    for epoch in &observations.epochs {
        let position_observations = position_observations(epoch);
        let receive_tow_s = epoch.gps_time().expect("epoch GPS time").tow_s;
        let solution = solver
            .try_solve_wls_with_broadcast_ionosphere(
                &position_observations,
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
        best_error_m = best_error_m.min(position_error_m);
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
}
