mod support;

use bijux_gnss_nav::api::{
    formal_protection_levels, PositionObservation, PositionSolver,
};

use support::public_spp_case::{ab43_public_spp_case, position_observations};

fn solve_public_epoch(observations: &[PositionObservation]) -> bijux_gnss_nav::api::PositionSolution {
    let case = ab43_public_spp_case();
    let epoch = case
        .observations
        .epochs
        .iter()
        .find(|epoch| epoch.sats.len() >= 6)
        .expect("AB43 epoch with at least six satellites");
    let solver = PositionSolver { raim: false, apply_troposphere: true, ..PositionSolver::new() };
    solver
        .try_solve_wls_with_gps_broadcast_navigation(
            observations,
            &case.navigation,
            epoch.gps_time().expect("AB43 epoch GPS time").tow_s,
        )
        .expect("public AB43 solution")
}

#[test]
fn public_solver_reports_formal_protection_levels_from_covariance() {
    let case = ab43_public_spp_case();
    let epoch = case
        .observations
        .epochs
        .iter()
        .find(|epoch| epoch.sats.len() >= 6)
        .expect("AB43 epoch with at least six satellites");
    let observations = position_observations(epoch);
    let solution = solve_public_epoch(&observations);
    let covariance = solution.position_covariance_ecef_m2.expect("position covariance");
    let expected = formal_protection_levels(
        [solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m],
        covariance,
    )
    .expect("formal protection levels");

    assert_eq!(solution.integrity_hpl_m, Some(expected.horizontal_m));
    assert_eq!(solution.integrity_vpl_m, Some(expected.vertical_m));
    assert!(expected.horizontal_m >= solution.horizontal_error_ellipse_major_axis_m.expect("ellipse") * 6.0);
    assert!(expected.vertical_m >= solution.sigma_v_m.expect("vertical sigma") * 6.0 - 1.0e-12);
}

#[test]
fn public_solver_hpl_worsens_for_restricted_satellite_geometry() {
    let case = ab43_public_spp_case();
    let epoch = case
        .observations
        .epochs
        .iter()
        .find(|epoch| epoch.sats.len() >= 6)
        .expect("AB43 epoch with at least six satellites");
    let observations = position_observations(epoch);
    let full_solution = solve_public_epoch(&observations);
    let full_hpl_m = full_solution.integrity_hpl_m.expect("full geometry HPL");
    let mut worst_subset_hpl_m = full_hpl_m;

    for excluded_index in 0..observations.len() {
        let subset = observations
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != excluded_index)
            .map(|(_, observation)| observation.clone())
            .collect::<Vec<_>>();
        let subset_solution = solve_public_epoch(&subset);
        let subset_hpl_m = subset_solution.integrity_hpl_m.expect("subset HPL");
        if subset_hpl_m > worst_subset_hpl_m {
            worst_subset_hpl_m = subset_hpl_m;
        }
    }

    assert!(worst_subset_hpl_m > full_hpl_m);
}
