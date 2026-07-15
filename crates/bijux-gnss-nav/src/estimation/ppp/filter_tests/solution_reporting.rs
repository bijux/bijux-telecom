use super::*;

#[test]
fn ppp_solution_epoch_exposes_ecef_position_covariance() {
    let mut filter = PppFilter::new(PppConfig::default());
    filter.seed_receiver_state([6_378_137.0, 10.0, 10.0], 4.0e-6);
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[0])] = 4.0;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[1])] = 9.0;
    filter.ekf.p[(filter.indices.pos[2], filter.indices.pos[2])] = 16.0;
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[1])] = 0.5;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[0])] = 0.5;

    let solution = filter.solution_epoch(
        7,
        7.0,
        Vec::new(),
        0,
        ppp_stochastic_evidence_from_config(&filter.config),
    );
    let covariance =
        solution.position_covariance_ecef_m2.expect("PPP solution should emit position covariance");

    assert_eq!(covariance[0][0], 4.0);
    assert_eq!(covariance[1][1], 9.0);
    assert_eq!(covariance[2][2], 16.0);
    assert_eq!(covariance[0][1], 0.5);
    assert_eq!(covariance[1][0], 0.5);
    assert!(solution.sigma_e_m.expect("east sigma").is_finite());
    assert!(solution.sigma_n_m.expect("north sigma").is_finite());
    assert!(solution.sigma_u_m.expect("up sigma").is_finite());
    assert!(solution
        .horizontal_error_ellipse_major_axis_m
        .expect("ellipse major axis")
        .is_finite());
    assert!(solution
        .horizontal_error_ellipse_minor_axis_m
        .expect("ellipse minor axis")
        .is_finite());
    assert!(solution.horizontal_error_ellipse_azimuth_deg.expect("ellipse azimuth").is_finite());
}
