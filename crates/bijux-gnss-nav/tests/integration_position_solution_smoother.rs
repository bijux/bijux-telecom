#![allow(missing_docs)]

use bijux_gnss_core::api::Constellation;
use bijux_gnss_nav::api::{
    geodetic_to_ecef, PositionFilterMotionClass, PositionSolution, PositionSolutionSmoother,
    PositionSolutionSmootherConfig,
};

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

fn root_mean_square(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn path_length_m(positions_ecef_m: &[(f64, f64, f64)]) -> f64 {
    positions_ecef_m
        .windows(2)
        .map(|window| {
            let dx = window[1].0 - window[0].0;
            let dy = window[1].1 - window[0].1;
            let dz = window[1].2 - window[0].2;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

fn deterministic_position_offset_m(epoch_index: usize) -> (f64, f64, f64) {
    let east = (((epoch_index * 17 + 3) % 11) as f64 - 5.0) * 0.95;
    let north = (((epoch_index * 13 + 5) % 9) as f64 - 4.0) * 0.75;
    let up = (((epoch_index * 19 + 7) % 7) as f64 - 3.0) * 0.60;
    (east, north, up)
}

fn translate_truth_ecef_m(
    truth_ecef_m: (f64, f64, f64),
    truth_velocity_mps: (f64, f64, f64),
    dt_s: f64,
) -> (f64, f64, f64) {
    (
        truth_ecef_m.0 + truth_velocity_mps.0 * dt_s,
        truth_ecef_m.1 + truth_velocity_mps.1 * dt_s,
        truth_ecef_m.2 + truth_velocity_mps.2 * dt_s,
    )
}

fn sample_position_solution(
    measured_ecef_m: (f64, f64, f64),
    sigma_h_m: f64,
    sigma_v_m: f64,
) -> PositionSolution {
    PositionSolution {
        ecef_x_m: measured_ecef_m.0,
        ecef_y_m: measured_ecef_m.1,
        ecef_z_m: measured_ecef_m.2,
        position_covariance_ecef_m2: Some([
            [sigma_h_m.powi(2), 0.4, 0.1],
            [0.4, sigma_h_m.powi(2), 0.2],
            [0.1, 0.2, sigma_v_m.powi(2)],
        ]),
        horizontal_error_ellipse_major_axis_m: Some(sigma_h_m * 1.2),
        horizontal_error_ellipse_minor_axis_m: Some(sigma_h_m),
        horizontal_error_ellipse_azimuth_deg: Some(17.0),
        sigma_e_m: Some(sigma_h_m),
        sigma_n_m: Some(sigma_h_m),
        sigma_u_m: Some(sigma_v_m),
        latitude_deg: 37.0,
        longitude_deg: -122.0,
        altitude_m: 10.0,
        clock_reference_constellation: Constellation::Gps,
        clock_bias_s: 2.75e-4,
        inter_system_biases: Vec::new(),
        pdop: 1.8,
        hdop: Some(1.2),
        vdop: Some(1.4),
        gdop: Some(2.0),
        tdop: Some(1.1),
        pre_fit_residual_rms_m: 1.0,
        post_fit_residual_rms_m: 0.8,
        rms_m: 0.8,
        sigma_h_m: Some(sigma_h_m),
        sigma_v_m: Some(sigma_v_m),
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        rejected: Vec::new(),
        raim_fault_detection: None,
        raim_fault_exclusion: None,
        separation_max_m: None,
        separation_suspect: None,
        covariance_symmetrized: false,
        covariance_clamped: false,
        covariance_max_variance: None,
        sat_count: 6,
        used_sat_count: 6,
        rejected_sat_count: 0,
    }
}

#[test]
fn position_solution_smoother_reduces_static_jitter() {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let mut config =
        PositionSolutionSmootherConfig::for_motion_class(PositionFilterMotionClass::Static);
    config.measurement_sigma_floor_m = 0.5;
    let mut smoother = PositionSolutionSmoother::new(config);
    let mut raw_errors_m = Vec::new();
    let mut smoothed_errors_m = Vec::new();
    let mut raw_positions_ecef_m = Vec::new();
    let mut smoothed_positions_ecef_m = Vec::new();

    for epoch_index in 0..80 {
        let offset_m = deterministic_position_offset_m(epoch_index);
        let measured_ecef_m = (
            truth_ecef_m.0 + offset_m.0,
            truth_ecef_m.1 + offset_m.1,
            truth_ecef_m.2 + offset_m.2,
        );
        let solution = sample_position_solution(measured_ecef_m, 2.8, 3.6);
        let smoothed = smoother.smooth_position_solution(100_000.0 + epoch_index as f64, &solution);

        raw_positions_ecef_m.push(measured_ecef_m);
        smoothed_positions_ecef_m
            .push((smoothed.ecef_x_m, smoothed.ecef_y_m, smoothed.ecef_z_m));
        raw_errors_m.push(position_error_3d_m(
            measured_ecef_m.0,
            measured_ecef_m.1,
            measured_ecef_m.2,
            truth_ecef_m,
        ));
        smoothed_errors_m.push(position_error_3d_m(
            smoothed.ecef_x_m,
            smoothed.ecef_y_m,
            smoothed.ecef_z_m,
            truth_ecef_m,
        ));
    }

    let raw_rms_error_m = root_mean_square(&raw_errors_m);
    let smoothed_rms_error_m = root_mean_square(&smoothed_errors_m);
    let raw_path_length_m = path_length_m(&raw_positions_ecef_m);
    let smoothed_path_length_m = path_length_m(&smoothed_positions_ecef_m);

    assert!(smoothed_rms_error_m < raw_rms_error_m * 0.7);
    assert!(smoothed_path_length_m < raw_path_length_m * 0.35);
}

#[test]
fn position_solution_smoother_preserves_linear_motion() {
    let truth_origin_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let truth_velocity_mps = (8.0, -3.0, 1.5);
    let mut config =
        PositionSolutionSmootherConfig::for_motion_class(PositionFilterMotionClass::Vehicle);
    config.measurement_sigma_floor_m = 0.5;
    let mut smoother = PositionSolutionSmoother::new(config);
    let mut raw_errors_m = Vec::new();
    let mut smoothed_errors_m = Vec::new();
    let mut smoothed_positions_ecef_m = Vec::new();
    let mut truth_positions_ecef_m = Vec::new();

    for epoch_index in 0..80 {
        let truth_ecef_m =
            translate_truth_ecef_m(truth_origin_ecef_m, truth_velocity_mps, epoch_index as f64);
        let offset_m = deterministic_position_offset_m(epoch_index);
        let measured_ecef_m = (
            truth_ecef_m.0 + offset_m.0,
            truth_ecef_m.1 + offset_m.1,
            truth_ecef_m.2 + offset_m.2,
        );
        let solution = sample_position_solution(measured_ecef_m, 2.8, 3.6);
        let smoothed = smoother.smooth_position_solution(100_500.0 + epoch_index as f64, &solution);

        truth_positions_ecef_m.push(truth_ecef_m);
        smoothed_positions_ecef_m
            .push((smoothed.ecef_x_m, smoothed.ecef_y_m, smoothed.ecef_z_m));
        raw_errors_m.push(position_error_3d_m(
            measured_ecef_m.0,
            measured_ecef_m.1,
            measured_ecef_m.2,
            truth_ecef_m,
        ));
        smoothed_errors_m.push(position_error_3d_m(
            smoothed.ecef_x_m,
            smoothed.ecef_y_m,
            smoothed.ecef_z_m,
            truth_ecef_m,
        ));
    }

    let truth_path_length_m = path_length_m(&truth_positions_ecef_m);
    let smoothed_path_length_m = path_length_m(&smoothed_positions_ecef_m);
    let smoothed_final = smoothed_positions_ecef_m.last().expect("smoothed final position");
    let truth_final = truth_positions_ecef_m.last().expect("truth final position");
    let final_position_error_m =
        position_error_3d_m(smoothed_final.0, smoothed_final.1, smoothed_final.2, *truth_final);

    assert!(root_mean_square(&smoothed_errors_m) <= root_mean_square(&raw_errors_m));
    assert!(smoothed_path_length_m > truth_path_length_m * 0.9);
    assert!(smoothed_path_length_m < truth_path_length_m * 1.15);
    assert!(final_position_error_m < 6.0, "final_position_error_m={final_position_error_m}");
}
