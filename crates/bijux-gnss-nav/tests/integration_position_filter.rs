#![allow(missing_docs)]
mod support;

use bijux_gnss_nav::api::{geodetic_to_ecef, GpsEphemeris, PositionFilter, PositionFilterConfig};

use support::position_truth::{
    gps_l1ca_doppler_from_truth, pseudorange_from_truth, receiver_clock_bias_with_drift_s,
    sample_ephemeris, timed_position_observation, timed_position_observation_with_doppler,
};

struct SequentialPositionEpoch {
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    truth_clock_bias_s: f64,
    truth_clock_drift_s_per_s: f64,
    observations: Vec<bijux_gnss_nav::api::PositionObservation>,
}

fn velocity_error_3d_mps(
    velocity_x_mps: f64,
    velocity_y_mps: f64,
    velocity_z_mps: f64,
    truth_velocity_mps: (f64, f64, f64),
) -> f64 {
    let dx = velocity_x_mps - truth_velocity_mps.0;
    let dy = velocity_y_mps - truth_velocity_mps.1;
    let dz = velocity_z_mps - truth_velocity_mps.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
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

fn clock_drift_spread_s_per_s(clock_drift_samples_s_per_s: &[f64]) -> f64 {
    let min_clock_drift_s_per_s =
        clock_drift_samples_s_per_s.iter().copied().fold(f64::INFINITY, f64::min);
    let max_clock_drift_s_per_s =
        clock_drift_samples_s_per_s.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    max_clock_drift_s_per_s - min_clock_drift_s_per_s
}

fn root_mean_square(values: &[f64]) -> f64 {
    (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
}

fn displacement_error_3d_m(
    previous_ecef_m: (f64, f64, f64),
    current_ecef_m: (f64, f64, f64),
    truth_velocity_mps: (f64, f64, f64),
    dt_s: f64,
) -> f64 {
    let dx = current_ecef_m.0 - previous_ecef_m.0 - truth_velocity_mps.0 * dt_s;
    let dy = current_ecef_m.1 - previous_ecef_m.1 - truth_velocity_mps.1 * dt_s;
    let dz = current_ecef_m.2 - previous_ecef_m.2 - truth_velocity_mps.2 * dt_s;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn sample_filter_ephemerides() -> Vec<GpsEphemeris> {
    let mut ephemerides = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
        sample_ephemeris(5, 3.2, 3.4),
        sample_ephemeris(6, 4.0, 4.2),
        sample_ephemeris(7, 4.8, 5.1),
    ];
    for ephemeris in &mut ephemerides {
        ephemeris.toe_s = 504_000.0;
        ephemeris.toc_s = 504_018.0;
    }
    ephemerides
}

fn deterministic_pseudorange_offset_m(epoch_index: usize, prn: u8) -> f64 {
    let pattern = ((epoch_index * 17 + prn as usize * 13) % 7) as f64 - 3.0;
    pattern * 0.35
}

fn deterministic_doppler_offset_hz(epoch_index: usize, prn: u8) -> f64 {
    let pattern = ((epoch_index * 11 + prn as usize * 19) % 9) as f64 - 4.0;
    pattern * 0.0
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

fn enu_velocity_to_ecef_mps(
    lat_deg: f64,
    lon_deg: f64,
    velocity_enu_mps: (f64, f64, f64),
) -> (f64, f64, f64) {
    let lat_rad = lat_deg.to_radians();
    let lon_rad = lon_deg.to_radians();
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    let east_mps = velocity_enu_mps.0;
    let north_mps = velocity_enu_mps.1;
    let up_mps = velocity_enu_mps.2;

    (
        -sin_lon * east_mps - sin_lat * cos_lon * north_mps + cos_lat * cos_lon * up_mps,
        cos_lon * east_mps - sin_lat * sin_lon * north_mps + cos_lat * sin_lon * up_mps,
        cos_lat * north_mps + sin_lat * up_mps,
    )
}

fn ecef_velocity_to_enu_mps(
    lat_deg: f64,
    lon_deg: f64,
    velocity_ecef_mps: (f64, f64, f64),
) -> (f64, f64, f64) {
    let lat_rad = lat_deg.to_radians();
    let lon_rad = lon_deg.to_radians();
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    let vx_mps = velocity_ecef_mps.0;
    let vy_mps = velocity_ecef_mps.1;
    let vz_mps = velocity_ecef_mps.2;

    (
        -sin_lon * vx_mps + cos_lon * vy_mps,
        -sin_lat * cos_lon * vx_mps - sin_lat * sin_lon * vy_mps + cos_lat * vz_mps,
        cos_lat * cos_lon * vx_mps + cos_lat * sin_lon * vy_mps + sin_lat * vz_mps,
    )
}

fn static_receiver_epochs(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
) -> Vec<SequentialPositionEpoch> {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + receiver_clock_bias_s;

    (0..epoch_count)
        .map(|epoch_index| {
            let t_rx_s = t0_rx_s + epoch_index as f64 * dt_s;
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m = pseudorange_from_truth(
                        ephemeris,
                        truth_ecef_m,
                        t_rx_s,
                        receiver_clock_bias_s,
                    );
                    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
                })
                .collect();
            SequentialPositionEpoch {
                t_rx_s,
                truth_ecef_m,
                truth_clock_bias_s: receiver_clock_bias_s,
                truth_clock_drift_s_per_s: 0.0,
                observations,
            }
        })
        .collect()
}

fn static_receiver_epochs_with_pseudorange_offsets(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
) -> Vec<SequentialPositionEpoch> {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + receiver_clock_bias_s;

    (0..epoch_count)
        .map(|epoch_index| {
            let t_rx_s = t0_rx_s + epoch_index as f64 * dt_s;
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        ) + deterministic_pseudorange_offset_m(epoch_index, ephemeris.sat.prn);
                    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
                })
                .collect();
            SequentialPositionEpoch {
                t_rx_s,
                truth_ecef_m,
                truth_clock_bias_s: receiver_clock_bias_s,
                truth_clock_drift_s_per_s: 0.0,
                observations,
            }
        })
        .collect()
}

fn moving_receiver_epochs(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
    truth_velocity_mps: (f64, f64, f64),
) -> Vec<SequentialPositionEpoch> {
    let truth_origin_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + receiver_clock_bias_s;

    (0..epoch_count)
        .map(|epoch_index| {
            let motion_dt_s = epoch_index as f64 * dt_s;
            let t_rx_s = t0_rx_s + motion_dt_s;
            let truth_ecef_m =
                translate_truth_ecef_m(truth_origin_ecef_m, truth_velocity_mps, motion_dt_s);
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        ) + deterministic_pseudorange_offset_m(epoch_index, ephemeris.sat.prn);
                    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
                })
                .collect();
            SequentialPositionEpoch {
                t_rx_s,
                truth_ecef_m,
                truth_clock_bias_s: receiver_clock_bias_s,
                truth_clock_drift_s_per_s: 0.0,
                observations,
            }
        })
        .collect()
}

fn moving_receiver_epochs_with_doppler(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
    truth_velocity_ecef_mps: (f64, f64, f64),
    receiver_clock_drift_s_per_s: f64,
) -> Vec<SequentialPositionEpoch> {
    let truth_origin_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let initial_receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + initial_receiver_clock_bias_s;

    (0..epoch_count)
        .map(|epoch_index| {
            let motion_dt_s = epoch_index as f64 * dt_s;
            let receiver_clock_bias_s = receiver_clock_bias_with_drift_s(
                initial_receiver_clock_bias_s,
                receiver_clock_drift_s_per_s,
                motion_dt_s,
            );
            let t_rx_s =
                t0_rx_s + motion_dt_s + receiver_clock_bias_s - initial_receiver_clock_bias_s;
            let truth_ecef_m =
                translate_truth_ecef_m(truth_origin_ecef_m, truth_velocity_ecef_mps, motion_dt_s);
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        ) + deterministic_pseudorange_offset_m(epoch_index, ephemeris.sat.prn);
                    let doppler_hz =
                        gps_l1ca_doppler_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            truth_velocity_ecef_mps,
                            t_rx_s,
                            receiver_clock_bias_s,
                            receiver_clock_drift_s_per_s,
                        ) + deterministic_doppler_offset_hz(epoch_index, ephemeris.sat.prn);
                    timed_position_observation_with_doppler(
                        ephemeris.sat,
                        pseudorange_m,
                        doppler_hz,
                        0.04,
                        t_rx_s,
                    )
                })
                .collect();
            SequentialPositionEpoch {
                t_rx_s,
                truth_ecef_m,
                truth_clock_bias_s: receiver_clock_bias_s,
                truth_clock_drift_s_per_s: receiver_clock_drift_s_per_s,
                observations,
            }
        })
        .collect()
}

fn piecewise_moving_receiver_epochs_with_doppler(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
    first_velocity_ecef_mps: (f64, f64, f64),
    second_velocity_ecef_mps: (f64, f64, f64),
    switch_epoch_index: usize,
    receiver_clock_drift_s_per_s: f64,
) -> Vec<SequentialPositionEpoch> {
    let truth_origin_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let initial_receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + initial_receiver_clock_bias_s;
    let mut truth_ecef_m = truth_origin_ecef_m;

    (0..epoch_count)
        .map(|epoch_index| {
            if epoch_index > 0 {
                let previous_velocity_ecef_mps = if epoch_index - 1 < switch_epoch_index {
                    first_velocity_ecef_mps
                } else {
                    second_velocity_ecef_mps
                };
                truth_ecef_m =
                    translate_truth_ecef_m(truth_ecef_m, previous_velocity_ecef_mps, dt_s);
            }

            let motion_dt_s = epoch_index as f64 * dt_s;
            let receiver_clock_bias_s = receiver_clock_bias_with_drift_s(
                initial_receiver_clock_bias_s,
                receiver_clock_drift_s_per_s,
                motion_dt_s,
            );
            let t_rx_s =
                t0_rx_s + motion_dt_s + receiver_clock_bias_s - initial_receiver_clock_bias_s;
            let truth_velocity_ecef_mps = if epoch_index < switch_epoch_index {
                first_velocity_ecef_mps
            } else {
                second_velocity_ecef_mps
            };
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        ) + deterministic_pseudorange_offset_m(epoch_index, ephemeris.sat.prn);
                    let doppler_hz =
                        gps_l1ca_doppler_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            truth_velocity_ecef_mps,
                            t_rx_s,
                            receiver_clock_bias_s,
                            receiver_clock_drift_s_per_s,
                        ) + deterministic_doppler_offset_hz(epoch_index, ephemeris.sat.prn);
                    timed_position_observation_with_doppler(
                        ephemeris.sat,
                        pseudorange_m,
                        doppler_hz,
                        0.04,
                        t_rx_s,
                    )
                })
                .collect();
            SequentialPositionEpoch {
                t_rx_s,
                truth_ecef_m,
                truth_clock_bias_s: receiver_clock_bias_s,
                truth_clock_drift_s_per_s: receiver_clock_drift_s_per_s,
                observations,
            }
        })
        .collect()
}

#[test]
fn sequential_position_filter_tracks_static_receiver_across_epochs() {
    let ephemerides = sample_filter_ephemerides();
    let epochs = static_receiver_epochs(&ephemerides, 6, 1.0);
    let mut filter = PositionFilter::new(PositionFilterConfig::default());
    let mut last_velocity_norm_mps = None;

    for epoch in &epochs {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("static epoch should solve");
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            epoch.truth_ecef_m,
        );
        let velocity_norm_mps = (solution.velocity_x_mps * solution.velocity_x_mps
            + solution.velocity_y_mps * solution.velocity_y_mps
            + solution.velocity_z_mps * solution.velocity_z_mps)
            .sqrt();

        assert!(position_error_m < 12.0, "position_error_m={position_error_m}");
        assert!(solution.used_sat_count >= 4);
        let covariance = solution
            .position_covariance_ecef_m2
            .expect("position filter should emit position covariance");
        for row in covariance {
            for value in row {
                assert!(value.is_finite());
            }
        }
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
        assert!(solution
            .horizontal_error_ellipse_azimuth_deg
            .expect("ellipse azimuth")
            .is_finite());
        last_velocity_norm_mps = Some(velocity_norm_mps);
    }

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("static epochs").t_rx_s));
    assert!(last_velocity_norm_mps.expect("final velocity norm") < 1.0);
}

#[test]
fn sequential_position_filter_tracks_moving_receiver_across_epochs() {
    let ephemerides = sample_filter_ephemerides();
    let truth_velocity_mps = (12.0, -4.0, 1.5);
    let epochs = moving_receiver_epochs(&ephemerides, 10, 1.0, truth_velocity_mps);
    let mut filter = PositionFilter::new(PositionFilterConfig::for_vehicle_receiver());
    let mut final_velocity_error_mps = None;
    let mut final_velocity_norm_mps = None;

    for epoch in &epochs {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("moving epoch should solve");
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            epoch.truth_ecef_m,
        );
        let velocity_error_mps = velocity_error_3d_mps(
            solution.velocity_x_mps,
            solution.velocity_y_mps,
            solution.velocity_z_mps,
            truth_velocity_mps,
        );
        let velocity_norm_mps = (solution.velocity_x_mps * solution.velocity_x_mps
            + solution.velocity_y_mps * solution.velocity_y_mps
            + solution.velocity_z_mps * solution.velocity_z_mps)
            .sqrt();

        assert!(position_error_m < 25.0, "position_error_m={position_error_m}");
        assert!(solution.used_sat_count >= 4);
        final_velocity_error_mps = Some(velocity_error_mps);
        final_velocity_norm_mps = Some(velocity_norm_mps);
    }

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("moving epochs").t_rx_s));
    assert!(final_velocity_norm_mps.expect("final velocity norm") > 3.0);
    assert!(final_velocity_error_mps.expect("final velocity error") < 6.0);
}

#[test]
fn sequential_position_filter_recovers_velocity_from_doppler_in_enu() {
    let ephemerides = sample_filter_ephemerides();
    let truth_velocity_enu_mps = (8.0, -2.5, 0.8);
    let truth_velocity_ecef_mps = enu_velocity_to_ecef_mps(37.0, -122.0, truth_velocity_enu_mps);
    let epochs =
        moving_receiver_epochs_with_doppler(&ephemerides, 30, 1.0, truth_velocity_ecef_mps, 0.0);
    let mut config = PositionFilterConfig::for_vehicle_receiver();
    config.base_pseudorange_sigma_m = 1.5;
    config.base_doppler_sigma_hz = 0.05;
    let mut filter = PositionFilter::new(config);
    let mut final_velocity_enu_mps = None;

    for epoch in &epochs {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("Doppler-supported moving epoch should solve");
        final_velocity_enu_mps = Some(ecef_velocity_to_enu_mps(
            37.0,
            -122.0,
            (solution.velocity_x_mps, solution.velocity_y_mps, solution.velocity_z_mps),
        ));
    }

    let final_velocity_enu_mps = final_velocity_enu_mps.expect("final ENU velocity");
    let east_error_mps = final_velocity_enu_mps.0 - truth_velocity_enu_mps.0;
    let north_error_mps = final_velocity_enu_mps.1 - truth_velocity_enu_mps.1;
    let up_error_mps = final_velocity_enu_mps.2 - truth_velocity_enu_mps.2;
    let velocity_enu_error_mps = (east_error_mps * east_error_mps
        + north_error_mps * north_error_mps
        + up_error_mps * up_error_mps)
        .sqrt();

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("Doppler epochs").t_rx_s));
    assert!(east_error_mps.abs() < 1.0, "east_error_mps={east_error_mps}");
    assert!(north_error_mps.abs() < 1.0, "north_error_mps={north_error_mps}");
    assert!(up_error_mps.abs() < 1.0, "up_error_mps={up_error_mps}");
    assert!(velocity_enu_error_mps < 1.25, "velocity_enu_error_mps={velocity_enu_error_mps}");
}

#[test]
fn sequential_position_filter_recovers_static_receiver_clock_drift_from_doppler() {
    let ephemerides = sample_filter_ephemerides();
    let truth_clock_drift_s_per_s = 5.0e-8;
    let epochs = moving_receiver_epochs_with_doppler(
        &ephemerides,
        40,
        1.0,
        (0.0, 0.0, 0.0),
        truth_clock_drift_s_per_s,
    );
    let mut config = PositionFilterConfig::default();
    config.base_pseudorange_sigma_m = 1.5;
    config.base_doppler_sigma_hz = 0.05;
    config.initial_velocity_sigma_mps = 5.0;
    config.initial_clock_drift_sigma_s_per_s = 1.0e-4;
    config.process_noise.vel_mps = 0.05;
    config.process_noise.clock_drift_s_per_s = 1.0e-10;
    let mut filter = PositionFilter::new(config);
    let mut steady_clock_drifts_s_per_s = Vec::new();
    let mut final_solution = None;

    for (epoch_index, epoch) in epochs.iter().enumerate() {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("static Doppler clock-drift epoch should solve");
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            epoch.truth_ecef_m,
        );

        assert!(position_error_m < 15.0, "position_error_m={position_error_m}");

        if epoch_index >= 10 {
            steady_clock_drifts_s_per_s.push(solution.clock_drift_s_per_s);
        }

        final_solution = Some(solution);
    }

    let final_solution = final_solution.expect("final static Doppler clock-drift solution");
    let final_clock_drift_error_s_per_s = (final_solution.clock_drift_s_per_s
        - epochs.last().expect("clock-drift epoch").truth_clock_drift_s_per_s)
        .abs();
    let final_clock_bias_error_s = (final_solution.clock_bias_s
        - epochs.last().expect("clock-drift epoch").truth_clock_bias_s)
        .abs();
    let steady_clock_drift_spread_s_per_s =
        clock_drift_spread_s_per_s(&steady_clock_drifts_s_per_s);

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("clock-drift epochs").t_rx_s));
    assert!(
        final_clock_drift_error_s_per_s < 1.0e-8,
        "final_clock_drift_error_s_per_s={final_clock_drift_error_s_per_s}"
    );
    assert!(
        final_clock_bias_error_s < 1.0e-7,
        "final_clock_bias_error_s={final_clock_bias_error_s}"
    );
    assert!(
        steady_clock_drift_spread_s_per_s < 1.0e-8,
        "steady_clock_drift_spread_s_per_s={steady_clock_drift_spread_s_per_s}"
    );
    assert!(
        final_solution.clock_drift_sigma_s_per_s < 5.0e-6,
        "clock_drift_sigma_s_per_s={}",
        final_solution.clock_drift_sigma_s_per_s
    );
}

#[test]
fn sequential_position_filter_maintains_clock_drift_consistency_while_moving() {
    let ephemerides = sample_filter_ephemerides();
    let truth_velocity_enu_mps = (8.0, -2.5, 0.8);
    let truth_velocity_ecef_mps = enu_velocity_to_ecef_mps(37.0, -122.0, truth_velocity_enu_mps);
    let truth_clock_drift_s_per_s = 5.0e-8;
    let epochs = moving_receiver_epochs_with_doppler(
        &ephemerides,
        40,
        1.0,
        truth_velocity_ecef_mps,
        truth_clock_drift_s_per_s,
    );
    let mut config = PositionFilterConfig::for_vehicle_receiver();
    config.base_pseudorange_sigma_m = 1.5;
    config.base_doppler_sigma_hz = 0.05;
    config.initial_clock_drift_sigma_s_per_s = 1.0e-4;
    config.process_noise.clock_drift_s_per_s = 1.0e-10;
    let mut filter = PositionFilter::new(config);
    let mut steady_clock_drifts_s_per_s = Vec::new();
    let mut final_solution = None;
    let mut final_velocity_enu_mps = None;

    for (epoch_index, epoch) in epochs.iter().enumerate() {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("moving Doppler clock-drift epoch should solve");
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            epoch.truth_ecef_m,
        );

        assert!(position_error_m < 30.0, "position_error_m={position_error_m}");

        if epoch_index >= 10 {
            steady_clock_drifts_s_per_s.push(solution.clock_drift_s_per_s);
        }

        final_velocity_enu_mps = Some(ecef_velocity_to_enu_mps(
            37.0,
            -122.0,
            (solution.velocity_x_mps, solution.velocity_y_mps, solution.velocity_z_mps),
        ));
        final_solution = Some(solution);
    }

    let final_solution = final_solution.expect("final moving Doppler clock-drift solution");
    let final_velocity_enu_mps = final_velocity_enu_mps.expect("final moving ENU velocity");
    let east_error_mps = final_velocity_enu_mps.0 - truth_velocity_enu_mps.0;
    let north_error_mps = final_velocity_enu_mps.1 - truth_velocity_enu_mps.1;
    let up_error_mps = final_velocity_enu_mps.2 - truth_velocity_enu_mps.2;
    let velocity_enu_error_mps = (east_error_mps * east_error_mps
        + north_error_mps * north_error_mps
        + up_error_mps * up_error_mps)
        .sqrt();
    let final_clock_drift_error_s_per_s = (final_solution.clock_drift_s_per_s
        - epochs.last().expect("clock-drift epoch").truth_clock_drift_s_per_s)
        .abs();
    let final_clock_bias_error_s = (final_solution.clock_bias_s
        - epochs.last().expect("clock-drift epoch").truth_clock_bias_s)
        .abs();
    let steady_clock_drift_spread_s_per_s =
        clock_drift_spread_s_per_s(&steady_clock_drifts_s_per_s);

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("clock-drift epochs").t_rx_s));
    assert!(east_error_mps.abs() < 1.0, "east_error_mps={east_error_mps}");
    assert!(north_error_mps.abs() < 1.0, "north_error_mps={north_error_mps}");
    assert!(up_error_mps.abs() < 1.0, "up_error_mps={up_error_mps}");
    assert!(velocity_enu_error_mps < 1.25, "velocity_enu_error_mps={velocity_enu_error_mps}");
    assert!(
        final_clock_drift_error_s_per_s < 1.0e-8,
        "final_clock_drift_error_s_per_s={final_clock_drift_error_s_per_s}"
    );
    assert!(
        final_clock_bias_error_s < 1.0e-7,
        "final_clock_bias_error_s={final_clock_bias_error_s}"
    );
    assert!(
        steady_clock_drift_spread_s_per_s < 1.0e-8,
        "steady_clock_drift_spread_s_per_s={steady_clock_drift_spread_s_per_s}"
    );
    assert!(
        final_solution.clock_drift_sigma_s_per_s < 5.0e-6,
        "clock_drift_sigma_s_per_s={}",
        final_solution.clock_drift_sigma_s_per_s
    );
}

#[test]
fn sequential_position_filter_motion_classes_change_velocity_adaptation() {
    let ephemerides = sample_filter_ephemerides();
    let first_velocity_enu_mps = (2.0, 0.5, 0.0);
    let second_velocity_enu_mps = (28.0, -8.0, 2.5);
    let first_velocity_ecef_mps = enu_velocity_to_ecef_mps(37.0, -122.0, first_velocity_enu_mps);
    let second_velocity_ecef_mps = enu_velocity_to_ecef_mps(37.0, -122.0, second_velocity_enu_mps);
    let switch_epoch_index = 8;
    let epochs = piecewise_moving_receiver_epochs_with_doppler(
        &ephemerides,
        24,
        1.0,
        first_velocity_ecef_mps,
        second_velocity_ecef_mps,
        switch_epoch_index,
        0.0,
    );
    let mut static_config = PositionFilterConfig::for_static_receiver();
    let mut pedestrian_config = PositionFilterConfig::for_pedestrian_receiver();
    let mut vehicle_config = PositionFilterConfig::for_vehicle_receiver();
    let mut airborne_config = PositionFilterConfig::for_airborne_receiver();

    for config in
        [&mut static_config, &mut pedestrian_config, &mut vehicle_config, &mut airborne_config]
    {
        config.base_pseudorange_sigma_m = 2.0;
        config.base_doppler_sigma_hz = 0.2;
    }

    let mut static_filter = PositionFilter::new(static_config);
    let mut pedestrian_filter = PositionFilter::new(pedestrian_config);
    let mut vehicle_filter = PositionFilter::new(vehicle_config);
    let mut airborne_filter = PositionFilter::new(airborne_config);
    let mut post_switch_velocity_errors_mps = None;

    for (epoch_index, epoch) in epochs.iter().enumerate() {
        let static_solution = static_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("static motion-class epoch should solve");
        let pedestrian_solution = pedestrian_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("pedestrian motion-class epoch should solve");
        let vehicle_solution = vehicle_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("vehicle motion-class epoch should solve");
        let airborne_solution = airborne_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("airborne motion-class epoch should solve");

        if epoch_index == switch_epoch_index + 2 {
            post_switch_velocity_errors_mps = Some([
                velocity_error_3d_mps(
                    static_solution.velocity_x_mps,
                    static_solution.velocity_y_mps,
                    static_solution.velocity_z_mps,
                    second_velocity_ecef_mps,
                ),
                velocity_error_3d_mps(
                    pedestrian_solution.velocity_x_mps,
                    pedestrian_solution.velocity_y_mps,
                    pedestrian_solution.velocity_z_mps,
                    second_velocity_ecef_mps,
                ),
                velocity_error_3d_mps(
                    vehicle_solution.velocity_x_mps,
                    vehicle_solution.velocity_y_mps,
                    vehicle_solution.velocity_z_mps,
                    second_velocity_ecef_mps,
                ),
                velocity_error_3d_mps(
                    airborne_solution.velocity_x_mps,
                    airborne_solution.velocity_y_mps,
                    airborne_solution.velocity_z_mps,
                    second_velocity_ecef_mps,
                ),
            ]);
        }
    }

    let post_switch_velocity_errors_mps =
        post_switch_velocity_errors_mps.expect("post-switch velocity errors");

    assert!(static_filter.initialized);
    assert!(pedestrian_filter.initialized);
    assert!(vehicle_filter.initialized);
    assert!(airborne_filter.initialized);
    assert_eq!(static_filter.last_t_rx_s, Some(epochs.last().expect("motion-class epochs").t_rx_s));
    assert_eq!(
        pedestrian_filter.last_t_rx_s,
        Some(epochs.last().expect("motion-class epochs").t_rx_s)
    );
    assert_eq!(
        vehicle_filter.last_t_rx_s,
        Some(epochs.last().expect("motion-class epochs").t_rx_s)
    );
    assert_eq!(
        airborne_filter.last_t_rx_s,
        Some(epochs.last().expect("motion-class epochs").t_rx_s)
    );
    assert!(
        post_switch_velocity_errors_mps[0] > post_switch_velocity_errors_mps[1],
        "static_error_mps={} pedestrian_error_mps={}",
        post_switch_velocity_errors_mps[0],
        post_switch_velocity_errors_mps[1]
    );
    assert!(
        post_switch_velocity_errors_mps[1] > post_switch_velocity_errors_mps[2],
        "pedestrian_error_mps={} vehicle_error_mps={}",
        post_switch_velocity_errors_mps[1],
        post_switch_velocity_errors_mps[2]
    );
    assert!(
        post_switch_velocity_errors_mps[2] > post_switch_velocity_errors_mps[3],
        "vehicle_error_mps={} airborne_error_mps={}",
        post_switch_velocity_errors_mps[2],
        post_switch_velocity_errors_mps[3]
    );
}

#[test]
fn sequential_position_filter_static_profile_reduces_late_position_scatter() {
    let ephemerides = sample_filter_ephemerides();
    let epochs = static_receiver_epochs_with_pseudorange_offsets(&ephemerides, 30, 1.0);
    let mut static_config = PositionFilterConfig::for_static_receiver();
    static_config.base_pseudorange_sigma_m = 1.5;
    let mut static_filter = PositionFilter::new(static_config);
    let mut static_errors_m = Vec::new();
    let mut final_static_velocity_norm_mps = None;

    for epoch in &epochs {
        let static_solution = static_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("static-profile epoch should solve");
        let static_error_m = position_error_3d_m(
            static_solution.ecef_x_m,
            static_solution.ecef_y_m,
            static_solution.ecef_z_m,
            epoch.truth_ecef_m,
        );

        static_errors_m.push(static_error_m);
        final_static_velocity_norm_mps = Some(
            (static_solution.velocity_x_mps * static_solution.velocity_x_mps
                + static_solution.velocity_y_mps * static_solution.velocity_y_mps
                + static_solution.velocity_z_mps * static_solution.velocity_z_mps)
                .sqrt(),
        );
    }

    let static_early_rms_m = root_mean_square(&static_errors_m[1..10]);
    let static_late_rms_m = root_mean_square(&static_errors_m[20..30]);

    assert!(static_filter.initialized);
    assert_eq!(
        static_filter.last_t_rx_s,
        Some(epochs.last().expect("static profile epochs").t_rx_s)
    );
    assert!(
        static_late_rms_m < static_early_rms_m,
        "static_early_rms_m={static_early_rms_m} static_late_rms_m={static_late_rms_m}"
    );
    assert!(static_late_rms_m < 0.35, "static_late_rms_m={static_late_rms_m}");
    assert!(
        final_static_velocity_norm_mps.expect("final static velocity norm") < 0.5,
        "final_static_velocity_norm_mps={:?}",
        final_static_velocity_norm_mps
    );
}

#[test]
fn sequential_position_filter_constant_velocity_profile_beats_independent_epochs_while_moving() {
    let ephemerides = sample_filter_ephemerides();
    let truth_velocity_enu_mps = (8.0, -2.5, 0.8);
    let truth_velocity_mps = enu_velocity_to_ecef_mps(37.0, -122.0, truth_velocity_enu_mps);
    let epochs =
        moving_receiver_epochs_with_doppler(&ephemerides, 20, 1.0, truth_velocity_mps, 0.0);
    let mut dynamic_config = PositionFilterConfig::for_vehicle_receiver();
    dynamic_config.base_pseudorange_sigma_m = 1.5;
    dynamic_config.base_doppler_sigma_hz = 0.05;
    let mut dynamic_filter = PositionFilter::new(dynamic_config.clone());
    let mut dynamic_positions_ecef_m = Vec::new();
    let mut independent_positions_ecef_m = Vec::new();
    let mut final_dynamic_velocity_error_mps = None;
    let mut final_independent_velocity_error_mps = None;

    for epoch in &epochs {
        let dynamic_solution = dynamic_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("dynamic moving epoch should solve");
        let mut independent_filter = PositionFilter::new(dynamic_config.clone());
        let independent_solution = independent_filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("independent moving epoch should solve");

        dynamic_positions_ecef_m.push((
            dynamic_solution.ecef_x_m,
            dynamic_solution.ecef_y_m,
            dynamic_solution.ecef_z_m,
        ));
        independent_positions_ecef_m.push((
            independent_solution.ecef_x_m,
            independent_solution.ecef_y_m,
            independent_solution.ecef_z_m,
        ));
        final_dynamic_velocity_error_mps = Some(velocity_error_3d_mps(
            dynamic_solution.velocity_x_mps,
            dynamic_solution.velocity_y_mps,
            dynamic_solution.velocity_z_mps,
            truth_velocity_mps,
        ));
        final_independent_velocity_error_mps = Some(velocity_error_3d_mps(
            independent_solution.velocity_x_mps,
            independent_solution.velocity_y_mps,
            independent_solution.velocity_z_mps,
            truth_velocity_mps,
        ));
    }

    let dynamic_step_errors_m = dynamic_positions_ecef_m
        .windows(2)
        .map(|window| displacement_error_3d_m(window[0], window[1], truth_velocity_mps, 1.0))
        .collect::<Vec<_>>();
    let independent_step_errors_m = independent_positions_ecef_m
        .windows(2)
        .map(|window| displacement_error_3d_m(window[0], window[1], truth_velocity_mps, 1.0))
        .collect::<Vec<_>>();
    let dynamic_late_step_rms_m = root_mean_square(&dynamic_step_errors_m[9..19]);
    let independent_late_step_rms_m = root_mean_square(&independent_step_errors_m[9..19]);
    let final_dynamic_velocity_error_mps =
        final_dynamic_velocity_error_mps.expect("final dynamic velocity error");
    let final_independent_velocity_error_mps =
        final_independent_velocity_error_mps.expect("final independent velocity error");

    assert!(dynamic_filter.initialized);
    assert_eq!(dynamic_filter.last_t_rx_s, Some(epochs.last().expect("moving epochs").t_rx_s));
    assert!(
        dynamic_late_step_rms_m < independent_late_step_rms_m,
        "dynamic_late_step_rms_m={dynamic_late_step_rms_m} independent_late_step_rms_m={independent_late_step_rms_m}"
    );
    assert!(
        final_dynamic_velocity_error_mps < final_independent_velocity_error_mps,
        "final_dynamic_velocity_error_mps={final_dynamic_velocity_error_mps} final_independent_velocity_error_mps={final_independent_velocity_error_mps}"
    );
    assert!(
        final_dynamic_velocity_error_mps < 2.0,
        "final_dynamic_velocity_error_mps={final_dynamic_velocity_error_mps}"
    );
}
