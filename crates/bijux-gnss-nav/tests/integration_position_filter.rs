#![allow(missing_docs)]
mod support;

use bijux_gnss_nav::api::{geodetic_to_ecef, GpsEphemeris, PositionFilter, PositionFilterConfig};

use support::position_truth::{
    gps_l1ca_doppler_from_truth, pseudorange_from_truth, receiver_clock_bias_with_drift_s,
    sample_ephemeris,
    timed_position_observation, timed_position_observation_with_doppler,
};

struct SequentialPositionEpoch {
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
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
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        );
                    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
                })
                .collect();
            SequentialPositionEpoch { t_rx_s, truth_ecef_m, observations }
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
            SequentialPositionEpoch { t_rx_s, truth_ecef_m, observations }
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
            let t_rx_s = t0_rx_s + motion_dt_s + receiver_clock_bias_s - initial_receiver_clock_bias_s;
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
            SequentialPositionEpoch { t_rx_s, truth_ecef_m, observations }
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
    let mut filter = PositionFilter::new(PositionFilterConfig::default());
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
    let truth_velocity_ecef_mps =
        enu_velocity_to_ecef_mps(37.0, -122.0, truth_velocity_enu_mps);
    let epochs = moving_receiver_epochs_with_doppler(&ephemerides, 30, 1.0, truth_velocity_ecef_mps, 0.0);
    let mut config = PositionFilterConfig::default();
    config.base_pseudorange_sigma_m = 1.5;
    config.base_doppler_sigma_hz = 0.05;
    config.initial_velocity_sigma_mps = 10.0;
    config.process_noise.vel_mps = 0.2;
    let mut filter = PositionFilter::new(config);
    let mut final_velocity_enu_mps = None;

    for epoch in &epochs {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("Doppler-supported moving epoch should solve");
        final_velocity_enu_mps = Some(ecef_velocity_to_enu_mps(
            37.0,
            -122.0,
            (
                solution.velocity_x_mps,
                solution.velocity_y_mps,
                solution.velocity_z_mps,
            ),
        ));
    }

    let final_velocity_enu_mps = final_velocity_enu_mps.expect("final ENU velocity");
    let east_error_mps = final_velocity_enu_mps.0 - truth_velocity_enu_mps.0;
    let north_error_mps = final_velocity_enu_mps.1 - truth_velocity_enu_mps.1;
    let up_error_mps = final_velocity_enu_mps.2 - truth_velocity_enu_mps.2;
    let velocity_enu_error_mps =
        (east_error_mps * east_error_mps + north_error_mps * north_error_mps + up_error_mps * up_error_mps)
            .sqrt();

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("Doppler epochs").t_rx_s));
    assert!(east_error_mps.abs() < 1.0, "east_error_mps={east_error_mps}");
    assert!(north_error_mps.abs() < 1.0, "north_error_mps={north_error_mps}");
    assert!(up_error_mps.abs() < 1.0, "up_error_mps={up_error_mps}");
    assert!(velocity_enu_error_mps < 1.25, "velocity_enu_error_mps={velocity_enu_error_mps}");
}
