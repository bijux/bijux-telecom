use bijux_gnss_nav::api::{GpsEphemeris, PositionObservation};

use crate::reference_models::gps_broadcast::sat_state_gps_l1ca;

use super::SPEED_OF_LIGHT_MPS;

pub fn iterative_pseudorange_residual_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        ephemeris,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        true,
    )
}

pub fn iterative_pseudorange_residual_without_earth_rotation_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
) -> f64 {
    iterative_pseudorange_residual_with_earth_rotation_mode_m(
        ephemeris,
        observation,
        receiver_ecef_m,
        receiver_clock_bias_s,
        t_rx_s,
        false,
    )
}

fn iterative_pseudorange_residual_with_earth_rotation_mode_m(
    ephemeris: &GpsEphemeris,
    observation: &PositionObservation,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    t_rx_s: f64,
    apply_earth_rotation: bool,
) -> f64 {
    let receive_tow_s =
        observation.gps_receive_time.map(|gps_time| gps_time.tow_s).unwrap_or(t_rx_s);
    let mut signal_travel_time_s = observation
        .signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(observation.pseudorange_m / SPEED_OF_LIGHT_MPS);
    let predicted_pseudorange_m = iterative_predicted_pseudorange_m(
        ephemeris,
        receiver_ecef_m,
        receiver_clock_bias_s,
        receive_tow_s,
        &mut signal_travel_time_s,
        apply_earth_rotation,
    );
    observation.pseudorange_m - predicted_pseudorange_m
}

fn iterative_predicted_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receiver_ecef_m: (f64, f64, f64),
    receiver_clock_bias_s: f64,
    receive_tow_s: f64,
    signal_travel_time_s: &mut f64,
    apply_earth_rotation: bool,
) -> f64 {
    let mut predicted_pseudorange_m = 0.0;
    for _ in 0..10 {
        let state = sat_state_gps_l1ca(
            ephemeris,
            receive_tow_s - *signal_travel_time_s,
            if apply_earth_rotation { *signal_travel_time_s } else { 0.0 },
        );
        let dx_m = receiver_ecef_m.0 - state.x_m;
        let dy_m = receiver_ecef_m.1 - state.y_m;
        let dz_m = receiver_ecef_m.2 - state.z_m;
        let range_m = (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt();
        predicted_pseudorange_m = range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
            - state.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_signal_travel_time_s = predicted_pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_signal_travel_time_s - *signal_travel_time_s).abs() < 1.0e-12 {
            break;
        }
        *signal_travel_time_s = next_signal_travel_time_s;
    }
    predicted_pseudorange_m
}
