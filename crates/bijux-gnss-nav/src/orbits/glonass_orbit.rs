use super::glonass::GlonassStateVector;

const GLONASS_MU_M3PS2: f64 = 398_600.44e9;
const GLONASS_EARTH_RADIUS_M: f64 = 6_378_136.0;
const GLONASS_C20: f64 = -1_082.63e-6;
pub(super) const GLONASS_EARTH_ROTATION_RATE_RAD_S: f64 = 7.292_115e-5;
pub(super) const GLONASS_DEFAULT_INTEGRATION_STEP_S: f64 = 60.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GlonassNumericalState {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
}

pub(super) fn propagate_glonass_orbit_fixed_step(
    state_vector: GlonassStateVector,
    delta_t_s: f64,
) -> GlonassNumericalState {
    propagate_glonass_orbit_with_step(state_vector, delta_t_s, GLONASS_DEFAULT_INTEGRATION_STEP_S)
}

fn propagate_glonass_orbit_with_step(
    state_vector: GlonassStateVector,
    delta_t_s: f64,
    integration_step_s: f64,
) -> GlonassNumericalState {
    let mut state = glonass_inertial_initial_state(state_vector);
    if delta_t_s.abs() <= f64::EPSILON {
        return glonass_ecef_state_from_inertial(state, delta_t_s);
    }

    let luni_solar_acceleration_mps2 =
        [state_vector.ax_mps2, state_vector.ay_mps2, state_vector.az_mps2];
    let mut remaining_s = delta_t_s;
    while remaining_s.abs() > f64::EPSILON {
        let step_s = remaining_s.signum() * remaining_s.abs().min(integration_step_s);
        state = rk4_step(state, step_s, luni_solar_acceleration_mps2, glonass_inertial_dynamics);
        remaining_s -= step_s;
    }
    glonass_ecef_state_from_inertial(state, delta_t_s)
}

fn rk4_step(
    state: [f64; 6],
    step_s: f64,
    luni_solar_acceleration_mps2: [f64; 3],
    dynamics: fn([f64; 6], [f64; 3]) -> [f64; 6],
) -> [f64; 6] {
    let k1 = dynamics(state, luni_solar_acceleration_mps2);
    let k2 = dynamics(add_scaled_state(state, k1, step_s * 0.5), luni_solar_acceleration_mps2);
    let k3 = dynamics(add_scaled_state(state, k2, step_s * 0.5), luni_solar_acceleration_mps2);
    let k4 = dynamics(add_scaled_state(state, k3, step_s), luni_solar_acceleration_mps2);

    let mut next_state = state;
    for index in 0..next_state.len() {
        next_state[index] +=
            step_s / 6.0 * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]);
    }
    next_state
}

fn add_scaled_state(state: [f64; 6], derivative: [f64; 6], scale: f64) -> [f64; 6] {
    let mut next = state;
    for index in 0..next.len() {
        next[index] += derivative[index] * scale;
    }
    next
}

fn glonass_inertial_initial_state(state_vector: GlonassStateVector) -> [f64; 6] {
    [
        state_vector.x_m,
        state_vector.y_m,
        state_vector.z_m,
        state_vector.vx_mps - GLONASS_EARTH_ROTATION_RATE_RAD_S * state_vector.y_m,
        state_vector.vy_mps + GLONASS_EARTH_ROTATION_RATE_RAD_S * state_vector.x_m,
        state_vector.vz_mps,
    ]
}

fn glonass_ecef_state_from_inertial(
    inertial_state: [f64; 6],
    delta_t_s: f64,
) -> GlonassNumericalState {
    let rotation_rad = GLONASS_EARTH_ROTATION_RATE_RAD_S * delta_t_s;
    let cos_rotation = rotation_rad.cos();
    let sin_rotation = rotation_rad.sin();
    let x_m = inertial_state[0] * cos_rotation + inertial_state[1] * sin_rotation;
    let y_m = -inertial_state[0] * sin_rotation + inertial_state[1] * cos_rotation;
    let z_m = inertial_state[2];
    let rotated_vx_mps = inertial_state[3] * cos_rotation + inertial_state[4] * sin_rotation;
    let rotated_vy_mps = -inertial_state[3] * sin_rotation + inertial_state[4] * cos_rotation;

    GlonassNumericalState {
        position_m: [x_m, y_m, z_m],
        velocity_mps: [
            rotated_vx_mps + GLONASS_EARTH_ROTATION_RATE_RAD_S * y_m,
            rotated_vy_mps - GLONASS_EARTH_ROTATION_RATE_RAD_S * x_m,
            inertial_state[5],
        ],
    }
}

fn glonass_inertial_dynamics(state: [f64; 6], luni_solar_acceleration_mps2: [f64; 3]) -> [f64; 6] {
    let x_m = state[0];
    let y_m = state[1];
    let z_m = state[2];
    let vx_mps = state[3];
    let vy_mps = state[4];
    let vz_mps = state[5];

    let radius_squared_m2 = x_m * x_m + y_m * y_m + z_m * z_m;
    let radius_m = radius_squared_m2.sqrt();
    let radius_cubed_m3 = radius_squared_m2 * radius_m;
    let radius_fifth_m5 = radius_cubed_m3 * radius_squared_m2;
    let z_ratio_squared = if radius_squared_m2 > 0.0 { z_m * z_m / radius_squared_m2 } else { 0.0 };
    let central_acceleration = -GLONASS_MU_M3PS2 / radius_cubed_m3;
    let j2_acceleration =
        1.5 * GLONASS_C20 * GLONASS_MU_M3PS2 * GLONASS_EARTH_RADIUS_M.powi(2) / radius_fifth_m5;

    let ax_mps2 = central_acceleration * x_m
        + j2_acceleration * x_m * (1.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[0];
    let ay_mps2 = central_acceleration * y_m
        + j2_acceleration * y_m * (1.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[1];
    let az_mps2 = central_acceleration * z_m
        + j2_acceleration * z_m * (3.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[2];

    [vx_mps, vy_mps, vz_mps, ax_mps2, ay_mps2, az_mps2]
}
