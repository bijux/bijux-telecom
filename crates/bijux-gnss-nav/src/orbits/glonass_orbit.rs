use super::glonass::GlonassStateVector;

const GLONASS_MU_M3PS2: f64 = 398_600.44e9;
const GLONASS_EARTH_RADIUS_M: f64 = 6_378_136.0;
const GLONASS_C20: f64 = -1_082.63e-6;
pub(super) const GLONASS_EARTH_ROTATION_RATE_RAD_S: f64 = 7.292_115e-5;
pub(super) const GLONASS_DEFAULT_INTEGRATION_STEP_S: f64 = 60.0;
const GLONASS_REFINEMENT_TOLERANCE_M: f64 = 1.0e-3;
const GLONASS_MAX_STEP_REFINEMENTS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GlonassNumericalState {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GlonassPropagationConfig {
    pub base_step_s: f64,
    pub position_tolerance_m: f64,
    pub max_refinements: usize,
}

impl Default for GlonassPropagationConfig {
    fn default() -> Self {
        Self {
            base_step_s: GLONASS_DEFAULT_INTEGRATION_STEP_S,
            position_tolerance_m: GLONASS_REFINEMENT_TOLERANCE_M,
            max_refinements: GLONASS_MAX_STEP_REFINEMENTS,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GlonassPropagationConvergence {
    pub delta_t_s: f64,
    pub base_step_s: f64,
    pub accepted_step_s: f64,
    pub refinements: usize,
    pub position_refinement_m: f64,
    pub position_tolerance_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct GlonassOrbitPropagation {
    pub state: GlonassNumericalState,
    pub convergence: GlonassPropagationConvergence,
}

pub(super) fn propagate_glonass_orbit_refined(
    state_vector: GlonassStateVector,
    delta_t_s: f64,
    config: GlonassPropagationConfig,
) -> Option<GlonassOrbitPropagation> {
    if !is_valid_state_vector(state_vector)
        || !delta_t_s.is_finite()
        || !config.base_step_s.is_finite()
        || config.base_step_s <= 0.0
        || !config.position_tolerance_m.is_finite()
        || config.position_tolerance_m <= 0.0
    {
        return None;
    }

    if delta_t_s.abs() <= f64::EPSILON {
        return Some(GlonassOrbitPropagation {
            state: propagate_glonass_orbit_with_step(state_vector, delta_t_s, config.base_step_s),
            convergence: GlonassPropagationConvergence {
                delta_t_s,
                base_step_s: config.base_step_s,
                accepted_step_s: 0.0,
                refinements: 0,
                position_refinement_m: 0.0,
                position_tolerance_m: config.position_tolerance_m,
            },
        });
    }

    let mut coarse_step_s = config.base_step_s;
    let mut coarse_state =
        propagate_glonass_orbit_with_step(state_vector, delta_t_s, coarse_step_s);
    for refinement_index in 0..=config.max_refinements {
        let fine_step_s = coarse_step_s * 0.5;
        let fine_state = propagate_glonass_orbit_with_step(state_vector, delta_t_s, fine_step_s);
        let position_refinement_m =
            position_difference_m(coarse_state.position_m, fine_state.position_m);
        if position_refinement_m <= config.position_tolerance_m {
            return Some(GlonassOrbitPropagation {
                state: fine_state,
                convergence: GlonassPropagationConvergence {
                    delta_t_s,
                    base_step_s: config.base_step_s,
                    accepted_step_s: fine_step_s,
                    refinements: refinement_index + 1,
                    position_refinement_m,
                    position_tolerance_m: config.position_tolerance_m,
                },
            });
        }

        coarse_step_s = fine_step_s;
        coarse_state = fine_state;
    }

    None
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

fn is_valid_state_vector(state_vector: GlonassStateVector) -> bool {
    [
        state_vector.x_m,
        state_vector.y_m,
        state_vector.z_m,
        state_vector.vx_mps,
        state_vector.vy_mps,
        state_vector.vz_mps,
        state_vector.ax_mps2,
        state_vector.ay_mps2,
        state_vector.az_mps2,
    ]
    .into_iter()
    .all(f64::is_finite)
}

fn position_difference_m(left: [f64; 3], right: [f64; 3]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    let dz = left[2] - right[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
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

#[cfg(test)]
mod tests {
    use super::{
        position_difference_m, propagate_glonass_orbit_refined, propagate_glonass_orbit_with_step,
        GlonassNumericalState, GlonassPropagationConfig, GLONASS_C20, GLONASS_EARTH_RADIUS_M,
        GLONASS_EARTH_ROTATION_RATE_RAD_S, GLONASS_MU_M3PS2,
    };
    use crate::orbits::glonass::GlonassStateVector;

    fn sample_state_vector() -> GlonassStateVector {
        GlonassStateVector {
            x_m: -7_557_760.253_906_25,
            y_m: -23_962_225.585_937_5,
            z_m: -4_337_567.871_093_75,
            vx_mps: 101.318_359_375,
            vy_mps: 602.112_770_080_566_4,
            vz_mps: -3_495.733_261_108_398_4,
            ax_mps2: -3.725_290_298_461_914e-6,
            ay_mps2: 0.0,
            az_mps2: 1.862_645_149_230_957e-6,
        }
    }

    #[test]
    fn glonass_refined_propagation_converges_within_policy() {
        let propagation = propagate_glonass_orbit_refined(
            sample_state_vector(),
            780.0,
            GlonassPropagationConfig::default(),
        )
        .expect("refined GLONASS propagation");

        assert!(propagation.convergence.refinements > 0);
        assert!(
            propagation.convergence.position_refinement_m
                <= propagation.convergence.position_tolerance_m
        );
        assert!(propagation.convergence.accepted_step_s <= 30.0);
        assert!(propagation.state.position_m.iter().all(|component| component.is_finite()));
        assert!(propagation.state.velocity_mps.iter().all(|component| component.is_finite()));
    }

    #[test]
    fn glonass_refinement_improves_fixed_step_solution() {
        let state_vector = sample_state_vector();
        let fixed = propagate_glonass_orbit_with_step(state_vector, 780.0, 60.0);
        let half_step = propagate_glonass_orbit_with_step(state_vector, 780.0, 30.0);
        let refined = propagate_glonass_orbit_refined(
            state_vector,
            780.0,
            GlonassPropagationConfig::default(),
        )
        .expect("refined GLONASS propagation");

        let fixed_to_half_m = position_difference_m(fixed.position_m, half_step.position_m);
        let refined_to_half_m =
            position_difference_m(refined.state.position_m, half_step.position_m);

        assert!(fixed_to_half_m > refined_to_half_m);
    }

    #[test]
    fn glonass_refined_propagation_refuses_unmet_tolerance() {
        let propagation = propagate_glonass_orbit_refined(
            sample_state_vector(),
            780.0,
            GlonassPropagationConfig {
                base_step_s: 600.0,
                position_tolerance_m: 1.0e-15,
                max_refinements: 0,
            },
        );

        assert!(propagation.is_none());
    }

    #[test]
    fn glonass_refined_propagation_matches_independent_reference_vectors() {
        let state_vector = sample_state_vector();

        for delta_t_s in [600.0, -420.0] {
            let propagation = propagate_glonass_orbit_refined(
                state_vector,
                delta_t_s,
                GlonassPropagationConfig::default(),
            )
            .expect("refined GLONASS propagation");
            let reference = independent_reference_propagation(state_vector, delta_t_s);

            assert!(
                position_difference_m(propagation.state.position_m, reference.position_m) < 0.05
            );
            assert!(
                position_difference_m(propagation.state.velocity_mps, reference.velocity_mps)
                    < 5.0e-4
            );
        }
    }

    fn independent_reference_propagation(
        state_vector: GlonassStateVector,
        delta_t_s: f64,
    ) -> GlonassNumericalState {
        let mut position_m = [state_vector.x_m, state_vector.y_m, state_vector.z_m];
        let mut velocity_mps = [
            state_vector.vx_mps - GLONASS_EARTH_ROTATION_RATE_RAD_S * state_vector.y_m,
            state_vector.vy_mps + GLONASS_EARTH_ROTATION_RATE_RAD_S * state_vector.x_m,
            state_vector.vz_mps,
        ];
        let broadcast_acceleration_mps2 =
            [state_vector.ax_mps2, state_vector.ay_mps2, state_vector.az_mps2];
        let mut acceleration_mps2 =
            independent_reference_acceleration(position_m, broadcast_acceleration_mps2);
        let mut remaining_s = delta_t_s;

        while remaining_s.abs() > f64::EPSILON {
            let step_s = remaining_s.signum() * remaining_s.abs().min(0.1);
            let next_position_m = [
                position_m[0]
                    + velocity_mps[0] * step_s
                    + 0.5 * acceleration_mps2[0] * step_s * step_s,
                position_m[1]
                    + velocity_mps[1] * step_s
                    + 0.5 * acceleration_mps2[1] * step_s * step_s,
                position_m[2]
                    + velocity_mps[2] * step_s
                    + 0.5 * acceleration_mps2[2] * step_s * step_s,
            ];
            let next_acceleration_mps2 =
                independent_reference_acceleration(next_position_m, broadcast_acceleration_mps2);
            velocity_mps = [
                velocity_mps[0] + 0.5 * (acceleration_mps2[0] + next_acceleration_mps2[0]) * step_s,
                velocity_mps[1] + 0.5 * (acceleration_mps2[1] + next_acceleration_mps2[1]) * step_s,
                velocity_mps[2] + 0.5 * (acceleration_mps2[2] + next_acceleration_mps2[2]) * step_s,
            ];
            position_m = next_position_m;
            acceleration_mps2 = next_acceleration_mps2;
            remaining_s -= step_s;
        }

        independent_reference_ecef_state(position_m, velocity_mps, delta_t_s)
    }

    fn independent_reference_acceleration(
        position_m: [f64; 3],
        broadcast_acceleration_mps2: [f64; 3],
    ) -> [f64; 3] {
        let radius_squared_m2 = position_m[0] * position_m[0]
            + position_m[1] * position_m[1]
            + position_m[2] * position_m[2];
        let radius_m = radius_squared_m2.sqrt();
        let radius_cubed_m3 = radius_squared_m2 * radius_m;
        let radius_fifth_m5 = radius_cubed_m3 * radius_squared_m2;
        let z_ratio_squared = position_m[2] * position_m[2] / radius_squared_m2;
        let central_acceleration = -GLONASS_MU_M3PS2 / radius_cubed_m3;
        let j2_acceleration =
            1.5 * GLONASS_C20 * GLONASS_MU_M3PS2 * GLONASS_EARTH_RADIUS_M.powi(2) / radius_fifth_m5;

        [
            central_acceleration * position_m[0]
                + j2_acceleration * position_m[0] * (1.0 - 5.0 * z_ratio_squared)
                + broadcast_acceleration_mps2[0],
            central_acceleration * position_m[1]
                + j2_acceleration * position_m[1] * (1.0 - 5.0 * z_ratio_squared)
                + broadcast_acceleration_mps2[1],
            central_acceleration * position_m[2]
                + j2_acceleration * position_m[2] * (3.0 - 5.0 * z_ratio_squared)
                + broadcast_acceleration_mps2[2],
        ]
    }

    fn independent_reference_ecef_state(
        position_m: [f64; 3],
        velocity_mps: [f64; 3],
        delta_t_s: f64,
    ) -> GlonassNumericalState {
        let rotation_rad = GLONASS_EARTH_ROTATION_RATE_RAD_S * delta_t_s;
        let cos_rotation = rotation_rad.cos();
        let sin_rotation = rotation_rad.sin();
        let x_m = position_m[0] * cos_rotation + position_m[1] * sin_rotation;
        let y_m = -position_m[0] * sin_rotation + position_m[1] * cos_rotation;
        let z_m = position_m[2];
        let rotated_vx_mps = velocity_mps[0] * cos_rotation + velocity_mps[1] * sin_rotation;
        let rotated_vy_mps = -velocity_mps[0] * sin_rotation + velocity_mps[1] * cos_rotation;

        GlonassNumericalState {
            position_m: [x_m, y_m, z_m],
            velocity_mps: [
                rotated_vx_mps + GLONASS_EARTH_ROTATION_RATE_RAD_S * y_m,
                rotated_vy_mps - GLONASS_EARTH_ROTATION_RATE_RAD_S * x_m,
                velocity_mps[2],
            ],
        }
    }
}
