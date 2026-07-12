#![allow(missing_docs)]

use crate::estimation::position::filter::{
    PositionFilterMotionClass, PositionFilterMotionModel, PositionFilterProcessNoise,
};
use crate::estimation::position::solver::PositionSolution;
use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, covariance_horizontal_vertical, horizontal_error_ellipse,
};

#[derive(Debug, Clone)]
pub struct PositionSolutionSmootherConfig {
    pub motion_class: PositionFilterMotionClass,
    pub process_noise: PositionFilterProcessNoise,
    pub motion_model: PositionFilterMotionModel,
    pub initial_position_sigma_m: f64,
    pub initial_velocity_sigma_mps: f64,
    pub measurement_sigma_floor_m: f64,
    pub min_dt_s: f64,
    pub reset_gap_s: f64,
}

impl Default for PositionSolutionSmootherConfig {
    fn default() -> Self {
        let mut config = Self {
            motion_class: PositionFilterMotionClass::Vehicle,
            process_noise: PositionFilterProcessNoise::default(),
            motion_model: PositionFilterMotionModel::ConstantVelocity,
            initial_position_sigma_m: 100.0,
            initial_velocity_sigma_mps: 50.0,
            measurement_sigma_floor_m: 1.0,
            min_dt_s: 1.0e-3,
            reset_gap_s: 5.0,
        };
        config.apply_motion_class(config.motion_class);
        config
    }
}

impl PositionSolutionSmootherConfig {
    pub fn for_motion_class(motion_class: PositionFilterMotionClass) -> Self {
        let mut config = Self::default();
        config.apply_motion_class(motion_class);
        config
    }

    pub fn apply_motion_class(&mut self, motion_class: PositionFilterMotionClass) {
        self.motion_class = motion_class;
        self.motion_model = motion_class.default_motion_model();
        self.process_noise = motion_class.process_noise();
        self.initial_velocity_sigma_mps = motion_class.initial_velocity_sigma_mps();
    }
}

#[derive(Debug, Clone)]
pub struct PositionSolutionSmootherEpoch {
    pub t_rx_s: f64,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    pub sigma_e_m: Option<f64>,
    pub sigma_n_m: Option<f64>,
    pub sigma_u_m: Option<f64>,
    pub horizontal_error_ellipse_major_axis_m: Option<f64>,
    pub horizontal_error_ellipse_minor_axis_m: Option<f64>,
    pub horizontal_error_ellipse_azimuth_deg: Option<f64>,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub velocity_x_mps: f64,
    pub velocity_y_mps: f64,
    pub velocity_z_mps: f64,
}

pub struct PositionSolutionSmoother {
    pub config: PositionSolutionSmootherConfig,
    state: [f64; 6],
    covariance: [[f64; 6]; 6],
    pub last_t_rx_s: Option<f64>,
    pub initialized: bool,
}

impl PositionSolutionSmoother {
    pub fn new(config: PositionSolutionSmootherConfig) -> Self {
        let mut covariance = [[0.0; 6]; 6];
        let position_variance_m2 = config.initial_position_sigma_m.powi(2);
        let velocity_variance_m2ps2 = config.initial_velocity_sigma_mps.powi(2);
        for axis in 0..3 {
            covariance[axis][axis] = position_variance_m2;
            covariance[axis + 3][axis + 3] = velocity_variance_m2ps2;
        }
        Self { config, state: [0.0; 6], covariance, last_t_rx_s: None, initialized: false }
    }

    pub fn smooth_position_solution(
        &mut self,
        t_rx_s: f64,
        solution: &PositionSolution,
    ) -> PositionSolutionSmootherEpoch {
        let measurement_position_m = [solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m];
        let measurement_covariance_m2 = measurement_covariance_m2(solution, &self.config);
        if should_reset(self.last_t_rx_s, t_rx_s, self.config.reset_gap_s) || !self.initialized {
            self.seed(measurement_position_m, measurement_covariance_m2, t_rx_s);
            return self.epoch_from_state(t_rx_s);
        }

        let dt_s = (t_rx_s - self.last_t_rx_s.unwrap_or(t_rx_s)).abs().max(self.config.min_dt_s);
        self.predict(dt_s);
        self.update(measurement_position_m, measurement_covariance_m2);
        self.last_t_rx_s = Some(t_rx_s);
        self.epoch_from_state(t_rx_s)
    }

    fn seed(
        &mut self,
        position_ecef_m: [f64; 3],
        measurement_covariance_m2: [[f64; 3]; 3],
        t_rx_s: f64,
    ) {
        self.state = [
            position_ecef_m[0],
            position_ecef_m[1],
            position_ecef_m[2],
            0.0,
            0.0,
            0.0,
        ];
        self.covariance = [[0.0; 6]; 6];
        for row in 0..3 {
            for col in 0..3 {
                self.covariance[row][col] = measurement_covariance_m2[row][col];
            }
            self.covariance[row + 3][row + 3] = self.config.initial_velocity_sigma_mps.powi(2);
        }
        self.last_t_rx_s = Some(t_rx_s);
        self.initialized = true;
    }

    fn predict(&mut self, dt_s: f64) {
        let velocity_decay = match &self.config.motion_model {
            PositionFilterMotionModel::ConstantVelocity => 1.0,
            PositionFilterMotionModel::StaticPosition(static_model) => {
                (-static_model.velocity_decay_per_s * dt_s).exp()
            }
        };
        let transition = state_transition(dt_s, velocity_decay);
        let predicted_state = mat6_vec_mul(&transition, &self.state);
        let predicted_covariance = mat6_add(
            &mat6_mul(&mat6_mul(&transition, &self.covariance), &mat6_transpose(&transition)),
            &process_noise_covariance(dt_s, velocity_decay, &self.config.process_noise),
        );
        self.state = predicted_state;
        self.covariance = predicted_covariance;
    }

    fn update(&mut self, measurement_position_m: [f64; 3], measurement_covariance_m2: [[f64; 3]; 3]) {
        let predicted_position_m = [self.state[0], self.state[1], self.state[2]];
        let innovation_m = [
            measurement_position_m[0] - predicted_position_m[0],
            measurement_position_m[1] - predicted_position_m[1],
            measurement_position_m[2] - predicted_position_m[2],
        ];
        let predicted_position_covariance_m2 = top_left_covariance(&self.covariance);
        let innovation_covariance_m2 =
            mat3_add(&predicted_position_covariance_m2, &measurement_covariance_m2);
        let innovation_covariance_inverse_m2 =
            invert_3x3(innovation_covariance_m2).unwrap_or_else(|| {
                invert_3x3(diagonal3(self.config.measurement_sigma_floor_m.powi(2)))
                    .expect("diagonal covariance is invertible")
            });
        let kalman_gain = kalman_gain(&self.covariance, &innovation_covariance_inverse_m2);
        self.state = add_state_delta(self.state, mat6x3_vec_mul(&kalman_gain, &innovation_m));
        self.covariance =
            joseph_covariance_update(&self.covariance, &kalman_gain, &measurement_covariance_m2);
    }

    fn epoch_from_state(&self, t_rx_s: f64) -> PositionSolutionSmootherEpoch {
        let position_ecef_m = [self.state[0], self.state[1], self.state[2]];
        let position_covariance_ecef_m2 = Some(top_left_covariance(&self.covariance));
        let (sigma_e_m, sigma_n_m, sigma_u_m) = position_covariance_ecef_m2
            .and_then(|covariance| {
                covariance_enu_standard_deviations_m(position_ecef_m, covariance)
            })
            .map(|(sigma_e_m, sigma_n_m, sigma_u_m)| {
                (Some(sigma_e_m), Some(sigma_n_m), Some(sigma_u_m))
            })
            .unwrap_or((None, None, None));
        let (
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
        ) = position_covariance_ecef_m2
            .and_then(|covariance| horizontal_error_ellipse(position_ecef_m, covariance))
            .map(|ellipse| {
                (
                    Some(ellipse.major_axis_m),
                    Some(ellipse.minor_axis_m),
                    Some(ellipse.azimuth_deg),
                )
            })
            .unwrap_or((None, None, None));
        let (sigma_h_m, sigma_v_m) = position_covariance_ecef_m2
            .and_then(|covariance| covariance_horizontal_vertical(position_ecef_m, covariance))
            .map(|(sigma_h_m, sigma_v_m)| (Some(sigma_h_m), Some(sigma_v_m)))
            .unwrap_or((None, None));

        PositionSolutionSmootherEpoch {
            t_rx_s,
            ecef_x_m: position_ecef_m[0],
            ecef_y_m: position_ecef_m[1],
            ecef_z_m: position_ecef_m[2],
            position_covariance_ecef_m2,
            sigma_e_m,
            sigma_n_m,
            sigma_u_m,
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
            sigma_h_m,
            sigma_v_m,
            velocity_x_mps: self.state[3],
            velocity_y_mps: self.state[4],
            velocity_z_mps: self.state[5],
        }
    }
}

fn should_reset(last_t_rx_s: Option<f64>, t_rx_s: f64, reset_gap_s: f64) -> bool {
    let Some(previous_t_rx_s) = last_t_rx_s else {
        return true;
    };
    !previous_t_rx_s.is_finite()
        || !t_rx_s.is_finite()
        || (t_rx_s - previous_t_rx_s).abs() > reset_gap_s
}

fn measurement_covariance_m2(
    solution: &PositionSolution,
    config: &PositionSolutionSmootherConfig,
) -> [[f64; 3]; 3] {
    let floor_variance_m2 = config.measurement_sigma_floor_m.powi(2);
    let covariance = solution
        .position_covariance_ecef_m2
        .map(sanitize_measurement_covariance)
        .unwrap_or_else(|| fallback_measurement_covariance(solution, floor_variance_m2));
    enforce_covariance_floor(covariance, floor_variance_m2)
}

fn fallback_measurement_covariance(
    solution: &PositionSolution,
    floor_variance_m2: f64,
) -> [[f64; 3]; 3] {
    let horizontal_variance_m2 = solution.sigma_h_m.map(|sigma_h_m| sigma_h_m.powi(2));
    let vertical_variance_m2 = solution.sigma_v_m.map(|sigma_v_m| sigma_v_m.powi(2));
    diagonal3([
        horizontal_variance_m2.unwrap_or(floor_variance_m2).max(floor_variance_m2),
        horizontal_variance_m2.unwrap_or(floor_variance_m2).max(floor_variance_m2),
        vertical_variance_m2.unwrap_or(floor_variance_m2).max(floor_variance_m2),
    ])
}

fn sanitize_measurement_covariance(covariance_m2: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut symmetrized = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            let left = covariance_m2[row][col];
            let right = covariance_m2[col][row];
            let average = if left.is_finite() && right.is_finite() {
                0.5 * (left + right)
            } else if left.is_finite() {
                left
            } else if right.is_finite() {
                right
            } else {
                0.0
            };
            symmetrized[row][col] = average;
        }
    }
    symmetrized
}

fn enforce_covariance_floor(
    mut covariance_m2: [[f64; 3]; 3],
    floor_variance_m2: f64,
) -> [[f64; 3]; 3] {
    for axis in 0..3 {
        if !covariance_m2[axis][axis].is_finite() || covariance_m2[axis][axis] < floor_variance_m2
        {
            covariance_m2[axis][axis] = floor_variance_m2;
        }
    }
    covariance_m2
}

fn state_transition(dt_s: f64, velocity_decay: f64) -> [[f64; 6]; 6] {
    let mut transition = [[0.0; 6]; 6];
    for axis in 0..3 {
        transition[axis][axis] = 1.0;
        transition[axis][axis + 3] = dt_s;
        transition[axis + 3][axis + 3] = velocity_decay;
    }
    transition
}

fn process_noise_covariance(
    dt_s: f64,
    velocity_decay: f64,
    process_noise: &PositionFilterProcessNoise,
) -> [[f64; 6]; 6] {
    let mut covariance = [[0.0; 6]; 6];
    let acceleration_sigma_mps2 = process_noise.vel_mps.max(1.0e-6);
    let acceleration_variance = acceleration_sigma_mps2.powi(2);
    let position_walk_variance = process_noise.pos_m.powi(2) * dt_s.max(1.0);
    let dt2 = dt_s * dt_s;
    let dt3 = dt2 * dt_s;
    let dt4 = dt2 * dt2;
    let q_pos = position_walk_variance + 0.25 * acceleration_variance * dt4;
    let q_cross = 0.5 * acceleration_variance * dt3 * velocity_decay.max(1.0e-3);
    let q_vel = acceleration_variance * dt2;
    for axis in 0..3 {
        covariance[axis][axis] = q_pos;
        covariance[axis][axis + 3] = q_cross;
        covariance[axis + 3][axis] = q_cross;
        covariance[axis + 3][axis + 3] = q_vel;
    }
    covariance
}

fn top_left_covariance(covariance: &[[f64; 6]; 6]) -> [[f64; 3]; 3] {
    let mut position_covariance = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            position_covariance[row][col] = covariance[row][col];
        }
    }
    position_covariance
}

fn kalman_gain(
    covariance: &[[f64; 6]; 6],
    innovation_covariance_inverse: &[[f64; 3]; 3],
) -> [[f64; 3]; 6] {
    let mut gain = [[0.0; 3]; 6];
    for row in 0..6 {
        for measurement_axis in 0..3 {
            gain[row][measurement_axis] = (0..3)
                .map(|column| covariance[row][column] * innovation_covariance_inverse[column][measurement_axis])
                .sum();
        }
    }
    gain
}

fn joseph_covariance_update(
    covariance: &[[f64; 6]; 6],
    kalman_gain: &[[f64; 3]; 6],
    measurement_covariance_m2: &[[f64; 3]; 3],
) -> [[f64; 6]; 6] {
    let mut identity_minus_kh = identity6();
    for row in 0..6 {
        for col in 0..3 {
            identity_minus_kh[row][col] -= kalman_gain[row][col];
        }
    }
    let left = mat6_mul(
        &mat6_mul(&identity_minus_kh, covariance),
        &mat6_transpose(&identity_minus_kh),
    );
    let mut right = [[0.0; 6]; 6];
    for row in 0..6 {
        for col in 0..6 {
            right[row][col] = (0..3)
                .flat_map(|mid_left| {
                    (0..3).map(move |mid_right| {
                        kalman_gain[row][mid_left]
                            * measurement_covariance_m2[mid_left][mid_right]
                            * kalman_gain[col][mid_right]
                    })
                })
                .sum();
        }
    }
    mat6_add(&left, &right)
}

fn diagonal3(diagonal: impl Into<Diagonal3>) -> [[f64; 3]; 3] {
    let diagonal = diagonal.into();
    [
        [diagonal.0[0], 0.0, 0.0],
        [0.0, diagonal.0[1], 0.0],
        [0.0, 0.0, diagonal.0[2]],
    ]
}

struct Diagonal3([f64; 3]);

impl From<f64> for Diagonal3 {
    fn from(value: f64) -> Self {
        Self([value; 3])
    }
}

impl From<[f64; 3]> for Diagonal3 {
    fn from(value: [f64; 3]) -> Self {
        Self(value)
    }
}

fn invert_3x3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    if !determinant.is_finite() || determinant.abs() < 1.0e-12 {
        return None;
    }
    let inverse_determinant = 1.0 / determinant;
    Some([
        [
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inverse_determinant,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inverse_determinant,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inverse_determinant,
        ],
        [
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inverse_determinant,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inverse_determinant,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inverse_determinant,
        ],
        [
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inverse_determinant,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inverse_determinant,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inverse_determinant,
        ],
    ])
}

fn add_state_delta(state: [f64; 6], delta: [f64; 6]) -> [f64; 6] {
    let mut updated_state = state;
    for index in 0..6 {
        updated_state[index] += delta[index];
    }
    updated_state
}

fn mat3_add(left: &[[f64; 3]; 3], right: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut sum = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            sum[row][col] = left[row][col] + right[row][col];
        }
    }
    sum
}

fn mat6_add(left: &[[f64; 6]; 6], right: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut sum = [[0.0; 6]; 6];
    for row in 0..6 {
        for col in 0..6 {
            sum[row][col] = left[row][col] + right[row][col];
        }
    }
    sum
}

fn mat6_mul(left: &[[f64; 6]; 6], right: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut product = [[0.0; 6]; 6];
    for row in 0..6 {
        for col in 0..6 {
            product[row][col] = (0..6).map(|mid| left[row][mid] * right[mid][col]).sum();
        }
    }
    product
}

fn mat6_transpose(matrix: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut transpose = [[0.0; 6]; 6];
    for row in 0..6 {
        for col in 0..6 {
            transpose[row][col] = matrix[col][row];
        }
    }
    transpose
}

fn mat6_vec_mul(matrix: &[[f64; 6]; 6], vector: &[f64; 6]) -> [f64; 6] {
    let mut product = [0.0; 6];
    for row in 0..6 {
        product[row] = (0..6).map(|col| matrix[row][col] * vector[col]).sum();
    }
    product
}

fn mat6x3_vec_mul(matrix: &[[f64; 3]; 6], vector: &[f64; 3]) -> [f64; 6] {
    let mut product = [0.0; 6];
    for row in 0..6 {
        product[row] = (0..3).map(|col| matrix[row][col] * vector[col]).sum();
    }
    product
}

fn identity6() -> [[f64; 6]; 6] {
    let mut identity = [[0.0; 6]; 6];
    for axis in 0..6 {
        identity[axis][axis] = 1.0;
    }
    identity
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoother_config_tracks_motion_class_defaults() {
        let mut config = PositionSolutionSmootherConfig::for_motion_class(
            PositionFilterMotionClass::Static,
        );
        assert!(matches!(
            config.motion_model,
            PositionFilterMotionModel::StaticPosition(_)
        ));
        config.apply_motion_class(PositionFilterMotionClass::Airborne);
        assert_eq!(config.motion_class, PositionFilterMotionClass::Airborne);
        assert!(matches!(
            config.motion_model,
            PositionFilterMotionModel::ConstantVelocity
        ));
    }

    #[test]
    fn smoother_epoch_exposes_smoothed_covariance_metrics() {
        let mut smoother = PositionSolutionSmoother::new(PositionSolutionSmootherConfig::default());
        let solution = sample_solution(1_115_194.907, -4_842_955.026, 3_985_351.523);
        let epoch = smoother.smooth_position_solution(100.0, &solution);
        assert!(epoch.position_covariance_ecef_m2.is_some());
        assert!(epoch.sigma_h_m.expect("horizontal sigma").is_finite());
        assert!(epoch.sigma_v_m.expect("vertical sigma").is_finite());
        assert!(epoch
            .horizontal_error_ellipse_major_axis_m
            .expect("ellipse major axis")
            .is_finite());
    }

    fn sample_solution(ecef_x_m: f64, ecef_y_m: f64, ecef_z_m: f64) -> PositionSolution {
        PositionSolution {
            ecef_x_m,
            ecef_y_m,
            ecef_z_m,
            position_covariance_ecef_m2: Some([
                [9.0, 0.5, 0.1],
                [0.5, 9.0, 0.2],
                [0.1, 0.2, 16.0],
            ]),
            horizontal_error_ellipse_major_axis_m: Some(3.5),
            horizontal_error_ellipse_minor_axis_m: Some(2.9),
            horizontal_error_ellipse_azimuth_deg: Some(21.0),
            sigma_e_m: Some(3.0),
            sigma_n_m: Some(3.0),
            sigma_u_m: Some(4.0),
            latitude_deg: 37.0,
            longitude_deg: -122.0,
            altitude_m: 10.0,
            clock_reference_constellation: bijux_gnss_core::api::Constellation::Gps,
            clock_bias_s: 2.5e-4,
            inter_system_biases: Vec::new(),
            pdop: 2.0,
            hdop: Some(1.4),
            vdop: Some(1.7),
            gdop: Some(2.4),
            tdop: Some(1.2),
            pre_fit_residual_rms_m: 1.0,
            post_fit_residual_rms_m: 0.8,
            rms_m: 0.8,
            sigma_h_m: Some(2.8),
            sigma_v_m: Some(4.0),
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
}
