#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    signal_id_wavelength_m, Constellation, InterSystemBias, MeasurementRejectReason, SatId, SigId,
    SignalBand, SignalCode,
};

use crate::estimation::ekf::models::{
    DopplerMeasurement, NavClockModel, ProcessNoiseConfig, PseudorangeMeasurement,
    StaticNavClockModel,
};
use crate::estimation::ekf::state::{Ekf, EkfConfig};
use crate::estimation::position::navigation::{
    corrected_pseudorange_m, default_position_signal_id, resolve_position_inputs,
    satellite_state_from_observation, unknown_inter_system_time_offset_sats, PositionSolveInput,
    SatelliteState,
};
use crate::estimation::position::solver::{
    elevation_azimuth_deg, position_broadcast_navigation_from_gps_ephemerides,
    position_measurement_weight, position_observation_has_valid_satellite_time,
    PositionBroadcastNavigation, PositionObservation, PositionSolveRefusal,
    PositionSolveRefusalKind, PositionSolver, WeightingConfig,
};
use crate::linalg::Matrix;
use crate::orbits::gps::GpsEphemeris;

#[derive(Debug, Clone)]
pub struct PositionFilterProcessNoise {
    pub pos_m: f64,
    pub vel_mps: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s_per_s: f64,
}

impl Default for PositionFilterProcessNoise {
    fn default() -> Self {
        Self { pos_m: 5.0, vel_mps: 1.0, clock_bias_s: 1.0e-6, clock_drift_s_per_s: 1.0e-7 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PositionFilterMotionClass {
    Static,
    Pedestrian,
    #[default]
    Vehicle,
    Airborne,
}

#[derive(Debug, Clone)]
pub struct PositionFilterStaticPositionModel {
    pub velocity_decay_per_s: f64,
}

impl Default for PositionFilterStaticPositionModel {
    fn default() -> Self {
        Self { velocity_decay_per_s: 4.0 }
    }
}

#[derive(Debug, Clone, Default)]
pub enum PositionFilterMotionModel {
    #[default]
    ConstantVelocity,
    StaticPosition(PositionFilterStaticPositionModel),
}

#[derive(Debug, Clone)]
pub struct PositionFilterConfig {
    pub motion_class: PositionFilterMotionClass,
    pub process_noise: PositionFilterProcessNoise,
    pub motion_model: PositionFilterMotionModel,
    pub weighting: WeightingConfig,
    pub base_pseudorange_sigma_m: f64,
    pub base_doppler_sigma_hz: f64,
    pub gating_chi2_code: Option<f64>,
    pub gating_chi2_doppler: Option<f64>,
    pub huber_k: Option<f64>,
    pub use_doppler: bool,
    pub apply_broadcast_group_delay: bool,
    pub initial_position_sigma_m: f64,
    pub initial_velocity_sigma_mps: f64,
    pub initial_clock_bias_sigma_s: f64,
    pub initial_clock_drift_sigma_s_per_s: f64,
    pub min_dt_s: f64,
}

impl Default for PositionFilterConfig {
    fn default() -> Self {
        let mut config = Self {
            motion_class: PositionFilterMotionClass::Vehicle,
            process_noise: PositionFilterProcessNoise::default(),
            motion_model: PositionFilterMotionModel::ConstantVelocity,
            weighting: WeightingConfig::default(),
            base_pseudorange_sigma_m: 5.0,
            base_doppler_sigma_hz: 1.0,
            gating_chi2_code: Some(100.0),
            gating_chi2_doppler: Some(100.0),
            huber_k: Some(30.0),
            use_doppler: true,
            apply_broadcast_group_delay: true,
            initial_position_sigma_m: 100.0,
            initial_velocity_sigma_mps: 50.0,
            initial_clock_bias_sigma_s: 1.0e-3,
            initial_clock_drift_sigma_s_per_s: 1.0e-4,
            min_dt_s: 1.0e-3,
        };
        config.apply_motion_class(config.motion_class);
        config
    }
}

impl PositionFilterConfig {
    pub fn for_motion_class(motion_class: PositionFilterMotionClass) -> Self {
        let mut config = Self::default();
        config.apply_motion_class(motion_class);
        config
    }

    pub fn for_constant_velocity_receiver() -> Self {
        Self::for_vehicle_receiver()
    }

    pub fn for_static_receiver() -> Self {
        Self::for_motion_class(PositionFilterMotionClass::Static)
    }

    pub fn for_pedestrian_receiver() -> Self {
        Self::for_motion_class(PositionFilterMotionClass::Pedestrian)
    }

    pub fn for_vehicle_receiver() -> Self {
        Self::for_motion_class(PositionFilterMotionClass::Vehicle)
    }

    pub fn for_airborne_receiver() -> Self {
        Self::for_motion_class(PositionFilterMotionClass::Airborne)
    }

    pub fn apply_motion_class(&mut self, motion_class: PositionFilterMotionClass) {
        self.motion_class = motion_class;
        self.motion_model = motion_class.default_motion_model();
        self.process_noise = motion_class.process_noise();
        self.initial_velocity_sigma_mps = motion_class.initial_velocity_sigma_mps();
    }
}

impl PositionFilterMotionClass {
    pub fn process_noise(self) -> PositionFilterProcessNoise {
        let mut process_noise = PositionFilterProcessNoise::default();
        match self {
            Self::Static => {
                process_noise.pos_m = 0.5;
                process_noise.vel_mps = 0.05;
            }
            Self::Pedestrian => {
                process_noise.pos_m = 1.5;
                process_noise.vel_mps = 0.35;
            }
            Self::Vehicle => {}
            Self::Airborne => {
                process_noise.pos_m = 12.0;
                process_noise.vel_mps = 3.0;
            }
        }
        process_noise
    }

    pub fn initial_velocity_sigma_mps(self) -> f64 {
        match self {
            Self::Static => 5.0,
            Self::Pedestrian => 12.0,
            Self::Vehicle => 50.0,
            Self::Airborne => 120.0,
        }
    }

    pub fn default_motion_model(self) -> PositionFilterMotionModel {
        match self {
            Self::Static => PositionFilterMotionModel::StaticPosition(
                PositionFilterStaticPositionModel::default(),
            ),
            Self::Pedestrian | Self::Vehicle | Self::Airborne => {
                PositionFilterMotionModel::ConstantVelocity
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct PositionFilterEpoch {
    pub t_rx_s: f64,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub velocity_x_mps: f64,
    pub velocity_y_mps: f64,
    pub velocity_z_mps: f64,
    pub clock_bias_s: f64,
    pub clock_bias_sigma_s: f64,
    pub clock_drift_s_per_s: f64,
    pub clock_drift_sigma_s_per_s: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub rms_m: f64,
    pub residuals: Vec<(SatId, f64)>,
    pub inter_system_biases: Vec<InterSystemBias>,
    pub used_sat_count: usize,
}

#[derive(Debug, Clone)]
pub struct PositionFilterIndices {
    pub pos: [usize; 3],
    pub vel: [usize; 3],
    pub clock_bias: usize,
    pub clock_drift: usize,
    pub isb: BTreeMap<Constellation, usize>,
}

pub struct PositionFilter {
    pub ekf: Ekf,
    pub config: PositionFilterConfig,
    pub indices: PositionFilterIndices,
    pub reference_constellation: Option<Constellation>,
    pub last_t_rx_s: Option<f64>,
    pub initialized: bool,
}

impl PositionFilter {
    pub fn new(config: PositionFilterConfig) -> Self {
        let state_len = 8;
        let mut covariance = Matrix::new(state_len, state_len, 0.0);
        covariance[(0, 0)] = config.initial_position_sigma_m.powi(2);
        covariance[(1, 1)] = config.initial_position_sigma_m.powi(2);
        covariance[(2, 2)] = config.initial_position_sigma_m.powi(2);
        covariance[(3, 3)] = config.initial_velocity_sigma_mps.powi(2);
        covariance[(4, 4)] = config.initial_velocity_sigma_mps.powi(2);
        covariance[(5, 5)] = config.initial_velocity_sigma_mps.powi(2);
        covariance[(6, 6)] = config.initial_clock_bias_sigma_s.powi(2);
        covariance[(7, 7)] = config.initial_clock_drift_sigma_s_per_s.powi(2);

        let ekf = Ekf::new(
            vec![0.0; state_len],
            covariance,
            EkfConfig {
                gating_chi2_code: config.gating_chi2_code,
                gating_chi2_phase: None,
                gating_chi2_doppler: config.gating_chi2_doppler,
                huber_k: config.huber_k,
                square_root: true,
                covariance_epsilon: 1.0e-12,
                divergence_max_variance: 1.0e12,
            },
        );

        Self {
            ekf,
            config,
            indices: PositionFilterIndices {
                pos: [0, 1, 2],
                vel: [3, 4, 5],
                clock_bias: 6,
                clock_drift: 7,
                isb: BTreeMap::new(),
            },
            reference_constellation: None,
            last_t_rx_s: None,
            initialized: false,
        }
    }

    pub fn seed_receiver_state(&mut self, ecef_m: [f64; 3], clock_bias_s: f64) {
        self.ekf.x[self.indices.pos[0]] = ecef_m[0];
        self.ekf.x[self.indices.pos[1]] = ecef_m[1];
        self.ekf.x[self.indices.pos[2]] = ecef_m[2];
        self.ekf.x[self.indices.clock_bias] = clock_bias_s;
        self.initialized = true;
    }

    pub fn solve_epoch(
        &mut self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
    ) -> Result<PositionFilterEpoch, PositionSolveRefusal> {
        let navigation = position_broadcast_navigation_from_gps_ephemerides(ephemerides);
        self.solve_epoch_with_navigation_data(observations, &navigation, t_rx_s)
    }

    pub fn solve_epoch_with_navigation_data(
        &mut self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
        t_rx_s: f64,
    ) -> Result<PositionFilterEpoch, PositionSolveRefusal> {
        let sat_count = observations.len();
        if sat_count < 4 {
            return Err(PositionSolveRefusal {
                kind: PositionSolveRefusalKind::InsufficientObservations,
                sat_count,
                used_sat_count: sat_count,
                rejected: Vec::new(),
            });
        }

        let mut observations = observations.to_vec();
        observations.sort_by_key(|obs| (obs.sat.constellation as u8, obs.sat.prn));
        let mut rejected = Vec::new();
        observations.retain(|obs| {
            if position_observation_has_valid_satellite_time(obs, t_rx_s) {
                true
            } else {
                rejected.push((obs.sat, MeasurementRejectReason::TimeInconsistency));
                false
            }
        });
        if observations.len() < 4 {
            let kind = if rejected.is_empty() {
                PositionSolveRefusalKind::InsufficientObservations
            } else {
                PositionSolveRefusalKind::InvalidSatelliteTime
            };
            return Err(PositionSolveRefusal {
                kind,
                sat_count,
                used_sat_count: observations.len(),
                rejected,
            });
        }

        let unknown_time_offset_sats =
            unknown_inter_system_time_offset_sats(&observations, navigation);
        if !unknown_time_offset_sats.is_empty() {
            rejected.extend(
                unknown_time_offset_sats
                    .iter()
                    .copied()
                    .map(|sat| (sat, MeasurementRejectReason::TimeInconsistency)),
            );
            return Err(PositionSolveRefusal {
                kind: PositionSolveRefusalKind::UnknownInterSystemTimeOffset,
                sat_count,
                used_sat_count: observations.len().saturating_sub(unknown_time_offset_sats.len()),
                rejected,
            });
        }

        let inputs = resolve_position_inputs(&observations, navigation, t_rx_s, &mut rejected);
        if inputs.len() < 4 {
            return Err(PositionSolveRefusal {
                kind: PositionSolveRefusalKind::InvalidEphemeris,
                sat_count,
                used_sat_count: inputs.len(),
                rejected,
            });
        }

        if !self.initialized {
            let seed = self.initialize_from_wls(&observations, navigation, t_rx_s)?;
            self.last_t_rx_s = Some(t_rx_s);
            return Ok(self.epoch_from_seed(t_rx_s, &seed));
        }

        self.ensure_reference_constellation(&inputs);
        self.ensure_isb_states(&inputs);
        let dt_s = (t_rx_s - self.last_t_rx_s.unwrap_or(t_rx_s)).max(self.config.min_dt_s);
        self.predict(dt_s);

        let mut residuals = Vec::new();
        let mut used_sat_count = 0;
        for input in &inputs {
            let corrected_pseudorange_m = corrected_pseudorange_m(
                &input.observation,
                &input.navigation,
                self.config.apply_broadcast_group_delay,
            );
            let Some(state) = satellite_state_from_observation(
                &input.navigation,
                input.receive_tow_s,
                corrected_pseudorange_m,
                input.observation.signal_timing,
            ) else {
                rejected.push((input.observation.sat, MeasurementRejectReason::InvalidEphemeris));
                continue;
            };

            let code_measurement =
                self.pseudorange_measurement(input, &state, corrected_pseudorange_m);
            if !self.ekf.update(&code_measurement) {
                rejected.push((input.observation.sat, MeasurementRejectReason::Outlier));
                continue;
            }
            used_sat_count += 1;

            if let Some(doppler_measurement) = self.doppler_measurement(input, &state) {
                let _ = self.ekf.update(&doppler_measurement);
            }
            residuals.push((input.observation.sat, self.ekf.health.innovation_rms));
        }

        self.last_t_rx_s = Some(t_rx_s);
        if used_sat_count < 4 {
            return Err(PositionSolveRefusal {
                kind: PositionSolveRefusalKind::InsufficientUsableSatellites,
                sat_count,
                used_sat_count,
                rejected,
            });
        }

        Ok(self.epoch_from_state(t_rx_s, residuals, used_sat_count))
    }

    pub(crate) fn process_noise_config(&self) -> ProcessNoiseConfig {
        ProcessNoiseConfig {
            pos_m: self.config.process_noise.pos_m,
            vel_mps: self.config.process_noise.vel_mps,
            clock_bias_s: self.config.process_noise.clock_bias_s,
            clock_drift_s: self.config.process_noise.clock_drift_s_per_s,
            ztd_m: 0.0,
        }
    }

    fn initialize_from_wls(
        &mut self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
        t_rx_s: f64,
    ) -> Result<crate::estimation::position::solver::PositionSolution, PositionSolveRefusal> {
        let solution = PositionSolver {
            apply_broadcast_group_delay: self.config.apply_broadcast_group_delay,
            ..PositionSolver::new()
        }
        .try_solve_wls_with_navigation_data(observations, navigation, t_rx_s)?;
        self.seed_from_solution(&solution);
        Ok(solution)
    }

    fn seed_from_solution(
        &mut self,
        solution: &crate::estimation::position::solver::PositionSolution,
    ) {
        self.seed_receiver_state(
            [solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m],
            solution.clock_bias_s,
        );
        self.reference_constellation = Some(solution.clock_reference_constellation);
        self.indices.isb.clear();
        for bias in &solution.inter_system_biases {
            let state_index = self.ekf.x.len();
            self.ekf.add_state(
                &format!("isb_{:?}", bias.constellation),
                bias.bias_s.0,
                self.config.initial_clock_bias_sigma_s.powi(2),
            );
            self.indices.isb.insert(bias.constellation, state_index);
        }
    }

    fn ensure_reference_constellation(&mut self, inputs: &[PositionSolveInput]) {
        if self.reference_constellation.is_some() {
            return;
        }
        self.reference_constellation = inputs
            .iter()
            .map(|input| input.observation.sat.constellation)
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .find(|constellation| *constellation == Constellation::Gps)
            .or_else(|| inputs.first().map(|input| input.observation.sat.constellation));
    }

    fn ensure_isb_states(&mut self, inputs: &[PositionSolveInput]) {
        let Some(reference_constellation) = self.reference_constellation else {
            return;
        };
        for constellation in inputs
            .iter()
            .map(|input| input.observation.sat.constellation)
            .collect::<std::collections::BTreeSet<_>>()
        {
            if constellation == reference_constellation
                || self.indices.isb.contains_key(&constellation)
            {
                continue;
            }
            let state_index = self.ekf.x.len();
            self.ekf.add_state(
                &format!("isb_{constellation:?}"),
                0.0,
                self.config.initial_clock_bias_sigma_s.powi(2),
            );
            self.indices.isb.insert(constellation, state_index);
        }
    }

    fn predict(&mut self, dt_s: f64) {
        match &self.config.motion_model {
            PositionFilterMotionModel::ConstantVelocity => {
                self.ekf.predict(&NavClockModel::new(self.process_noise_config()), dt_s);
            }
            PositionFilterMotionModel::StaticPosition(static_position_model) => {
                self.ekf.predict(
                    &StaticNavClockModel::new(
                        self.process_noise_config(),
                        static_position_model.velocity_decay_per_s,
                    ),
                    dt_s,
                );
            }
        }
    }

    fn pseudorange_measurement(
        &self,
        input: &PositionSolveInput,
        state: &SatelliteState,
        corrected_pseudorange_m: f64,
    ) -> PseudorangeMeasurement {
        let signal_id = resolved_signal_id(&input.observation);
        let elevation_deg = input
            .observation
            .elevation_deg
            .or_else(|| current_elevation_deg(&self.ekf.x, &self.indices.pos, state));
        let sigma_m = pseudorange_sigma_m(&input.observation, elevation_deg, &self.config);
        let isb_index = self
            .reference_constellation
            .filter(|reference_constellation| {
                input.observation.sat.constellation != *reference_constellation
            })
            .and_then(|_| self.indices.isb.get(&input.observation.sat.constellation).copied());

        PseudorangeMeasurement {
            sig: signal_id,
            z_m: corrected_pseudorange_m,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m,
            elevation_deg,
            ztd_index: None,
            isb_index,
        }
    }

    fn doppler_measurement(
        &self,
        input: &PositionSolveInput,
        state: &SatelliteState,
    ) -> Option<DopplerMeasurement> {
        if !self.config.use_doppler {
            return None;
        }

        let raw_doppler_hz = input.observation.doppler_hz?;
        let signal_id = resolved_signal_id(&input.observation);
        let wavelength_m = signal_id_wavelength_m(signal_id)?.0;
        let elevation_deg = input
            .observation
            .elevation_deg
            .or_else(|| current_elevation_deg(&self.ekf.x, &self.indices.pos, state));
        let sigma_hz = doppler_sigma_hz(&input.observation, elevation_deg, &self.config);

        Some(DopplerMeasurement {
            sig: signal_id,
            z_hz: raw_doppler_hz + 299_792_458.0 * state.clock_drift_s_per_s / wavelength_m,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_vel_mps: [state.vx_mps, state.vy_mps, state.vz_mps],
            wavelength_m,
            sigma_hz,
        })
    }

    fn epoch_from_seed(
        &self,
        t_rx_s: f64,
        solution: &crate::estimation::position::solver::PositionSolution,
    ) -> PositionFilterEpoch {
        PositionFilterEpoch {
            t_rx_s,
            ecef_x_m: solution.ecef_x_m,
            ecef_y_m: solution.ecef_y_m,
            ecef_z_m: solution.ecef_z_m,
            velocity_x_mps: self.ekf.x[self.indices.vel[0]],
            velocity_y_mps: self.ekf.x[self.indices.vel[1]],
            velocity_z_mps: self.ekf.x[self.indices.vel[2]],
            clock_bias_s: solution.clock_bias_s,
            clock_bias_sigma_s: self.ekf.p[(self.indices.clock_bias, self.indices.clock_bias)]
                .abs()
                .sqrt(),
            clock_drift_s_per_s: self.ekf.x[self.indices.clock_drift],
            clock_drift_sigma_s_per_s: self.ekf.p
                [(self.indices.clock_drift, self.indices.clock_drift)]
                .abs()
                .sqrt(),
            sigma_h_m: solution.sigma_h_m,
            sigma_v_m: solution.sigma_v_m,
            rms_m: solution.rms_m,
            residuals: solution
                .residuals
                .iter()
                .map(|(sat, residual_m, _)| (*sat, *residual_m))
                .collect(),
            inter_system_biases: solution.inter_system_biases.clone(),
            used_sat_count: solution.used_sat_count,
        }
    }

    fn epoch_from_state(
        &self,
        t_rx_s: f64,
        residuals: Vec<(SatId, f64)>,
        used_sat_count: usize,
    ) -> PositionFilterEpoch {
        let sigma_x_m = self.ekf.p[(self.indices.pos[0], self.indices.pos[0])].abs().sqrt();
        let sigma_y_m = self.ekf.p[(self.indices.pos[1], self.indices.pos[1])].abs().sqrt();
        let sigma_z_m = self.ekf.p[(self.indices.pos[2], self.indices.pos[2])].abs().sqrt();
        let clock_bias_sigma_s =
            self.ekf.p[(self.indices.clock_bias, self.indices.clock_bias)].abs().sqrt();
        let clock_drift_sigma_s_per_s =
            self.ekf.p[(self.indices.clock_drift, self.indices.clock_drift)].abs().sqrt();
        PositionFilterEpoch {
            t_rx_s,
            ecef_x_m: self.ekf.x[self.indices.pos[0]],
            ecef_y_m: self.ekf.x[self.indices.pos[1]],
            ecef_z_m: self.ekf.x[self.indices.pos[2]],
            velocity_x_mps: self.ekf.x[self.indices.vel[0]],
            velocity_y_mps: self.ekf.x[self.indices.vel[1]],
            velocity_z_mps: self.ekf.x[self.indices.vel[2]],
            clock_bias_s: self.ekf.x[self.indices.clock_bias],
            clock_bias_sigma_s,
            clock_drift_s_per_s: self.ekf.x[self.indices.clock_drift],
            clock_drift_sigma_s_per_s,
            sigma_h_m: Some((sigma_x_m * sigma_x_m + sigma_y_m * sigma_y_m).sqrt()),
            sigma_v_m: Some(sigma_z_m),
            rms_m: self.ekf.health.innovation_rms,
            residuals,
            inter_system_biases: self.inter_system_biases(),
            used_sat_count,
        }
    }

    fn inter_system_biases(&self) -> Vec<InterSystemBias> {
        self.indices
            .isb
            .iter()
            .map(|(constellation, index)| InterSystemBias {
                constellation: *constellation,
                band: Some(constellation_primary_band(*constellation)),
                bias_s: bijux_gnss_core::api::Seconds(
                    self.ekf.x.get(*index).copied().unwrap_or(0.0),
                ),
            })
            .collect()
    }
}

fn current_elevation_deg(
    state: &[f64],
    pos: &[usize; 3],
    sat_state: &SatelliteState,
) -> Option<f64> {
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        state[pos[0]],
        state[pos[1]],
        state[pos[2]],
        sat_state.x_m,
        sat_state.y_m,
        sat_state.z_m,
    );
    (azimuth_deg.is_finite() && elevation_deg.is_finite()).then_some(elevation_deg)
}

fn pseudorange_sigma_m(
    observation: &PositionObservation,
    elevation_deg: Option<f64>,
    config: &PositionFilterConfig,
) -> f64 {
    let geometry_weight =
        position_measurement_weight(Some(observation.cn0_dbhz), elevation_deg, None, config.weighting);
    let total_weight = (geometry_weight * observation.weight.max(1.0e-6)).max(1.0e-6);
    (config.base_pseudorange_sigma_m / total_weight.sqrt()).max(1.0e-3)
}

fn doppler_sigma_hz(
    observation: &PositionObservation,
    elevation_deg: Option<f64>,
    config: &PositionFilterConfig,
) -> f64 {
    let geometry_weight =
        position_measurement_weight(Some(observation.cn0_dbhz), elevation_deg, None, config.weighting);
    let total_weight = (geometry_weight * observation.weight.max(1.0e-6)).max(1.0e-6);
    let modeled_sigma_hz = (config.base_doppler_sigma_hz / total_weight.sqrt()).max(1.0e-3);
    let observed_sigma_hz = observation
        .doppler_var_hz2
        .filter(|variance_hz2| variance_hz2.is_finite() && *variance_hz2 > 0.0)
        .map(f64::sqrt)
        .unwrap_or(0.0);
    modeled_sigma_hz.max(observed_sigma_hz)
}

fn constellation_primary_band(constellation: Constellation) -> SignalBand {
    match constellation {
        Constellation::Gps => SignalBand::L1,
        Constellation::Galileo => SignalBand::E1,
        Constellation::Glonass => SignalBand::L1,
        Constellation::Beidou => SignalBand::B1,
        Constellation::Unknown => SignalBand::Unknown,
    }
}

fn default_signal_id(sat: SatId) -> SigId {
    SigId { sat, band: constellation_primary_band(sat.constellation), code: SignalCode::Unknown }
}

fn resolved_signal_id(observation: &PositionObservation) -> SigId {
    observation
        .signal_id
        .or_else(|| default_position_signal_id(observation.sat))
        .unwrap_or_else(|| default_signal_id(observation.sat))
}

#[cfg(test)]
mod tests {
    use super::{
        pseudorange_sigma_m, PositionFilter, PositionFilterConfig, PositionFilterMotionClass,
        PositionFilterMotionModel, PositionFilterStaticPositionModel,
    };
    use crate::estimation::position::solver::{
        PositionObservation, PositionSolveRefusalKind, WeightingConfig,
    };
    use bijux_gnss_core::api::{Constellation, SatId};

    fn sample_position_observation(elevation_deg: Option<f64>) -> PositionObservation {
        PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            pseudorange_m: 24_000_000.0,
            doppler_hz: Some(0.0),
            doppler_var_hz2: Some(1.0),
            cn0_dbhz: 45.0,
            elevation_deg,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        }
    }

    #[test]
    fn position_filter_uses_eight_state_layout() {
        let filter = PositionFilter::new(PositionFilterConfig::default());

        assert_eq!(filter.ekf.x.len(), 8);
        assert_eq!(filter.indices.pos, [0, 1, 2]);
        assert_eq!(filter.indices.vel, [3, 4, 5]);
        assert_eq!(filter.indices.clock_bias, 6);
        assert_eq!(filter.indices.clock_drift, 7);
        assert_eq!(filter.ekf.config.gating_chi2_doppler, Some(100.0));
        assert!(!filter.initialized);
    }

    #[test]
    fn position_filter_enables_doppler_updates_by_default() {
        let config = PositionFilterConfig::default();

        assert!(config.use_doppler);
        assert_eq!(config.motion_class, PositionFilterMotionClass::Vehicle);
        assert!(matches!(config.motion_model, PositionFilterMotionModel::ConstantVelocity));
        assert_eq!(config.base_doppler_sigma_hz, 1.0);
        assert_eq!(config.gating_chi2_doppler, Some(100.0));
    }

    #[test]
    fn position_filter_static_receiver_profile_enables_static_model() {
        let config = PositionFilterConfig::for_static_receiver();
        let PositionFilterMotionModel::StaticPosition(static_model) = config.motion_model else {
            panic!("static receiver profile should use the static position motion model");
        };

        assert_eq!(config.motion_class, PositionFilterMotionClass::Static);
        assert_eq!(
            static_model.velocity_decay_per_s,
            PositionFilterStaticPositionModel::default().velocity_decay_per_s
        );
        assert_eq!(config.process_noise.pos_m, 0.5);
        assert_eq!(config.process_noise.vel_mps, 0.05);
        assert_eq!(config.initial_velocity_sigma_mps, 5.0);
    }

    #[test]
    fn position_filter_constant_velocity_profile_enables_constant_velocity_model() {
        let config = PositionFilterConfig::for_constant_velocity_receiver();

        assert_eq!(config.motion_class, PositionFilterMotionClass::Vehicle);
        assert!(matches!(config.motion_model, PositionFilterMotionModel::ConstantVelocity));
        assert_eq!(
            config.process_noise.vel_mps,
            PositionFilterConfig::default().process_noise.vel_mps
        );
        assert_eq!(
            config.initial_velocity_sigma_mps,
            PositionFilterConfig::default().initial_velocity_sigma_mps
        );
    }

    #[test]
    fn position_filter_motion_class_profiles_define_distinct_process_noise() {
        let static_config = PositionFilterConfig::for_static_receiver();
        let pedestrian_config = PositionFilterConfig::for_pedestrian_receiver();
        let vehicle_config = PositionFilterConfig::for_vehicle_receiver();
        let airborne_config = PositionFilterConfig::for_airborne_receiver();

        assert!(matches!(static_config.motion_model, PositionFilterMotionModel::StaticPosition(_)));
        assert!(matches!(
            pedestrian_config.motion_model,
            PositionFilterMotionModel::ConstantVelocity
        ));
        assert!(matches!(vehicle_config.motion_model, PositionFilterMotionModel::ConstantVelocity));
        assert!(matches!(
            airborne_config.motion_model,
            PositionFilterMotionModel::ConstantVelocity
        ));

        assert!(static_config.process_noise.pos_m < pedestrian_config.process_noise.pos_m);
        assert!(pedestrian_config.process_noise.pos_m < vehicle_config.process_noise.pos_m);
        assert!(vehicle_config.process_noise.pos_m < airborne_config.process_noise.pos_m);

        assert!(static_config.process_noise.vel_mps < pedestrian_config.process_noise.vel_mps);
        assert!(pedestrian_config.process_noise.vel_mps < vehicle_config.process_noise.vel_mps);
        assert!(vehicle_config.process_noise.vel_mps < airborne_config.process_noise.vel_mps);

        assert!(
            static_config.initial_velocity_sigma_mps < pedestrian_config.initial_velocity_sigma_mps
        );
        assert!(
            pedestrian_config.initial_velocity_sigma_mps
                < vehicle_config.initial_velocity_sigma_mps
        );
        assert!(
            vehicle_config.initial_velocity_sigma_mps < airborne_config.initial_velocity_sigma_mps
        );
    }

    #[test]
    fn position_filter_apply_motion_class_replaces_tuning_consistently() {
        let mut config = PositionFilterConfig::for_vehicle_receiver();
        config.base_pseudorange_sigma_m = 2.5;
        config.base_doppler_sigma_hz = 0.25;

        config.apply_motion_class(PositionFilterMotionClass::Pedestrian);

        assert_eq!(config.motion_class, PositionFilterMotionClass::Pedestrian);
        assert!(matches!(config.motion_model, PositionFilterMotionModel::ConstantVelocity));
        assert_eq!(
            config.process_noise.pos_m,
            PositionFilterMotionClass::Pedestrian.process_noise().pos_m
        );
        assert_eq!(
            config.process_noise.vel_mps,
            PositionFilterMotionClass::Pedestrian.process_noise().vel_mps
        );
        assert_eq!(
            config.initial_velocity_sigma_mps,
            PositionFilterMotionClass::Pedestrian.initial_velocity_sigma_mps()
        );
        assert_eq!(config.base_pseudorange_sigma_m, 2.5);
        assert_eq!(config.base_doppler_sigma_hz, 0.25);
    }

    #[test]
    fn position_filter_motion_classes_order_prediction_covariance_growth() {
        let mut static_filter = PositionFilter::new(PositionFilterConfig::for_static_receiver());
        let mut pedestrian_filter =
            PositionFilter::new(PositionFilterConfig::for_pedestrian_receiver());
        let mut vehicle_filter = PositionFilter::new(PositionFilterConfig::for_vehicle_receiver());
        let mut airborne_filter =
            PositionFilter::new(PositionFilterConfig::for_airborne_receiver());

        for filter in
            [&mut static_filter, &mut pedestrian_filter, &mut vehicle_filter, &mut airborne_filter]
        {
            filter.seed_receiver_state([0.0, 0.0, 0.0], 0.0);
            filter.ekf.x[3] = 15.0;
            filter.ekf.x[4] = -4.0;
            filter.ekf.x[5] = 1.0;
            for index in 0..8 {
                filter.ekf.p[(index, index)] = 1.0;
            }
        }

        static_filter.predict(1.0);
        pedestrian_filter.predict(1.0);
        vehicle_filter.predict(1.0);
        airborne_filter.predict(1.0);

        let position_variances = [
            static_filter.ekf.p[(0, 0)],
            pedestrian_filter.ekf.p[(0, 0)],
            vehicle_filter.ekf.p[(0, 0)],
            airborne_filter.ekf.p[(0, 0)],
        ];
        let velocity_variances = [
            static_filter.ekf.p[(3, 3)],
            pedestrian_filter.ekf.p[(3, 3)],
            vehicle_filter.ekf.p[(3, 3)],
            airborne_filter.ekf.p[(3, 3)],
        ];

        assert!(position_variances[0] < position_variances[1]);
        assert!(position_variances[1] < position_variances[2]);
        assert!(position_variances[2] < position_variances[3]);

        assert!(velocity_variances[0] < velocity_variances[1]);
        assert!(velocity_variances[1] < velocity_variances[2]);
        assert!(velocity_variances[2] < velocity_variances[3]);
    }

    #[test]
    fn position_filter_seed_receiver_state_updates_position_and_clock() {
        let mut filter = PositionFilter::new(PositionFilterConfig::default());

        filter.seed_receiver_state([1.0, 2.0, 3.0], 4.0e-4);

        assert_eq!(filter.ekf.x[0], 1.0);
        assert_eq!(filter.ekf.x[1], 2.0);
        assert_eq!(filter.ekf.x[2], 3.0);
        assert_eq!(filter.ekf.x[6], 4.0e-4);
        assert!(filter.initialized);
    }

    #[test]
    fn position_filter_assigns_larger_sigma_to_low_elevation_pseudorange() {
        let mut config = PositionFilterConfig::default();
        config.base_pseudorange_sigma_m = 3.0;
        config.weighting = WeightingConfig::default();

        let low_elevation_sigma =
            pseudorange_sigma_m(&sample_position_observation(Some(10.0)), Some(10.0), &config);
        let high_elevation_sigma =
            pseudorange_sigma_m(&sample_position_observation(Some(75.0)), Some(75.0), &config);

        assert!(low_elevation_sigma.is_finite());
        assert!(high_elevation_sigma.is_finite());
        assert!(low_elevation_sigma > high_elevation_sigma);
    }

    #[test]
    fn position_filter_epoch_exposes_clock_state_uncertainty() {
        let filter = PositionFilter::new(PositionFilterConfig::default());

        let epoch = filter.epoch_from_state(0.0, Vec::new(), 0);

        assert_eq!(epoch.clock_bias_sigma_s, filter.config.initial_clock_bias_sigma_s);
        assert_eq!(
            epoch.clock_drift_sigma_s_per_s,
            filter.config.initial_clock_drift_sigma_s_per_s
        );
    }

    #[test]
    fn position_filter_static_profile_predicts_without_position_drift() {
        let mut filter = PositionFilter::new(PositionFilterConfig::for_static_receiver());
        filter.seed_receiver_state([10.0, 20.0, 30.0], 0.0);
        filter.ekf.x[3] = 3.0;
        filter.ekf.x[4] = 4.0;
        filter.ekf.x[5] = 5.0;
        filter.ekf.x[7] = 2.0e-4;

        filter.predict(2.0);

        assert_eq!(filter.ekf.x[0], 10.0);
        assert_eq!(filter.ekf.x[1], 20.0);
        assert_eq!(filter.ekf.x[2], 30.0);
        assert!(filter.ekf.x[3].abs() < 3.0);
        assert!(filter.ekf.x[4].abs() < 4.0);
        assert!(filter.ekf.x[5].abs() < 5.0);
        assert_eq!(filter.ekf.x[6], 4.0e-4);
    }

    #[test]
    fn position_filter_constant_velocity_profile_predicts_position_from_velocity() {
        let mut filter = PositionFilter::new(PositionFilterConfig::for_vehicle_receiver());
        filter.seed_receiver_state([10.0, 20.0, 30.0], 0.0);
        filter.ekf.x[3] = 3.0;
        filter.ekf.x[4] = 4.0;
        filter.ekf.x[5] = 5.0;
        filter.ekf.x[7] = 2.0e-4;

        filter.predict(2.0);

        assert_eq!(filter.ekf.x[0], 16.0);
        assert_eq!(filter.ekf.x[1], 28.0);
        assert_eq!(filter.ekf.x[2], 40.0);
        assert_eq!(filter.ekf.x[3], 3.0);
        assert_eq!(filter.ekf.x[4], 4.0);
        assert_eq!(filter.ekf.x[5], 5.0);
        assert_eq!(filter.ekf.x[6], 4.0e-4);
    }

    #[test]
    fn position_filter_refuses_underconstrained_epoch() {
        let mut filter = PositionFilter::new(PositionFilterConfig::default());
        let observations = vec![PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        }];

        let refusal = filter
            .solve_epoch(&observations, &[], 0.0)
            .expect_err("one observation cannot initialize the sequential filter");

        assert_eq!(refusal.kind, PositionSolveRefusalKind::InsufficientObservations);
        assert_eq!(refusal.sat_count, 1);
    }
}
