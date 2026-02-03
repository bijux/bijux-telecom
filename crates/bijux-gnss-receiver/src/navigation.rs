use bijux_gnss_core::{NavResidual, NavSolutionEpoch, ObsEpoch};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, GpsEphemeris,
    PositionObservation, PositionSolver, WeightingConfig,
};

use crate::config::ReceiverConfig;

/// Navigation solution derived from observation epochs.
pub struct Navigation {
    #[allow(dead_code)]
    config: ReceiverConfig,
    solver: PositionSolver,
    clock: ClockModel,
    last_ecef: Option<(f64, f64, f64)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavigationState {
    ColdStart,
    Coarse,
    Converged,
    Degraded,
    Recovered,
}

pub struct NavigationEngine {
    pub state: NavigationState,
}

impl NavigationEngine {
    pub fn new() -> Self {
        Self {
            state: NavigationState::ColdStart,
        }
    }

    pub fn transition(&mut self, sats: usize, rms_m: f64) {
        self.state = match (self.state, sats, rms_m) {
            (_, s, _) if s < 4 => NavigationState::Degraded,
            (NavigationState::ColdStart, _, r) if r.is_finite() => NavigationState::Coarse,
            (NavigationState::Coarse, _, r) if r < 50.0 => NavigationState::Converged,
            (NavigationState::Converged, _, r) if r > 200.0 => NavigationState::Degraded,
            (NavigationState::Degraded, _, r) if r < 80.0 => NavigationState::Recovered,
            (state, _, _) => state,
        };
    }
}

impl Default for NavigationEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct EkfState {
    pub position_ecef_m: [f64; 3],
    pub velocity_ecef_mps: [f64; 3],
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
}

impl EkfState {
    pub fn new() -> Self {
        Self {
            position_ecef_m: [0.0; 3],
            velocity_ecef_mps: [0.0; 3],
            clock_bias_s: 0.0,
            clock_drift_s: 0.0,
        }
    }
}

impl Default for EkfState {
    fn default() -> Self {
        Self::new()
    }
}

impl Navigation {
    pub fn new(config: ReceiverConfig) -> Self {
        let mut solver = PositionSolver::new();
        solver.robust = config.robust_solver;
        solver.huber_k = config.huber_k;
        solver.raim = config.raim;
        Self {
            config,
            solver,
            clock: ClockModel::new(),
            last_ecef: None,
        }
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        eph: &[GpsEphemeris],
    ) -> Option<NavSolutionEpoch> {
        let observations: Vec<PositionObservation> = obs
            .sats
            .iter()
            .filter_map(|s| {
                let mut elevation = s.elevation_deg;
                if elevation.is_none() {
                    if let Some((rx_x, rx_y, rx_z)) = self.last_ecef {
                        if let Some(eph) = eph.iter().find(|e| e.sat == s.signal_id.sat) {
                            let sat = sat_state_gps_l1ca(eph, obs.t_rx_s, 0.0);
                            let (_az, el) =
                                elevation_azimuth_deg(rx_x, rx_y, rx_z, sat.x_m, sat.y_m, sat.z_m);
                            elevation = Some(el);
                        }
                    }
                }
                let weight_config = WeightingConfig {
                    enabled: self.config.weighting.enabled,
                    min_elev_deg: self.config.weighting.min_elev_deg,
                    elev_exponent: self.config.weighting.elev_exponent,
                    cn0_ref_dbhz: self.config.weighting.cn0_ref_dbhz,
                    min_weight: self.config.weighting.min_weight,
                };
                if let Some(el) = elevation {
                    if el < self.config.weighting.elev_mask_deg {
                        return None;
                    }
                }
                let tracking_mode_weight = if s.metadata.tracking_mode.contains("vector") {
                    self.config.weighting.tracking_mode_vector_weight
                } else {
                    self.config.weighting.tracking_mode_scalar_weight
                };
                let weight = elevation
                    .map(|el| weight_from_cn0_elev(s.cn0_dbhz, el, weight_config))
                    .unwrap_or(1.0);
                Some(PositionObservation {
                    sat: s.signal_id.sat,
                    pseudorange_m: s.pseudorange_m,
                    cn0_dbhz: s.cn0_dbhz,
                    elevation_deg: elevation,
                    weight: weight * tracking_mode_weight,
                })
            })
            .collect();
        let solution = self.solver.solve_wls(&observations, eph, obs.t_rx_s)?;
        let clock_bias_s = self.clock.update(solution.clock_bias_s, 0.001);
        self.last_ecef = Some((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        Some(NavSolutionEpoch {
            epoch: bijux_gnss_core::Epoch {
                index: obs.epoch_idx,
            },
            ecef_x_m: solution.ecef_x_m,
            ecef_y_m: solution.ecef_y_m,
            ecef_z_m: solution.ecef_z_m,
            latitude_deg: solution.latitude_deg,
            longitude_deg: solution.longitude_deg,
            altitude_m: solution.altitude_m,
            clock_bias_s,
            pdop: solution.pdop,
            rms_m: solution.rms_m,
            sigma_h_m: solution.sigma_h_m,
            sigma_v_m: solution.sigma_v_m,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            residuals: solution
                .residuals
                .into_iter()
                .map(|(sat, residual_m)| NavResidual {
                    sat,
                    residual_m,
                    rejected: false,
                })
                .chain(solution.rejected.into_iter().map(|sat| NavResidual {
                    sat,
                    residual_m: 0.0,
                    rejected: true,
                }))
                .collect(),
            isb: Vec::new(),
        })
    }
}

#[derive(Debug, Clone)]
struct ClockModel {
    bias_s: f64,
    drift_s: f64,
}

impl ClockModel {
    fn new() -> Self {
        Self {
            bias_s: 0.0,
            drift_s: 0.0,
        }
    }

    fn update(&mut self, measurement_bias_s: f64, dt_s: f64) -> f64 {
        let alpha = 0.1;
        let beta = 0.01;
        let mut bias = self.bias_s + self.drift_s * dt_s;
        let residual = measurement_bias_s - bias;
        bias += alpha * residual;
        let drift = self.drift_s + (beta * residual / dt_s.max(1e-6));
        self.bias_s = bias;
        self.drift_s = drift;
        self.bias_s
    }
}
