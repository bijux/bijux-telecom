#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, InterSystemBias, SatId};

use crate::estimation::ekf::models::ProcessNoiseConfig;
use crate::estimation::ekf::state::{Ekf, EkfConfig};
use crate::estimation::position::solver::WeightingConfig;
use crate::linalg::Matrix;

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

#[derive(Debug, Clone)]
pub struct PositionFilterConfig {
    pub process_noise: PositionFilterProcessNoise,
    pub weighting: WeightingConfig,
    pub base_pseudorange_sigma_m: f64,
    pub gating_chi2_code: Option<f64>,
    pub huber_k: Option<f64>,
    pub apply_broadcast_group_delay: bool,
    pub initial_position_sigma_m: f64,
    pub initial_velocity_sigma_mps: f64,
    pub initial_clock_bias_sigma_s: f64,
    pub initial_clock_drift_sigma_s_per_s: f64,
    pub min_dt_s: f64,
}

impl Default for PositionFilterConfig {
    fn default() -> Self {
        Self {
            process_noise: PositionFilterProcessNoise::default(),
            weighting: WeightingConfig::default(),
            base_pseudorange_sigma_m: 5.0,
            gating_chi2_code: Some(100.0),
            huber_k: Some(30.0),
            apply_broadcast_group_delay: true,
            initial_position_sigma_m: 100.0,
            initial_velocity_sigma_mps: 50.0,
            initial_clock_bias_sigma_s: 1.0e-3,
            initial_clock_drift_sigma_s_per_s: 1.0e-4,
            min_dt_s: 1.0e-3,
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
    pub clock_drift_s_per_s: f64,
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
                gating_chi2_doppler: None,
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

    pub(crate) fn process_noise_config(&self) -> ProcessNoiseConfig {
        ProcessNoiseConfig {
            pos_m: self.config.process_noise.pos_m,
            vel_mps: self.config.process_noise.vel_mps,
            clock_bias_s: self.config.process_noise.clock_bias_s,
            clock_drift_s: self.config.process_noise.clock_drift_s_per_s,
            ztd_m: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PositionFilter, PositionFilterConfig};

    #[test]
    fn position_filter_uses_eight_state_layout() {
        let filter = PositionFilter::new(PositionFilterConfig::default());

        assert_eq!(filter.ekf.x.len(), 8);
        assert_eq!(filter.indices.pos, [0, 1, 2]);
        assert_eq!(filter.indices.vel, [3, 4, 5]);
        assert_eq!(filter.indices.clock_bias, 6);
        assert_eq!(filter.indices.clock_drift, 7);
        assert!(!filter.initialized);
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
}
