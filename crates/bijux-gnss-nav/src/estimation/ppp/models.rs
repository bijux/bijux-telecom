#![allow(missing_docs)]

use super::config::PppProcessNoise;
use crate::corrections::Corrections;
use crate::estimation::ekf::traits::StateModel;
use crate::linalg::Matrix;

pub struct PppProcessModel {
    pub pos: [usize; 3],
    pub vel: [usize; 3],
    pub clock_bias: usize,
    pub clock_drift: usize,
    pub ztd: usize,
    pub inter_system_biases: Vec<usize>,
    pub ionospheres: Vec<usize>,
    pub ambiguities: Vec<usize>,
    pub process: PppProcessNoise,
}

impl StateModel for PppProcessModel {
    fn state_dim(&self) -> usize {
        9
    }

    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64) {
        for i in 0..3 {
            let pos = self.pos[i];
            let vel = self.vel[i];
            if vel < x.len() && pos < x.len() {
                x[pos] += x[vel] * dt_s;
            }
        }
        if self.clock_bias < x.len() && self.clock_drift < x.len() {
            x[self.clock_bias] += x[self.clock_drift] * dt_s;
        }

        let mut q = Matrix::new(p.rows(), p.cols(), 0.0);
        let dt_scale = dt_s.max(1.0e-3);
        for index in self.pos {
            add_process_variance(&mut q, index, self.process.position_m, dt_scale);
        }
        for index in self.vel {
            add_process_variance(&mut q, index, self.process.velocity_mps, dt_scale);
        }
        add_process_variance(&mut q, self.clock_bias, self.process.clock_bias_s, dt_scale);
        add_process_variance(&mut q, self.clock_drift, self.process.clock_drift_s, dt_scale);
        add_process_variance(&mut q, self.ztd, self.process.ztd_m, dt_scale);
        for index in &self.inter_system_biases {
            add_process_variance(&mut q, *index, self.process.inter_system_bias_s, dt_scale);
        }
        for index in &self.ionospheres {
            add_process_variance(&mut q, *index, self.process.iono_m, dt_scale);
        }
        for index in &self.ambiguities {
            add_process_variance(&mut q, *index, self.process.ambiguity_cycles, dt_scale);
        }
        let f = Matrix::identity(p.rows());
        *p = f.mul(p).mul(&f.transpose()).add(&q);
    }
}

fn add_process_variance(q: &mut Matrix, index: usize, sigma: f64, dt_scale: f64) {
    if index < q.rows() && sigma.is_finite() && sigma >= 0.0 {
        q[(index, index)] = sigma.powi(2) * dt_scale;
    }
}

#[derive(Debug, Clone)]
pub struct PppCodeMeasurement {
    pub z_m: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub antenna_range_correction_m: f64,
    pub sigma_m: f64,
    pub troposphere_mapping: f64,
    pub ionosphere_scale: f64,
    pub iono_index: Option<usize>,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
    pub corr: Corrections,
}

#[cfg(test)]
mod tests {
    use super::{PppProcessModel, PppProcessNoise};
    use crate::estimation::ekf::traits::StateModel;
    use crate::linalg::Matrix;

    fn process_noise() -> PppProcessNoise {
        PppProcessNoise {
            position_m: 0.2,
            velocity_mps: 0.03,
            clock_bias_s: 1.0e-7,
            clock_drift_s: 2.0e-8,
            inter_system_bias_s: 3.0e-9,
            ztd_m: 0.04,
            iono_m: 0.5,
            ambiguity_cycles: 0.07,
        }
    }

    #[test]
    fn ppp_process_model_uses_declared_dynamic_state_noise() {
        let model = PppProcessModel {
            pos: [0, 1, 2],
            vel: [3, 4, 5],
            clock_bias: 6,
            clock_drift: 7,
            ztd: 8,
            inter_system_biases: vec![9],
            ionospheres: vec![10],
            ambiguities: vec![11],
            process: process_noise(),
        };
        let mut state = vec![0.0; 12];
        let mut covariance = Matrix::identity(12);

        model.propagate(&mut state, &mut covariance, 4.0);

        assert!((covariance[(9, 9)] - (1.0 + (3.0e-9_f64).powi(2) * 4.0)).abs() < 1.0e-18);
        assert!((covariance[(10, 10)] - (1.0 + 0.5_f64.powi(2) * 4.0)).abs() < 1.0e-12);
        assert!((covariance[(11, 11)] - (1.0 + 0.07_f64.powi(2) * 4.0)).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_process_model_does_not_add_unconfigured_blanket_floor() {
        let model = PppProcessModel {
            pos: [0, 1, 2],
            vel: [3, 4, 5],
            clock_bias: 6,
            clock_drift: 7,
            ztd: 8,
            inter_system_biases: Vec::new(),
            ionospheres: Vec::new(),
            ambiguities: Vec::new(),
            process: PppProcessNoise {
                position_m: 0.0,
                velocity_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s: 0.0,
                inter_system_bias_s: 0.0,
                ztd_m: 0.0,
                iono_m: 0.0,
                ambiguity_cycles: 0.0,
            },
        };
        let mut state = vec![0.0; 9];
        let mut covariance = Matrix::identity(9);

        model.propagate(&mut state, &mut covariance, 10.0);

        for index in 0..9 {
            assert_eq!(covariance[(index, index)], 1.0);
        }
    }
}
