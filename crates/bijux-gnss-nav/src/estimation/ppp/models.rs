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
        if self.clock_drift < q.rows() {
            q[(self.clock_drift, self.clock_drift)] = self.process.clock_drift_s.powi(2);
        }
        if self.ztd < q.rows() {
            q[(self.ztd, self.ztd)] = self.process.ztd_m.powi(2);
        }
        for i in 0..q.rows() {
            if q[(i, i)] == 0.0 {
                q[(i, i)] = 1e-6;
            }
        }
        let f = Matrix::identity(p.rows());
        *p = f.mul(p).mul(&f.transpose()).add(&q);
    }
}

#[derive(Debug, Clone)]
pub struct PppCodeMeasurement {
    pub z_m: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub sigma_m: f64,
    pub iono_index: Option<usize>,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
    pub corr: Corrections,
}
