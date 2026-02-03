use super::config::PppProcessNoise;
use crate::estimation::ekf::StateModel;
use crate::{Corrections, Matrix};

pub(crate) struct PppProcessModel {
    pub(crate) pos: [usize; 3],
    pub(crate) vel: [usize; 3],
    pub(crate) clock_bias: usize,
    pub(crate) clock_drift: usize,
    pub(crate) ztd: usize,
    pub(crate) process: PppProcessNoise,
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
pub(crate) struct PppCodeMeasurement {
    pub(crate) z_m: f64,
    pub(crate) sat_pos_m: [f64; 3],
    pub(crate) sat_clock_s: f64,
    pub(crate) sigma_m: f64,
    pub(crate) iono_index: Option<usize>,
    pub(crate) ztd_index: Option<usize>,
    pub(crate) isb_index: Option<usize>,
    pub(crate) corr: Corrections,
}
