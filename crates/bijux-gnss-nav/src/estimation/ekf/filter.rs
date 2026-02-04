#![allow(missing_docs)]

use super::state::{Ekf, EkfCheckpoint, EkfConfig, EkfHealth, MeasurementKind, RejectionReason};
use super::traits::{MeasurementModel, StateModel};
use crate::linalg::Matrix;
use bijux_gnss_core::api::NavHealthEvent;

impl Ekf {
    pub fn new(x: Vec<f64>, p: Matrix, config: EkfConfig) -> Self {
        Self {
            x,
            p,
            config,
            health: EkfHealth {
                innovation_rms: 0.0,
                rejected: 0,
                last_rejection: None,
                rejection_reasons: Vec::new(),
                last_rejection_code: None,
                condition_number: None,
                whiteness_ratio: None,
                predicted_variance: None,
                observed_variance: None,
                events: Vec::new(),
            },
            labels: Vec::new(),
        }
    }

    pub fn checkpoint(&self) -> EkfCheckpoint {
        EkfCheckpoint {
            x: self.x.clone(),
            rows: self.p.rows(),
            cols: self.p.cols(),
            data: self.p.data().to_vec(),
            labels: self.labels.clone(),
        }
    }

    pub fn restore(checkpoint: EkfCheckpoint, config: EkfConfig) -> Self {
        Self {
            x: checkpoint.x,
            p: Matrix::from_parts(checkpoint.rows, checkpoint.cols, checkpoint.data),
            config,
            health: EkfHealth {
                innovation_rms: 0.0,
                rejected: 0,
                last_rejection: None,
                rejection_reasons: Vec::new(),
                last_rejection_code: None,
                condition_number: None,
                whiteness_ratio: None,
                predicted_variance: None,
                observed_variance: None,
                events: Vec::new(),
            },
            labels: checkpoint.labels,
        }
    }

    pub fn add_state(&mut self, label: &str, value: f64, variance: f64) {
        let n = self.x.len();
        self.x.push(value);
        let mut p_new = Matrix::new(n + 1, n + 1, 0.0);
        for r in 0..n {
            for c in 0..n {
                p_new[(r, c)] = self.p[(r, c)];
            }
        }
        p_new[(n, n)] = variance;
        self.p = p_new;
        self.labels.push(label.to_string());
    }

    pub fn predict<M: StateModel>(&mut self, model: &M, dt_s: f64) {
        model.propagate(&mut self.x, &mut self.p, dt_s);
        self.sanitize_covariance();
    }

    pub fn update<M: MeasurementModel>(&mut self, model: &M) -> bool {
        let m = model.measurement_dim();
        let n = self.x.len();
        let mut h_pred = vec![0.0; m];
        model.h(&self.x, &mut h_pred);
        let z = model.observation();
        let mut y = vec![0.0; m];
        for i in 0..m {
            y[i] = z[i] - h_pred[i];
        }

        let mut h = Matrix::new(m, n, 0.0);
        model.jacobian(&self.x, &mut h);
        let mut r = Matrix::new(m, m, 0.0);
        model.covariance(&self.x, &mut r);

        let ht = h.transpose();
        let s = h.mul(&self.p).mul(&ht).add(&r);
        let mut max_diag = 0.0;
        let mut min_diag = f64::MAX;
        for i in 0..m {
            let v = s[(i, i)].abs();
            if v > max_diag {
                max_diag = v;
            }
            if v < min_diag {
                min_diag = v;
            }
        }
        if min_diag > 0.0 {
            self.health.condition_number = Some(max_diag / min_diag);
        }
        let Some(s_inv) = s.invert() else {
            self.health.rejected += 1;
            let reason = format!("{}: singular S", model.name());
            self.health.last_rejection = Some(reason.clone());
            self.health.rejection_reasons.push(reason.clone());
            self.health.last_rejection_code = Some(RejectionReason::SingularS);
            self.health
                .events
                .push(NavHealthEvent::InnovationRejected { reason });
            return false;
        };

        let chi2_gate = match model.kind() {
            MeasurementKind::Code => self.config.gating_chi2_code,
            MeasurementKind::Doppler => self.config.gating_chi2_doppler,
            MeasurementKind::Phase => self.config.gating_chi2_phase,
        };
        if let Some(chi2) = chi2_gate {
            let mut chi = 0.0;
            for i in 0..m {
                for j in 0..m {
                    chi += y[i] * s_inv[(i, j)] * y[j];
                }
            }
            if chi > chi2 {
                self.health.rejected += 1;
                let reason = format!("{}: chi2 gate", model.name());
                self.health.last_rejection = Some(reason.clone());
                self.health.rejection_reasons.push(reason.clone());
                self.health.last_rejection_code = Some(RejectionReason::Chi2Gate);
                self.health
                    .events
                    .push(NavHealthEvent::InnovationRejected { reason });
                return false;
            }
        }

        if let Some(k) = self.config.huber_k {
            for val in y.iter_mut().take(m) {
                let a = val.abs();
                if a > k {
                    *val *= k / a;
                }
            }
        }

        let k_gain = self.p.mul(&ht).mul(&s_inv);
        for i in 0..n {
            let mut dx = 0.0;
            for j in 0..m {
                dx += k_gain[(i, j)] * y[j];
            }
            self.x[i] += dx;
        }

        let i_mat = Matrix::identity(n);
        let kh = k_gain.mul(&h);
        let p_new = i_mat.sub(&kh).mul(&self.p);
        self.p = p_new;
        self.sanitize_covariance();

        let rms = if m > 0 {
            (y.iter().map(|v| v * v).sum::<f64>() / m as f64).sqrt()
        } else {
            0.0
        };
        self.health.innovation_rms = rms;
        let predicted = if m > 0 {
            let mut sum = 0.0;
            for i in 0..m {
                sum += s[(i, i)];
            }
            sum / m as f64
        } else {
            0.0
        };
        let observed = if m > 0 {
            y.iter().map(|v| v * v).sum::<f64>() / m as f64
        } else {
            0.0
        };
        self.health.predicted_variance = Some(predicted);
        self.health.observed_variance = Some(observed);
        if predicted > 0.0 {
            self.health.whiteness_ratio = Some(observed / predicted);
        }
        true
    }

    pub fn sanitize_covariance(&mut self) {
        let n = self.p.rows();
        for r in 0..n {
            for c in (r + 1)..n {
                let sym = 0.5 * (self.p[(r, c)] + self.p[(c, r)]);
                if (self.p[(r, c)] - self.p[(c, r)]).abs() > 1e-12 {
                    self.health
                        .events
                        .push(NavHealthEvent::CovarianceSymmetrized);
                }
                self.p[(r, c)] = sym;
                self.p[(c, r)] = sym;
            }
        }

        let mut max_var: f64 = 0.0;
        for i in 0..n {
            if self.p[(i, i)] < self.config.covariance_epsilon {
                self.p[(i, i)] = self.config.covariance_epsilon;
                self.health.events.push(NavHealthEvent::CovarianceClamped {
                    min_eigenvalue: self.config.covariance_epsilon,
                });
            }
            max_var = max_var.max(self.p[(i, i)].abs());
        }
        if max_var > self.config.divergence_max_variance {
            self.health.events.push(NavHealthEvent::CovarianceDiverged {
                max_variance: max_var,
            });
        }

        if self.config.square_root {
            if let Some(l) = self.p.cholesky() {
                let lt = l.transpose();
                self.p = l.mul(&lt);
            } else {
                self.health.events.push(NavHealthEvent::CovarianceClamped {
                    min_eigenvalue: self.config.covariance_epsilon,
                });
            }
        }
    }
}
