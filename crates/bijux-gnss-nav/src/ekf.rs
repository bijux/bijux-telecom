use std::collections::BTreeMap;

use crate::linalg::Matrix;
use bijux_gnss_core::NavHealthEvent;
use bijux_gnss_core::SigId;

#[derive(Debug, Clone)]
pub struct EkfConfig {
    pub gating_chi2_code: Option<f64>,
    pub gating_chi2_phase: Option<f64>,
    pub gating_chi2_doppler: Option<f64>,
    pub huber_k: Option<f64>,
    pub square_root: bool,
    pub covariance_epsilon: f64,
    pub divergence_max_variance: f64,
}

#[derive(Debug, Clone)]
pub struct EkfHealth {
    pub innovation_rms: f64,
    pub rejected: usize,
    pub last_rejection: Option<String>,
    pub rejection_reasons: Vec<String>,
    pub last_rejection_code: Option<RejectionReason>,
    pub condition_number: Option<f64>,
    pub whiteness_ratio: Option<f64>,
    pub predicted_variance: Option<f64>,
    pub observed_variance: Option<f64>,
    pub events: Vec<NavHealthEvent>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionReason {
    SingularS,
    Chi2Gate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementKind {
    Code,
    Doppler,
    Phase,
}

pub trait StateModel {
    fn state_dim(&self) -> usize;
    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64);
}

pub trait MeasurementModel {
    fn name(&self) -> &'static str;
    fn kind(&self) -> MeasurementKind;
    fn measurement_dim(&self) -> usize;
    fn observation(&self) -> &[f64];
    fn h(&self, x: &[f64], out: &mut [f64]);
    fn jacobian(&self, x: &[f64], h: &mut Matrix);
    fn covariance(&self, x: &[f64], r: &mut Matrix);
}

#[derive(Debug, Clone)]
pub struct Ekf {
    pub x: Vec<f64>,
    pub p: Matrix,
    pub config: EkfConfig,
    pub health: EkfHealth,
    pub labels: Vec<String>,
}

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

    pub(crate) fn sanitize_covariance(&mut self) {
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

#[derive(Debug, Clone)]
pub struct ProcessNoiseConfig {
    pub pos_m: f64,
    pub vel_mps: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
    pub ztd_m: f64,
}

#[derive(Debug, Clone)]
pub struct NavClockModel {
    pub noise: ProcessNoiseConfig,
}

impl NavClockModel {
    pub fn new(noise: ProcessNoiseConfig) -> Self {
        Self { noise }
    }
}

impl StateModel for NavClockModel {
    fn state_dim(&self) -> usize {
        8
    }

    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64) {
        if x.len() < 8 {
            return;
        }
        x[0] += x[3] * dt_s;
        x[1] += x[4] * dt_s;
        x[2] += x[5] * dt_s;
        x[6] += x[7] * dt_s;

        let mut f = Matrix::identity(8);
        f[(0, 3)] = dt_s;
        f[(1, 4)] = dt_s;
        f[(2, 5)] = dt_s;
        f[(6, 7)] = dt_s;

        let mut q = Matrix::new(8, 8, 0.0);
        q[(0, 0)] = self.noise.pos_m * self.noise.pos_m;
        q[(1, 1)] = self.noise.pos_m * self.noise.pos_m;
        q[(2, 2)] = self.noise.pos_m * self.noise.pos_m;
        q[(3, 3)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(4, 4)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(5, 5)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(6, 6)] = self.noise.clock_bias_s * self.noise.clock_bias_s;
        q[(7, 7)] = self.noise.clock_drift_s * self.noise.clock_drift_s;

        if x.len() > 8 && self.noise.ztd_m > 0.0 {
            let idx = 8;
            if idx < q.rows() {
                q[(idx, idx)] = self.noise.ztd_m * self.noise.ztd_m;
            }
        }

        let ft = f.transpose();
        let p_new = f.mul(p).mul(&ft).add(&q);
        *p = p_new;
    }
}

#[derive(Debug, Clone)]
pub struct PseudorangeMeasurement {
    pub sig: SigId,
    pub z_m: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub tropo_m: f64,
    pub iono_m: f64,
    pub sigma_m: f64,
    pub elevation_deg: Option<f64>,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
}

impl MeasurementModel for PseudorangeMeasurement {
    fn name(&self) -> &'static str {
        "pseudorange"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Code
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_m)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = self.tropo_m;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if let Some(ztd) = x.get(idx) {
                    tropo = ztd * m;
                }
            }
        }
        let mut pred = range + 299_792_458.0 * (x[6] - self.sat_clock_s) + tropo - self.iono_m;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += 299_792_458.0 * isb;
            }
        }
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        h[(0, 0)] = dx / range;
        h[(0, 1)] = dy / range;
        h[(0, 2)] = dz / range;
        h[(0, 6)] = 299_792_458.0;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if idx < h.cols() {
                    h[(0, idx)] = m;
                }
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = 299_792_458.0;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
pub struct DopplerMeasurement {
    pub sig: SigId,
    pub z_hz: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_vel_mps: [f64; 3],
    pub wavelength_m: f64,
    pub sigma_hz: f64,
}

impl MeasurementModel for DopplerMeasurement {
    fn name(&self) -> &'static str {
        "doppler"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Doppler
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_hz)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let los = [dx / range, dy / range, dz / range];
        let rel_vel = [
            x[3] - self.sat_vel_mps[0],
            x[4] - self.sat_vel_mps[1],
            x[5] - self.sat_vel_mps[2],
        ];
        let range_rate = los[0] * rel_vel[0] + los[1] * rel_vel[1] + los[2] * rel_vel[2];
        let pred = -range_rate / self.wavelength_m + 299_792_458.0 * x[7] / self.wavelength_m;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let los = [dx / range, dy / range, dz / range];
        h[(0, 3)] = -los[0] / self.wavelength_m;
        h[(0, 4)] = -los[1] / self.wavelength_m;
        h[(0, 5)] = -los[2] / self.wavelength_m;
        h[(0, 7)] = 299_792_458.0 / self.wavelength_m;
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_hz * self.sigma_hz;
    }
}

#[derive(Debug, Clone)]
pub struct CarrierPhaseMeasurement {
    pub sig: SigId,
    pub z_cycles: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub tropo_m: f64,
    pub iono_m: f64,
    pub wavelength_m: f64,
    pub ambiguity_index: Option<usize>,
    pub sigma_cycles: f64,
    pub elevation_deg: Option<f64>,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
}

impl MeasurementModel for CarrierPhaseMeasurement {
    fn name(&self) -> &'static str {
        "carrier_phase"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Phase
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z_cycles)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let n = self
            .ambiguity_index
            .and_then(|idx| x.get(idx).copied())
            .unwrap_or(0.0);
        let mut tropo = self.tropo_m;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if let Some(ztd) = x.get(idx) {
                    tropo = ztd * m;
                }
            }
        }
        let mut pred = (range + 299_792_458.0 * (x[6] - self.sat_clock_s) + tropo - self.iono_m)
            / self.wavelength_m
            + n;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += 299_792_458.0 * isb / self.wavelength_m;
            }
        }
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        h[(0, 0)] = (dx / range) / self.wavelength_m;
        h[(0, 1)] = (dy / range) / self.wavelength_m;
        h[(0, 2)] = (dz / range) / self.wavelength_m;
        h[(0, 6)] = 299_792_458.0 / self.wavelength_m;
        if let Some(idx) = self.ztd_index {
            if let Some(elev) = self.elevation_deg {
                let m = 1.0 / elev.to_radians().sin().max(0.1);
                if idx < h.cols() {
                    h[(0, idx)] = m / self.wavelength_m;
                }
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = 299_792_458.0 / self.wavelength_m;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            h[(0, idx)] = 1.0;
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

#[derive(Debug, Clone)]
pub struct AmbiguityManager {
    pub indices: BTreeMap<String, usize>,
}

impl AmbiguityManager {
    pub fn new() -> Self {
        Self {
            indices: BTreeMap::new(),
        }
    }

    pub fn get_or_add(&mut self, ekf: &mut Ekf, key: &str, value: f64, variance: f64) -> usize {
        if let Some(idx) = self.indices.get(key) {
            return *idx;
        }
        let idx = ekf.x.len();
        ekf.add_state(key, value, variance);
        self.indices.insert(key.to_string(), idx);
        idx
    }

    pub fn apply_fix_hold(&self, ekf: &mut Ekf, indices: &[usize], variance: f64) {
        for &idx in indices {
            if idx >= ekf.x.len() {
                continue;
            }
            let z = ekf.x[idx].round();
            let sigma = variance.max(1e-6).sqrt();
            let meas = FixHoldMeasurement {
                index: idx,
                z,
                sigma,
            };
            let _ = ekf.update(&meas);
            let v = variance.max(1e-6);
            ekf.p[(idx, idx)] = v;
            for j in 0..ekf.x.len() {
                if j == idx {
                    continue;
                }
                ekf.p[(idx, j)] *= 0.2;
                ekf.p[(j, idx)] = ekf.p[(idx, j)];
            }
        }
        ekf.sanitize_covariance();
    }
}

impl Default for AmbiguityManager {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct InterSystemBiasManager {
    pub indices: BTreeMap<String, usize>,
}

impl InterSystemBiasManager {
    pub fn new() -> Self {
        Self {
            indices: BTreeMap::new(),
        }
    }

    pub fn get_or_add(&mut self, ekf: &mut Ekf, key: &str, value: f64, variance: f64) -> usize {
        if let Some(idx) = self.indices.get(key) {
            return *idx;
        }
        let idx = ekf.x.len();
        ekf.add_state(key, value, variance);
        self.indices.insert(key.to_string(), idx);
        idx
    }
}

impl Default for InterSystemBiasManager {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
struct FixHoldMeasurement {
    index: usize,
    z: f64,
    sigma: f64,
}

impl MeasurementModel for FixHoldMeasurement {
    fn name(&self) -> &'static str {
        "fix_hold"
    }

    fn kind(&self) -> MeasurementKind {
        MeasurementKind::Phase
    }

    fn measurement_dim(&self) -> usize {
        1
    }

    fn observation(&self) -> &[f64] {
        std::slice::from_ref(&self.z)
    }

    fn h(&self, x: &[f64], out: &mut [f64]) {
        out[0] = x.get(self.index).copied().unwrap_or(0.0);
    }

    fn jacobian(&self, _x: &[f64], h: &mut Matrix) {
        h[(0, self.index)] = 1.0;
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma * self.sigma;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::{Constellation, SatId, SignalBand};

    #[test]
    fn matrix_inversion_identity() {
        let m = Matrix::identity(3);
        let inv = m.invert().expect("invert");
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv[(i, j)] - expected).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn ekf_update_runs() {
        let x = vec![0.0; 8];
        let p = Matrix::identity(8);
        let mut ekf = Ekf::new(
            x,
            p,
            EkfConfig {
                gating_chi2_code: Some(100.0),
                gating_chi2_phase: Some(100.0),
                gating_chi2_doppler: Some(100.0),
                huber_k: Some(10.0),
                square_root: true,
                covariance_epsilon: 1e-6,
                divergence_max_variance: 1e12,
            },
        );
        let meas = PseudorangeMeasurement {
            sig: SigId {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn: 1,
                },
                band: SignalBand::L1,
            },
            z_m: 20_200_000.0,
            sat_pos_m: [15_000_000.0, 0.0, 21_000_000.0],
            sat_clock_s: 0.0,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m: 10.0,
            elevation_deg: None,
            ztd_index: None,
            isb_index: None,
        };
        assert!(ekf.update(&meas));
    }
}
