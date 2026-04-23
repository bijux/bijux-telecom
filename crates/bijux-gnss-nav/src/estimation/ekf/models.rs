#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::SigId;

use super::state::{Ekf, MeasurementKind};
use super::traits::{MeasurementModel, StateModel};
use crate::linalg::Matrix;

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
        let rel_vel =
            [x[3] - self.sat_vel_mps[0], x[4] - self.sat_vel_mps[1], x[5] - self.sat_vel_mps[2]];
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
        let n = self.ambiguity_index.and_then(|idx| x.get(idx).copied()).unwrap_or(0.0);
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
        Self { indices: BTreeMap::new() }
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
            let meas = FixHoldMeasurement { index: idx, z, sigma };
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
        Self { indices: BTreeMap::new() }
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
