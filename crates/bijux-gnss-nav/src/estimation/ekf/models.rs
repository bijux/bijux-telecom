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
        let n = x.len();
        if n < 8 {
            return;
        }
        x[0] += x[3] * dt_s;
        x[1] += x[4] * dt_s;
        x[2] += x[5] * dt_s;
        x[6] += x[7] * dt_s;

        let mut f = Matrix::identity(n);
        f[(0, 3)] = dt_s;
        f[(1, 4)] = dt_s;
        f[(2, 5)] = dt_s;
        f[(6, 7)] = dt_s;

        let mut q = Matrix::new(n, n, 0.0);
        q[(0, 0)] = self.noise.pos_m * self.noise.pos_m;
        q[(1, 1)] = self.noise.pos_m * self.noise.pos_m;
        q[(2, 2)] = self.noise.pos_m * self.noise.pos_m;
        q[(3, 3)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(4, 4)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(5, 5)] = self.noise.vel_mps * self.noise.vel_mps;
        q[(6, 6)] = self.noise.clock_bias_s * self.noise.clock_bias_s;
        q[(7, 7)] = self.noise.clock_drift_s * self.noise.clock_drift_s;

        if n > 8 && self.noise.ztd_m > 0.0 {
            let idx = 8;
            q[(idx, idx)] = self.noise.ztd_m * self.noise.ztd_m;
        }

        let ft = f.transpose();
        let p_new = f.mul(p).mul(&ft).add(&q);
        *p = p_new;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

    fn sample_sig_id() -> SigId {
        SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            code: SignalCode::Ca,
        }
    }

    #[test]
    fn nav_clock_model_propagates_extended_state_covariance() {
        let model = NavClockModel::new(ProcessNoiseConfig {
            pos_m: 1.0,
            vel_mps: 0.5,
            clock_bias_s: 1.0e-4,
            clock_drift_s: 1.0e-5,
            ztd_m: 0.25,
        });
        let mut x = vec![0.0_f64; 9];
        x[3] = 3.0;
        x[4] = 4.0;
        x[5] = 5.0;
        x[7] = 2.0e-4;
        x[8] = 2.3;
        let mut p = Matrix::identity(9);

        model.propagate(&mut x, &mut p, 2.0);

        assert_eq!(x[0], 6.0);
        assert_eq!(x[1], 8.0);
        assert_eq!(x[2], 10.0);
        assert_eq!(x[6], 4.0e-4);
        assert_eq!(x[8], 2.3);
        assert_eq!(p.rows(), 9);
        assert_eq!(p.cols(), 9);
        assert!(p[(8, 8)] > 1.0);
    }

    #[test]
    fn pseudorange_measurement_subtracts_satellite_clock_bias() {
        let measurement = PseudorangeMeasurement {
            sig: sample_sig_id(),
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, -1_500_000.0, 21_300_000.0],
            sat_clock_s: 240.0e-9,
            tropo_m: 3.2,
            iono_m: 1.4,
            sigma_m: 2.0,
            elevation_deg: Some(45.0),
            ztd_index: None,
            isb_index: None,
        };
        let state = [1_117_194.907, -4_842_953.615, 3_985_351.233, 0.0, 0.0, 0.0, 2.75e-4, 0.0];
        let mut corrected = [0.0];
        let mut zero_clock = [0.0];

        measurement.h(&state, &mut corrected);
        PseudorangeMeasurement { sat_clock_s: 0.0, ..measurement.clone() }
            .h(&state, &mut zero_clock);

        let expected_delta_m = 299_792_458.0 * measurement.sat_clock_s;

        assert!((zero_clock[0] - corrected[0] - expected_delta_m).abs() < 1.0e-6);
    }

    #[test]
    fn carrier_phase_measurement_subtracts_satellite_clock_bias() {
        let measurement = CarrierPhaseMeasurement {
            sig: sample_sig_id(),
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, -1_500_000.0, 21_300_000.0],
            sat_clock_s: -170.0e-9,
            tropo_m: 2.6,
            iono_m: 0.8,
            wavelength_m: 0.190_293_672_798_364_87,
            ambiguity_index: None,
            sigma_cycles: 0.05,
            elevation_deg: Some(45.0),
            ztd_index: None,
            isb_index: None,
        };
        let state = [1_117_194.907, -4_842_953.615, 3_985_351.233, 0.0, 0.0, 0.0, 2.75e-4, 0.0];
        let mut corrected = [0.0];
        let mut zero_clock = [0.0];

        measurement.h(&state, &mut corrected);
        CarrierPhaseMeasurement { sat_clock_s: 0.0, ..measurement.clone() }
            .h(&state, &mut zero_clock);

        let expected_delta_cycles =
            299_792_458.0 * measurement.sat_clock_s / measurement.wavelength_m;

        assert!((zero_clock[0] - corrected[0] - expected_delta_cycles).abs() < 1.0e-6);
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
