#![allow(missing_docs)]

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId, SignalBand};

use crate::corrections::Corrections;
use crate::estimation::ekf::state::MeasurementKind;
use crate::estimation::ekf::traits::MeasurementModel;
use crate::linalg::Matrix;

use super::config::SPEED_OF_LIGHT_MPS;
use super::models::PppCodeMeasurement;

#[derive(Debug, Clone, Copy)]
pub struct IonoFreeObservation {
    pub code_m: f64,
    pub phase_m: f64,
    pub f1_hz: f64,
    pub f2_hz: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct WideLaneObservation {
    pub cycles: f64,
    pub variance: f64,
}

#[derive(Clone, Copy)]
struct DualFrequencyObservationPair<'a> {
    first: &'a ObsSatellite,
    second: &'a ObsSatellite,
}

impl MeasurementModel for PppCodeMeasurement {
    fn name(&self) -> &'static str {
        "ppp_code"
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
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v;
            }
        }
        let mut pred = range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo - iono;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb;
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = -1.0;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
pub struct PppPhaseMeasurement {
    pub z_cycles: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub sigma_cycles: f64,
    pub iono_index: Option<usize>,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
    pub ambiguity_index: Option<usize>,
    pub corr: Corrections,
    pub wavelength_m: f64,
}

impl MeasurementModel for PppPhaseMeasurement {
    fn name(&self) -> &'static str {
        "ppp_phase"
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
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v;
            }
        }
        let mut pred = (range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo - iono)
            / self.wavelength_m;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb / self.wavelength_m;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if let Some(n) = x.get(idx) {
                pred += *n;
            }
        }
        pred += self.corr.phase_windup_cycles;
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS / self.wavelength_m;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0 / self.wavelength_m;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = -1.0 / self.wavelength_m;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS / self.wavelength_m;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

#[derive(Debug, Clone)]
pub struct PppIonoFreeCodeMeasurement {
    pub z_m: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub sigma_m: f64,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
    pub corr: Corrections,
}

impl MeasurementModel for PppIonoFreeCodeMeasurement {
    fn name(&self) -> &'static str {
        "ppp_if_code"
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
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut tropo = 0.0;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                tropo = *ztd;
            }
        }
        let mut pred = range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo;
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb;
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
        h[(0, 6)] = SPEED_OF_LIGHT_MPS;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_m * self.sigma_m;
    }
}

#[derive(Debug, Clone)]
pub struct PppIonoFreePhaseMeasurement {
    pub z_cycles: f64,
    pub sat_pos_m: [f64; 3],
    pub sat_clock_s: f64,
    pub sigma_cycles: f64,
    pub ztd_index: Option<usize>,
    pub isb_index: Option<usize>,
    pub ambiguity_index: Option<usize>,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub corr: Corrections,
}

impl MeasurementModel for PppIonoFreePhaseMeasurement {
    fn name(&self) -> &'static str {
        "ppp_if_phase"
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
        let rx = [
            x[0] + self.corr.earth_tide_m[0],
            x[1] + self.corr.earth_tide_m[1],
            x[2] + self.corr.earth_tide_m[2],
        ];
        let dx = rx[0] - self.sat_pos_m[0];
        let dy = rx[1] - self.sat_pos_m[1];
        let dz = rx[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let lambda_if = SPEED_OF_LIGHT_MPS
            / ((self.f1_hz * self.f1_hz - self.f2_hz * self.f2_hz) / (self.f1_hz - self.f2_hz));
        let mut pred = (range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s)) / lambda_if;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                pred += *ztd / lambda_if;
            }
        }
        if let Some(idx) = self.isb_index {
            if let Some(isb) = x.get(idx) {
                pred += SPEED_OF_LIGHT_MPS * isb / lambda_if;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if let Some(n) = x.get(idx) {
                pred += *n;
            }
        }
        pred += self.corr.phase_windup_cycles;
        out[0] = pred;
    }

    fn jacobian(&self, x: &[f64], h: &mut Matrix) {
        let dx = x[0] - self.sat_pos_m[0];
        let dy = x[1] - self.sat_pos_m[1];
        let dz = x[2] - self.sat_pos_m[2];
        let range = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let lambda_if = SPEED_OF_LIGHT_MPS
            / ((self.f1_hz * self.f1_hz - self.f2_hz * self.f2_hz) / (self.f1_hz - self.f2_hz));
        h[(0, 0)] = (dx / range) / lambda_if;
        h[(0, 1)] = (dy / range) / lambda_if;
        h[(0, 2)] = (dz / range) / lambda_if;
        h[(0, 6)] = SPEED_OF_LIGHT_MPS / lambda_if;
        if let Some(idx) = self.ztd_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0 / lambda_if;
            }
        }
        if let Some(idx) = self.isb_index {
            if idx < h.cols() {
                h[(0, idx)] = SPEED_OF_LIGHT_MPS / lambda_if;
            }
        }
        if let Some(idx) = self.ambiguity_index {
            if idx < h.cols() {
                h[(0, idx)] = 1.0;
            }
        }
    }

    fn covariance(&self, _x: &[f64], r: &mut Matrix) {
        r[(0, 0)] = self.sigma_cycles * self.sigma_cycles;
    }
}

pub fn iono_free_from_obs(obs: &ObsEpoch, sat: SatId) -> Option<IonoFreeObservation> {
    let pair = select_dual_frequency_pair(obs, sat)?;
    let f1 = pair.first.metadata.signal.carrier_hz.value();
    let f2 = pair.second.metadata.signal.carrier_hz.value();
    let f1_2 = f1 * f1;
    let f2_2 = f2 * f2;
    let denom = (f1_2 - f2_2).max(1.0);
    let if_code = (f1_2 * pair.first.pseudorange_m.0 - f2_2 * pair.second.pseudorange_m.0) / denom;
    let lambda1 = SPEED_OF_LIGHT_MPS / f1;
    let lambda2 = SPEED_OF_LIGHT_MPS / f2;
    let phi1_m = pair.first.carrier_phase_cycles.0 * lambda1;
    let phi2_m = pair.second.carrier_phase_cycles.0 * lambda2;
    let if_phase = (f1_2 * phi1_m - f2_2 * phi2_m) / denom;
    Some(IonoFreeObservation { code_m: if_code, phase_m: if_phase, f1_hz: f1, f2_hz: f2 })
}

pub fn wide_lane_from_obs(obs: &ObsEpoch, sat: SatId) -> Option<WideLaneObservation> {
    let pair = select_dual_frequency_pair(obs, sat)?;
    let f1 = pair.first.metadata.signal.carrier_hz.value();
    let f2 = pair.second.metadata.signal.carrier_hz.value();
    let lambda1 = SPEED_OF_LIGHT_MPS / f1;
    let lambda2 = SPEED_OF_LIGHT_MPS / f2;
    let phi1_m = pair.first.carrier_phase_cycles.0 * lambda1;
    let phi2_m = pair.second.carrier_phase_cycles.0 * lambda2;
    let lambda_wl = SPEED_OF_LIGHT_MPS / (f1 - f2).abs().max(1.0);
    let wl_cycles = (phi1_m - phi2_m) / lambda_wl;
    let variance = pair.first.carrier_phase_var_cycles2 + pair.second.carrier_phase_var_cycles2;
    Some(WideLaneObservation { cycles: wl_cycles, variance })
}

pub fn ratio_fix(float: f64, variance: f64) -> (f64, i64) {
    let n0 = float.round() as i64;
    let n1 = if float > n0 as f64 { n0 + 1 } else { n0 - 1 };
    let cost0 = ((float - n0 as f64).powi(2)) / variance.max(1e-6);
    let cost1 = ((float - n1 as f64).powi(2)) / variance.max(1e-6);
    let ratio = if cost0 > 0.0 { cost1 / cost0 } else { 0.0 };
    (ratio, n0)
}

fn select_dual_frequency_pair(
    obs: &ObsEpoch,
    sat: SatId,
) -> Option<DualFrequencyObservationPair<'_>> {
    for (first_band, second_band) in preferred_dual_frequency_pairs(sat) {
        let first = obs.sats.iter().find(|observation| {
            observation.signal_id.sat == sat && observation.signal_id.band == *first_band
        });
        let second = obs.sats.iter().find(|observation| {
            observation.signal_id.sat == sat && observation.signal_id.band == *second_band
        });
        if let (Some(first), Some(second)) = (first, second) {
            return Some(DualFrequencyObservationPair { first, second });
        }
    }
    None
}

fn preferred_dual_frequency_pairs(sat: SatId) -> &'static [(SignalBand, SignalBand)] {
    match sat.constellation {
        bijux_gnss_core::api::Constellation::Gps => &[(SignalBand::L1, SignalBand::L2)],
        bijux_gnss_core::api::Constellation::Galileo => &[(SignalBand::E1, SignalBand::E5)],
        _ => &[],
    }
}
