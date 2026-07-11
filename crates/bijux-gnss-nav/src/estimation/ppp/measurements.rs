#![allow(missing_docs)]

use bijux_gnss_core::api::{ObsEpoch, ObsSatellite, SatId, SignalBand};

use crate::corrections::iono_free_code::iono_free_code_from_pair;
use crate::corrections::iono_free_phase::iono_free_phase_from_pair;
use crate::corrections::Corrections;
use crate::estimation::ekf::state::MeasurementKind;
use crate::estimation::ekf::traits::MeasurementModel;
use crate::linalg::Matrix;

use super::config::SPEED_OF_LIGHT_MPS;
use super::models::PppCodeMeasurement;

#[derive(Debug, Clone, Copy)]
pub struct IonoFreeObservation {
    pub code_m: f64,
    pub phase_cycles: f64,
    pub code_sigma_m: f64,
    pub phase_sigma_cycles: f64,
    pub phase_wavelength_m: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
}

#[derive(Debug, Clone, Copy)]
pub struct WideLaneObservation {
    pub cycles: f64,
    pub variance: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
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
    pub wavelength_m: f64,
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
        let mut pred = (range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s)) / self.wavelength_m;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                pred += *ztd / self.wavelength_m;
            }
        }
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

pub fn iono_free_from_obs(obs: &ObsEpoch, sat: SatId) -> Option<IonoFreeObservation> {
    let pair = select_dual_frequency_pair(obs, sat)?;
    let code = iono_free_code_from_pair(
        obs.epoch_idx,
        obs.t_rx_s.0,
        sat,
        pair.first.signal_id.band,
        pair.second.signal_id.band,
        Some(pair.first),
        Some(pair.second),
    );
    let phase = iono_free_phase_from_pair(
        obs.epoch_idx,
        obs.t_rx_s.0,
        sat,
        pair.first.signal_id.band,
        pair.second.signal_id.band,
        Some(pair.first),
        Some(pair.second),
    );
    if code.status != "ok" || phase.status != "ok" {
        return None;
    }
    Some(IonoFreeObservation {
        code_m: code.code_m?,
        phase_cycles: phase.phase_cycles?,
        code_sigma_m: code.variance_m2?.sqrt(),
        phase_sigma_cycles: phase.variance_cycles2?.sqrt(),
        phase_wavelength_m: phase.narrow_lane_wavelength_m?,
        band_1: pair.first.signal_id.band,
        band_2: pair.second.signal_id.band,
    })
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
    Some(WideLaneObservation {
        cycles: wl_cycles,
        variance,
        band_1: pair.first.signal_id.band,
        band_2: pair.second.signal_id.band,
    })
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
        bijux_gnss_core::api::Constellation::Gps => {
            &[(SignalBand::L1, SignalBand::L2), (SignalBand::L1, SignalBand::L5)]
        }
        bijux_gnss_core::api::Constellation::Galileo => &[(SignalBand::E1, SignalBand::E5)],
        _ => &[],
    }
}

#[cfg(test)]
mod tests {
    use super::{iono_free_from_obs, wide_lane_from_obs, SPEED_OF_LIGHT_MPS};
    use bijux_gnss_core::api::{
        signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5, Constellation, Cycles,
        Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite, ObservationEpochDecision,
        ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, SigId, SignalCode,
    };

    fn make_gps_dual_frequency_epoch(
        second_band: bijux_gnss_core::api::SignalBand,
        second_code: SignalCode,
        second_signal: bijux_gnss_core::api::SignalSpec,
        pseudorange_1_m: f64,
        pseudorange_2_m: f64,
        phase_1_cycles: f64,
        phase_2_cycles: f64,
    ) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_1 = signal_spec_gps_l1_ca();
        ObsEpoch {
            t_rx_s: bijux_gnss_core::api::Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                make_satellite(
                    sat,
                    bijux_gnss_core::api::SignalBand::L1,
                    SignalCode::Ca,
                    signal_1,
                    pseudorange_1_m,
                    phase_1_cycles,
                ),
                make_satellite(
                    sat,
                    second_band,
                    second_code,
                    second_signal,
                    pseudorange_2_m,
                    phase_2_cycles,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn make_satellite(
        sat: SatId,
        band: bijux_gnss_core::api::SignalBand,
        code: SignalCode,
        signal: bijux_gnss_core::api::SignalSpec,
        pseudorange_m: f64,
        phase_cycles: f64,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(phase_cycles),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
                ..ObsMetadata::default()
            },
        }
    }

    fn carrier_cycles(range_m: f64, iono_m: f64, wavelength_m: f64, ambiguity_cycles: f64) -> f64 {
        (range_m - iono_m) / wavelength_m + ambiguity_cycles
    }

    #[test]
    fn iono_free_phase_recovers_range_for_l1_l2() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let iono_l2_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / l2.carrier_hz.value();
        let epoch = make_gps_dual_frequency_epoch(
            bijux_gnss_core::api::SignalBand::L2,
            SignalCode::Py,
            l2,
            base_range_m + iono_l1_m,
            base_range_m + iono_l2_m,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            carrier_cycles(base_range_m, iono_l2_m, lambda2, 0.0),
        );

        let iono_free =
            iono_free_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("iono-free observation");

        assert_eq!(iono_free.band_1, bijux_gnss_core::api::SignalBand::L1);
        assert_eq!(iono_free.band_2, bijux_gnss_core::api::SignalBand::L2);
        assert!(
            (iono_free.phase_cycles * iono_free.phase_wavelength_m - base_range_m).abs() < 1.0e-6
        );
    }

    #[test]
    fn iono_free_phase_recovers_range_for_l1_l5() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let iono_l5_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l5.carrier_hz.value() * l5.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda5 = SPEED_OF_LIGHT_MPS / l5.carrier_hz.value();
        let epoch = make_gps_dual_frequency_epoch(
            bijux_gnss_core::api::SignalBand::L5,
            SignalCode::Unknown,
            l5,
            base_range_m + iono_l1_m,
            base_range_m + iono_l5_m,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            carrier_cycles(base_range_m, iono_l5_m, lambda5, 0.0),
        );

        let iono_free =
            iono_free_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("iono-free observation");

        assert_eq!(iono_free.band_1, bijux_gnss_core::api::SignalBand::L1);
        assert_eq!(iono_free.band_2, bijux_gnss_core::api::SignalBand::L5);
        assert!(
            (iono_free.phase_cycles * iono_free.phase_wavelength_m - base_range_m).abs() < 1.0e-6
        );
    }

    #[test]
    fn iono_free_phase_preserves_l1_l2_ambiguity_term() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let ambiguity_l1_cycles = 17.0;
        let ambiguity_l2_cycles = 11.0;
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let iono_l2_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / l2.carrier_hz.value();
        let epoch = make_gps_dual_frequency_epoch(
            bijux_gnss_core::api::SignalBand::L2,
            SignalCode::Py,
            l2,
            base_range_m + iono_l1_m,
            base_range_m + iono_l2_m,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            carrier_cycles(base_range_m, iono_l2_m, lambda2, ambiguity_l2_cycles),
        );

        let iono_free =
            iono_free_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("iono-free observation");

        let f1_2 = l1.carrier_hz.value() * l1.carrier_hz.value();
        let f2_2 = l2.carrier_hz.value() * l2.carrier_hz.value();
        let expected_ambiguity_m = (f1_2 * lambda1 * ambiguity_l1_cycles
            - f2_2 * lambda2 * ambiguity_l2_cycles)
            / (f1_2 - f2_2);

        assert!(
            (iono_free.phase_cycles * iono_free.phase_wavelength_m
                - (base_range_m + expected_ambiguity_m))
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn iono_free_phase_preserves_l1_l5_ambiguity_term() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let ambiguity_l1_cycles = 19.0;
        let ambiguity_l5_cycles = 7.0;
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let iono_l5_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l5.carrier_hz.value() * l5.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda5 = SPEED_OF_LIGHT_MPS / l5.carrier_hz.value();
        let epoch = make_gps_dual_frequency_epoch(
            bijux_gnss_core::api::SignalBand::L5,
            SignalCode::Unknown,
            l5,
            base_range_m + iono_l1_m,
            base_range_m + iono_l5_m,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            carrier_cycles(base_range_m, iono_l5_m, lambda5, ambiguity_l5_cycles),
        );

        let iono_free =
            iono_free_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("iono-free observation");

        let f1_2 = l1.carrier_hz.value() * l1.carrier_hz.value();
        let f5_2 = l5.carrier_hz.value() * l5.carrier_hz.value();
        let expected_ambiguity_m = (f1_2 * lambda1 * ambiguity_l1_cycles
            - f5_2 * lambda5 * ambiguity_l5_cycles)
            / (f1_2 - f5_2);

        assert!(
            (iono_free.phase_cycles * iono_free.phase_wavelength_m
                - (base_range_m + expected_ambiguity_m))
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn iono_free_prefers_l1_l2_when_l5_is_also_present() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();
        let l5 = signal_spec_gps_l5();
        let epoch = ObsEpoch {
            t_rx_s: bijux_gnss_core::api::Seconds(0.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                make_satellite(
                    sat,
                    bijux_gnss_core::api::SignalBand::L1,
                    SignalCode::Ca,
                    l1,
                    1.0,
                    1.0,
                ),
                make_satellite(
                    sat,
                    bijux_gnss_core::api::SignalBand::L2,
                    SignalCode::Py,
                    l2,
                    2.0,
                    2.0,
                ),
                make_satellite(
                    sat,
                    bijux_gnss_core::api::SignalBand::L5,
                    SignalCode::Unknown,
                    l5,
                    3.0,
                    3.0,
                ),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let iono_free = iono_free_from_obs(&epoch, sat).expect("iono-free observation");

        assert_eq!(iono_free.band_1, bijux_gnss_core::api::SignalBand::L1);
        assert_eq!(iono_free.band_2, bijux_gnss_core::api::SignalBand::L2);
    }

    #[test]
    fn wide_lane_falls_back_to_l1_l5_when_l2_is_missing() {
        let base_range_m = 20_200_000.0;
        let iono_l1_m = 5.0;
        let l1 = signal_spec_gps_l1_ca();
        let l5 = signal_spec_gps_l5();
        let iono_l5_m = iono_l1_m * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l5.carrier_hz.value() * l5.carrier_hz.value());
        let lambda1 = SPEED_OF_LIGHT_MPS / l1.carrier_hz.value();
        let lambda5 = SPEED_OF_LIGHT_MPS / l5.carrier_hz.value();
        let ambiguity_l1_cycles = 12.0;
        let ambiguity_l5_cycles = 4.0;
        let epoch = make_gps_dual_frequency_epoch(
            bijux_gnss_core::api::SignalBand::L5,
            SignalCode::Unknown,
            l5,
            base_range_m + iono_l1_m,
            base_range_m + iono_l5_m,
            carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            carrier_cycles(base_range_m, iono_l5_m, lambda5, ambiguity_l5_cycles),
        );

        let wide_lane =
            wide_lane_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("wide-lane observation");

        let lambda_wl = SPEED_OF_LIGHT_MPS / (l1.carrier_hz.value() - l5.carrier_hz.value()).abs();
        let expected_cycles = ((-iono_l1_m + iono_l5_m) + lambda1 * ambiguity_l1_cycles
            - lambda5 * ambiguity_l5_cycles)
            / lambda_wl;

        assert_eq!(wide_lane.band_1, bijux_gnss_core::api::SignalBand::L1);
        assert_eq!(wide_lane.band_2, bijux_gnss_core::api::SignalBand::L5);
        assert!((wide_lane.cycles - expected_cycles).abs() < 1.0e-6);
    }
}
