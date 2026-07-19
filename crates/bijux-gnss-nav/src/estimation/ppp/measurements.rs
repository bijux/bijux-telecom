#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GpsTime, ObsEpoch, ObsSatellite, SatId, SigId, SignalBand, SignalSpec,
};
use bijux_gnss_signal::api::{
    signal_cycles_to_meters, signal_spec_beidou_b1i, signal_spec_galileo_e1b,
    signal_spec_gps_l1_ca, signal_wavelength_m,
    supported_dual_frequency_band_pairs_for_constellation,
};

use crate::corrections::biases::PhaseBiasProvider;
use crate::corrections::combinations::wide_lane_wavelength_m_from_frequencies;
use crate::corrections::iono_free_code::iono_free_code_from_pair;
use crate::corrections::iono_free_phase::iono_free_phase_from_pair;
use crate::corrections::Corrections;
use crate::estimation::ekf::state::MeasurementKind;
use crate::estimation::ekf::traits::MeasurementModel;
use crate::linalg::Matrix;

use super::config::SPEED_OF_LIGHT_MPS;
use super::models::PppCodeMeasurement;

#[derive(Debug, Clone, Copy)]
pub struct IonoFreeCodeMeasurementObservation {
    pub gps_time: Option<GpsTime>,
    pub signal_1: SigId,
    pub signal_2: SigId,
    pub code_m: f64,
    pub code_sigma_m: f64,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
}

#[derive(Debug, Clone, Copy)]
pub struct IonoFreePhaseMeasurementObservation {
    pub phase_cycles: f64,
    pub phase_sigma_cycles: f64,
    pub phase_wavelength_m: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
}

#[derive(Debug, Clone, Copy)]
pub struct IonoFreeObservation {
    pub signal_1: SigId,
    pub signal_2: SigId,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub code_sigma_m: f64,
    pub phase_sigma_cycles: f64,
    pub phase_wavelength_m: f64,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
}

#[derive(Debug, Clone, Copy)]
pub struct WideLaneObservation {
    pub signal_1: SigId,
    pub signal_2: SigId,
    pub cycles: f64,
    pub variance: f64,
    pub wavelength_m: f64,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub phase_bias_provenance_complete: bool,
}

#[derive(Clone, Copy)]
struct DualFrequencyObservationPair<'a> {
    first: &'a ObsSatellite,
    second: &'a ObsSatellite,
}

pub fn ppp_ionosphere_delay_scale(signal: SignalSpec) -> f64 {
    let reference_hz = ppp_reference_ionosphere_carrier_hz(signal);
    let carrier_hz = signal.carrier_hz.value();
    if reference_hz.is_finite() && reference_hz > 0.0 && carrier_hz.is_finite() && carrier_hz > 0.0
    {
        (reference_hz / carrier_hz).powi(2)
    } else {
        1.0
    }
}

fn ppp_reference_ionosphere_carrier_hz(signal: SignalSpec) -> f64 {
    match signal.constellation {
        Constellation::Gps => signal_spec_gps_l1_ca().carrier_hz.value(),
        Constellation::Galileo => signal_spec_galileo_e1b().carrier_hz.value(),
        Constellation::Beidou => signal_spec_beidou_b1i().carrier_hz.value(),
        Constellation::Glonass | Constellation::Unknown => signal.carrier_hz.value(),
    }
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
                tropo = *ztd * self.troposphere_mapping;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v * self.ionosphere_scale;
            }
        }
        let mut pred = range + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s) + tropo + iono;
        pred += self.antenna_range_correction_m;
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
                h[(0, idx)] = self.troposphere_mapping;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = self.ionosphere_scale;
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
    pub antenna_range_correction_m: f64,
    pub sigma_cycles: f64,
    pub troposphere_mapping: f64,
    pub ionosphere_scale: f64,
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
                tropo = *ztd * self.troposphere_mapping;
            }
        }
        let mut iono = 0.0;
        if let Some(idx) = self.iono_index {
            if let Some(v) = x.get(idx) {
                iono = *v * self.ionosphere_scale;
            }
        }
        let mut pred = (range
            + self.antenna_range_correction_m
            + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s)
            + tropo
            - iono)
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
                h[(0, idx)] = self.troposphere_mapping / self.wavelength_m;
            }
        }
        if let Some(idx) = self.iono_index {
            if idx < h.cols() {
                h[(0, idx)] = -self.ionosphere_scale / self.wavelength_m;
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
    pub antenna_range_correction_m: f64,
    pub sigma_m: f64,
    pub troposphere_mapping: f64,
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
                tropo = *ztd * self.troposphere_mapping;
            }
        }
        let mut pred = range
            + self.antenna_range_correction_m
            + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s)
            + tropo;
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
                h[(0, idx)] = self.troposphere_mapping;
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
    pub antenna_range_correction_m: f64,
    pub sigma_cycles: f64,
    pub troposphere_mapping: f64,
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
        let mut pred = (range
            + self.antenna_range_correction_m
            + SPEED_OF_LIGHT_MPS * (x[6] - self.sat_clock_s))
            / self.wavelength_m;
        if let Some(idx) = self.ztd_index {
            if let Some(ztd) = x.get(idx) {
                pred += *ztd * self.troposphere_mapping / self.wavelength_m;
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
                h[(0, idx)] = self.troposphere_mapping / self.wavelength_m;
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
    let code = iono_free_code_observation_from_obs(obs, sat)?;
    let phase = iono_free_phase_observation_from_obs(obs, sat)?;
    Some(IonoFreeObservation {
        signal_1: code.signal_1,
        signal_2: code.signal_2,
        code_m: code.code_m,
        phase_cycles: phase.phase_cycles,
        code_sigma_m: code.code_sigma_m,
        phase_sigma_cycles: phase.phase_sigma_cycles,
        phase_wavelength_m: phase.phase_wavelength_m,
        f1_hz: code.f1_hz,
        f2_hz: code.f2_hz,
        band_1: code.band_1,
        band_2: code.band_2,
    })
}

pub fn iono_free_code_observation_from_obs(
    obs: &ObsEpoch,
    sat: SatId,
) -> Option<IonoFreeCodeMeasurementObservation> {
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
    if code.status != "ok" {
        return None;
    }
    Some(IonoFreeCodeMeasurementObservation {
        gps_time: obs.gps_week.zip(obs.tow_s).map(|(week, tow_s)| GpsTime { week, tow_s: tow_s.0 }),
        signal_1: pair.first.signal_id,
        signal_2: pair.second.signal_id,
        code_m: code.code_m?,
        code_sigma_m: code.variance_m2?.sqrt(),
        f1_hz: pair.first.metadata.signal.carrier_hz.value(),
        f2_hz: pair.second.metadata.signal.carrier_hz.value(),
        band_1: pair.first.signal_id.band,
        band_2: pair.second.signal_id.band,
    })
}

pub fn iono_free_phase_observation_from_obs(
    obs: &ObsEpoch,
    sat: SatId,
) -> Option<IonoFreePhaseMeasurementObservation> {
    let pair = select_dual_frequency_pair(obs, sat)?;
    let phase = iono_free_phase_from_pair(
        obs.epoch_idx,
        obs.t_rx_s.0,
        sat,
        pair.first.signal_id.band,
        pair.second.signal_id.band,
        Some(pair.first),
        Some(pair.second),
    );
    if phase.status != "ok" {
        return None;
    }
    Some(IonoFreePhaseMeasurementObservation {
        phase_cycles: phase.phase_cycles?,
        phase_sigma_cycles: phase.variance_cycles2?.sqrt(),
        phase_wavelength_m: phase.narrow_lane_wavelength_m?,
        band_1: pair.first.signal_id.band,
        band_2: pair.second.signal_id.band,
    })
}

pub fn wide_lane_from_obs(obs: &ObsEpoch, sat: SatId) -> Option<WideLaneObservation> {
    wide_lane_from_obs_with_phase_biases(obs, sat, None)
}

pub fn wide_lane_from_obs_with_phase_biases(
    obs: &ObsEpoch,
    sat: SatId,
    phase_biases: Option<&dyn PhaseBiasProvider>,
) -> Option<WideLaneObservation> {
    let pair = select_dual_frequency_pair(obs, sat)?;
    let f1 = pair.first.metadata.signal.carrier_hz.value();
    let f2 = pair.second.metadata.signal.carrier_hz.value();
    let lambda_1 = signal_wavelength_m(pair.first.metadata.signal).0;
    let lambda_2 = signal_wavelength_m(pair.second.metadata.signal).0;
    let (bias_1_cycles, bias_1_provenance) =
        resolved_ar_phase_bias_cycles(phase_biases, pair.first.signal_id, obs.gps_time());
    let (bias_2_cycles, bias_2_provenance) =
        resolved_ar_phase_bias_cycles(phase_biases, pair.second.signal_id, obs.gps_time());
    let phi1_m = signal_cycles_to_meters(
        bijux_gnss_core::api::Cycles(pair.first.carrier_phase_cycles.0 - bias_1_cycles),
        pair.first.metadata.signal,
    )
    .0;
    let phi2_m = signal_cycles_to_meters(
        bijux_gnss_core::api::Cycles(pair.second.carrier_phase_cycles.0 - bias_2_cycles),
        pair.second.metadata.signal,
    )
    .0;
    let lambda_wl = wide_lane_wavelength_m_from_frequencies(f1, f2)?;
    let wl_cycles = (phi1_m - phi2_m) / lambda_wl;
    let variance = (lambda_1 / lambda_wl).powi(2) * pair.first.carrier_phase_var_cycles2
        + (lambda_2 / lambda_wl).powi(2) * pair.second.carrier_phase_var_cycles2;
    Some(WideLaneObservation {
        signal_1: pair.first.signal_id,
        signal_2: pair.second.signal_id,
        cycles: wl_cycles,
        variance,
        wavelength_m: lambda_wl,
        band_1: pair.first.signal_id.band,
        band_2: pair.second.signal_id.band,
        phase_bias_provenance_complete: bias_1_provenance && bias_2_provenance,
    })
}

fn resolved_ar_phase_bias_cycles(
    phase_biases: Option<&dyn PhaseBiasProvider>,
    signal: SigId,
    gps_time: Option<GpsTime>,
) -> (f64, bool) {
    phase_biases
        .and_then(|provider| provider.phase_bias_for_ambiguity_resolution(signal, gps_time))
        .map(|bias| (bias.bias_cycles, true))
        .unwrap_or((0.0, false))
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
    for (first_band, second_band) in
        supported_dual_frequency_band_pairs_for_constellation(sat.constellation)
    {
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

#[cfg(test)]
mod tests {
    use super::{
        iono_free_code_observation_from_obs, iono_free_from_obs,
        iono_free_phase_observation_from_obs, ppp_ionosphere_delay_scale, wide_lane_from_obs,
        PppIonoFreeCodeMeasurement, PppIonoFreePhaseMeasurement, PppPhaseMeasurement,
        SPEED_OF_LIGHT_MPS,
    };
    use crate::corrections::Corrections;
    use crate::estimation::ekf::traits::MeasurementModel;
    use crate::estimation::ppp::models::PppCodeMeasurement;
    use crate::linalg::Matrix;
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        SigId, SignalCode, SignalSpec,
    };
    use bijux_gnss_signal::api::{
        signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
        signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
    };

    struct DualFrequencySignalObservation {
        band: bijux_gnss_core::api::SignalBand,
        code: SignalCode,
        signal: SignalSpec,
        pseudorange_m: f64,
        phase_cycles: f64,
    }

    struct DualFrequencyEpochRequest {
        sat: SatId,
        first_observation: DualFrequencySignalObservation,
        second_observation: DualFrequencySignalObservation,
    }

    fn make_dual_frequency_epoch(request: DualFrequencyEpochRequest) -> ObsEpoch {
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
                    request.sat,
                    request.first_observation.band,
                    request.first_observation.code,
                    request.first_observation.signal,
                    request.first_observation.pseudorange_m,
                    request.first_observation.phase_cycles,
                ),
                make_satellite(
                    request.sat,
                    request.second_observation.band,
                    request.second_observation.code,
                    request.second_observation.signal,
                    request.second_observation.pseudorange_m,
                    request.second_observation.phase_cycles,
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

    #[test]
    fn iono_free_code_observation_survives_missing_carrier_phase() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let mut epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat,
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: signal_spec_gps_l1_ca(),
                pseudorange_m: 22_000_000.0,
                phase_cycles: 22_000_100.0,
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L2,
                code: SignalCode::Py,
                signal: signal_spec_gps_l2_py(),
                pseudorange_m: 100_000.0,
                phase_cycles: 80_000.0,
            },
        });
        epoch.sats[0].lock_flags.carrier_lock = false;
        epoch.sats[1].lock_flags.carrier_lock = false;

        let code = iono_free_code_observation_from_obs(&epoch, sat).expect("iono-free code");
        let phase = iono_free_phase_observation_from_obs(&epoch, sat);

        assert!(code.code_m.is_finite());
        assert!(phase.is_none());
        assert!(iono_free_from_obs(&epoch, sat).is_none());
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
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: l1,
                pseudorange_m: base_range_m + iono_l1_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L2,
                code: SignalCode::Py,
                signal: l2,
                pseudorange_m: base_range_m + iono_l2_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l2_m, lambda2, 0.0),
            },
        });

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
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: l1,
                pseudorange_m: base_range_m + iono_l1_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l1_m, lambda1, 0.0),
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L5,
                code: SignalCode::Unknown,
                signal: l5,
                pseudorange_m: base_range_m + iono_l5_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l5_m, lambda5, 0.0),
            },
        });

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
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: l1,
                pseudorange_m: base_range_m + iono_l1_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L2,
                code: SignalCode::Py,
                signal: l2,
                pseudorange_m: base_range_m + iono_l2_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l2_m, lambda2, ambiguity_l2_cycles),
            },
        });

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
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: l1,
                pseudorange_m: base_range_m + iono_l1_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L5,
                code: SignalCode::Unknown,
                signal: l5,
                pseudorange_m: base_range_m + iono_l5_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l5_m, lambda5, ambiguity_l5_cycles),
            },
        });

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
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
                signal: l1,
                pseudorange_m: base_range_m + iono_l1_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l1_m, lambda1, ambiguity_l1_cycles),
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::L5,
                code: SignalCode::Unknown,
                signal: l5,
                pseudorange_m: base_range_m + iono_l5_m,
                phase_cycles: carrier_cycles(base_range_m, iono_l5_m, lambda5, ambiguity_l5_cycles),
            },
        });

        let wide_lane =
            wide_lane_from_obs(&epoch, SatId { constellation: Constellation::Gps, prn: 3 })
                .expect("wide-lane observation");

        let lambda_wl = SPEED_OF_LIGHT_MPS / (l1.carrier_hz.value() - l5.carrier_hz.value()).abs();
        let expected_cycles = ((-iono_l1_m + iono_l5_m) + lambda1 * ambiguity_l1_cycles
            - lambda5 * ambiguity_l5_cycles)
            / lambda_wl;

        assert_eq!(wide_lane.band_1, bijux_gnss_core::api::SignalBand::L1);
        assert_eq!(wide_lane.band_2, bijux_gnss_core::api::SignalBand::L5);
        assert!((wide_lane.wavelength_m - lambda_wl).abs() < 1.0e-12);
        assert!((wide_lane.cycles - expected_cycles).abs() < 1.0e-6);
        assert!(
            (wide_lane.variance
                - ((lambda1 / lambda_wl).powi(2) + (lambda5 / lambda_wl).powi(2)) * 0.01)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn wide_lane_uses_galileo_e1_e5_wavelength_and_variance() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 19 };
        let e1 = signal_spec_galileo_e1b();
        let e5 = signal_spec_galileo_e5a();
        let lambda1 = SPEED_OF_LIGHT_MPS / e1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / e5.carrier_hz.value();
        let ambiguity_1_cycles = 23.0;
        let ambiguity_2_cycles = 9.0;
        let base_range_m = 24_000_000.0;
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat,
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::E1,
                code: SignalCode::E1B,
                signal: e1,
                pseudorange_m: base_range_m,
                phase_cycles: base_range_m / lambda1 + ambiguity_1_cycles,
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::E5,
                code: SignalCode::E5a,
                signal: e5,
                pseudorange_m: base_range_m,
                phase_cycles: base_range_m / lambda2 + ambiguity_2_cycles,
            },
        });

        let wide_lane = wide_lane_from_obs(&epoch, sat).expect("galileo wide-lane observation");
        let lambda_wl = SPEED_OF_LIGHT_MPS / (e1.carrier_hz.value() - e5.carrier_hz.value()).abs();
        let expected_cycles =
            (lambda1 * ambiguity_1_cycles - lambda2 * ambiguity_2_cycles) / lambda_wl;
        let expected_variance =
            ((lambda1 / lambda_wl).powi(2) + (lambda2 / lambda_wl).powi(2)) * 0.01;

        assert_eq!(wide_lane.band_1, bijux_gnss_core::api::SignalBand::E1);
        assert_eq!(wide_lane.band_2, bijux_gnss_core::api::SignalBand::E5);
        assert!((wide_lane.wavelength_m - lambda_wl).abs() < 1.0e-12);
        assert!((wide_lane.cycles - expected_cycles).abs() < 1.0e-6);
        assert!((wide_lane.variance - expected_variance).abs() < 1.0e-12);
    }

    #[test]
    fn wide_lane_uses_beidou_b1_b2_wavelength_and_variance() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 7 };
        let b1 = signal_spec_beidou_b1i();
        let b2 = signal_spec_beidou_b2i();
        let lambda1 = SPEED_OF_LIGHT_MPS / b1.carrier_hz.value();
        let lambda2 = SPEED_OF_LIGHT_MPS / b2.carrier_hz.value();
        let ambiguity_1_cycles = 21.0;
        let ambiguity_2_cycles = 8.0;
        let base_range_m = 24_000_000.0;
        let epoch = make_dual_frequency_epoch(DualFrequencyEpochRequest {
            sat,
            first_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::B1,
                code: SignalCode::B1I,
                signal: b1,
                pseudorange_m: base_range_m,
                phase_cycles: base_range_m / lambda1 + ambiguity_1_cycles,
            },
            second_observation: DualFrequencySignalObservation {
                band: bijux_gnss_core::api::SignalBand::B2,
                code: SignalCode::B2I,
                signal: b2,
                pseudorange_m: base_range_m,
                phase_cycles: base_range_m / lambda2 + ambiguity_2_cycles,
            },
        });

        let wide_lane = wide_lane_from_obs(&epoch, sat).expect("beidou wide-lane observation");
        let lambda_wl = SPEED_OF_LIGHT_MPS / (b1.carrier_hz.value() - b2.carrier_hz.value()).abs();
        let expected_cycles =
            (lambda1 * ambiguity_1_cycles - lambda2 * ambiguity_2_cycles) / lambda_wl;
        let expected_variance =
            ((lambda1 / lambda_wl).powi(2) + (lambda2 / lambda_wl).powi(2)) * 0.01;

        assert_eq!(wide_lane.band_1, bijux_gnss_core::api::SignalBand::B1);
        assert_eq!(wide_lane.band_2, bijux_gnss_core::api::SignalBand::B2);
        assert!((wide_lane.wavelength_m - lambda_wl).abs() < 1.0e-12);
        assert!((wide_lane.cycles - expected_cycles).abs() < 1.0e-6);
        assert!((wide_lane.variance - expected_variance).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_code_measurement_maps_zenith_delay_by_elevation() {
        let measurement = PppCodeMeasurement {
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.4,
            sigma_m: 1.0,
            troposphere_mapping: 2.75,
            ionosphere_scale: 1.0,
            iono_index: None,
            ztd_index: Some(8),
            isb_index: None,
            corr: Corrections::default(),
        };
        let mut state = vec![0.0; 9];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 2.4;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx: f64 = state[0] - measurement.sat_pos_m[0];
        let dy: f64 = state[1] - measurement.sat_pos_m[1];
        let dz: f64 = state[2] - measurement.sat_pos_m[2];
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let expected = geometric_range_m + 0.4 + 2.75 * 2.4;

        assert!((predicted[0] - expected).abs() < 1.0e-9);

        let mut jacobian = Matrix::new(1, 9, 0.0);
        measurement.jacobian(&state, &mut jacobian);

        assert!((jacobian[(0, 8)] - 2.75).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_code_measurement_applies_earth_tide_displacement_to_geometry() {
        let measurement = PppCodeMeasurement {
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_m: 1.0,
            troposphere_mapping: 1.0,
            ionosphere_scale: 1.0,
            iono_index: None,
            ztd_index: None,
            isb_index: None,
            corr: Corrections { earth_tide_m: [0.02, -0.03, 0.04], ..Corrections::default() },
        };
        let mut state = vec![0.0; 9];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] + 0.02 - measurement.sat_pos_m[0];
        let dy = state[1] - 0.03 - measurement.sat_pos_m[1];
        let dz = state[2] + 0.04 - measurement.sat_pos_m[2];
        let expected = (dx * dx + dy * dy + dz * dz).sqrt();

        assert!((predicted[0] - expected).abs() < 1.0e-9);
    }

    #[test]
    fn ppp_ionosphere_delay_scale_uses_reference_carrier_frequency() {
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();

        let l1_scale = ppp_ionosphere_delay_scale(l1);
        let l2_scale = ppp_ionosphere_delay_scale(l2);
        let expected_l2_scale = (l1.carrier_hz.value() / l2.carrier_hz.value()).powi(2);

        assert!((l1_scale - 1.0).abs() < 1.0e-12);
        assert!((l2_scale - expected_l2_scale).abs() < 1.0e-12);
        assert!(l2_scale > l1_scale);
    }

    #[test]
    fn ppp_code_measurement_adds_scaled_ionosphere_delay() {
        let ionosphere_scale = ppp_ionosphere_delay_scale(signal_spec_gps_l2_py());
        let measurement = PppCodeMeasurement {
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_m: 1.0,
            troposphere_mapping: 1.0,
            ionosphere_scale,
            iono_index: Some(8),
            ztd_index: None,
            isb_index: None,
            corr: Corrections::default(),
        };
        let mut state = vec![0.0; 9];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 5.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] - measurement.sat_pos_m[0];
        let dy = state[1] - measurement.sat_pos_m[1];
        let dz = state[2] - measurement.sat_pos_m[2];
        let expected = (dx * dx + dy * dy + dz * dz).sqrt() + ionosphere_scale * state[8];

        assert!((predicted[0] - expected).abs() < 1.0e-9);

        let mut jacobian = Matrix::new(1, 9, 0.0);
        measurement.jacobian(&state, &mut jacobian);

        assert!((jacobian[(0, 8)] - ionosphere_scale).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_phase_measurement_maps_zenith_delay_by_elevation() {
        let measurement = PppPhaseMeasurement {
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: -0.25,
            sigma_cycles: 0.01,
            troposphere_mapping: 3.1,
            ionosphere_scale: 1.0,
            iono_index: None,
            ztd_index: Some(8),
            isb_index: None,
            ambiguity_index: Some(9),
            corr: Corrections::default(),
            wavelength_m: 0.190_293_672_798,
        };
        let mut state = vec![0.0; 10];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 2.1;
        state[9] = 12.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] - measurement.sat_pos_m[0];
        let dy = state[1] - measurement.sat_pos_m[1];
        let dz = state[2] - measurement.sat_pos_m[2];
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let expected = (geometric_range_m - 0.25 + 3.1 * 2.1) / measurement.wavelength_m + state[9];

        assert!((predicted[0] - expected).abs() < 1.0e-9);

        let mut jacobian = Matrix::new(1, 10, 0.0);
        measurement.jacobian(&state, &mut jacobian);

        assert!((jacobian[(0, 8)] - 3.1 / measurement.wavelength_m).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_phase_measurement_applies_earth_tide_displacement_to_geometry() {
        let measurement = PppPhaseMeasurement {
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_cycles: 0.01,
            troposphere_mapping: 1.0,
            ionosphere_scale: 1.0,
            iono_index: None,
            ztd_index: None,
            isb_index: None,
            ambiguity_index: Some(9),
            corr: Corrections { earth_tide_m: [0.02, -0.03, 0.04], ..Corrections::default() },
            wavelength_m: 0.190_293_672_798,
        };
        let mut state = vec![0.0; 10];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[9] = 12.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] + 0.02 - measurement.sat_pos_m[0];
        let dy = state[1] - 0.03 - measurement.sat_pos_m[1];
        let dz = state[2] + 0.04 - measurement.sat_pos_m[2];
        let expected = (dx * dx + dy * dy + dz * dz).sqrt() / measurement.wavelength_m + state[9];

        assert!((predicted[0] - expected).abs() < 1.0e-9);
    }

    #[test]
    fn ppp_phase_measurement_subtracts_scaled_ionosphere_advance() {
        let signal = signal_spec_gps_l2_py();
        let ionosphere_scale = ppp_ionosphere_delay_scale(signal);
        let wavelength_m = SPEED_OF_LIGHT_MPS / signal.carrier_hz.value();
        let measurement = PppPhaseMeasurement {
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_cycles: 0.01,
            troposphere_mapping: 1.0,
            ionosphere_scale,
            iono_index: Some(8),
            ztd_index: None,
            isb_index: None,
            ambiguity_index: Some(9),
            corr: Corrections::default(),
            wavelength_m,
        };
        let mut state = vec![0.0; 10];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 5.0;
        state[9] = 12.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] - measurement.sat_pos_m[0];
        let dy = state[1] - measurement.sat_pos_m[1];
        let dz = state[2] - measurement.sat_pos_m[2];
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let expected = (range_m - ionosphere_scale * state[8]) / wavelength_m + state[9];

        assert!((predicted[0] - expected).abs() < 1.0e-9);

        let mut jacobian = Matrix::new(1, 10, 0.0);
        measurement.jacobian(&state, &mut jacobian);

        assert!((jacobian[(0, 8)] + ionosphere_scale / wavelength_m).abs() < 1.0e-12);
        assert!((jacobian[(0, 9)] - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_iono_free_phase_measurement_maps_zenith_delay_by_elevation() {
        let measurement = PppIonoFreePhaseMeasurement {
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.15,
            sigma_cycles: 0.01,
            troposphere_mapping: 2.2,
            ztd_index: Some(8),
            isb_index: None,
            ambiguity_index: Some(9),
            wavelength_m: 0.107,
            corr: Corrections::default(),
        };
        let mut state = vec![0.0; 10];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 2.8;
        state[9] = 6.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] - measurement.sat_pos_m[0];
        let dy = state[1] - measurement.sat_pos_m[1];
        let dz = state[2] - measurement.sat_pos_m[2];
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let expected = (geometric_range_m + 0.15 + 2.2 * 2.8) / measurement.wavelength_m + state[9];

        assert!((predicted[0] - expected).abs() < 1.0e-9);

        let mut jacobian = Matrix::new(1, 10, 0.0);
        measurement.jacobian(&state, &mut jacobian);

        assert!((jacobian[(0, 8)] - 2.2 / measurement.wavelength_m).abs() < 1.0e-12);
    }

    #[test]
    fn ppp_iono_free_phase_measurement_applies_earth_tide_displacement_to_geometry() {
        let measurement = PppIonoFreePhaseMeasurement {
            z_cycles: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_cycles: 0.01,
            troposphere_mapping: 1.0,
            ztd_index: None,
            isb_index: None,
            ambiguity_index: Some(9),
            wavelength_m: 0.107,
            corr: Corrections { earth_tide_m: [0.02, -0.03, 0.04], ..Corrections::default() },
        };
        let mut state = vec![0.0; 10];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[9] = 6.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] + 0.02 - measurement.sat_pos_m[0];
        let dy = state[1] - 0.03 - measurement.sat_pos_m[1];
        let dz = state[2] + 0.04 - measurement.sat_pos_m[2];
        let expected = (dx * dx + dy * dy + dz * dz).sqrt() / measurement.wavelength_m + state[9];

        assert!((predicted[0] - expected).abs() < 1.0e-9);
    }

    #[test]
    fn ppp_iono_free_code_measurement_applies_antenna_range_correction() {
        let measurement = PppIonoFreeCodeMeasurement {
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: -0.3,
            sigma_m: 1.0,
            troposphere_mapping: 2.4,
            ztd_index: Some(8),
            isb_index: None,
            corr: Corrections::default(),
        };
        let mut state = vec![0.0; 9];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;
        state[8] = 1.8;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] - measurement.sat_pos_m[0];
        let dy = state[1] - measurement.sat_pos_m[1];
        let dz = state[2] - measurement.sat_pos_m[2];
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let expected = geometric_range_m - 0.3 + 2.4 * 1.8;

        assert!((predicted[0] - expected).abs() < 1.0e-9);
    }

    #[test]
    fn ppp_iono_free_code_measurement_applies_earth_tide_displacement_to_geometry() {
        let measurement = PppIonoFreeCodeMeasurement {
            z_m: 0.0,
            sat_pos_m: [20_200_000.0, 14_000_000.0, 21_700_000.0],
            sat_clock_s: 0.0,
            antenna_range_correction_m: 0.0,
            sigma_m: 1.0,
            troposphere_mapping: 1.0,
            ztd_index: None,
            isb_index: None,
            corr: Corrections { earth_tide_m: [0.02, -0.03, 0.04], ..Corrections::default() },
        };
        let mut state = vec![0.0; 9];
        state[0] = 1_111_111.0;
        state[1] = -4_222_222.0;
        state[2] = 4_333_333.0;

        let mut predicted = [0.0];
        measurement.h(&state, &mut predicted);

        let dx = state[0] + 0.02 - measurement.sat_pos_m[0];
        let dy = state[1] - 0.03 - measurement.sat_pos_m[1];
        let dz = state[2] + 0.04 - measurement.sat_pos_m[2];
        let expected = (dx * dx + dy * dy + dz * dz).sqrt();

        assert!((predicted[0] - expected).abs() < 1.0e-9);
    }
}
