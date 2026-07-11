#![allow(missing_docs)]

use std::collections::BTreeMap;

use super::metrics::{jitter_summary, BaselineSolution, JitterSummary};
use bijux_gnss_core::api::{Constellation, ObsEpoch, ReceiverRole, SigId};
use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation,
    rtk_double_differences_by_constellation, rtk_double_differences_from_single_differences,
    rtk_float_baseline_from_double_differences, rtk_single_differences_from_obs_epochs,
    RtkDoubleDifferenceObservation, RtkFloatBaselineSolution, RtkSingleDifferenceObservation,
};

#[derive(Debug, Clone)]
pub struct BaselineConfig {
    pub base_ecef_m: Option<[f64; 3]>,
    pub rover_prior_ecef_m: Option<[f64; 3]>,
    pub align_tolerance_s: f64,
    pub ref_policy: RefSatPolicy,
}

impl Default for BaselineConfig {
    fn default() -> Self {
        Self {
            base_ecef_m: None,
            rover_prior_ecef_m: None,
            align_tolerance_s: 0.0005,
            ref_policy: RefSatPolicy::Global,
        }
    }
}

#[derive(Debug, Clone)]
pub struct EpochAligner {
    tolerance_s: f64,
    pub aligned: usize,
    pub dropped_base: usize,
    pub dropped_rover: usize,
    jitter_s: Vec<f64>,
}

#[derive(Debug, Clone, Copy)]
pub enum RefSatPolicy {
    Global,
    PerConstellation,
}

#[derive(Debug, Clone)]
pub struct RefSatSelector {
    pub last_ref: Option<SigId>,
    pub hold_epochs: usize,
    pub since_change: usize,
}

impl RefSatSelector {
    pub fn new(hold_epochs: usize) -> Self {
        Self { last_ref: None, hold_epochs, since_change: 0 }
    }

    pub fn choose(&mut self, sd: &[SdObservation]) -> Option<SigId> {
        if sd.is_empty() {
            return None;
        }
        let best = choose_ref_sat(sd)?;
        if let Some(last) = self.last_ref {
            if sd.iter().any(|s| s.sig == last) && self.since_change < self.hold_epochs {
                self.since_change += 1;
                return Some(last);
            }
        }
        if self.last_ref != Some(best) {
            self.last_ref = Some(best);
            self.since_change = 0;
        }
        Some(best)
    }
}

impl EpochAligner {
    pub fn new(tolerance_s: f64) -> Self {
        Self { tolerance_s, aligned: 0, dropped_base: 0, dropped_rover: 0, jitter_s: Vec::new() }
    }

    pub fn align(&mut self, base: &[ObsEpoch], rover: &[ObsEpoch]) -> Vec<(ObsEpoch, ObsEpoch)> {
        let mut out = Vec::new();
        let mut i = 0;
        let mut j = 0;
        while i < base.len() && j < rover.len() {
            let b = &base[i];
            let r = &rover[j];
            let dt = (b.t_rx_s.0 - r.t_rx_s.0).abs();
            if dt <= self.tolerance_s {
                out.push((b.clone(), r.clone()));
                self.aligned += 1;
                self.jitter_s.push(dt);
                i += 1;
                j += 1;
            } else if b.t_rx_s.0 < r.t_rx_s.0 {
                self.dropped_base += 1;
                i += 1;
            } else {
                self.dropped_rover += 1;
                j += 1;
            }
        }
        out
    }

    pub fn report(&self, base_len: usize, rover_len: usize) -> AlignmentReport {
        let total = base_len.max(rover_len).max(1) as f64;
        let matched_pct = (self.aligned as f64) / total;
        AlignmentReport {
            base: AlignmentDiagnostic {
                role: ReceiverRole::Base,
                aligned: self.aligned,
                dropped: self.dropped_base,
            },
            rover: AlignmentDiagnostic {
                role: ReceiverRole::Rover,
                aligned: self.aligned,
                dropped: self.dropped_rover,
            },
            matched_pct,
            jitter: jitter_summary(&self.jitter_s),
        }
    }
}

pub type SdObservation = RtkSingleDifferenceObservation;

pub type DdObservation = RtkDoubleDifferenceObservation;

#[derive(Debug, Clone)]
pub struct SolutionSeparation {
    pub sig: SigId,
    pub delta_enu_m: f64,
}

pub fn build_sd(base: &ObsEpoch, rover: &ObsEpoch) -> Vec<SdObservation> {
    rtk_single_differences_from_obs_epochs(base, rover)
}

pub fn choose_ref_sat(sd: &[SdObservation]) -> Option<SigId> {
    choose_rtk_single_difference_reference_signal(sd)
}

pub fn choose_ref_sat_per_constellation(sd: &[SdObservation]) -> BTreeMap<Constellation, SigId> {
    choose_rtk_single_difference_reference_signals_by_constellation(sd)
}

pub fn build_dd(sd: &[SdObservation], ref_sig: SigId) -> Vec<DdObservation> {
    rtk_double_differences_from_single_differences(sd, ref_sig)
}

pub fn build_dd_per_constellation(
    sd: &[SdObservation],
    refs: &BTreeMap<Constellation, SigId>,
) -> Vec<DdObservation> {
    rtk_double_differences_by_constellation(sd, refs)
}

#[derive(Debug, Clone)]
pub struct DdCovarianceModel {
    pub variance_code: f64,
    pub variance_phase: f64,
    pub inter_frequency_corr: Option<f64>,
}

pub fn dd_covariance(dd: &DdObservation) -> DdCovarianceModel {
    DdCovarianceModel {
        variance_code: dd.code_variance_m2,
        variance_phase: dd.phase_variance_cycles2,
        inter_frequency_corr: None,
    }
}

#[derive(Debug, Clone)]
pub struct InnovationDiagnostics {
    pub predicted_variance: f64,
    pub observed_variance: f64,
    pub scale_suggestion: f64,
}

pub fn innovation_diagnostics(residuals: &[f64], predicted_variance: f64) -> InnovationDiagnostics {
    let observed = if residuals.is_empty() {
        0.0
    } else {
        residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64
    };
    let scale = if predicted_variance > 0.0 { (observed / predicted_variance).sqrt() } else { 1.0 };
    InnovationDiagnostics {
        predicted_variance,
        observed_variance: observed,
        scale_suggestion: scale,
    }
}

pub fn solve_baseline_dd(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    t_rx_s: f64,
) -> Option<BaselineSolution> {
    let solution = solve_float_baseline_dd(dd, base_ecef_m, ephs, t_rx_s)?;
    Some(BaselineSolution {
        enu_m: solution.enu_m,
        covariance_m2: Some(solution.covariance_enu_m2),
        fixed: false,
    })
}

pub fn solve_float_baseline_dd(
    dd: &[DdObservation],
    base_ecef_m: [f64; 3],
    ephs: &[bijux_gnss_nav::api::GpsEphemeris],
    t_rx_s: f64,
) -> Option<RtkFloatBaselineSolution> {
    rtk_float_baseline_from_double_differences(dd, base_ecef_m, ephs, t_rx_s)
}

pub fn los_unit(base: [f64; 3], sat: [f64; 3]) -> [f64; 3] {
    let dx = base[0] - sat[0];
    let dy = base[1] - sat[1];
    let dz = base[2] - sat[2];
    let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    [dx / r, dy / r, dz / r]
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AlignmentDiagnostic {
    pub role: ReceiverRole,
    pub aligned: usize,
    pub dropped: usize,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AlignmentReport {
    pub base: AlignmentDiagnostic,
    pub rover: AlignmentDiagnostic,
    pub matched_pct: f64,
    pub jitter: JitterSummary,
}
