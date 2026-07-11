#![allow(missing_docs)]

use std::collections::BTreeMap;

use super::metrics::{baseline_from_ecef, jitter_summary, BaselineSolution, JitterSummary};
use bijux_gnss_core::api::{
    AmbiguityId, Constellation, ObsEpoch, ObsSignalTiming, ReceiverRole, SigId,
};
use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation,
    rtk_single_differences_from_obs_epochs, RtkSingleDifferenceObservation,
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

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DdObservation {
    pub sig: SigId,
    pub ref_sig: SigId,
    pub signal_pseudorange_m: f64,
    pub signal_timing: Option<ObsSignalTiming>,
    pub ref_signal_pseudorange_m: f64,
    pub ref_signal_timing: Option<ObsSignalTiming>,
    pub code_m: f64,
    pub phase_cycles: f64,
    pub doppler_hz: f64,
    pub variance_code: f64,
    pub variance_phase: f64,
    pub canceled: Vec<AmbiguityId>,
}

impl bijux_gnss_core::api::ArtifactPayloadValidate for DdObservation {
    fn validate_payload(&self) -> Vec<bijux_gnss_core::api::DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
        {
            events.push(bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "RTK_DD_NUMERIC_INVALID",
                "dd observation contains NaN/Inf",
            ));
        }
        if self.variance_code < 0.0 || self.variance_phase < 0.0 {
            events.push(bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "RTK_DD_VARIANCE_INVALID",
                "dd observation variance is negative",
            ));
        }
        events
    }
}

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
    let mut ref_sd = None;
    for s in sd {
        if s.sig == ref_sig {
            ref_sd = Some(s);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for s in sd {
        if s.sig == ref_sig {
            continue;
        }
        if s.sig.sat.constellation != ref_sd.sig.sat.constellation {
            continue;
        }
        let corr = 0.0;
        let code_var = s.code_variance_m2 + ref_sd.code_variance_m2
            - 2.0 * corr * (s.code_variance_m2 * ref_sd.code_variance_m2).sqrt();
        let phase_var = s.phase_variance_cycles2 + ref_sd.phase_variance_cycles2
            - 2.0 * corr * (s.phase_variance_cycles2 * ref_sd.phase_variance_cycles2).sqrt();
        out.push(DdObservation {
            sig: s.sig,
            ref_sig: ref_sd.sig,
            signal_pseudorange_m: s.base_pseudorange_m,
            signal_timing: s.base_signal_timing,
            ref_signal_pseudorange_m: ref_sd.base_pseudorange_m,
            ref_signal_timing: ref_sd.base_signal_timing,
            code_m: s.code_m - ref_sd.code_m,
            phase_cycles: s.phase_cycles - ref_sd.phase_cycles,
            doppler_hz: s.doppler_hz - ref_sd.doppler_hz,
            variance_code: code_var,
            variance_phase: phase_var,
            canceled: vec![
                s.ambiguity_rover.clone(),
                s.ambiguity_base.clone(),
                ref_sd.ambiguity_rover.clone(),
                ref_sd.ambiguity_base.clone(),
            ],
        });
    }
    out
}

pub fn build_dd_per_constellation(
    sd: &[SdObservation],
    refs: &BTreeMap<Constellation, SigId>,
) -> Vec<DdObservation> {
    let mut out = Vec::new();
    for (constellation, ref_sig) in refs {
        let subset: Vec<SdObservation> = sd
            .iter()
            .filter(|s| s.ambiguity_rover.sig.sat.constellation == *constellation)
            .cloned()
            .collect();
        out.extend(build_dd(&subset, *ref_sig));
    }
    out
}

#[derive(Debug, Clone)]
pub struct DdCovarianceModel {
    pub variance_code: f64,
    pub variance_phase: f64,
    pub inter_frequency_corr: Option<f64>,
}

pub fn dd_covariance(dd: &DdObservation) -> DdCovarianceModel {
    DdCovarianceModel {
        variance_code: dd.variance_code,
        variance_phase: dd.variance_phase,
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
    if dd.len() < 3 {
        return None;
    }
    let mut h = Vec::new();
    let mut v = Vec::new();
    for obs in dd {
        let eph = ephs.iter().find(|e| e.sat == obs.sig.sat)?;
        let eph_ref = ephs.iter().find(|e| e.sat == obs.ref_sig.sat)?;
        let sat = bijux_gnss_nav::api::sat_state_gps_l1ca_from_observation(
            eph,
            t_rx_s,
            obs.signal_pseudorange_m,
            obs.signal_timing,
        );
        let sat_ref = bijux_gnss_nav::api::sat_state_gps_l1ca_from_observation(
            eph_ref,
            t_rx_s,
            obs.ref_signal_pseudorange_m,
            obs.ref_signal_timing,
        );
        let u = los_unit(base_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
        let u_ref = los_unit(base_ecef_m, [sat_ref.x_m, sat_ref.y_m, sat_ref.z_m]);
        h.push([u_ref[0] - u[0], u_ref[1] - u[1], u_ref[2] - u[2]]);
        v.push(obs.code_m);
    }
    let (bx, by, bz, cov) = solve_3x3(&h, &v)?;
    let mut baseline = baseline_from_ecef(
        base_ecef_m,
        [base_ecef_m[0] + bx, base_ecef_m[1] + by, base_ecef_m[2] + bz],
    );
    baseline.covariance_m2 = Some(cov);
    Some(baseline)
}

pub fn los_unit(base: [f64; 3], sat: [f64; 3]) -> [f64; 3] {
    let dx = base[0] - sat[0];
    let dy = base[1] - sat[1];
    let dz = base[2] - sat[2];
    let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    [dx / r, dy / r, dz / r]
}

fn solve_3x3(h: &[[f64; 3]], v: &[f64]) -> Option<(f64, f64, f64, [[f64; 3]; 3])> {
    let mut n = [[0.0_f64; 3]; 3];
    let mut u = [0.0_f64; 3];
    for (i, row) in h.iter().enumerate() {
        let wi = 1.0_f64;
        for r in 0..3 {
            u[r] += row[r] * v[i] * wi;
            for c in 0..3 {
                n[r][c] += row[r] * row[c] * wi;
            }
        }
    }
    let inv = invert_3x3(n)?;
    let dx = inv[0][0] * u[0] + inv[0][1] * u[1] + inv[0][2] * u[2];
    let dy = inv[1][0] * u[0] + inv[1][1] * u[1] + inv[1][2] * u[2];
    let dz = inv[2][0] * u[0] + inv[2][1] * u[1] + inv[2][2] * u[2];
    Some((dx, dy, dz, inv))
}

fn invert_3x3(a: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let mut inv = [[0.0_f64; 3]; 3];
    inv[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det;
    inv[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det;
    inv[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det;
    inv[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det;
    inv[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det;
    inv[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det;
    inv[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det;
    Some(inv)
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
