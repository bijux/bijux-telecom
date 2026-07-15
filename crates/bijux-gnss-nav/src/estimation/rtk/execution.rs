#![allow(missing_docs)]

use std::collections::BTreeMap;

use super::quality::{jitter_summary, BaselineSolution, JitterSummary};
use crate::api::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation,
    rtk_double_differences_by_constellation, rtk_double_differences_from_single_differences,
    rtk_float_baseline_from_double_differences,
    rtk_single_differences_from_aligned_obs_epochs, rtk_single_differences_from_obs_epochs,
    GpsEphemeris, RtkDoubleDifferenceObservation, RtkFloatBaselineSolution,
    RtkSingleDifferenceObservation,
};
use bijux_gnss_core::api::{
    Constellation, Cycles, DoubleDifference, Hertz, Meters, ObsEpoch, ReceiverRole, SigId,
    SingleDifference,
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
        if !self.tolerance_s.is_finite() || self.tolerance_s < 0.0 {
            self.dropped_base += base.len();
            self.dropped_rover += rover.len();
            return out;
        }
        while i < base.len() && j < rover.len() {
            let b = &base[i];
            let r = &rover[j];
            if !b.t_rx_s.0.is_finite() {
                self.dropped_base += 1;
                i += 1;
                continue;
            }
            if !r.t_rx_s.0.is_finite() {
                self.dropped_rover += 1;
                j += 1;
                continue;
            }
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
        self.dropped_base += base.len().saturating_sub(i);
        self.dropped_rover += rover.len().saturating_sub(j);
        out
    }

    pub fn report(&self, base_len: usize, rover_len: usize) -> AlignmentReport {
        let total = base_len.max(rover_len).max(1) as f64;
        let matched_pct = (self.aligned as f64) / total;
        AlignmentReport {
            tolerance_s: self.tolerance_s,
            attempted_base_epochs: base_len,
            attempted_rover_epochs: rover_len,
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

pub fn build_sd_with_alignment_tolerance(
    base: &ObsEpoch,
    rover: &ObsEpoch,
    tolerance_s: f64,
) -> Vec<SdObservation> {
    rtk_single_differences_from_aligned_obs_epochs(base, rover, tolerance_s)
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

pub fn single_difference(rover: &ObsEpoch, base: &ObsEpoch) -> Vec<SingleDifference> {
    build_sd(base, rover)
        .into_iter()
        .map(|observation| SingleDifference {
            sig: observation.sig,
            code_m: Meters(observation.code_m),
            phase_cycles: Cycles(observation.phase_cycles),
            doppler_hz: Hertz(observation.doppler_hz),
            ambiguity_rover: observation.ambiguity_rover,
            ambiguity_base: observation.ambiguity_base,
        })
        .collect()
}

pub fn double_difference(
    single_differences: &[SingleDifference],
    ref_sig: SigId,
) -> Vec<DoubleDifference> {
    let mut ref_sd = None;
    for sd in single_differences {
        if sd.sig == ref_sig {
            ref_sd = Some(sd);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for sd in single_differences {
        if sd.sig == ref_sig {
            continue;
        }
        out.push(DoubleDifference {
            ref_sig: ref_sd.sig,
            sig: sd.sig,
            code_m: Meters(sd.code_m.0 - ref_sd.code_m.0),
            phase_cycles: Cycles(sd.phase_cycles.0 - ref_sd.phase_cycles.0),
            doppler_hz: Hertz(sd.doppler_hz.0 - ref_sd.doppler_hz.0),
            canceled: vec![
                ref_sd.ambiguity_rover.clone(),
                ref_sd.ambiguity_base.clone(),
                sd.ambiguity_rover.clone(),
                sd.ambiguity_base.clone(),
            ],
        });
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
    ephs: &[GpsEphemeris],
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
    ephs: &[GpsEphemeris],
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
    pub tolerance_s: f64,
    pub attempted_base_epochs: usize,
    pub attempted_rover_epochs: usize,
    pub base: AlignmentDiagnostic,
    pub rover: AlignmentDiagnostic,
    pub matched_pct: f64,
    pub jitter: JitterSummary,
}

#[cfg(test)]
mod tests {
    use super::EpochAligner;
    use bijux_gnss_core::api::{
        ObsEpoch, ObservationEpochDecision, ReceiverRole, ReceiverSampleTrace, Seconds,
    };

    #[test]
    fn epoch_aligner_reports_unmatched_tail_epochs() {
        let base = vec![empty_epoch(0.0), empty_epoch(0.001), empty_epoch(0.002)];
        let rover = vec![empty_epoch(0.0)];
        let mut aligner = EpochAligner::new(0.0001);

        let aligned = aligner.align(&base, &rover);
        let report = aligner.report(base.len(), rover.len());

        assert_eq!(aligned.len(), 1);
        assert_eq!(report.base.aligned, 1);
        assert_eq!(report.base.dropped, 2);
        assert_eq!(report.rover.dropped, 0);
        assert_eq!(report.attempted_base_epochs, 3);
        assert_eq!(report.attempted_rover_epochs, 1);
        assert_eq!(report.tolerance_s, 0.0001);
    }

    #[test]
    fn epoch_aligner_refuses_invalid_tolerance() {
        let base = vec![empty_epoch(0.0)];
        let rover = vec![empty_epoch(0.0)];
        let mut aligner = EpochAligner::new(f64::NAN);

        let aligned = aligner.align(&base, &rover);
        let report = aligner.report(base.len(), rover.len());

        assert!(aligned.is_empty());
        assert_eq!(report.base.dropped, 1);
        assert_eq!(report.rover.dropped, 1);
    }

    fn empty_epoch(t_rx_s: f64) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(t_rx_s),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: Vec::new(),
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("test_epoch".to_string()),
            manifest: None,
        }
    }
}
