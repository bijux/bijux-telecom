use bijux_gnss_core::api::{AcqHypothesis, AcqResult, SatId};

use crate::pipeline::acquisition::peak_metrics::DelayedSecondaryPeakDiagnostic;
use crate::pipeline::acquisition::threshold_resolution::ResolvedAcquisitionThresholds;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AcquisitionDecisionReason {
    AcceptedByRatioThresholds,
    AmbiguousRatioThresholds,
    MultipathSuspect,
    LowPeakMetric,
}

impl AcquisitionDecisionReason {
    pub(super) fn as_str(self) -> &'static str {
        match self {
            AcquisitionDecisionReason::AcceptedByRatioThresholds => "accepted_by_ratio_thresholds",
            AcquisitionDecisionReason::AmbiguousRatioThresholds => "ambiguous_ratio_thresholds",
            AcquisitionDecisionReason::MultipathSuspect => "multipath_suspect",
            AcquisitionDecisionReason::LowPeakMetric => "low_peak_metric",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct AcquisitionDecision {
    pub(super) hypothesis: AcqHypothesis,
    pub(super) reason: AcquisitionDecisionReason,
    pub(super) score: f32,
}

pub(super) fn acquisition_decision(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    local_peak_separation_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &ResolvedAcquisitionThresholds,
) -> AcquisitionDecision {
    if peak_mean_ratio < thresholds.peak_mean_threshold {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Rejected,
            reason: AcquisitionDecisionReason::LowPeakMetric,
            score: 0.0,
        };
    }
    if local_peak_separation_ratio.is_nan() || competing_peak_ratio.is_nan() {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Ambiguous,
            reason: AcquisitionDecisionReason::AmbiguousRatioThresholds,
            score: 0.25 * peak_mean_ratio,
        };
    }
    let limiting_ratio =
        peak_second_ratio.min(local_peak_separation_ratio).min(competing_peak_ratio);
    if peak_second_ratio < thresholds.peak_second_threshold
        || local_peak_separation_ratio < thresholds.peak_second_threshold
        || competing_peak_ratio < thresholds.peak_second_threshold
    {
        return AcquisitionDecision {
            hypothesis: AcqHypothesis::Ambiguous,
            reason: AcquisitionDecisionReason::AmbiguousRatioThresholds,
            score: (peak_mean_ratio * 0.35) + (limiting_ratio * 0.15),
        };
    }
    AcquisitionDecision {
        hypothesis: AcqHypothesis::Accepted,
        reason: AcquisitionDecisionReason::AcceptedByRatioThresholds,
        score: (peak_mean_ratio * 0.5) + (peak_second_ratio * 0.5),
    }
}

pub(super) fn selected_reason_for_candidate(candidate: &AcqResult) -> &'static str {
    if let Some(reason) = candidate.explain_selection_reason.as_deref() {
        if let Some(reason_prefix) = known_selection_reason_prefix(reason) {
            return reason_prefix;
        }
    }
    match candidate.hypothesis {
        AcqHypothesis::Accepted => AcquisitionDecisionReason::AcceptedByRatioThresholds.as_str(),
        AcqHypothesis::Ambiguous => AcquisitionDecisionReason::AmbiguousRatioThresholds.as_str(),
        AcqHypothesis::Rejected => AcquisitionDecisionReason::LowPeakMetric.as_str(),
        AcqHypothesis::Deferred => "deferred",
    }
}

fn known_selection_reason_prefix(reason: &str) -> Option<&'static str> {
    let reason_prefix = reason.split_once(':').map_or(reason, |(prefix, _)| prefix);
    match reason_prefix {
        "accepted_by_ratio_thresholds" => Some("accepted_by_ratio_thresholds"),
        "ambiguous_ratio_thresholds" => Some("ambiguous_ratio_thresholds"),
        "multipath_suspect" => Some("multipath_suspect"),
        "low_peak_metric" => Some("low_peak_metric"),
        "ranked_alternative" => Some("ranked_alternative"),
        "wrong_prn_correlation" => Some("wrong_prn_correlation"),
        "missing_glonass_frequency_channel" => Some("missing_glonass_frequency_channel"),
        "insufficient_frame" => Some("insufficient_frame"),
        "unsupported_coherent_integration_ms" => Some("unsupported_coherent_integration_ms"),
        _ => None,
    }
}

pub(super) fn ranked_alternative_candidate_reason(
    primary: &AcqResult,
    alternative: &AcqResult,
) -> String {
    format!(
        "ranked_alternative: preserved rank {} candidate behind rank {} primary (peak_mean_ratio {:.6} < {:.6}, peak_second_ratio {:.6} vs {:.6})",
        alternative.candidate_rank,
        primary.candidate_rank,
        alternative.peak_mean_ratio,
        primary.peak_mean_ratio,
        alternative.peak_second_ratio,
        primary.peak_second_ratio,
    )
}

pub(super) fn wrong_prn_candidate_reason(
    candidate_sat: SatId,
    dominant_sat: SatId,
    candidate_peak_mean_ratio: f32,
    dominant_peak_mean_ratio: f32,
    dominance_ratio: f32,
    candidate_peak_second_ratio: f32,
) -> String {
    format!(
        "wrong_prn_correlation: prn {} suppressed by stronger prn {} (peak_mean_ratio {:.6} vs {:.6}, dominance_ratio {:.6}, peak_second_ratio {:.6})",
        candidate_sat.prn,
        dominant_sat.prn,
        candidate_peak_mean_ratio,
        dominant_peak_mean_ratio,
        dominance_ratio,
        candidate_peak_second_ratio,
    )
}

pub(super) fn selected_candidate_reason(
    decision: AcquisitionDecision,
    peak_mean_ratio: f32,
    local_peak_separation_ratio: f32,
    competing_peak_ratio: f32,
    thresholds: &ResolvedAcquisitionThresholds,
) -> String {
    match decision.reason {
        AcquisitionDecisionReason::AcceptedByRatioThresholds
        | AcquisitionDecisionReason::AmbiguousRatioThresholds => format!(
            "{}: peak_mean_ratio={:.6}, local_peak_separation_ratio={:.6}, competing_peak_ratio={:.6}, score={:.6}",
            decision.reason.as_str(),
            peak_mean_ratio,
            local_peak_separation_ratio,
            competing_peak_ratio,
            decision.score,
        ),
        AcquisitionDecisionReason::LowPeakMetric => format!(
            "{}: peak_mean_ratio={:.6} below threshold {:.6}",
            decision.reason.as_str(),
            peak_mean_ratio,
            thresholds.peak_mean_threshold,
        ),
        AcquisitionDecisionReason::MultipathSuspect => unreachable!(
            "multipath_suspect reasons must be rendered through multipath_candidate_reason"
        ),
    }
}

pub(super) fn multipath_suspect_decision(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    diagnostic: &DelayedSecondaryPeakDiagnostic,
) -> AcquisitionDecision {
    let delay_penalty = 1.0 / (1.0 + diagnostic.delay_samples as f32);
    let limiting_ratio = peak_second_ratio.min(competing_peak_ratio);
    AcquisitionDecision {
        hypothesis: AcqHypothesis::Ambiguous,
        reason: AcquisitionDecisionReason::MultipathSuspect,
        score: (peak_mean_ratio * 0.35) + (limiting_ratio * 0.15) - delay_penalty.min(0.1),
    }
}

pub(super) fn multipath_candidate_reason(
    peak_mean_ratio: f32,
    peak_second_ratio: f32,
    competing_peak_ratio: f32,
    diagnostic: &DelayedSecondaryPeakDiagnostic,
    samples_per_code: usize,
    code_length: usize,
    score: f32,
) -> String {
    let delay_chips = diagnostic.delay_samples as f64 * code_length.max(1) as f64
        / samples_per_code.max(1) as f64;
    format!(
        "multipath_suspect: delayed secondary peak at sample {} (+{} samples, {:.3} chips) with peak_mean_ratio={:.6}, peak_second_ratio={:.6}, competing_peak_ratio={:.6}, score={:.6}",
        diagnostic.secondary_code_phase_samples,
        diagnostic.delay_samples,
        delay_chips,
        peak_mean_ratio,
        peak_second_ratio,
        competing_peak_ratio,
        score,
    )
}
