use bijux_gnss_core::api::{
    stable_acq_result_keys, AcqExplain, AcqExplainCandidate, AcqHypothesis,
};

use crate::engine::runtime::TraceRecord;

use super::candidate_decision::selected_reason_for_candidate;
use super::search_window::{SearchWindowDimension, SearchWindowEdge};
use super::wrong_prn_suppression::AcquisitionSatEvaluation;
use super::{Acquisition, AcquisitionRun};

impl Acquisition {
    pub(super) fn report_satellite_evaluations(
        &self,
        sat_evaluations: Vec<AcquisitionSatEvaluation>,
        emit_explanations: bool,
    ) -> AcquisitionRun {
        let mut results = Vec::with_capacity(sat_evaluations.len());
        let mut explains = Vec::with_capacity(sat_evaluations.len());
        for evaluation in sat_evaluations {
            let sat = evaluation.sat;
            let search_window_diagnostic = evaluation.search_window_diagnostic;
            let candidates = evaluation.candidates;
            if candidates.is_empty() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_sat_done",
                    fields: vec![
                        ("constellation", format!("{:?}", sat.constellation)),
                        ("prn", sat.prn.to_string()),
                        ("outcome", "no_candidates".to_string()),
                    ],
                });
                self.with_stats(|stats| {
                    stats.deferred_count = stats.deferred_count.saturating_add(1);
                });
                if emit_explanations {
                    explains.push(AcqExplain {
                        sat,
                        selected_rank: None,
                        selected_reason: "no_candidates".to_string(),
                        candidate_count: 0,
                        candidates: Vec::new(),
                    });
                }
                results.push(candidates);
                continue;
            }

            let best = &candidates[0];
            if emit_explanations {
                let selected_reason = match search_window_diagnostic.as_ref() {
                    Some(_) => "signal_outside_search_range",
                    None => selected_reason_for_candidate(best),
                };
                explains.push(AcqExplain {
                    sat,
                    selected_rank: Some(best.candidate_rank),
                    selected_reason: selected_reason.to_string(),
                    candidate_count: candidates.len(),
                    candidates: candidates
                        .iter()
                        .map(|candidate| AcqExplainCandidate {
                            rank: candidate.candidate_rank,
                            code_phase_samples: candidate.code_phase_samples,
                            carrier_hz: candidate.carrier_hz.0,
                            peak: candidate.peak,
                            peak_mean_ratio: candidate.peak_mean_ratio,
                            peak_second_ratio: candidate.peak_second_ratio,
                            second_peak_ratio: (if candidate.peak == 0.0 {
                                f32::INFINITY
                            } else {
                                candidate.peak / (candidate.second_peak + 1e-6)
                            }),
                            mean: candidate.mean,
                            hypothesis: candidate.hypothesis,
                            score: candidate.score,
                            threshold_hit: matches!(candidate.hypothesis, AcqHypothesis::Accepted),
                            reason: candidate
                                .explain_selection_reason
                                .clone()
                                .unwrap_or_else(|| "discarded".to_string()),
                        })
                        .collect(),
                });
            }

            let outcome = match best.hypothesis {
                AcqHypothesis::Accepted => "accepted",
                AcqHypothesis::Ambiguous => "ambiguous",
                AcqHypothesis::Rejected => "rejected",
                AcqHypothesis::Deferred => "deferred",
            };
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_sat_done",
                fields: vec![
                    ("constellation", format!("{:?}", best.sat.constellation)),
                    ("prn", best.sat.prn.to_string()),
                    ("outcome", outcome.to_string()),
                    (
                        "reason",
                        search_window_diagnostic.as_ref().map_or_else(
                            || selected_reason_for_candidate(best).to_string(),
                            |_| "signal_outside_search_range".to_string(),
                        ),
                    ),
                    ("carrier_hz", format!("{:.3}", best.carrier_hz.0)),
                    ("score", format!("{:.6}", best.score)),
                    ("peak_mean_ratio", format!("{:.6}", best.peak_mean_ratio)),
                ],
            });
            if let Some(diagnostic) = search_window_diagnostic.as_ref() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_search_window_rejection",
                    fields: vec![
                        ("constellation", format!("{:?}", best.sat.constellation)),
                        ("prn", best.sat.prn.to_string()),
                        ("reason", "signal_outside_search_range".to_string()),
                        ("edge", search_window_edge_name(diagnostic.edge).to_string()),
                        (
                            "dimension",
                            search_window_dimension_name(diagnostic.dimension).to_string(),
                        ),
                        ("best_axis_value", format!("{:.3}", diagnostic.best_axis_value)),
                        ("interior_axis_value", format!("{:.3}", diagnostic.interior_axis_value)),
                        ("best_peak_mean_ratio", format!("{:.6}", diagnostic.best_peak_mean_ratio)),
                        (
                            "interior_peak_mean_ratio",
                            format!("{:.6}", diagnostic.interior_peak_mean_ratio),
                        ),
                    ],
                });
            }
            if !best.evidence.is_empty() {
                self.runtime.trace.record(TraceRecord {
                    name: "acquisition_evidence",
                    fields: vec![
                        ("constellation", format!("{:?}", best.sat.constellation)),
                        ("prn", best.sat.prn.to_string()),
                        ("rank", format!("{}", best.evidence.first().map(|e| e.rank).unwrap_or(0))),
                        ("peak", format!("{:.6}", best.peak)),
                        ("second_peak", format!("{:.6}", best.second_peak)),
                        ("mean", format!("{:.6}", best.mean)),
                    ],
                });
            }
            self.with_stats(|stats| match best.hypothesis {
                AcqHypothesis::Accepted => {
                    stats.accepted_count = stats.accepted_count.saturating_add(1)
                }
                AcqHypothesis::Ambiguous => {
                    stats.ambiguous_count = stats.ambiguous_count.saturating_add(1)
                }
                AcqHypothesis::Rejected => {
                    stats.rejected_count = stats.rejected_count.saturating_add(1)
                }
                AcqHypothesis::Deferred => {
                    stats.deferred_count = stats.deferred_count.saturating_add(1)
                }
            });
            let stable_keys = stable_acq_result_keys(&candidates);
            self.runtime.trace.record(TraceRecord {
                name: "acquisition_stability_signature",
                fields: vec![
                    ("constellation", format!("{:?}", sat.constellation)),
                    ("prn", sat.prn.to_string()),
                    ("candidate_count", stable_keys.len().to_string()),
                    (
                        "top_signature",
                        stable_keys.first().cloned().unwrap_or_else(|| "none".to_string()),
                    ),
                ],
            });
            results.push(candidates);
        }
        AcquisitionRun { results, explains }
    }
}

fn search_window_edge_name(edge: SearchWindowEdge) -> &'static str {
    match edge {
        SearchWindowEdge::Lower => "lower",
        SearchWindowEdge::Upper => "upper",
    }
}

fn search_window_dimension_name(dimension: SearchWindowDimension) -> &'static str {
    match dimension {
        SearchWindowDimension::Doppler => "doppler",
        SearchWindowDimension::DopplerRate => "doppler_rate",
        SearchWindowDimension::CodePhase => "code_phase",
    }
}
