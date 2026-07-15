use bijux_gnss_core::api::{AcqHypothesis, AcqRequest, AcqResult};

use crate::pipeline::acquisition_assistance::ResolvedAcquisitionSearchBounds;

use super::selected_reason_for_candidate;

pub(super) const SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION: f32 = 0.9;
pub(super) const SEARCH_EDGE_RISE_RATIO_EPSILON: f32 = 0.05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SearchWindowEdge {
    Lower,
    Upper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SearchWindowDimension {
    Doppler,
    DopplerRate,
    CodePhase,
}

#[derive(Debug, Clone)]
pub(super) struct SearchWindowDiagnostic {
    pub(super) dimension: SearchWindowDimension,
    pub(super) edge: SearchWindowEdge,
    pub(super) best_axis_value: f64,
    pub(super) interior_axis_value: f64,
    pub(super) best_peak_mean_ratio: f32,
    pub(super) interior_peak_mean_ratio: f32,
}

fn candidate_is_trackable(candidate: &AcqResult) -> bool {
    matches!(candidate.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
}

pub(super) fn should_retry_assisted_search(
    request: AcqRequest,
    resolved_bounds: &ResolvedAcquisitionSearchBounds,
    primary_candidate: Option<&AcqResult>,
    search_window_diagnostic: Option<&SearchWindowDiagnostic>,
) -> bool {
    request.assistance_bounds.is_some()
        && resolved_bounds.search_domain_reduced
        && (search_window_diagnostic.is_some()
            || !primary_candidate.is_some_and(candidate_is_trackable))
}

pub(super) fn search_window_candidate_reason(diagnostic: &SearchWindowDiagnostic) -> String {
    let edge = match diagnostic.edge {
        SearchWindowEdge::Lower => "lower",
        SearchWindowEdge::Upper => "upper",
    };
    match diagnostic.dimension {
        SearchWindowDimension::Doppler => format!(
            "signal_outside_search_range: best carrier {:.3} Hz sits on the {edge} search edge and exceeds the interior neighbor at {:.3} Hz (peak_mean_ratio {:.6} > {:.6})",
            diagnostic.best_axis_value,
            diagnostic.interior_axis_value,
            diagnostic.best_peak_mean_ratio,
            diagnostic.interior_peak_mean_ratio,
        ),
        SearchWindowDimension::DopplerRate => format!(
            "signal_outside_search_range: best Doppler rate {:.3} Hz/s sits on the {edge} search edge and exceeds the interior neighbor at {:.3} Hz/s (peak_mean_ratio {:.6} > {:.6})",
            diagnostic.best_axis_value,
            diagnostic.interior_axis_value,
            diagnostic.best_peak_mean_ratio,
            diagnostic.interior_peak_mean_ratio,
        ),
        SearchWindowDimension::CodePhase => format!(
            "signal_outside_search_range: best code phase {:.0} samples sits on the {edge} search edge and exceeds the interior neighbor at {:.0} samples (peak_mean_ratio {:.6} > {:.6})",
            diagnostic.best_axis_value,
            diagnostic.interior_axis_value,
            diagnostic.best_peak_mean_ratio,
            diagnostic.interior_peak_mean_ratio,
        ),
    }
}

pub(super) fn assisted_search_fallback_reason(
    primary_candidate: Option<&AcqResult>,
    search_window_diagnostic: Option<&SearchWindowDiagnostic>,
) -> String {
    match search_window_diagnostic {
        Some(diagnostic) => {
            format!("assistance_bounds_fallback: {}", search_window_candidate_reason(diagnostic))
        }
        None => {
            let candidate_reason = primary_candidate
                .and_then(|candidate| candidate.explain_selection_reason.as_deref())
                .unwrap_or("bounded search returned no trackable candidate");
            format!("assistance_bounds_fallback: {candidate_reason}")
        }
    }
}

pub(super) fn append_assisted_search_fallback_reason(
    candidates: &mut [AcqResult],
    fallback_reason: &str,
) {
    if let Some(primary_candidate) = candidates.first_mut() {
        let existing_reason = primary_candidate
            .explain_selection_reason
            .clone()
            .unwrap_or_else(|| selected_reason_for_candidate(primary_candidate).to_string());
        primary_candidate.explain_selection_reason =
            Some(format!("{existing_reason}; {fallback_reason}"));
    }
}

pub(super) fn code_phase_outside_search_range(
    correlation_profile: &[f32],
    search_start_sample: usize,
    search_step_samples: usize,
    search_bins: usize,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if correlation_profile.len() < 2
        || search_bins < 2
        || search_bins >= correlation_profile.len()
        || search_step_samples == 0
    {
        return None;
    }

    let mean = correlation_profile.iter().copied().sum::<f32>() / correlation_profile.len() as f32;
    let peak_mean_ratio = |idx: usize| correlation_profile[idx] / (mean + 1.0e-6);
    let best_peak_mean_ratio =
        correlation_profile.iter().copied().fold(0.0_f32, f32::max) / (mean + 1.0e-6);
    let hint_threshold =
        peak_mean_threshold.max(best_peak_mean_ratio * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION);
    let lower_edge_index = search_start_sample % correlation_profile.len();
    let lower_interior_index = (lower_edge_index + search_step_samples) % correlation_profile.len();
    let upper_edge_index =
        (search_start_sample + (search_bins - 1) * search_step_samples) % correlation_profile.len();
    let upper_interior_index = (upper_edge_index + correlation_profile.len()
        - search_step_samples % correlation_profile.len())
        % correlation_profile.len();

    [
        (SearchWindowEdge::Lower, lower_edge_index, lower_interior_index),
        (SearchWindowEdge::Upper, upper_edge_index, upper_interior_index),
    ]
    .into_iter()
    .filter_map(|(edge, edge_index, interior_index)| {
        let edge_peak_mean_ratio = peak_mean_ratio(edge_index);
        let interior_peak_mean_ratio = peak_mean_ratio(interior_index);
        if edge_peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_peak_mean_ratio <= interior_peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            dimension: SearchWindowDimension::CodePhase,
            edge,
            best_axis_value: edge_index as f64,
            interior_axis_value: interior_index as f64,
            best_peak_mean_ratio: edge_peak_mean_ratio,
            interior_peak_mean_ratio,
        })
    })
    .max_by(|left, right| {
        left.best_peak_mean_ratio
            .partial_cmp(&right.best_peak_mean_ratio)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}
