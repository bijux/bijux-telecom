use bijux_gnss_core::api::{AcqHypothesis, AcqRequest, AcqResult, SamplesFrame};
use bijux_gnss_signal::api::AcquisitionSignalModel;

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::acquisition_assistance::ResolvedAcquisitionSearchBounds;
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;

use crate::pipeline::acquisition::candidate_decision::selected_reason_for_candidate;

use super::code_phase_profile::{measure_code_phase_profile, CodePhaseProfileRequest};
use super::doppler_refinement::{
    find_best_candidate_by_carrier_hz_and_rate, find_candidate_by_carrier_hz,
};

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

pub(super) struct AssistedCodePhaseWindowDiagnosticRequest<'a> {
    pub(super) config: &'a ReceiverPipelineConfig,
    pub(super) signal_model: &'a AcquisitionSignalModel,
    pub(super) frame: &'a SamplesFrame,
    pub(super) carrier_hz: f64,
    pub(super) doppler_rate_hz_per_s: f64,
    pub(super) coherent_ms: u32,
    pub(super) noncoherent: u32,
    pub(super) resolved_bounds: &'a ResolvedAcquisitionSearchBounds,
    pub(super) peak_mean_threshold: f32,
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

pub(super) fn signal_outside_search_range(
    candidates: &[AcqResult],
    intermediate_freq_hz: f64,
    doppler_search_hz: i32,
    doppler_step_hz: i32,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if candidates.len() < 2 || doppler_search_hz <= 0 || doppler_step_hz <= 0 {
        return None;
    }

    let best_peak_mean_ratio =
        candidates.iter().map(|candidate| candidate.peak_mean_ratio).fold(0.0_f32, f32::max);
    let hint_threshold =
        peak_mean_threshold.max(best_peak_mean_ratio * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION);
    [
        (
            SearchWindowEdge::Lower,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, -(doppler_search_hz as f64)),
            carrier_hz_from_doppler_hz(
                intermediate_freq_hz,
                -((doppler_search_hz - doppler_step_hz) as f64),
            ),
        ),
        (
            SearchWindowEdge::Upper,
            carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_search_hz as f64),
            carrier_hz_from_doppler_hz(
                intermediate_freq_hz,
                (doppler_search_hz - doppler_step_hz) as f64,
            ),
        ),
    ]
    .into_iter()
    .filter_map(|(edge, edge_carrier_hz, interior_carrier_hz)| {
        let edge_candidate = find_candidate_by_carrier_hz(candidates, edge_carrier_hz)?;
        let interior_candidate = find_candidate_by_carrier_hz(candidates, interior_carrier_hz)?;
        if edge_candidate.peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_candidate.peak_mean_ratio
            <= interior_candidate.peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            dimension: SearchWindowDimension::Doppler,
            edge,
            best_axis_value: edge_candidate.carrier_hz.0,
            interior_axis_value: interior_candidate.carrier_hz.0,
            best_peak_mean_ratio: edge_candidate.peak_mean_ratio,
            interior_peak_mean_ratio: interior_candidate.peak_mean_ratio,
        })
    })
    .max_by(|left, right| {
        left.best_peak_mean_ratio
            .partial_cmp(&right.best_peak_mean_ratio)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}

pub(super) fn assisted_code_phase_search_window_diagnostic(
    request: AssistedCodePhaseWindowDiagnosticRequest<'_>,
) -> Option<SearchWindowDiagnostic> {
    let AssistedCodePhaseWindowDiagnosticRequest {
        config,
        signal_model,
        frame,
        carrier_hz,
        doppler_rate_hz_per_s,
        coherent_ms,
        noncoherent,
        resolved_bounds,
        peak_mean_threshold,
    } = request;
    if resolved_bounds.code_phase_search_bins == 0
        || resolved_bounds.code_phase_search_mode == "full_code"
    {
        return None;
    }
    let correlation_profile = measure_code_phase_profile(CodePhaseProfileRequest {
        config,
        signal_model,
        frame,
        carrier_hz,
        doppler_rate_hz_per_s,
        coherent_ms,
        noncoherent,
    })?;
    code_phase_outside_search_range(
        &correlation_profile,
        resolved_bounds.code_phase_search_start_sample,
        resolved_bounds.code_phase_search_step_samples,
        resolved_bounds.code_phase_search_bins,
        peak_mean_threshold,
    )
}

pub(super) fn signal_outside_doppler_rate_search_range(
    candidates: &[AcqResult],
    carrier_hz: f64,
    doppler_rate_center_hz_per_s: f64,
    doppler_rate_search_hz_per_s: i32,
    doppler_rate_step_hz_per_s: i32,
    peak_mean_threshold: f32,
) -> Option<SearchWindowDiagnostic> {
    if candidates.len() < 2 || doppler_rate_search_hz_per_s <= 0 || doppler_rate_step_hz_per_s <= 0
    {
        return None;
    }

    let best_peak_mean_ratio = candidates
        .iter()
        .filter(|candidate| (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON)
        .map(|candidate| candidate.peak_mean_ratio)
        .fold(0.0_f32, f32::max);
    if best_peak_mean_ratio <= f32::EPSILON {
        return None;
    }
    let hint_threshold =
        peak_mean_threshold.max(best_peak_mean_ratio * SEARCH_EDGE_HINT_PEAK_MEAN_RATIO_FRACTION);
    [
        (
            SearchWindowEdge::Lower,
            doppler_rate_center_hz_per_s - doppler_rate_search_hz_per_s as f64,
            doppler_rate_center_hz_per_s
                - (doppler_rate_search_hz_per_s - doppler_rate_step_hz_per_s) as f64,
        ),
        (
            SearchWindowEdge::Upper,
            doppler_rate_center_hz_per_s + doppler_rate_search_hz_per_s as f64,
            doppler_rate_center_hz_per_s
                + (doppler_rate_search_hz_per_s - doppler_rate_step_hz_per_s) as f64,
        ),
    ]
    .into_iter()
    .filter_map(|(edge, edge_rate_hz_per_s, interior_rate_hz_per_s)| {
        let edge_candidate =
            find_best_candidate_by_carrier_hz_and_rate(candidates, carrier_hz, edge_rate_hz_per_s)?;
        let interior_candidate = find_best_candidate_by_carrier_hz_and_rate(
            candidates,
            carrier_hz,
            interior_rate_hz_per_s,
        )?;
        if edge_candidate.peak_mean_ratio < hint_threshold {
            return None;
        }
        if edge_candidate.peak_mean_ratio
            <= interior_candidate.peak_mean_ratio * (1.0 + SEARCH_EDGE_RISE_RATIO_EPSILON)
        {
            return None;
        }
        Some(SearchWindowDiagnostic {
            dimension: SearchWindowDimension::DopplerRate,
            edge,
            best_axis_value: edge_candidate.doppler_rate_hz_per_s,
            interior_axis_value: interior_candidate.doppler_rate_hz_per_s,
            best_peak_mean_ratio: edge_candidate.peak_mean_ratio,
            interior_peak_mean_ratio: interior_candidate.peak_mean_ratio,
        })
    })
    .max_by(|left, right| {
        left.best_peak_mean_ratio
            .partial_cmp(&right.best_peak_mean_ratio)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
}
