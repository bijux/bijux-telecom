use bijux_gnss_core::api::{AcqExplain, AcqHypothesis, AcqResult};

use crate::pipeline::acquisition_assistance::RelatedSignalFollowUpRequest;

fn related_signal_follow_up_label(follow_up_request: &RelatedSignalFollowUpRequest) -> String {
    format!(
        "same_satellite_cross_band_assistance(source={:?}:{:?},target={:?}:{:?})",
        follow_up_request.source_signal_band,
        follow_up_request.source_signal_code,
        follow_up_request.request.signal_band,
        follow_up_request.request.signal_code,
    )
}

fn related_signal_hypothesis_rank(hypothesis: AcqHypothesis) -> u8 {
    match hypothesis {
        AcqHypothesis::Accepted => 3,
        AcqHypothesis::Ambiguous => 2,
        AcqHypothesis::Rejected => 1,
        AcqHypothesis::Deferred => 0,
    }
}

pub(super) fn should_replace_related_signal_row(
    existing_candidates: &[AcqResult],
    replacement_candidates: &[AcqResult],
) -> bool {
    let Some(replacement_primary) = replacement_candidates.first() else {
        return false;
    };
    let Some(existing_primary) = existing_candidates.first() else {
        return true;
    };
    let replacement_rank = related_signal_hypothesis_rank(replacement_primary.hypothesis);
    let existing_rank = related_signal_hypothesis_rank(existing_primary.hypothesis);
    replacement_rank > existing_rank
        || (replacement_rank == existing_rank
            && (replacement_primary.score > existing_primary.score
                || (existing_primary
                    .assumptions
                    .as_ref()
                    .and_then(|assumptions| assumptions.assistance_bounds)
                    .is_none()
                    && replacement_primary
                        .assumptions
                        .as_ref()
                        .and_then(|assumptions| assumptions.assistance_bounds)
                        .is_some())))
}

pub(super) fn annotate_related_signal_follow_up_candidates(
    candidates: &mut [AcqResult],
    follow_up_request: &RelatedSignalFollowUpRequest,
) {
    let label = related_signal_follow_up_label(follow_up_request);
    for candidate in candidates {
        let reason = candidate
            .explain_selection_reason
            .take()
            .map(|existing| format!("{label}; {existing}"))
            .unwrap_or_else(|| label.clone());
        candidate.explain_selection_reason = Some(reason);
    }
}

pub(super) fn annotate_related_signal_follow_up_explain(
    mut explain: AcqExplain,
    follow_up_request: &RelatedSignalFollowUpRequest,
) -> AcqExplain {
    let label = related_signal_follow_up_label(follow_up_request);
    explain.selected_reason = format!("{label}; {}", explain.selected_reason);
    for candidate in &mut explain.candidates {
        candidate.reason = format!("{label}; {}", candidate.reason);
    }
    explain
}
