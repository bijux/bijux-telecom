use bijux_gnss_core::api::{AcqHypothesis, AcqResult, SatId};

use crate::pipeline::acquisition::candidate_decision::wrong_prn_candidate_reason;
use crate::pipeline::acquisition::search_window::SearchWindowDiagnostic;

const WRONG_PRN_DOMINANCE_RATIO_MIN: f32 = 4.0;
const WRONG_PRN_PEAK_SECOND_RATIO_MAX: f32 = 1.1;

#[derive(Debug, Clone)]
pub(super) struct AcquisitionSatEvaluation {
    pub(super) sat: SatId,
    pub(super) candidates: Vec<AcqResult>,
    pub(super) search_window_diagnostic: Option<SearchWindowDiagnostic>,
}

pub(super) fn suppress_wrong_prn_correlations(sat_evaluations: &mut [AcquisitionSatEvaluation]) {
    let dominant = sat_evaluations
        .iter()
        .filter_map(|evaluation| evaluation.candidates.first())
        .filter(|candidate| {
            matches!(candidate.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
        })
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|candidate| (candidate.sat, candidate.peak_mean_ratio));

    let Some((dominant_sat, dominant_peak_mean_ratio)) = dominant else {
        return;
    };

    for evaluation in sat_evaluations {
        if evaluation.search_window_diagnostic.is_some() {
            continue;
        }
        let Some(candidate) = evaluation.candidates.first_mut() else {
            continue;
        };
        if candidate.sat == dominant_sat
            || !matches!(candidate.hypothesis, AcqHypothesis::Ambiguous)
        {
            continue;
        }
        if candidate.peak_second_ratio >= WRONG_PRN_PEAK_SECOND_RATIO_MAX
            || candidate.peak_mean_ratio <= f32::EPSILON
        {
            continue;
        }
        let dominance_ratio = dominant_peak_mean_ratio / candidate.peak_mean_ratio;
        if dominance_ratio < WRONG_PRN_DOMINANCE_RATIO_MIN {
            continue;
        }
        candidate.hypothesis = AcqHypothesis::Rejected;
        candidate.score = 0.0;
        candidate.explain_selection_reason = Some(wrong_prn_candidate_reason(
            candidate.sat,
            dominant_sat,
            candidate.peak_mean_ratio,
            dominant_peak_mean_ratio,
            dominance_ratio,
            candidate.peak_second_ratio,
        ));
    }
}
