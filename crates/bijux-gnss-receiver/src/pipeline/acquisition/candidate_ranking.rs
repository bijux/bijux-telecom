use bijux_gnss_core::api::AcqResult;

pub(super) fn competing_candidate_ratio(candidates: &[AcqResult]) -> f32 {
    let Some(best) = candidates.first() else {
        return f32::INFINITY;
    };

    candidates
        .iter()
        .skip(1)
        .find(|candidate| {
            !same_acquisition_hypothesis(best, candidate)
                && candidate.peak_mean_ratio > f32::EPSILON
        })
        .map_or(f32::INFINITY, |competing| best.peak_mean_ratio / competing.peak_mean_ratio)
}

pub(super) fn has_strong_same_hypothesis_component_alternative(
    candidates: &[AcqResult],
    peak_mean_threshold: f32,
    peak_second_threshold: f32,
) -> bool {
    let Some(best) = candidates.first() else {
        return false;
    };
    candidates.iter().skip(1).any(|candidate| {
        same_acquisition_hypothesis(best, candidate)
            && component_strategy_differs(best, candidate)
            && candidate.peak_mean_ratio >= peak_mean_threshold
            && candidate.peak_second_ratio >= peak_second_threshold
    })
}

pub(super) fn same_acquisition_hypothesis(left: &AcqResult, right: &AcqResult) -> bool {
    left.sat == right.sat
        && left.signal_band == right.signal_band
        && left.signal_code == right.signal_code
        && left.glonass_frequency_channel == right.glonass_frequency_channel
        && left.code_phase_samples == right.code_phase_samples
        && (left.doppler_rate_hz_per_s - right.doppler_rate_hz_per_s).abs() <= f64::EPSILON
        && (left.carrier_hz.0 - right.carrier_hz.0).abs() <= f64::EPSILON
}

fn component_strategy_differs(left: &AcqResult, right: &AcqResult) -> bool {
    let Some(left_provenance) = left.component_provenance() else {
        return false;
    };
    let Some(right_provenance) = right.component_provenance() else {
        return false;
    };
    left_provenance.combination_mode != right_provenance.combination_mode
        || left_provenance.components.len() != right_provenance.components.len()
        || left_provenance
            .components
            .iter()
            .zip(right_provenance.components.iter())
            .any(|(left_component, right_component)| left_component.role != right_component.role)
}
