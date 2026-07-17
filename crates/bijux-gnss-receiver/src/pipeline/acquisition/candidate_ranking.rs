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

fn same_acquisition_hypothesis(left: &AcqResult, right: &AcqResult) -> bool {
    left.sat == right.sat
        && left.signal_band == right.signal_band
        && left.signal_code == right.signal_code
        && left.glonass_frequency_channel == right.glonass_frequency_channel
        && left.code_phase_samples == right.code_phase_samples
        && same_component_provenance(left, right)
        && (left.doppler_rate_hz_per_s - right.doppler_rate_hz_per_s).abs() <= f64::EPSILON
        && (left.carrier_hz.0 - right.carrier_hz.0).abs() <= f64::EPSILON
}

fn same_component_provenance(left: &AcqResult, right: &AcqResult) -> bool {
    match (left.component_provenance(), right.component_provenance()) {
        (Some(left), Some(right)) => {
            left.combination_mode == right.combination_mode
                && left.components.len() == right.components.len()
                && left.components.iter().zip(&right.components).all(
                    |(left_component, right_component)| {
                        left_component.role == right_component.role
                            && left_component.secondary_code_phase_periods
                                == right_component.secondary_code_phase_periods
                    },
                )
        }
        (None, None) => true,
        _ => false,
    }
}
