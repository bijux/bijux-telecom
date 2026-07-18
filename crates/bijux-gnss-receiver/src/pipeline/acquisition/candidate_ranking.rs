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
        && (left.doppler_rate_hz_per_s - right.doppler_rate_hz_per_s).abs() <= f64::EPSILON
        && (left.carrier_hz.0 - right.carrier_hz.0).abs() <= f64::EPSILON
}
