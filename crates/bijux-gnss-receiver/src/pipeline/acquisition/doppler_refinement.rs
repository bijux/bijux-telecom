use bijux_gnss_core::api::{AcqDopplerRefinement, AcqResult, Hertz};

const DOPPLER_REFINEMENT_METHOD: &str = "parabolic_peak";
const DOPPLER_REFINEMENT_MAX_OFFSET_BINS: f64 = 0.5;
const DOPPLER_REFINEMENT_EPSILON: f64 = 1.0e-12;

pub(super) fn estimate_acquisition_doppler_refinement(
    coarse_carrier_hz: f64,
    coarse_doppler_rate_hz_per_s: f64,
    grid_candidates: &[AcqResult],
    doppler_step_hz: i32,
) -> Option<AcqDopplerRefinement> {
    if doppler_step_hz <= 0 {
        return None;
    }
    let step_hz = doppler_step_hz as f64;
    let left = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz - step_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    let center = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    let right = find_best_candidate_by_carrier_hz_and_rate(
        grid_candidates,
        coarse_carrier_hz + step_hz,
        coarse_doppler_rate_hz_per_s,
    )?;
    if center.peak_mean_ratio < left.peak_mean_ratio
        || center.peak_mean_ratio < right.peak_mean_ratio
    {
        return None;
    }

    let left_peak_mean_ratio = left.peak_mean_ratio as f64;
    let center_peak_mean_ratio = center.peak_mean_ratio as f64;
    let right_peak_mean_ratio = right.peak_mean_ratio as f64;
    let denominator = left_peak_mean_ratio - (2.0 * center_peak_mean_ratio) + right_peak_mean_ratio;
    if !denominator.is_finite() || denominator.abs() <= DOPPLER_REFINEMENT_EPSILON {
        return None;
    }

    let raw_offset_bins = 0.5 * (left_peak_mean_ratio - right_peak_mean_ratio) / denominator;
    if !raw_offset_bins.is_finite() {
        return None;
    }
    let offset_bins = raw_offset_bins
        .clamp(-DOPPLER_REFINEMENT_MAX_OFFSET_BINS, DOPPLER_REFINEMENT_MAX_OFFSET_BINS);

    Some(AcqDopplerRefinement {
        method: DOPPLER_REFINEMENT_METHOD.to_string(),
        coarse_carrier_hz: Hertz(coarse_carrier_hz),
        offset_hz: offset_bins * step_hz,
        offset_bins,
        left_peak_mean_ratio: left.peak_mean_ratio,
        center_peak_mean_ratio: center.peak_mean_ratio,
        right_peak_mean_ratio: right.peak_mean_ratio,
    })
}

pub(super) fn find_candidate_by_carrier_hz(
    candidates: &[AcqResult],
    carrier_hz: f64,
) -> Option<&AcqResult> {
    candidates
        .iter()
        .filter(|candidate| (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON)
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

pub(super) fn find_best_candidate_by_carrier_hz_and_rate(
    candidates: &[AcqResult],
    carrier_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> Option<&AcqResult> {
    candidates
        .iter()
        .filter(|candidate| {
            (candidate.carrier_hz.0 - carrier_hz).abs() <= f64::EPSILON
                && (candidate.doppler_rate_hz_per_s - doppler_rate_hz_per_s).abs() <= f64::EPSILON
        })
        .max_by(|left, right| {
            left.peak_mean_ratio
                .partial_cmp(&right.peak_mean_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}
