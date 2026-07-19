#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CorrelationMetrics {
    pub(super) peak_idx: usize,
    pub(super) peak: f32,
    pub(super) second_idx: usize,
    pub(super) second: f32,
    pub(super) mean: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct DelayedSecondaryPeakDiagnostic {
    pub(super) secondary_code_phase_samples: usize,
    pub(super) delay_samples: usize,
}

const MULTIPATH_SECONDARY_GUARD_CHIPS: usize = 2;

pub(super) fn correlation_metrics(corr: &[f32]) -> CorrelationMetrics {
    let mut peak_idx = 0;
    let mut peak = 0.0f32;
    let mut second_idx = 0usize;
    let mut second = 0.0f32;
    let mut sum = 0.0f32;

    for (idx, &mag) in corr.iter().enumerate() {
        sum += mag;
        if mag > peak {
            second_idx = peak_idx;
            second = peak;
            peak = mag;
            peak_idx = idx;
        } else if mag > second {
            second_idx = idx;
            second = mag;
        }
    }

    let mean = sum / corr.len().max(1) as f32;
    CorrelationMetrics { peak_idx, peak, second_idx, second, mean }
}

pub(super) fn correlation_metrics_in_window(
    corr: &[f32],
    start_sample: usize,
    step_samples: usize,
    search_bins: usize,
) -> CorrelationMetrics {
    if corr.is_empty() {
        return CorrelationMetrics {
            peak_idx: 0,
            peak: 0.0,
            second_idx: 0,
            second: 0.0,
            mean: 0.0,
        };
    }
    if search_bins == 0 || search_bins >= corr.len() && step_samples <= 1 {
        return correlation_metrics(corr);
    }

    let mut peak_idx = start_sample % corr.len();
    let mut peak = f32::NEG_INFINITY;
    let mut second_idx = peak_idx;
    let mut second = f32::NEG_INFINITY;
    let step_samples = step_samples.max(1);
    let mut visited = 0usize;

    while visited < search_bins {
        let idx = (start_sample + visited * step_samples) % corr.len();
        let mag = corr[idx];
        if mag > peak {
            second_idx = peak_idx;
            second = peak;
            peak = mag;
            peak_idx = idx;
        } else if mag > second {
            second_idx = idx;
            second = mag;
        }
        visited += 1;
    }

    let mean = corr.iter().copied().sum::<f32>() / corr.len().max(1) as f32;
    CorrelationMetrics { peak_idx, peak, second_idx, second, mean }
}

pub(super) fn delayed_secondary_peak_diagnostic(
    correlation_profile: &[f32],
    peak_idx: usize,
    samples_per_code: usize,
    code_length: usize,
) -> Option<DelayedSecondaryPeakDiagnostic> {
    if correlation_profile.len() < 3 || samples_per_code == 0 || code_length == 0 {
        return None;
    }
    let min_delay_samples = samples_per_chip(samples_per_code, code_length)
        .saturating_mul(MULTIPATH_SECONDARY_GUARD_CHIPS);
    let period_samples = correlation_profile.len();
    let mut best_secondary_idx = None;
    let mut best_secondary_peak = 0.0f32;

    for (idx, &magnitude) in correlation_profile.iter().enumerate() {
        let delay_samples = wrapped_code_phase_offset_samples(peak_idx, idx, period_samples);
        if delay_samples < min_delay_samples || delay_samples > period_samples / 2 {
            continue;
        }
        if !is_local_code_phase_peak(correlation_profile, idx) {
            continue;
        }
        if magnitude > best_secondary_peak {
            best_secondary_peak = magnitude;
            best_secondary_idx = Some((idx, delay_samples));
        }
    }

    let (secondary_code_phase_samples, delay_samples) = best_secondary_idx?;
    Some(DelayedSecondaryPeakDiagnostic { secondary_code_phase_samples, delay_samples })
}

fn is_local_code_phase_peak(correlation_profile: &[f32], idx: usize) -> bool {
    if correlation_profile.len() < 3 {
        return false;
    }
    let period_samples = correlation_profile.len();
    let left = correlation_profile[(idx + period_samples - 1) % period_samples];
    let center = correlation_profile[idx];
    let right = correlation_profile[(idx + 1) % period_samples];

    (center >= left && center >= right) && (center > left || center > right)
}

fn samples_per_chip(samples_per_code: usize, code_length: usize) -> usize {
    ((samples_per_code + code_length.saturating_sub(1)) / code_length.max(1)).max(1)
}

fn wrapped_code_phase_offset_samples(
    primary_code_phase_samples: usize,
    secondary_code_phase_samples: usize,
    period_samples: usize,
) -> usize {
    if period_samples == 0 {
        return 0;
    }
    (secondary_code_phase_samples + period_samples - primary_code_phase_samples) % period_samples
}
