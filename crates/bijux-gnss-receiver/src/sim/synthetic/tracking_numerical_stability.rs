use crate::pipeline::tracking::{TrackingArtifacts, TrackingResult};

const TRACKING_NUMERICAL_CODE_PHASE_EPSILON_SAMPLES: f64 = 1.0e-9;
const TRACKING_NUMERICAL_NCO_PHASE_STEP_BOUND_CYCLES: f64 = 0.5;
const TRACKING_NUMERICAL_SECONDARY_CODE_PHASE_BOUND_PERIODS: f64 = 10_000.0;

/// Summarize long-duration numerical boundedness across receiver tracking outputs.
pub fn summarize_tracking_numerical_stability(
    scenario_id: &str,
    config: &ReceiverPipelineConfig,
    expected_input_samples: u64,
    required_epoch_count: usize,
    artifacts: &TrackingArtifacts,
) -> SyntheticTrackingNumericalStabilityReport {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let satellites = artifacts
        .tracking
        .iter()
        .map(|track| {
            summarize_tracking_signal_numerical_stability(
                config,
                period_samples,
                expected_input_samples,
                required_epoch_count,
                track,
            )
        })
        .collect::<Vec<_>>();
    let pass = artifacts.processed_input_samples == expected_input_samples
        && !satellites.is_empty()
        && satellites.iter().all(|satellite| satellite.pass);

    SyntheticTrackingNumericalStabilityReport {
        scenario_id: scenario_id.to_string(),
        sample_rate_hz: config.sampling_freq_hz,
        period_samples,
        expected_input_samples,
        processed_input_samples: artifacts.processed_input_samples,
        required_epoch_count,
        tracked_signal_count: satellites.len(),
        satellites,
        pass,
    }
}

fn summarize_tracking_signal_numerical_stability(
    config: &ReceiverPipelineConfig,
    period_samples: usize,
    expected_input_samples: u64,
    required_epoch_count: usize,
    track: &TrackingResult,
) -> SyntheticTrackingNumericalStabilitySatellite {
    let signal = track.epochs.first();
    let timestamps = timestamp_summary(&track.epochs, expected_input_samples, required_epoch_count);
    let code_phase = code_phase_summary(&track.epochs, period_samples, required_epoch_count);
    let carrier_phase = carrier_phase_summary(&track.epochs, config, required_epoch_count);
    let nco_state = nco_state_summary(&track.epochs, required_epoch_count);
    let secondary_code_phase = secondary_code_phase_summary(&track.epochs, required_epoch_count);
    let pass = timestamps.pass
        && code_phase.pass
        && carrier_phase.pass
        && nco_state.pass
        && secondary_code_phase.as_ref().is_none_or(|summary| summary.pass);

    SyntheticTrackingNumericalStabilitySatellite {
        sat: track.sat,
        signal_band: signal.map(|epoch| epoch.signal_band).unwrap_or_else(|| {
            bijux_gnss_core::api::default_signal_band_for_constellation(track.sat.constellation)
        }),
        signal_code: signal
            .map(|epoch| epoch.signal_code)
            .unwrap_or(bijux_gnss_core::api::SignalCode::Unknown),
        glonass_frequency_channel: signal.and_then(|epoch| epoch.glonass_frequency_channel),
        epoch_count: track.epochs.len(),
        timestamps,
        code_phase,
        carrier_phase,
        nco_state,
        secondary_code_phase,
        pass,
    }
}

fn timestamp_summary(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    expected_input_samples: u64,
    required_epoch_count: usize,
) -> SyntheticTrackingNumericalStateSummary {
    let mut previous_sample_index = None;
    let mut valid_epoch_count = 0usize;
    let mut values = Vec::with_capacity(epochs.len());

    for epoch in epochs {
        let monotonic = previous_sample_index
            .map(|previous| epoch.sample_index > previous)
            .unwrap_or(true);
        previous_sample_index = Some(epoch.sample_index);
        let source_time_valid = epoch.source_time.validate().is_ok()
            && epoch.source_time.sample_index == epoch.sample_index;
        if monotonic && source_time_valid && epoch.sample_index <= expected_input_samples {
            valid_epoch_count += 1;
        }
        values.push(epoch.sample_index as f64);
    }

    bounded_summary(values, valid_epoch_count, required_epoch_count)
}

fn code_phase_summary(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    period_samples: usize,
    required_epoch_count: usize,
) -> SyntheticTrackingNumericalStateSummary {
    let period_samples_f64 = period_samples as f64;
    let mut valid_epoch_count = 0usize;
    let mut values = Vec::with_capacity(epochs.len());
    let mut max_abs_step = 0.0f64;
    let mut previous: Option<f64> = None;

    for epoch in epochs {
        let code_phase = epoch.code_phase_samples.0;
        if code_phase.is_finite()
            && code_phase >= -TRACKING_NUMERICAL_CODE_PHASE_EPSILON_SAMPLES
            && code_phase < period_samples_f64 + TRACKING_NUMERICAL_CODE_PHASE_EPSILON_SAMPLES
        {
            valid_epoch_count += 1;
        }
        if let Some(previous_code_phase) = previous {
            let step = bijux_gnss_signal::api::wrapped_code_phase_delta_samples(
                code_phase,
                previous_code_phase,
                period_samples,
            )
            .abs();
            if step.is_finite() {
                max_abs_step = max_abs_step.max(step);
            }
        }
        previous = Some(code_phase);
        values.push(code_phase);
    }

    bounded_summary_with_step(values, valid_epoch_count, required_epoch_count, max_abs_step)
}

fn carrier_phase_summary(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    config: &ReceiverPipelineConfig,
    required_epoch_count: usize,
) -> SyntheticTrackingNumericalStateSummary {
    let integration_s = config.tracking_integration_ms.max(1) as f64 / 1000.0;
    let max_step_cycles = (config.intermediate_freq_hz.abs()
        + f64::from(config.acquisition_doppler_search_hz.abs())
        + 1.0)
        * integration_s;
    let mut valid_epoch_count = 0usize;
    let mut values = Vec::with_capacity(epochs.len());
    let mut max_abs_step = 0.0f64;
    let mut previous: Option<f64> = None;

    for epoch in epochs {
        let carrier_phase = epoch.carrier_phase_cycles.0;
        if carrier_phase.is_finite() {
            valid_epoch_count += 1;
        }
        if let Some(previous_carrier_phase) = previous {
            let step = (carrier_phase - previous_carrier_phase).abs();
            if step.is_finite() {
                max_abs_step = max_abs_step.max(step);
            }
        }
        previous = Some(carrier_phase);
        values.push(carrier_phase);
    }

    let mut summary =
        bounded_summary_with_step(values, valid_epoch_count, required_epoch_count, max_abs_step);
    summary.pass = summary.pass && max_abs_step <= max_step_cycles.max(1.0);
    summary
}

fn nco_state_summary(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    required_epoch_count: usize,
) -> SyntheticTrackingNumericalStateSummary {
    let mut valid_epoch_count = 0usize;
    let mut values = Vec::with_capacity(epochs.len());
    let mut max_abs_step = 0.0f64;
    let mut previous: Option<f64> = None;

    for epoch in epochs {
        let wrapped_carrier_phase = epoch.carrier_phase_cycles.0.rem_euclid(1.0);
        let carrier_hz = epoch.carrier_hz.0;
        let code_rate_hz = epoch.code_rate_hz.0;
        if wrapped_carrier_phase.is_finite()
            && (0.0..1.0).contains(&wrapped_carrier_phase)
            && carrier_hz.is_finite()
            && code_rate_hz.is_finite()
            && code_rate_hz > 0.0
        {
            valid_epoch_count += 1;
        }
        if let Some(previous_wrapped_phase) = previous {
            let step = bijux_gnss_signal::api::wrap_phase_cycles_signed(
                wrapped_carrier_phase - previous_wrapped_phase,
            )
            .abs();
            if step.is_finite() {
                max_abs_step = max_abs_step.max(step);
            }
        }
        previous = Some(wrapped_carrier_phase);
        values.push(wrapped_carrier_phase);
    }

    let mut summary =
        bounded_summary_with_step(values, valid_epoch_count, required_epoch_count, max_abs_step);
    summary.pass =
        summary.pass && summary.max_abs_step <= TRACKING_NUMERICAL_NCO_PHASE_STEP_BOUND_CYCLES;
    summary
}

fn secondary_code_phase_summary(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    required_epoch_count: usize,
) -> Option<SyntheticTrackingNumericalStateSummary> {
    let signal_has_secondary_code = epochs
        .iter()
        .any(|epoch| epoch.tracking_provenance.contains("secondary_code=true"));
    if !signal_has_secondary_code {
        return None;
    }

    let mut valid_epoch_count = 0usize;
    let mut values = Vec::with_capacity(epochs.len());
    let mut max_abs_step = 0.0f64;
    let mut previous: Option<f64> = None;

    for epoch in epochs {
        if let Some(phase_periods) =
            tracking_provenance_usize(&epoch.tracking_provenance, "secondary_code_phase_periods=")
        {
            let phase_periods_f64 = phase_periods as f64;
            if phase_periods_f64.is_finite()
                && phase_periods_f64 >= 0.0
                && phase_periods_f64 <= TRACKING_NUMERICAL_SECONDARY_CODE_PHASE_BOUND_PERIODS
            {
                valid_epoch_count += 1;
            }
            if let Some(previous_phase_periods) = previous {
                let step = (phase_periods_f64 - previous_phase_periods).abs();
                if step.is_finite() {
                    max_abs_step = max_abs_step.max(step);
                }
            }
            previous = Some(phase_periods_f64);
            values.push(phase_periods_f64);
        }
    }

    Some(bounded_summary_with_step(
        values,
        valid_epoch_count,
        required_epoch_count,
        max_abs_step,
    ))
}

fn bounded_summary(
    values: Vec<f64>,
    finite_epoch_count: usize,
    required_epoch_count: usize,
) -> SyntheticTrackingNumericalStateSummary {
    let max_abs_step = values
        .windows(2)
        .filter_map(|window| {
            let step = (window[1] - window[0]).abs();
            step.is_finite().then_some(step)
        })
        .fold(0.0f64, f64::max);
    bounded_summary_with_step(values, finite_epoch_count, required_epoch_count, max_abs_step)
}

fn bounded_summary_with_step(
    values: Vec<f64>,
    finite_epoch_count: usize,
    required_epoch_count: usize,
    max_abs_step: f64,
) -> SyntheticTrackingNumericalStateSummary {
    let epoch_count = values.len();
    let min_value = values.iter().copied().fold(f64::INFINITY, f64::min);
    let max_value = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let pass = epoch_count >= required_epoch_count
        && finite_epoch_count == epoch_count
        && epoch_count > 0
        && min_value.is_finite()
        && max_value.is_finite()
        && max_abs_step.is_finite();

    SyntheticTrackingNumericalStateSummary {
        epoch_count,
        finite_epoch_count,
        min_value: if min_value.is_finite() { min_value } else { 0.0 },
        max_value: if max_value.is_finite() { max_value } else { 0.0 },
        max_abs_step,
        pass,
    }
}

fn tracking_provenance_usize(provenance: &str, key: &str) -> Option<usize> {
    let start = provenance.find(key)? + key.len();
    let value = provenance[start..].split_whitespace().next()?;
    value.parse::<usize>().ok()
}
