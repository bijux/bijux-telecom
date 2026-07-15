fn receiver_oscillator_model_from_legacy_bias(
    receiver_clock_frequency_bias_hz: f64,
) -> SyntheticReceiverOscillatorModel {
    SyntheticReceiverOscillatorModel::with_carrier_frequency_bias_hz(
        receiver_clock_frequency_bias_hz,
    )
}

fn receiver_oscillator_truth_for_capture(
    model: &SyntheticReceiverOscillatorModel,
    sample_rate_hz: f64,
    sample_count: u64,
) -> SyntheticReceiverOscillatorTruth {
    let sample_indices =
        receiver_oscillator_truth_sample_indices(model, sample_rate_hz, sample_count);
    let noise_knots = receiver_oscillator_noise_knots(model, sample_count, sample_rate_hz);
    SyntheticReceiverOscillatorTruth {
        carrier_frequency_bias_hz: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: model.carrier_frequency_bias_hz,
            })
            .collect(),
        carrier_frequency_drift_hz_per_s: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: model.carrier_frequency_drift_hz_per_s,
            })
            .collect(),
        phase_noise_rad: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: receiver_oscillator_phase_noise_rad_at_sample(
                    model,
                    sample_index,
                    sample_count,
                ) + receiver_oscillator_noise_phase_rad_from_knots(
                    &noise_knots,
                    sample_index,
                    sample_rate_hz,
                ),
            })
            .collect(),
        frequency_noise_hz: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: receiver_oscillator_frequency_noise_hz_from_knots(
                    &noise_knots,
                    sample_index,
                ),
            })
            .collect(),
        sampling_clock_time_error_s: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: sampling_clock_time_error_s_at_nominal_time(
                    sample_index as f64 / sample_rate_hz,
                    model,
                ),
            })
            .collect(),
    }
}

fn sampling_clock_time_error_s_at_nominal_time(
    nominal_elapsed_s: f64,
    model: &SyntheticReceiverOscillatorModel,
) -> f64 {
    nominal_elapsed_s * model.sampling_clock_fractional_error
        + 0.5 * model.sampling_clock_fractional_drift_per_s * nominal_elapsed_s * nominal_elapsed_s
}

fn receiver_oscillator_truth_sample_indices(
    model: &SyntheticReceiverOscillatorModel,
    sample_rate_hz: f64,
    sample_count: u64,
) -> Vec<u64> {
    if sample_count == 0 {
        return Vec::new();
    }

    let mut sample_indices = vec![0];
    let last_sample_index = sample_count.saturating_sub(1);
    let truth_step_samples = ((sample_rate_hz * 0.001).round() as u64).max(1);
    let mut stride = truth_step_samples;
    if model.phase_noise.is_enabled() {
        stride = stride.min(model.phase_noise.knot_interval_samples.max(1));
    }
    if model.noise.is_enabled() {
        stride = stride.min(model.noise.update_interval_samples.max(1));
    }
    let mut sample_index = stride;
    while sample_index < last_sample_index {
        sample_indices.push(sample_index);
        sample_index += stride;
    }
    if last_sample_index > 0 {
        sample_indices.push(last_sample_index);
    }
    sample_indices
}

fn receiver_oscillator_phase_noise_rad_at_sample(
    model: &SyntheticReceiverOscillatorModel,
    sample_index: u64,
    sample_count: u64,
) -> f64 {
    if !model.phase_noise.is_enabled() || sample_count == 0 {
        return 0.0;
    }

    let knots = receiver_oscillator_phase_noise_knots(model, sample_count);
    receiver_oscillator_phase_noise_rad_from_knots(&knots, sample_index)
}

fn receiver_oscillator_phase_noise_knots(
    model: &SyntheticReceiverOscillatorModel,
    sample_count: u64,
) -> Vec<(u64, f64)> {
    if !model.phase_noise.is_enabled() || sample_count == 0 {
        return vec![(0, 0.0)];
    }

    let last_sample_index = sample_count.saturating_sub(1);
    let step_samples = model.phase_noise.knot_interval_samples.max(1);
    let mut rng = XorShift64::new(model.phase_noise.seed);
    let mut knots = vec![(0, 0.0)];
    let mut phase_noise_rad = 0.0;
    let mut sample_index = step_samples;
    while sample_index <= last_sample_index {
        phase_noise_rad += rng.next_gaussian() as f64 * model.phase_noise.step_std_rad;
        knots.push((sample_index, phase_noise_rad));
        sample_index += step_samples;
    }
    if knots.last().map(|(sample_index, _)| *sample_index) != Some(last_sample_index) {
        phase_noise_rad += rng.next_gaussian() as f64 * model.phase_noise.step_std_rad;
        knots.push((last_sample_index, phase_noise_rad));
    }
    knots
}

fn receiver_oscillator_phase_noise_rad_from_knots(knots: &[(u64, f64)], sample_index: u64) -> f64 {
    if let Some((_, value)) =
        knots.iter().find(|(knot_sample_index, _)| *knot_sample_index == sample_index)
    {
        return *value;
    }

    for window in knots.windows(2) {
        let (start_sample_index, start_value) = window[0];
        let (end_sample_index, end_value) = window[1];
        if sample_index >= start_sample_index && sample_index <= end_sample_index {
            let span = (end_sample_index - start_sample_index).max(1) as f64;
            let alpha = (sample_index - start_sample_index) as f64 / span;
            return start_value + alpha * (end_value - start_value);
        }
    }

    knots.last().map(|(_, value)| *value).unwrap_or(0.0)
}

fn receiver_oscillator_noise_knots(
    model: &SyntheticReceiverOscillatorModel,
    sample_count: u64,
    sample_rate_hz: f64,
) -> ReceiverOscillatorNoiseKnots {
    if !model.noise.is_enabled() || sample_count == 0 {
        return ReceiverOscillatorNoiseKnots {
            white_phase_rad: vec![(0, 0.0)],
            frequency_noise_hz: vec![(0, 0.0)],
            frequency_noise_phase_rad: vec![(0, 0.0)],
        };
    }

    let last_sample_index = sample_count.saturating_sub(1);
    let step_samples = model.noise.update_interval_samples.max(1);
    let mut rng = XorShift64::new(model.noise.seed);
    let mut white_phase_rad = Vec::new();
    let mut frequency_noise_hz = Vec::new();
    let mut frequency_noise_phase_rad = Vec::new();
    let mut random_walk_frequency_hz = 0.0;
    let mut integrated_phase_rad = 0.0;
    let mut previous_sample_index = 0;
    let mut previous_frequency_noise_hz = 0.0;
    let mut sample_index = 0;

    loop {
        if sample_index > 0 {
            let elapsed_s = (sample_index - previous_sample_index) as f64 / sample_rate_hz;
            integrated_phase_rad +=
                std::f64::consts::TAU * previous_frequency_noise_hz * elapsed_s;
            random_walk_frequency_hz += rng.next_gaussian() as f64
                * model.noise.random_walk_frequency_step_std_hz;
        }

        let white_phase = rng.next_gaussian() as f64 * model.noise.white_phase_std_rad;
        let white_frequency = rng.next_gaussian() as f64 * model.noise.white_frequency_std_hz;
        let frequency_noise = random_walk_frequency_hz + white_frequency;

        white_phase_rad.push((sample_index, white_phase));
        frequency_noise_hz.push((sample_index, frequency_noise));
        frequency_noise_phase_rad.push((sample_index, integrated_phase_rad));

        if sample_index == last_sample_index {
            break;
        }

        previous_sample_index = sample_index;
        previous_frequency_noise_hz = frequency_noise;
        sample_index = (sample_index + step_samples).min(last_sample_index);
    }

    ReceiverOscillatorNoiseKnots {
        white_phase_rad,
        frequency_noise_hz,
        frequency_noise_phase_rad,
    }
}

fn receiver_oscillator_piecewise_value_from_knots(knots: &[(u64, f64)], sample_index: u64) -> f64 {
    knots
        .iter()
        .rev()
        .find_map(|(knot_sample_index, value)| {
            (*knot_sample_index <= sample_index).then_some(*value)
        })
        .unwrap_or(0.0)
}

fn receiver_oscillator_frequency_noise_hz_from_knots(
    knots: &ReceiverOscillatorNoiseKnots,
    sample_index: u64,
) -> f64 {
    receiver_oscillator_piecewise_value_from_knots(&knots.frequency_noise_hz, sample_index)
}

fn receiver_oscillator_frequency_noise_phase_rad_from_knots(
    knots: &ReceiverOscillatorNoiseKnots,
    sample_index: u64,
    sample_rate_hz: f64,
) -> f64 {
    let start_sample_index = knots
        .frequency_noise_phase_rad
        .iter()
        .rev()
        .find_map(|(knot_sample_index, _)| {
            (*knot_sample_index <= sample_index).then_some(*knot_sample_index)
        })
        .unwrap_or(0);
    let start_phase = receiver_oscillator_piecewise_value_from_knots(
        &knots.frequency_noise_phase_rad,
        sample_index,
    );
    let frequency_noise_hz =
        receiver_oscillator_frequency_noise_hz_from_knots(knots, sample_index);
    let elapsed_s = sample_index.saturating_sub(start_sample_index) as f64 / sample_rate_hz;
    start_phase + std::f64::consts::TAU * frequency_noise_hz * elapsed_s
}

fn receiver_oscillator_noise_phase_rad_from_knots(
    knots: &ReceiverOscillatorNoiseKnots,
    sample_index: u64,
    sample_rate_hz: f64,
) -> f64 {
    receiver_oscillator_piecewise_value_from_knots(&knots.white_phase_rad, sample_index)
        + receiver_oscillator_frequency_noise_phase_rad_from_knots(
            knots,
            sample_index,
            sample_rate_hz,
        )
}
