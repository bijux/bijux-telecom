include!("signal_generation/capture_generation.rs");

include!("signal_generation/signal_frames.rs");
include!("signal_generation/streaming_source.rs");

include!("signal_generation/satellite_state.rs");

include!("signal_generation/frame_regeneration.rs");

include!("signal_generation/navigation_symbols.rs");

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

fn peak_component(samples: &[Complex<f32>]) -> f32 {
    samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max)
}

fn nav_bit_segments(
    sample_rate_hz: f64,
    sample_count: u64,
    params: &SyntheticSignalParams,
    nav_bit_mode: SyntheticNavBitMode,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }

    if matches!(
        nav_bit_mode,
        SyntheticNavBitMode::GlonassL1FixedDataString
            | SyntheticNavBitMode::GlonassL1AlternatingDataString
            | SyntheticNavBitMode::GlonassL1CustomDataString
    ) {
        return symbol_period_segments(
            sample_rate_hz,
            sample_count,
            Some(bijux_gnss_signal::api::GLONASS_L1_SYMBOL_PERIOD_S),
            |start_s| {
                nav_bit_sign_with_symbol_period_at_time_s(
                    &params.navigation_data,
                    nav_bit_mode,
                    None,
                    start_s,
                )
            },
        );
    }

    if let Some(primary_epoch_period_s) = native_epoch_period_for_signal(params) {
        return symbol_period_segments(
            sample_rate_hz,
            sample_count,
            Some(primary_epoch_period_s),
            |start_s| {
                let primary_code_period_index = (start_s / primary_epoch_period_s).floor() as usize;
                emitted_epoch_symbol(params, primary_code_period_index)
            },
        );
    }

    symbol_period_segments(
        sample_rate_hz,
        sample_count,
        nav_symbol_period_for_signal(params, nav_bit_mode),
        |start_s| {
            nav_bit_sign_with_symbol_period_at_time_s(
                &params.navigation_data,
                nav_bit_mode,
                nav_symbol_period_for_signal(params, nav_bit_mode),
                start_s,
            )
        },
    )
}

fn symbol_period_segments<F>(
    sample_rate_hz: f64,
    sample_count: u64,
    symbol_period_s: Option<f64>,
    symbol_at_time_s: F,
) -> Vec<SyntheticNavBitSegment>
where
    F: Fn(f64) -> i8,
{
    let Some(symbol_period_s) = symbol_period_s else {
        return vec![SyntheticNavBitSegment {
            start_sample: 0,
            end_sample: sample_count,
            start_s: 0.0,
            end_s: sample_count as f64 / sample_rate_hz,
            bit: 1,
        }];
    };

    let mut segments = Vec::new();
    let mut bit_index = 0u64;
    loop {
        let start_sample = ((bit_index as f64 * symbol_period_s * sample_rate_hz).ceil()) as u64;
        if start_sample >= sample_count {
            break;
        }
        let end_sample =
            ((((bit_index + 1) as f64) * symbol_period_s * sample_rate_hz).ceil()) as u64;
        let clamped_end = end_sample.min(sample_count);
        let start_s = start_sample as f64 / sample_rate_hz;
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: symbol_at_time_s(start_s),
        });
        bit_index += 1;
    }
    segments
}

fn native_epoch_period_for_signal(params: &SyntheticSignalParams) -> Option<f64> {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        params.signal_band,
        signal_code,
        params.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let component = registry_entry.default_component()?;

    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I)
        | (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            Some(component.primary_code_period_s)
        }
        _ => None,
    }
}

fn emitted_epoch_symbol(params: &SyntheticSignalParams, primary_code_period_index: usize) -> i8 {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I) => {
            bijux_gnss_signal::api::gps_l5_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::gps_l5_i_data_symbol_index(primary_code_period_index),
                )],
                primary_code_period_index,
            )
            .expect("synthetic GPS L5I epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => {
            bijux_gnss_signal::api::gps_l5_q_epoch_symbol(primary_code_period_index)
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            bijux_gnss_signal::api::galileo_e5a_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::galileo_e5a_i_data_symbol_index(
                        primary_code_period_index,
                    ),
                )],
                primary_code_period_index,
            )
            .expect("synthetic Galileo E5a epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            bijux_gnss_signal::api::galileo_e5b_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::galileo_e5b_i_data_symbol_index(
                        primary_code_period_index,
                    ),
                )],
                primary_code_period_index,
            )
            .expect("synthetic Galileo E5b epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            bijux_gnss_signal::api::beidou_d1_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::beidou_d1_data_symbol_index(primary_code_period_index),
                )],
                primary_code_period_index,
            )
            .expect("synthetic BeiDou D1 epoch symbols must be valid")
        }
        _ => {
            let nav_bit_mode = nav_bit_mode(params);
            nav_bit_sign_with_symbol_period_at_time_s(
                &params.navigation_data,
                nav_bit_mode,
                nav_symbol_period_for_signal(params, nav_bit_mode),
                0.0,
            )
        }
    }
}

fn nav_symbol_period_for_signal(
    params: &SyntheticSignalParams,
    nav_bit_mode: SyntheticNavBitMode,
) -> Option<f64> {
    if matches!(
        nav_bit_mode,
        SyntheticNavBitMode::ConstantPositive | SyntheticNavBitMode::ConstantNegative
    ) {
        return None;
    }

    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        params.signal_band,
        signal_code,
        params.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let component = registry_entry.default_component()?;

    match nav_bit_mode {
        SyntheticNavBitMode::GpsL5QNh20 => {
            component.secondary_code.map(|secondary| secondary.chip_period_s)
        }
        _ => component.symbol_period_s,
    }
}

#[cfg(test)]
fn legacy_nav_symbol_period_s(nav_bit_mode: SyntheticNavBitMode) -> Option<f64> {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive | SyntheticNavBitMode::ConstantNegative => None,
        SyntheticNavBitMode::GlonassL1FixedDataString
        | SyntheticNavBitMode::GlonassL1AlternatingDataString
        | SyntheticNavBitMode::GlonassL1CustomDataString => {
            Some(bijux_gnss_signal::api::GLONASS_L1_SYMBOL_PERIOD_S)
        }
        SyntheticNavBitMode::AlternatingGpsLnav20ms => Some(GPS_L1_CA_NAV_BIT_PERIOD_S),
        SyntheticNavBitMode::AlternatingGalileoInav4ms => Some(0.004),
        SyntheticNavBitMode::AlternatingGpsL5I10ms => Some(0.010),
        SyntheticNavBitMode::NativeSymbolSequence => Some(GPS_L1_CA_NAV_BIT_PERIOD_S),
        SyntheticNavBitMode::GpsL5QNh20 => Some(0.001),
    }
}

fn resolved_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
) -> bijux_gnss_core::api::SignalCode {
    if signal_code != bijux_gnss_core::api::SignalCode::Unknown {
        return signal_code;
    }
    match (sat.constellation, signal_band) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L1) => {
            bijux_gnss_core::api::SignalCode::Ca
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L2) => {
            bijux_gnss_core::api::SignalCode::L2C
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5) => {
            bijux_gnss_core::api::SignalCode::L5I
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1) => {
            bijux_gnss_core::api::SignalCode::E1B
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5) => {
            bijux_gnss_core::api::SignalCode::E5a
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1) => {
            bijux_gnss_core::api::SignalCode::B1I
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2) => {
            bijux_gnss_core::api::SignalCode::B2I
        }
        _ => bijux_gnss_core::api::SignalCode::Unknown,
    }
}

#[cfg(test)]
fn nav_bit_mode_from_flip(data_bit_flip: bool) -> SyntheticNavBitMode {
    if data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

#[cfg(test)]
fn navigation_data_for_nav_mode(nav_bit_mode: SyntheticNavBitMode) -> SyntheticNavigationData {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::ConstantNegative => SyntheticNavigationData::ConstantNegative,
        SyntheticNavBitMode::GlonassL1FixedDataString => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::GlonassL1AlternatingDataString => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::GlonassL1CustomDataString => {
            SyntheticNavigationData::GlonassL1String {
                raw_data_bits: glonass_l1_raw_data_bits(
                    &SyntheticNavigationData::AlternatingStartPositive,
                )
                .to_vec(),
            }
        }
        SyntheticNavBitMode::AlternatingGpsLnav20ms
        | SyntheticNavBitMode::AlternatingGalileoInav4ms
        | SyntheticNavBitMode::AlternatingGpsL5I10ms => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::NativeSymbolSequence => {
            SyntheticNavigationData::SymbolSequence(vec![1, -1])
        }
        SyntheticNavBitMode::GpsL5QNh20 => SyntheticNavigationData::ConstantPositive,
    }
}

fn glonass_l1_raw_data_bits(
    navigation_data: &SyntheticNavigationData,
) -> [i8; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS] {
    let len = bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS;
    match navigation_data {
        SyntheticNavigationData::ConstantPositive => {
            [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS]
        }
        SyntheticNavigationData::AlternatingStartPositive => {
            let mut raw_data_bits = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (index, bit) in raw_data_bits.iter_mut().enumerate().skip(1) {
                *bit = if index % 2 == 0 { 1 } else { -1 };
            }
            raw_data_bits
        }
        SyntheticNavigationData::GlonassL1String { raw_data_bits } => {
            assert!(
                raw_data_bits.len() == len,
                "synthetic glonass l1 raw data string must contain {len} bits"
            );
            let mut validated = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (slot, bit) in validated.iter_mut().zip(raw_data_bits.iter().copied()) {
                *slot = validate_navigation_symbol(bit);
            }
            validated
        }
        _ => {
            panic!("glonass l1 string modulation requires constant positive, alternating start positive, or an explicit raw data string")
        }
    }
}

#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}

#[cfg(test)]
mod signal_generation_tests {
    use super::{synthetic_replica_model, SyntheticNavigationData, SyntheticSignalParams};
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
    use bijux_gnss_signal::api::ReplicaCodeModel;

    fn synthetic_signal_params(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
    ) -> SyntheticSignalParams {
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: SyntheticNavigationData::ConstantPositive,
        }
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5a_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5a,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5aQpsk { .. }), "{model:?}");
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5b_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5b,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5bQpsk { .. }), "{model:?}");
    }
}
