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
    let (signal_band, signal_code) =
        resolved_signal_identity(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        signal_band,
        signal_code,
        params.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let component = registry_entry.default_component()?;

    match (params.sat.constellation, signal_band, signal_code) {
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
    let (signal_band, signal_code) =
        resolved_signal_identity(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, signal_band, signal_code) {
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

    let (signal_band, signal_code) =
        resolved_signal_identity(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        signal_band,
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
