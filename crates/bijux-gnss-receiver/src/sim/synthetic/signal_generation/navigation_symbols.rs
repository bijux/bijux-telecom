fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => {
            SyntheticNavBitMode::GpsL5QNh20
        }
        (bijux_gnss_core::api::Constellation::Glonass, SignalBand::L1, _) => {
            match &params.navigation_data {
                SyntheticNavigationData::ConstantPositive => {
                    SyntheticNavBitMode::GlonassL1FixedDataString
                }
                SyntheticNavigationData::AlternatingStartPositive => {
                    SyntheticNavBitMode::GlonassL1AlternatingDataString
                }
                SyntheticNavigationData::GlonassL1String { raw_data_bits } => {
                    if raw_data_bits.iter().all(|&bit| bit == 1) {
                        SyntheticNavBitMode::GlonassL1FixedDataString
                    } else {
                        SyntheticNavBitMode::GlonassL1CustomDataString
                    }
                }
                _ => SyntheticNavBitMode::GlonassL1CustomDataString,
            }
        }
        _ => match &params.navigation_data {
            SyntheticNavigationData::ConstantPositive => SyntheticNavBitMode::ConstantPositive,
            SyntheticNavigationData::ConstantNegative => SyntheticNavBitMode::ConstantNegative,
            SyntheticNavigationData::AlternatingStartPositive => {
                match (params.sat.constellation, params.signal_band, signal_code) {
                    (
                        bijux_gnss_core::api::Constellation::Galileo,
                        SignalBand::E5,
                        SignalCode::E5b,
                    ) => SyntheticNavBitMode::AlternatingGalileoInav4ms,
                    (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I) => {
                        SyntheticNavBitMode::AlternatingGpsL5I10ms
                    }
                    _ => SyntheticNavBitMode::AlternatingGpsLnav20ms,
                }
            }
            SyntheticNavigationData::AlternatingStartNegative
            | SyntheticNavigationData::SymbolSequence(_)
            | SyntheticNavigationData::GlonassL1String { .. } => {
                SyntheticNavBitMode::NativeSymbolSequence
            }
        },
    }
}

#[cfg(test)]
fn nav_bit_sign_for_mode_at_time_s(nav_bit_mode: SyntheticNavBitMode, time_s: f64) -> i8 {
    let navigation_data = navigation_data_for_nav_mode(nav_bit_mode);
    nav_bit_sign_with_navigation_data_for_mode_at_time_s(&navigation_data, nav_bit_mode, time_s)
}

#[cfg(test)]
fn nav_bit_sign_with_navigation_data_for_mode_at_time_s(
    navigation_data: &SyntheticNavigationData,
    nav_bit_mode: SyntheticNavBitMode,
    time_s: f64,
) -> i8 {
    nav_bit_sign_with_symbol_period_at_time_s(
        navigation_data,
        nav_bit_mode,
        legacy_nav_symbol_period_s(nav_bit_mode),
        time_s,
    )
}

fn nav_bit_sign_with_symbol_period_at_time_s(
    navigation_data: &SyntheticNavigationData,
    nav_bit_mode: SyntheticNavBitMode,
    symbol_period_s: Option<f64>,
    time_s: f64,
) -> i8 {
    match nav_bit_mode {
        SyntheticNavBitMode::GlonassL1FixedDataString
        | SyntheticNavBitMode::GlonassL1AlternatingDataString
        | SyntheticNavBitMode::GlonassL1CustomDataString => {
            bijux_gnss_signal::api::glonass_l1_string_symbol_at_time_s(
                &glonass_l1_raw_data_bits(navigation_data),
                time_s.max(0.0),
            )
            .expect("synthetic GLONASS L1 string schedule must be valid")
        }
        SyntheticNavBitMode::GpsL5QNh20 => bijux_gnss_signal::api::gps_l5_q_epoch_symbol(
            navigation_symbol_index_at_time_s(symbol_period_s, time_s),
        ),
        _ => navigation_symbol_at_index(
            navigation_data,
            navigation_symbol_index_at_time_s(symbol_period_s, time_s),
        ),
    }
}

fn navigation_symbol_index_at_time_s(symbol_period_s: Option<f64>, time_s: f64) -> usize {
    let Some(symbol_period_s) = symbol_period_s else {
        return 0;
    };
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / symbol_period_s).floor() as usize
}

#[cfg(test)]
fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    let navigation_data = SyntheticNavigationData::from(data_bit_flip);
    nav_bit_sign_with_navigation_data_for_mode_at_time_s(
        &navigation_data,
        nav_bit_mode_from_flip(data_bit_flip),
        time_s,
    )
}

#[cfg(test)]
fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    navigation_symbol_index_at_time_s(Some(GPS_L1_CA_NAV_BIT_PERIOD_S), time_s) as u64
}

fn navigation_symbol_at_index(
    navigation_data: &SyntheticNavigationData,
    symbol_index: usize,
) -> i8 {
    match navigation_data {
        SyntheticNavigationData::ConstantPositive => 1,
        SyntheticNavigationData::ConstantNegative => -1,
        SyntheticNavigationData::AlternatingStartPositive => {
            if symbol_index % 2 == 0 {
                1
            } else {
                -1
            }
        }
        SyntheticNavigationData::AlternatingStartNegative => {
            if symbol_index % 2 == 0 {
                -1
            } else {
                1
            }
        }
        SyntheticNavigationData::SymbolSequence(symbols) => {
            assert!(!symbols.is_empty(), "synthetic navigation symbol sequence must not be empty");
            let symbol = symbols.get(symbol_index % symbols.len()).copied().unwrap_or_else(|| {
                panic!("synthetic navigation symbol sequence must not be empty")
            });
            validate_navigation_symbol(symbol)
        }
        SyntheticNavigationData::GlonassL1String { .. } => {
            panic!("glonass l1 raw data strings are only valid on glonass l1 signals")
        }
    }
}

fn validate_navigation_symbol(symbol: i8) -> i8 {
    match symbol {
        -1 | 1 => symbol,
        other => panic!("synthetic navigation symbol must be -1 or 1, found {other}"),
    }
}
