fn resolved_signal_identity(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
) -> (SignalBand, bijux_gnss_core::api::SignalCode) {
    if signal_code != bijux_gnss_core::api::SignalCode::Unknown {
        return (signal_band, signal_code);
    }

    let resolved_code = default_signal_code_for_band(sat.constellation, signal_band);
    if bijux_gnss_signal::api::resolved_signal_registry_entry(
        sat,
        signal_band,
        resolved_code,
        None,
    )
        .ok()
        .flatten()
        .is_some()
    {
        return (signal_band, resolved_code);
    }

    let fallback_band = default_signal_band_for_constellation(sat.constellation);
    (fallback_band, default_signal_code_for_band(sat.constellation, fallback_band))
}

fn resolved_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
) -> bijux_gnss_core::api::SignalCode {
    resolved_signal_identity(sat, signal_band, signal_code).1
}

fn default_signal_code_for_band(
    constellation: bijux_gnss_core::api::Constellation,
    signal_band: SignalBand,
) -> bijux_gnss_core::api::SignalCode {
    match (constellation, signal_band) {
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

fn default_signal_band_for_constellation(
    constellation: bijux_gnss_core::api::Constellation,
) -> SignalBand {
    match constellation {
        bijux_gnss_core::api::Constellation::Gps => SignalBand::L1,
        bijux_gnss_core::api::Constellation::Galileo => SignalBand::E1,
        bijux_gnss_core::api::Constellation::Glonass => SignalBand::L1,
        bijux_gnss_core::api::Constellation::Beidou => SignalBand::B1,
        bijux_gnss_core::api::Constellation::Unknown => SignalBand::L1,
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
