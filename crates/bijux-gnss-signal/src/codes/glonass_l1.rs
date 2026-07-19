#![allow(missing_docs)]

//! GLONASS L1 standard-precision spreading code generation.
//!
//! The open-service GLONASS L1 FDMA signal uses a 511-chip maximum-length
//! sequence clocked at 511 kchips/s. All satellites share the same spreading
//! code; satellites are distinguished by carrier frequency channel instead.

use crate::error::SignalError;

/// Number of chips in one GLONASS L1 standard-precision code period.
pub const GLONASS_L1_ST_CODE_CHIPS: usize = 511;

/// Code rate of the GLONASS L1 standard-precision ranging code.
pub const GLONASS_L1_ST_CODE_RATE_HZ: f64 = 511_000.0;

/// Duration of one GLONASS navigation string.
pub const GLONASS_L1_STRING_DURATION_S: f64 = 2.0;
/// Duration of one GLONASS L1 raw navigation-data bit before meander expansion.
pub const GLONASS_L1_DATA_BIT_PERIOD_S: f64 = 0.020;
/// Duration of one GLONASS L1 meander or time-mark symbol.
pub const GLONASS_L1_SYMBOL_PERIOD_S: f64 = 0.010;
/// Number of raw navigation-data bits transmitted in the first 1.7 seconds of one string.
pub const GLONASS_L1_STRING_DATA_BITS: usize = 85;
/// Number of 10 ms symbols occupied by the first 1.7 seconds of one string.
pub const GLONASS_L1_STRING_DATA_SYMBOLS: usize = 170;
/// Number of 10 ms symbols carried by the GLONASS time mark.
pub const GLONASS_L1_TIME_MARK_SYMBOLS: usize = 30;
/// Total number of 10 ms symbols in one GLONASS string.
pub const GLONASS_L1_STRING_SYMBOLS: usize =
    GLONASS_L1_STRING_DATA_SYMBOLS + GLONASS_L1_TIME_MARK_SYMBOLS;
/// Published 30-symbol GLONASS L1 time mark in bipolar form.
pub const GLONASS_L1_TIME_MARK: [i8; GLONASS_L1_TIME_MARK_SYMBOLS] = [
    -1, -1, -1, -1, -1, 1, 1, 1, -1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1, 1,
    -1, -1, 1,
];

/// Generate one full-period GLONASS L1 standard-precision spreading code.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_glonass_l1_st_code() -> Vec<i8> {
    generate_glonass_l1_st_code_chips(GLONASS_L1_ST_CODE_CHIPS)
}

/// Generate a GLONASS L1 standard-precision spreading code sequence of
/// arbitrary length.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_glonass_l1_st_code_chips(chip_count: usize) -> Vec<i8> {
    let mut register = [1u8; 9];
    let mut code = Vec::with_capacity(chip_count);

    for _ in 0..chip_count {
        // The ICD defines the output at the seventh stage of the 9-stage
        // register with polynomial x^9 + x^5 + 1 and all-ones initialization.
        let chip = register[6];
        code.push(if chip == 0 { 1 } else { -1 });

        let feedback = register[4] ^ register[8];
        shift_register(&mut register, feedback);
    }

    code
}

/// Sample the GLONASS L1 standard-precision code at an arbitrary sample rate
/// from a chip-phase origin.
pub fn sample_glonass_l1_st_code(
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, crate::error::SignalError> {
    let code = generate_glonass_l1_st_code();
    crate::dsp::signal::sample_code(
        &code,
        sample_rate_hz,
        GLONASS_L1_ST_CODE_RATE_HZ,
        start_chip_phase,
        sample_count,
    )
}

/// Return the published 100 Hz meander sign at one 10 ms symbol index.
pub fn glonass_l1_meander_symbol(symbol_index: usize) -> i8 {
    if symbol_index % 2 == 0 {
        1
    } else {
        -1
    }
}

/// Differentially encode one GLONASS raw navigation-data bit against the previous raw bit.
pub fn glonass_l1_relative_data_symbol(
    previous_raw_bit: i8,
    raw_bit: i8,
) -> Result<i8, SignalError> {
    validate_bipolar_navigation_symbol(previous_raw_bit)?;
    validate_bipolar_navigation_symbol(raw_bit)?;
    Ok(previous_raw_bit * raw_bit)
}

/// Return the bipolar GLONASS L1 modulation symbol for one 10 ms position within a string.
///
/// The first 170 symbols carry 85 raw 20 ms data bits after one-bit-delay relative encoding
/// and 100 Hz meander expansion. The final 30 symbols carry the published time mark.
pub fn glonass_l1_string_symbol(
    raw_data_bits: &[i8; GLONASS_L1_STRING_DATA_BITS],
    symbol_index: usize,
) -> Result<i8, SignalError> {
    let index_in_string = symbol_index % GLONASS_L1_STRING_SYMBOLS;

    if index_in_string >= GLONASS_L1_STRING_DATA_SYMBOLS {
        return Ok(GLONASS_L1_TIME_MARK[index_in_string - GLONASS_L1_STRING_DATA_SYMBOLS]);
    }

    let bit_index = index_in_string / 2;
    let previous_raw_bit = if bit_index == 0 { 1 } else { raw_data_bits[bit_index - 1] };
    let relative_symbol =
        glonass_l1_relative_data_symbol(previous_raw_bit, raw_data_bits[bit_index])?;

    Ok(relative_symbol * glonass_l1_meander_symbol(index_in_string))
}

/// Return the bipolar GLONASS L1 modulation symbol at an elapsed time.
pub fn glonass_l1_string_symbol_at_time_s(
    raw_data_bits: &[i8; GLONASS_L1_STRING_DATA_BITS],
    elapsed_s: f64,
) -> Result<i8, SignalError> {
    if !elapsed_s.is_finite() || elapsed_s < 0.0 {
        return Err(SignalError::InvalidElapsedDuration);
    }

    let symbol_index = (elapsed_s / GLONASS_L1_SYMBOL_PERIOD_S).floor() as usize;
    glonass_l1_string_symbol(raw_data_bits, symbol_index)
}

fn shift_register(register: &mut [u8; 9], feedback: u8) {
    for index in (1..register.len()).rev() {
        register[index] = register[index - 1];
    }
    register[0] = feedback;
}

fn validate_bipolar_navigation_symbol(symbol: i8) -> Result<(), SignalError> {
    if matches!(symbol, -1 | 1) {
        Ok(())
    } else {
        Err(SignalError::InvalidNavigationSymbol(symbol))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        generate_glonass_l1_st_code_chips, glonass_l1_relative_data_symbol,
        glonass_l1_string_symbol, GLONASS_L1_STRING_DATA_BITS, GLONASS_L1_STRING_DATA_SYMBOLS,
        GLONASS_L1_STRING_SYMBOLS, GLONASS_L1_ST_CODE_CHIPS, GLONASS_L1_TIME_MARK,
    };

    #[test]
    fn glonass_l1_reference_prefix_matches_icd_sequence() {
        let code = generate_glonass_l1_st_code_chips(9);
        let bits =
            code.into_iter().map(|chip| if chip > 0 { '0' } else { '1' }).collect::<String>();

        assert_eq!(bits, "111111100");
    }

    #[test]
    fn glonass_l1_period_repeats_after_511_chips() {
        let two_periods = generate_glonass_l1_st_code_chips(GLONASS_L1_ST_CODE_CHIPS * 2);
        let (first_period, second_period) = two_periods.split_at(GLONASS_L1_ST_CODE_CHIPS);

        assert_eq!(first_period, second_period);
    }

    #[test]
    fn glonass_l1_relative_data_symbol_applies_one_bit_delay_xor_logic() {
        assert_eq!(glonass_l1_relative_data_symbol(1, 1).expect("valid bipolar symbols"), 1);
        assert_eq!(glonass_l1_relative_data_symbol(1, -1).expect("valid bipolar symbols"), -1);
        assert_eq!(glonass_l1_relative_data_symbol(-1, -1).expect("valid bipolar symbols"), 1);
    }

    #[test]
    fn glonass_l1_time_mark_matches_published_sequence() {
        let bits = GLONASS_L1_TIME_MARK
            .into_iter()
            .map(|chip| if chip > 0 { '0' } else { '1' })
            .collect::<String>();

        assert_eq!(bits, "111110001101110101000010010110");
    }

    #[test]
    fn glonass_l1_string_switches_from_relative_data_to_time_mark_after_1p7_seconds() {
        let raw_data_bits = [1; GLONASS_L1_STRING_DATA_BITS];

        assert_eq!(
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_DATA_SYMBOLS - 2)
                .expect("data symbol"),
            1
        );
        assert_eq!(
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_DATA_SYMBOLS - 1)
                .expect("data symbol"),
            -1
        );
        assert_eq!(
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_DATA_SYMBOLS)
                .expect("time mark symbol"),
            GLONASS_L1_TIME_MARK[0]
        );
        assert_eq!(
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_SYMBOLS - 1)
                .expect("last time mark symbol"),
            GLONASS_L1_TIME_MARK[GLONASS_L1_TIME_MARK.len() - 1]
        );
        assert_eq!(
            glonass_l1_string_symbol(&raw_data_bits, GLONASS_L1_STRING_SYMBOLS)
                .expect("wrapped symbol"),
            1
        );
    }
}
