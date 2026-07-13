#![allow(missing_docs)]

//! Galileo E5 primary-code, secondary-code, and narrowband QPSK helpers.
//!
//! Clean-room implementation derived from the public Galileo OS SIS ICD v2.2.

use num_complex::Complex;

use crate::codes::ca_code::periodic_correlation;
use crate::codes::galileo_e5_tables::{
    GALILEO_E5A_I_INITIAL_SEQUENCE_HEX, GALILEO_E5A_I_SECONDARY_HEX,
    GALILEO_E5A_I_START_VALUES_OCTAL, GALILEO_E5A_Q_INITIAL_SEQUENCE_HEX,
    GALILEO_E5A_Q_SECONDARY_HEX, GALILEO_E5A_Q_START_VALUES_OCTAL,
    GALILEO_E5B_I_INITIAL_SEQUENCE_HEX, GALILEO_E5B_I_SECONDARY_HEX,
    GALILEO_E5B_I_START_VALUES_OCTAL, GALILEO_E5B_Q_INITIAL_SEQUENCE_HEX,
    GALILEO_E5B_Q_SECONDARY_HEX, GALILEO_E5B_Q_START_VALUES_OCTAL,
};
use crate::dsp::signal::{code_value_at_phase, sample_code};
use crate::error::SignalError;

pub const GALILEO_E5A_PRIMARY_CODE_CHIPS: usize = 10_230;
pub const GALILEO_E5A_CODE_RATE_HZ: f64 = 10_230_000.0;
pub const GALILEO_E5A_I_SECONDARY_CODE_CHIPS: usize = 20;
pub const GALILEO_E5A_Q_SECONDARY_CODE_CHIPS: usize = 100;
pub const GALILEO_E5A_I_PRIMARY_EPOCHS_PER_SYMBOL: usize = GALILEO_E5A_I_SECONDARY_CODE_CHIPS;
pub const GALILEO_E5B_PRIMARY_CODE_CHIPS: usize = 10_230;
pub const GALILEO_E5B_CODE_RATE_HZ: f64 = 10_230_000.0;
pub const GALILEO_E5B_I_SECONDARY_CODE_CHIPS: usize = 4;
pub const GALILEO_E5B_Q_SECONDARY_CODE_CHIPS: usize = 100;
pub const GALILEO_E5B_I_PRIMARY_EPOCHS_PER_SYMBOL: usize = GALILEO_E5B_I_SECONDARY_CODE_CHIPS;

const GALILEO_E5A_REGISTER_STAGES: usize = 14;
const GALILEO_E5A_REGISTER1_TAPS: [usize; 4] = [1, 6, 8, 14];
const GALILEO_E5A_REGISTER2_TAPS: [usize; 6] = [4, 5, 7, 8, 12, 14];
const GALILEO_E5B_REGISTER1_TAPS: [usize; 4] = [4, 11, 13, 14];
const GALILEO_E5B_I_REGISTER2_TAPS: [usize; 6] = [2, 5, 8, 9, 12, 14];
const GALILEO_E5B_Q_REGISTER2_TAPS: [usize; 6] = [1, 5, 6, 9, 10, 14];
const GALILEO_E5A_COMPONENT_POWER_SCALE: f32 = std::f32::consts::FRAC_1_SQRT_2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5aICodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5aQCodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
    pub secondary_code_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5bICodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GalileoE5bQCodeAssignment {
    pub prn: u8,
    pub register_2_start_octal: &'static str,
    pub initial_sequence_hex: &'static str,
    pub secondary_code_hex: &'static str,
}

pub const GALILEO_E5A_I_CODE_ASSIGNMENTS: [GalileoE5aICodeAssignment; 50] =
    build_e5a_i_code_assignments();
pub const GALILEO_E5A_Q_CODE_ASSIGNMENTS: [GalileoE5aQCodeAssignment; 50] =
    build_e5a_q_code_assignments();
pub const GALILEO_E5B_I_CODE_ASSIGNMENTS: [GalileoE5bICodeAssignment; 50] =
    build_e5b_i_code_assignments();
pub const GALILEO_E5B_Q_CODE_ASSIGNMENTS: [GalileoE5bQCodeAssignment; 50] =
    build_e5b_q_code_assignments();

const fn build_e5a_i_code_assignments() -> [GalileoE5aICodeAssignment; 50] {
    let mut assignments = [GalileoE5aICodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5aICodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5A_I_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5A_I_INITIAL_SEQUENCE_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5a_q_code_assignments() -> [GalileoE5aQCodeAssignment; 50] {
    let mut assignments = [GalileoE5aQCodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
        secondary_code_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5aQCodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5A_Q_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5A_Q_INITIAL_SEQUENCE_HEX[index],
            secondary_code_hex: GALILEO_E5A_Q_SECONDARY_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5b_i_code_assignments() -> [GalileoE5bICodeAssignment; 50] {
    let mut assignments = [GalileoE5bICodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5bICodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5B_I_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5B_I_INITIAL_SEQUENCE_HEX[index],
        };
        index += 1;
    }
    assignments
}

const fn build_e5b_q_code_assignments() -> [GalileoE5bQCodeAssignment; 50] {
    let mut assignments = [GalileoE5bQCodeAssignment {
        prn: 1,
        register_2_start_octal: "",
        initial_sequence_hex: "",
        secondary_code_hex: "",
    }; 50];
    let mut index = 0usize;
    while index < assignments.len() {
        assignments[index] = GalileoE5bQCodeAssignment {
            prn: (index + 1) as u8,
            register_2_start_octal: GALILEO_E5B_Q_START_VALUES_OCTAL[index],
            initial_sequence_hex: GALILEO_E5B_Q_INITIAL_SEQUENCE_HEX[index],
            secondary_code_hex: GALILEO_E5B_Q_SECONDARY_HEX[index],
        };
        index += 1;
    }
    assignments
}

pub fn galileo_e5a_i_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5aICodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5A_I_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5a_q_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5aQCodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5A_Q_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5a_i_code_assignments() -> &'static [GalileoE5aICodeAssignment; 50] {
    &GALILEO_E5A_I_CODE_ASSIGNMENTS
}

pub fn galileo_e5a_q_code_assignments() -> &'static [GalileoE5aQCodeAssignment; 50] {
    &GALILEO_E5A_Q_CODE_ASSIGNMENTS
}

pub fn galileo_e5b_i_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5bICodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5B_I_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5b_q_code_assignment(
    prn: u8,
) -> Result<&'static GalileoE5bQCodeAssignment, SignalError> {
    let index = e5_prn_index(prn)?;
    Ok(&GALILEO_E5B_Q_CODE_ASSIGNMENTS[index])
}

pub fn galileo_e5b_i_code_assignments() -> &'static [GalileoE5bICodeAssignment; 50] {
    &GALILEO_E5B_I_CODE_ASSIGNMENTS
}

pub fn galileo_e5b_q_code_assignments() -> &'static [GalileoE5bQCodeAssignment; 50] {
    &GALILEO_E5B_Q_CODE_ASSIGNMENTS
}

pub fn generate_galileo_e5a_i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let assignment = galileo_e5a_i_code_assignment(prn)?;
    generate_e5_primary_code(
        assignment.register_2_start_octal,
        &GALILEO_E5A_REGISTER1_TAPS,
        &GALILEO_E5A_REGISTER2_TAPS,
    )
}

pub fn generate_galileo_e5a_q_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let assignment = galileo_e5a_q_code_assignment(prn)?;
    generate_e5_primary_code(
        assignment.register_2_start_octal,
        &GALILEO_E5A_REGISTER1_TAPS,
        &GALILEO_E5A_REGISTER2_TAPS,
    )
}

pub fn generate_galileo_e5b_i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let assignment = galileo_e5b_i_code_assignment(prn)?;
    generate_e5_primary_code(
        assignment.register_2_start_octal,
        &GALILEO_E5B_REGISTER1_TAPS,
        &GALILEO_E5B_I_REGISTER2_TAPS,
    )
}

pub fn generate_galileo_e5b_q_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let assignment = galileo_e5b_q_code_assignment(prn)?;
    generate_e5_primary_code(
        assignment.register_2_start_octal,
        &GALILEO_E5B_REGISTER1_TAPS,
        &GALILEO_E5B_Q_REGISTER2_TAPS,
    )
}

pub fn galileo_e5a_i_secondary_code() -> [i8; GALILEO_E5A_I_SECONDARY_CODE_CHIPS] {
    decode_hex_sequence::<GALILEO_E5A_I_SECONDARY_CODE_CHIPS>(GALILEO_E5A_I_SECONDARY_HEX)
}

pub fn galileo_e5a_q_secondary_code(
    prn: u8,
) -> Result<[i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS], SignalError> {
    let assignment = galileo_e5a_q_code_assignment(prn)?;
    Ok(decode_hex_sequence::<GALILEO_E5A_Q_SECONDARY_CODE_CHIPS>(assignment.secondary_code_hex))
}

pub fn galileo_e5b_i_secondary_code() -> [i8; GALILEO_E5B_I_SECONDARY_CODE_CHIPS] {
    decode_hex_sequence::<GALILEO_E5B_I_SECONDARY_CODE_CHIPS>(GALILEO_E5B_I_SECONDARY_HEX)
}

pub fn galileo_e5b_q_secondary_code(
    prn: u8,
) -> Result<[i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS], SignalError> {
    let assignment = galileo_e5b_q_code_assignment(prn)?;
    Ok(decode_hex_sequence::<GALILEO_E5B_Q_SECONDARY_CODE_CHIPS>(assignment.secondary_code_hex))
}

pub fn galileo_e5a_i_secondary_chip(primary_code_period_index: usize) -> i8 {
    let secondary = galileo_e5a_i_secondary_code();
    secondary[primary_code_period_index % secondary.len()]
}

pub fn galileo_e5a_q_secondary_chip(
    secondary_code: &[i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
    primary_code_period_index: usize,
) -> i8 {
    secondary_code[primary_code_period_index % secondary_code.len()]
}

pub fn galileo_e5b_i_secondary_chip(primary_code_period_index: usize) -> i8 {
    let secondary = galileo_e5b_i_secondary_code();
    secondary[primary_code_period_index % secondary.len()]
}

pub fn galileo_e5b_q_secondary_chip(
    secondary_code: &[i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
    primary_code_period_index: usize,
) -> i8 {
    secondary_code[primary_code_period_index % secondary_code.len()]
}

pub fn galileo_e5a_i_data_symbol_index(primary_code_period_index: usize) -> usize {
    primary_code_period_index / GALILEO_E5A_I_PRIMARY_EPOCHS_PER_SYMBOL
}

pub fn galileo_e5b_i_data_symbol_index(primary_code_period_index: usize) -> usize {
    primary_code_period_index / GALILEO_E5B_I_PRIMARY_EPOCHS_PER_SYMBOL
}

pub fn galileo_e5a_i_epoch_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    epoch_symbol(
        data_symbols,
        galileo_e5a_i_data_symbol_index(primary_code_period_index),
        galileo_e5a_i_secondary_chip(primary_code_period_index),
    )
}

pub fn galileo_e5a_q_epoch_symbol(
    secondary_code: &[i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
    primary_code_period_index: usize,
) -> i8 {
    galileo_e5a_q_secondary_chip(secondary_code, primary_code_period_index)
}

pub fn galileo_e5b_i_epoch_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    epoch_symbol(
        data_symbols,
        galileo_e5b_i_data_symbol_index(primary_code_period_index),
        galileo_e5b_i_secondary_chip(primary_code_period_index),
    )
}

pub fn galileo_e5b_q_epoch_symbol(
    secondary_code: &[i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
    primary_code_period_index: usize,
) -> i8 {
    galileo_e5b_q_secondary_chip(secondary_code, primary_code_period_index)
}

pub fn sample_galileo_e5a_i_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_galileo_e5a_i_code(prn)?;
    sample_code(&code, sample_rate_hz, GALILEO_E5A_CODE_RATE_HZ, start_chip_phase, sample_count)
}

pub fn sample_galileo_e5a_q_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_galileo_e5a_q_code(prn)?;
    sample_code(&code, sample_rate_hz, GALILEO_E5A_CODE_RATE_HZ, start_chip_phase, sample_count)
}

pub fn sample_galileo_e5b_i_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_galileo_e5b_i_code(prn)?;
    sample_code(&code, sample_rate_hz, GALILEO_E5B_CODE_RATE_HZ, start_chip_phase, sample_count)
}

pub fn sample_galileo_e5b_q_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_galileo_e5b_q_code(prn)?;
    sample_code(&code, sample_rate_hz, GALILEO_E5B_CODE_RATE_HZ, start_chip_phase, sample_count)
}

pub fn galileo_e5a_i_value(
    primary_code: &[i8],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_symbols: &[i8],
) -> Result<f32, SignalError> {
    Ok(code_value_at_phase(primary_code, chip_phase)?
        * galileo_e5a_i_epoch_symbol(data_symbols, primary_code_period_index)? as f32
        * GALILEO_E5A_COMPONENT_POWER_SCALE)
}

pub fn galileo_e5a_q_value(
    primary_code: &[i8],
    secondary_code: &[i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
    chip_phase: f64,
    primary_code_period_index: usize,
) -> Result<f32, SignalError> {
    Ok(code_value_at_phase(primary_code, chip_phase)?
        * galileo_e5a_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
        * GALILEO_E5A_COMPONENT_POWER_SCALE)
}

pub fn galileo_e5a_qpsk_value(
    e5ai_primary_code: &[i8],
    e5aq_primary_code: &[i8],
    e5aq_secondary_code: &[i8; GALILEO_E5A_Q_SECONDARY_CODE_CHIPS],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_symbols: &[i8],
) -> Result<Complex<f32>, SignalError> {
    Ok(Complex::new(
        galileo_e5a_i_value(
            e5ai_primary_code,
            chip_phase,
            primary_code_period_index,
            data_symbols,
        )?,
        galileo_e5a_q_value(
            e5aq_primary_code,
            e5aq_secondary_code,
            chip_phase,
            primary_code_period_index,
        )?,
    ))
}

pub fn galileo_e5a_primary_autocorrelation(primary_code: &[i8]) -> Result<Vec<i16>, SignalError> {
    periodic_correlation(primary_code, primary_code)
}

pub fn galileo_e5b_i_value(
    primary_code: &[i8],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_symbols: &[i8],
) -> Result<f32, SignalError> {
    Ok(code_value_at_phase(primary_code, chip_phase)?
        * galileo_e5b_i_epoch_symbol(data_symbols, primary_code_period_index)? as f32
        * GALILEO_E5A_COMPONENT_POWER_SCALE)
}

pub fn galileo_e5b_q_value(
    primary_code: &[i8],
    secondary_code: &[i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
    chip_phase: f64,
    primary_code_period_index: usize,
) -> Result<f32, SignalError> {
    Ok(code_value_at_phase(primary_code, chip_phase)?
        * galileo_e5b_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
        * GALILEO_E5A_COMPONENT_POWER_SCALE)
}

pub fn galileo_e5b_qpsk_value(
    e5bi_primary_code: &[i8],
    e5bq_primary_code: &[i8],
    e5bq_secondary_code: &[i8; GALILEO_E5B_Q_SECONDARY_CODE_CHIPS],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_symbols: &[i8],
) -> Result<Complex<f32>, SignalError> {
    Ok(Complex::new(
        galileo_e5b_i_value(
            e5bi_primary_code,
            chip_phase,
            primary_code_period_index,
            data_symbols,
        )?,
        galileo_e5b_q_value(
            e5bq_primary_code,
            e5bq_secondary_code,
            chip_phase,
            primary_code_period_index,
        )?,
    ))
}

pub fn galileo_e5b_primary_autocorrelation(primary_code: &[i8]) -> Result<Vec<i16>, SignalError> {
    periodic_correlation(primary_code, primary_code)
}

fn e5_prn_index(prn: u8) -> Result<usize, SignalError> {
    if prn == 0 || prn > 50 {
        return Err(SignalError::UnsupportedPrn(prn));
    }
    Ok(usize::from(prn - 1))
}

fn generate_e5_primary_code(
    register_2_start_octal: &str,
    register_1_taps: &[usize],
    register_2_taps: &[usize],
) -> Result<Vec<i8>, SignalError> {
    let mut register_1 = [1_u8; GALILEO_E5A_REGISTER_STAGES];
    let mut register_2 = register_start_from_octal(register_2_start_octal);
    let mut code = Vec::with_capacity(GALILEO_E5A_PRIMARY_CODE_CHIPS);

    for _ in 0..GALILEO_E5A_PRIMARY_CODE_CHIPS {
        let output_bit = register_1[GALILEO_E5A_REGISTER_STAGES - 1]
            ^ register_2[GALILEO_E5A_REGISTER_STAGES - 1];
        code.push(bit_to_chip(output_bit));
        register_1 = step_register(register_1, register_1_taps);
        register_2 = step_register(register_2, register_2_taps);
    }

    Ok(code)
}

fn epoch_symbol(
    data_symbols: &[i8],
    data_symbol_index: usize,
    secondary_chip: i8,
) -> Result<i8, SignalError> {
    if data_symbols.is_empty() {
        return Err(SignalError::EmptyNavigationSymbolStream);
    }
    let symbol = data_symbols[data_symbol_index % data_symbols.len()];
    if !matches!(symbol, -1 | 1) {
        return Err(SignalError::InvalidNavigationSymbol(symbol));
    }
    Ok(symbol * secondary_chip)
}

fn register_start_from_octal(octal: &str) -> [u8; GALILEO_E5A_REGISTER_STAGES] {
    let parsed = u16::from_str_radix(octal, 8).expect("ICD table uses valid octal values");
    let mut start = [0_u8; GALILEO_E5A_REGISTER_STAGES];
    for (index, slot) in start.iter_mut().enumerate() {
        *slot = ((parsed >> index) & 1) as u8;
    }
    start
}

fn step_register(
    register: [u8; GALILEO_E5A_REGISTER_STAGES],
    taps: &[usize],
) -> [u8; GALILEO_E5A_REGISTER_STAGES] {
    let mut feedback = 0_u8;
    for tap in taps {
        feedback ^= register[*tap - 1];
    }

    let mut next = [0_u8; GALILEO_E5A_REGISTER_STAGES];
    next[0] = feedback;
    next[1..].copy_from_slice(&register[..GALILEO_E5A_REGISTER_STAGES - 1]);
    next
}

fn bit_to_chip(bit: u8) -> i8 {
    1 - (2 * bit as i8)
}

fn decode_hex_sequence<const N: usize>(hex: &str) -> [i8; N] {
    let mut chips = [0_i8; N];
    let expected_hex_digits = N.div_ceil(4);
    debug_assert_eq!(hex.len(), expected_hex_digits);

    let mut chip_index = 0usize;
    for nibble in hex.bytes() {
        let value =
            (nibble as char).to_digit(16).expect("ICD table uses valid hexadecimal values") as u8;
        for bit in (0..4).rev() {
            if chip_index == N {
                break;
            }
            chips[chip_index] = bit_to_chip((value >> bit) & 1);
            chip_index += 1;
        }
    }
    chips
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn e5a_primary_code_rejects_out_of_range_prns() {
        assert_eq!(generate_galileo_e5a_i_code(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_galileo_e5a_q_code(51), Err(SignalError::UnsupportedPrn(51)));
        assert_eq!(generate_galileo_e5b_i_code(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_galileo_e5b_q_code(51), Err(SignalError::UnsupportedPrn(51)));
    }

    #[test]
    fn e5a_i_primary_code_matches_published_prefixes() {
        for (prn, expected_hex) in [(1_u8, "3CEA9D"), (25, "A7D629"), (50, "A5029C")] {
            let code = generate_galileo_e5a_i_code(prn).expect("valid E5a-I PRN");
            assert_eq!(code.len(), GALILEO_E5A_PRIMARY_CODE_CHIPS);
            assert_eq!(first_24_chips_hex(&code), expected_hex, "prn={prn}");
        }
    }

    #[test]
    fn e5a_q_primary_code_matches_published_prefixes() {
        for (prn, expected_hex) in [(1_u8, "515537"), (25, "DCD55C"), (50, "53DA0E")] {
            let code = generate_galileo_e5a_q_code(prn).expect("valid E5a-Q PRN");
            assert_eq!(code.len(), GALILEO_E5A_PRIMARY_CODE_CHIPS);
            assert_eq!(first_24_chips_hex(&code), expected_hex, "prn={prn}");
        }
    }

    #[test]
    fn e5b_i_primary_code_matches_published_prefixes() {
        for (prn, expected_hex) in [(1_u8, "C5BEA1"), (25, "1969C0"), (50, "AFC22B")] {
            let code = generate_galileo_e5b_i_code(prn).expect("valid E5b-I PRN");
            assert_eq!(code.len(), GALILEO_E5B_PRIMARY_CODE_CHIPS);
            assert_eq!(first_24_chips_hex(&code), expected_hex, "prn={prn}");
        }
    }

    #[test]
    fn e5b_q_primary_code_matches_published_prefixes() {
        for (prn, expected_hex) in [(1_u8, "E49AF0"), (25, "71DE13"), (50, "37AF4F")] {
            let code = generate_galileo_e5b_q_code(prn).expect("valid E5b-Q PRN");
            assert_eq!(code.len(), GALILEO_E5B_PRIMARY_CODE_CHIPS);
            assert_eq!(first_24_chips_hex(&code), expected_hex, "prn={prn}");
        }
    }

    #[test]
    fn e5a_secondary_codes_match_published_lengths_and_wrap() {
        let e5ai = galileo_e5a_i_secondary_code();
        let e5aq = galileo_e5a_q_secondary_code(1).expect("valid E5a-Q PRN");

        assert_eq!(e5ai.len(), GALILEO_E5A_I_SECONDARY_CODE_CHIPS);
        assert_eq!(e5aq.len(), GALILEO_E5A_Q_SECONDARY_CODE_CHIPS);
        assert_eq!(galileo_e5a_i_secondary_chip(0), e5ai[0]);
        assert_eq!(
            galileo_e5a_q_secondary_chip(&e5aq, GALILEO_E5A_Q_SECONDARY_CODE_CHIPS),
            e5aq[0]
        );
    }

    #[test]
    fn e5b_secondary_codes_match_published_lengths_and_wrap() {
        let e5bi = galileo_e5b_i_secondary_code();
        let e5bq = galileo_e5b_q_secondary_code(1).expect("valid E5b-Q PRN");

        assert_eq!(e5bi.len(), GALILEO_E5B_I_SECONDARY_CODE_CHIPS);
        assert_eq!(e5bq.len(), GALILEO_E5B_Q_SECONDARY_CODE_CHIPS);
        assert_eq!(galileo_e5b_i_secondary_chip(0), e5bi[0]);
        assert_eq!(
            galileo_e5b_q_secondary_chip(&e5bq, GALILEO_E5B_Q_SECONDARY_CODE_CHIPS),
            e5bq[0]
        );
    }

    #[test]
    fn e5a_i_epoch_symbol_combines_data_and_secondary_code() {
        let data_symbols = [1_i8, -1];
        assert_eq!(galileo_e5a_i_data_symbol_index(0), 0);
        assert_eq!(galileo_e5a_i_data_symbol_index(19), 0);
        assert_eq!(galileo_e5a_i_data_symbol_index(20), 1);
        assert_eq!(
            galileo_e5a_i_epoch_symbol(&data_symbols, 0).expect("valid first epoch"),
            galileo_e5a_i_secondary_chip(0)
        );
        assert_eq!(
            galileo_e5a_i_epoch_symbol(&data_symbols, 20).expect("valid second data symbol"),
            -galileo_e5a_i_secondary_chip(0)
        );
    }

    #[test]
    fn e5a_i_epoch_symbol_wraps_single_symbol_stream() {
        assert_eq!(
            galileo_e5a_i_epoch_symbol(&[1], 20).expect("wrapped data symbol"),
            galileo_e5a_i_secondary_chip(20)
        );
    }

    #[test]
    fn e5b_i_epoch_symbol_combines_data_and_secondary_code() {
        let data_symbols = [1_i8, -1];
        assert_eq!(galileo_e5b_i_data_symbol_index(0), 0);
        assert_eq!(galileo_e5b_i_data_symbol_index(3), 0);
        assert_eq!(galileo_e5b_i_data_symbol_index(4), 1);
        assert_eq!(
            galileo_e5b_i_epoch_symbol(&data_symbols, 0).expect("valid first epoch"),
            galileo_e5b_i_secondary_chip(0)
        );
        assert_eq!(
            galileo_e5b_i_epoch_symbol(&data_symbols, 4).expect("valid second data symbol"),
            -galileo_e5b_i_secondary_chip(0)
        );
    }

    #[test]
    fn e5b_i_epoch_symbol_wraps_single_symbol_stream() {
        assert_eq!(
            galileo_e5b_i_epoch_symbol(&[1], 4).expect("wrapped data symbol"),
            galileo_e5b_i_secondary_chip(4)
        );
    }

    #[test]
    fn e5a_qpsk_value_preserves_unit_power_when_both_components_present() {
        let e5ai = generate_galileo_e5a_i_code(1).expect("valid E5a-I PRN");
        let e5aq = generate_galileo_e5a_q_code(1).expect("valid E5a-Q PRN");
        let secondary = galileo_e5a_q_secondary_code(1).expect("valid E5a-Q PRN");

        let value =
            galileo_e5a_qpsk_value(&e5ai, &e5aq, &secondary, 0.0, 0, &[1]).expect("QPSK value");

        assert!((value.norm() - 1.0).abs() < 1.0e-6, "{value:?}");
    }

    #[test]
    fn e5b_qpsk_value_preserves_unit_power_when_both_components_present() {
        let e5bi = generate_galileo_e5b_i_code(1).expect("valid E5b-I PRN");
        let e5bq = generate_galileo_e5b_q_code(1).expect("valid E5b-Q PRN");
        let secondary = galileo_e5b_q_secondary_code(1).expect("valid E5b-Q PRN");

        let value =
            galileo_e5b_qpsk_value(&e5bi, &e5bq, &secondary, 0.0, 0, &[1]).expect("QPSK value");

        assert!((value.norm() - 1.0).abs() < 1.0e-6, "{value:?}");
    }

    #[test]
    fn e5a_primary_codes_keep_full_period_autocorrelation_peak() {
        let code = generate_galileo_e5a_i_code(7).expect("valid E5a-I PRN");
        let correlation = galileo_e5a_primary_autocorrelation(&code).expect("autocorrelation");

        assert_eq!(correlation[0], GALILEO_E5A_PRIMARY_CODE_CHIPS as i16);
        assert!(correlation[1].abs() < correlation[0]);
    }

    #[test]
    fn e5b_primary_codes_keep_full_period_autocorrelation_peak() {
        let code = generate_galileo_e5b_i_code(7).expect("valid E5b-I PRN");
        let correlation = galileo_e5b_primary_autocorrelation(&code).expect("autocorrelation");

        assert_eq!(correlation[0], GALILEO_E5B_PRIMARY_CODE_CHIPS as i16);
        assert!(correlation[1].abs() < correlation[0]);
    }

    fn first_24_chips_hex(code: &[i8]) -> String {
        debug_assert!(code.len() >= 24);
        let mut value = 0_u32;
        for chip in code.iter().take(24) {
            value <<= 1;
            if *chip == -1 {
                value |= 1;
            }
        }
        format!("{value:06X}")
    }
}
