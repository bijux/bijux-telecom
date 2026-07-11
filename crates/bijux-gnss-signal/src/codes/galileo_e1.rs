#![allow(missing_docs)]

use crate::codes::galileo_e1_tables::{E1B_HEX, E1C_HEX, E1C_SECONDARY_CODE};
use crate::dsp::signal::code_value_at_phase;
use crate::error::SignalError;

pub const GALILEO_E1_PRIMARY_CODE_CHIPS: usize = 4092;
pub const GALILEO_E1_SECONDARY_CODE_CHIPS: usize = 25;
pub const GALILEO_E1_CODE_RATE_HZ: f64 = 1_023_000.0;
pub const GALILEO_E1_PRIMARY_PERIOD_MS: u32 = 4;
pub const GALILEO_E1_CBOC_ALPHA: f32 = 0.953_462_6;
pub const GALILEO_E1_CBOC_BETA: f32 = 0.301_511_35;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GalileoE1Channel {
    E1B,
    E1C,
}

pub fn generate_galileo_e1b_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    generate_galileo_e1_primary_code(prn, GalileoE1Channel::E1B)
}

pub fn generate_galileo_e1c_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    generate_galileo_e1_primary_code(prn, GalileoE1Channel::E1C)
}

pub fn generate_galileo_e1_primary_code(
    prn: u8,
    channel: GalileoE1Channel,
) -> Result<Vec<i8>, SignalError> {
    if prn == 0 {
        return Err(SignalError::UnsupportedPrn(prn));
    }
    let index = usize::from(prn - 1);
    let hex = match channel {
        GalileoE1Channel::E1B => E1B_HEX.get(index),
        GalileoE1Channel::E1C => E1C_HEX.get(index),
    }
    .ok_or(SignalError::UnsupportedPrn(prn))?;

    let mut code = Vec::with_capacity(GALILEO_E1_PRIMARY_CODE_CHIPS);
    for ch in hex.bytes() {
        let nibble = (ch as char).to_digit(16).ok_or(SignalError::UnsupportedPrn(prn))? as u8;
        for bit in (0..4).rev() {
            code.push(1 - (2 * ((nibble >> bit) & 1) as i8));
        }
    }
    debug_assert_eq!(code.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    Ok(code)
}

pub fn galileo_e1c_secondary_code() -> [i8; GALILEO_E1_SECONDARY_CODE_CHIPS] {
    let mut code = [0_i8; GALILEO_E1_SECONDARY_CODE_CHIPS];
    debug_assert_eq!(E1C_SECONDARY_CODE.len(), GALILEO_E1_SECONDARY_CODE_CHIPS);
    for (chip, bit) in code.iter_mut().zip(E1C_SECONDARY_CODE.bytes()) {
        *chip = 1 - (2 * (bit - b'0') as i8);
    }
    code
}

pub fn galileo_e1c_secondary_chip(primary_code_period_index: usize) -> i8 {
    let secondary = galileo_e1c_secondary_code();
    secondary[primary_code_period_index % secondary.len()]
}

pub fn boc_subcarrier_value(chip_phase: f64, cycles_per_chip: u32) -> Result<f32, SignalError> {
    if cycles_per_chip == 0 {
        return Err(SignalError::InvalidCodeRate);
    }
    if !chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let wrapped = chip_phase.rem_euclid(1.0);
    let half_cycle_count = usize::try_from(cycles_per_chip).expect("u32 fits usize") * 2;
    let half_cycle_phase = wrapped * half_cycle_count as f64;
    let half_cycle_index = half_cycle_phase.floor() as usize;
    if half_cycle_index % 2 == 0 {
        Ok(1.0)
    } else {
        Ok(-1.0)
    }
}

pub fn sample_boc_code(
    code: &[i8],
    sample_rate_hz: f64,
    code_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
    cycles_per_chip: u32,
) -> Result<Vec<f32>, SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !code_rate_hz.is_finite() || code_rate_hz <= 0.0 {
        return Err(SignalError::InvalidCodeRate);
    }
    if code.is_empty() {
        return Err(SignalError::EmptyCodeSequence);
    }
    if !start_chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let chips_per_sample = code_rate_hz / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        let chip = code_value_at_phase(code, chip_phase)?;
        let subcarrier = boc_subcarrier_value(chip_phase, cycles_per_chip)?;
        samples.push(chip * subcarrier);
    }
    Ok(samples)
}

pub fn sample_galileo_e1_boc11_code(
    prn: u8,
    channel: GalileoE1Channel,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_galileo_e1_primary_code(prn, channel)?;
    sample_boc_code(
        &code,
        sample_rate_hz,
        GALILEO_E1_CODE_RATE_HZ,
        start_chip_phase,
        sample_count,
        1,
    )
}

pub fn galileo_e1_cboc_value(
    e1b_code: &[i8],
    e1c_code: &[i8],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_bit: i8,
) -> Result<f32, SignalError> {
    let data = code_value_at_phase(e1b_code, chip_phase)? * data_bit as f32;
    let pilot = code_value_at_phase(e1c_code, chip_phase)?
        * galileo_e1c_secondary_chip(primary_code_period_index) as f32;
    let boc11 = boc_subcarrier_value(chip_phase, 1)?;
    let boc61 = boc_subcarrier_value(chip_phase, 6)?;

    Ok(std::f32::consts::FRAC_1_SQRT_2
        * ((data * ((GALILEO_E1_CBOC_ALPHA * boc11) + (GALILEO_E1_CBOC_BETA * boc61)))
            - (pilot * ((GALILEO_E1_CBOC_ALPHA * boc11) - (GALILEO_E1_CBOC_BETA * boc61)))))
}

pub fn sample_galileo_e1_cboc(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
    primary_code_period_index: usize,
    data_bit: i8,
) -> Result<Vec<f32>, SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !start_chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let e1b = generate_galileo_e1b_code(prn)?;
    let e1c = generate_galileo_e1c_code(prn)?;
    let chips_per_sample = GALILEO_E1_CODE_RATE_HZ / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        samples.push(galileo_e1_cboc_value(
            &e1b,
            &e1c,
            chip_phase,
            primary_code_period_index,
            data_bit,
        )?);
    }
    Ok(samples)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primary_code_decode_lengths_match_icd() {
        let e1b = generate_galileo_e1b_code(1).expect("valid Galileo E1-B PRN");
        let e1c = generate_galileo_e1c_code(1).expect("valid Galileo E1-C PRN");

        assert_eq!(e1b.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
        assert_eq!(e1c.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
        assert!(e1b.iter().all(|chip| *chip == -1 || *chip == 1));
        assert!(e1c.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn primary_code_rejects_out_of_range_prns() {
        assert_eq!(
            generate_galileo_e1_primary_code(0, GalileoE1Channel::E1B),
            Err(SignalError::UnsupportedPrn(0))
        );
        assert_eq!(
            generate_galileo_e1_primary_code(51, GalileoE1Channel::E1C),
            Err(SignalError::UnsupportedPrn(51))
        );
    }

    #[test]
    fn secondary_code_matches_published_length_and_signs() {
        let secondary = galileo_e1c_secondary_code();
        assert_eq!(secondary.len(), GALILEO_E1_SECONDARY_CODE_CHIPS);
        assert_eq!(secondary[0], 1);
        assert_eq!(secondary[2], -1);
        assert_eq!(secondary[24], 1);
    }

    #[test]
    fn boc_subcarrier_flips_halfway_through_each_chip() {
        assert_eq!(boc_subcarrier_value(0.0, 1).expect("valid phase"), 1.0);
        assert_eq!(boc_subcarrier_value(0.49, 1).expect("valid phase"), 1.0);
        assert_eq!(boc_subcarrier_value(0.50, 1).expect("valid phase"), -1.0);
        assert_eq!(boc_subcarrier_value(0.99, 1).expect("valid phase"), -1.0);
    }

    #[test]
    fn secondary_code_wraps_by_primary_period_index() {
        assert_eq!(galileo_e1c_secondary_chip(0), 1);
        assert_eq!(galileo_e1c_secondary_chip(2), -1);
        assert_eq!(
            galileo_e1c_secondary_chip(GALILEO_E1_SECONDARY_CODE_CHIPS),
            galileo_e1c_secondary_chip(0)
        );
    }
}
