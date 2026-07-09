#![allow(missing_docs)]

//! Signal processing utilities.

use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::error::SignalError;

/// Compute the number of samples per C/A code period (1 ms) given sampling
/// frequency and code parameters.
pub fn samples_per_code(
    sampling_freq_hz: f64,
    code_freq_basis_hz: f64,
    code_length: usize,
) -> usize {
    (sampling_freq_hz / (code_freq_basis_hz / code_length as f64)).round() as usize
}

/// Advance a wrapped code phase by a number of sampled intervals.
pub fn advance_code_phase_chips(
    start_chip_phase: f64,
    sample_rate_hz: f64,
    code_rate_hz: f64,
    sample_count: usize,
    code_length: usize,
) -> Result<f64, SignalError> {
    validate_rate_inputs(sample_rate_hz, code_rate_hz)?;
    validate_code_phase(start_chip_phase)?;
    validate_code_length(code_length)?;

    let chip_advance = sample_count as f64 * code_rate_hz / sample_rate_hz;
    Ok((start_chip_phase + chip_advance).rem_euclid(code_length as f64))
}

/// Return the code value at a wrapped chip phase.
pub fn code_value_at_phase(code: &[i8], chip_phase: f64) -> Result<f32, SignalError> {
    validate_code_phase(chip_phase)?;
    validate_code(code)?;

    Ok(code_value_at_phase_unchecked(code, chip_phase))
}

/// Sample a spreading code at arbitrary sample and code rates from a chip phase origin.
pub fn sample_code(
    code: &[i8],
    sample_rate_hz: f64,
    code_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    validate_rate_inputs(sample_rate_hz, code_rate_hz)?;
    validate_code_phase(start_chip_phase)?;
    validate_code(code)?;

    let chips_per_sample = code_rate_hz / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);

    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        samples.push(code_value_at_phase_unchecked(code, chip_phase));
    }

    Ok(samples)
}

/// Sample a GPS L1 C/A code at arbitrary sample and code rates from a chip phase origin.
pub fn sample_ca_code(
    prn: Prn,
    sample_rate_hz: f64,
    code_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_ca_code(prn)?;
    sample_code(&code, sample_rate_hz, code_rate_hz, start_chip_phase, sample_count)
}

fn code_value_at_phase_unchecked(code: &[i8], chip_phase: f64) -> f32 {
    let wrapped = chip_phase.rem_euclid(code.len() as f64);
    let chip_index = wrapped.floor() as usize;
    code[chip_index] as f32
}

fn validate_rate_inputs(sample_rate_hz: f64, code_rate_hz: f64) -> Result<(), SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !code_rate_hz.is_finite() || code_rate_hz <= 0.0 {
        return Err(SignalError::InvalidCodeRate);
    }
    Ok(())
}

fn validate_code_phase(chip_phase: f64) -> Result<(), SignalError> {
    if !chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }
    Ok(())
}

fn validate_code(code: &[i8]) -> Result<(), SignalError> {
    validate_code_length(code.len())
}

fn validate_code_length(code_length: usize) -> Result<(), SignalError> {
    if code_length == 0 {
        return Err(SignalError::EmptyCodeSequence);
    }
    Ok(())
}
