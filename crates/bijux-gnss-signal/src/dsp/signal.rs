#![allow(missing_docs)]

//! Signal processing utilities.

use crate::codes::ca_code::{generate_ca_code, Prn};
use crate::dsp::nco::Nco;
use crate::error::SignalError;
use num_complex::Complex;

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
    validate_sample_rate(sample_rate_hz)?;
    validate_code_rate(code_rate_hz)?;
    validate_code_phase(start_chip_phase)?;
    validate_code_length(code_length)?;

    let chip_advance = sample_count as f64 * code_rate_hz / sample_rate_hz;
    Ok((start_chip_phase + chip_advance).rem_euclid(code_length as f64))
}

/// Advance a wrapped code phase by an elapsed duration in seconds.
pub fn advance_code_phase_seconds(
    start_chip_phase: f64,
    code_rate_hz: f64,
    elapsed_seconds: f64,
    code_length: usize,
) -> Result<f64, SignalError> {
    validate_code_rate(code_rate_hz)?;
    validate_code_phase(start_chip_phase)?;
    validate_elapsed_seconds(elapsed_seconds)?;
    validate_code_length(code_length)?;

    let chip_advance = elapsed_seconds * code_rate_hz;
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
    validate_sample_rate(sample_rate_hz)?;
    validate_code_rate(code_rate_hz)?;
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

/// Mix a carrier down to baseband from an absolute sample origin.
pub fn wipeoff_carrier(
    samples: &[Complex<f32>],
    carrier_hz: f64,
    sample_rate_hz: f64,
    start_sample_index: u64,
    phase_offset_rad: f64,
) -> Result<Vec<Complex<f32>>, SignalError> {
    validate_sample_rate(sample_rate_hz)?;
    validate_carrier_frequency(carrier_hz)?;
    validate_code_phase(phase_offset_rad)?;

    let mut nco =
        Nco::from_sample_index(carrier_hz, sample_rate_hz, start_sample_index, phase_offset_rad);
    let mut mixed = Vec::with_capacity(samples.len());
    for sample in samples {
        let (sin, cos) = nco.next_sin_cos();
        let rot = Complex::new(cos as f32, -sin as f32);
        mixed.push(*sample * rot);
    }
    Ok(mixed)
}

fn code_value_at_phase_unchecked(code: &[i8], chip_phase: f64) -> f32 {
    let wrapped = chip_phase.rem_euclid(code.len() as f64);
    let chip_index = wrapped.floor() as usize;
    code[chip_index] as f32
}

fn validate_sample_rate(sample_rate_hz: f64) -> Result<(), SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    Ok(())
}

fn validate_code_rate(code_rate_hz: f64) -> Result<(), SignalError> {
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

fn validate_carrier_frequency(carrier_hz: f64) -> Result<(), SignalError> {
    if !carrier_hz.is_finite() {
        return Err(SignalError::InvalidCarrierFrequency);
    }
    Ok(())
}

fn validate_elapsed_seconds(elapsed_seconds: f64) -> Result<(), SignalError> {
    if !elapsed_seconds.is_finite() || elapsed_seconds < 0.0 {
        return Err(SignalError::InvalidElapsedDuration);
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
