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

/// Wrap a receiver code phase into one code period expressed in samples.
pub fn wrap_code_phase_samples(code_phase_samples: f64, samples_per_code: usize) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    code_phase_samples.rem_euclid(code_period_samples)
}

/// Convert receiver-aligned code phase into the epoch-start convention used by tracking.
pub fn epoch_start_code_phase_samples_from_receiver_phase(
    receiver_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    (code_period_samples - receiver_code_phase_samples.rem_euclid(code_period_samples))
        .rem_euclid(code_period_samples)
}

/// Convert epoch-start code phase back into the receiver-aligned search convention.
pub fn receiver_code_phase_samples_from_epoch_start_phase(
    epoch_start_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    (code_period_samples - epoch_start_code_phase_samples.rem_euclid(code_period_samples))
        .rem_euclid(code_period_samples)
}

/// Measure the shortest wrapped code-phase delta between two sample-domain phases.
pub fn wrapped_code_phase_delta_samples(
    next_code_phase_samples: f64,
    previous_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    let mut delta =
        (next_code_phase_samples - previous_code_phase_samples).rem_euclid(code_period_samples);
    if delta > code_period_samples / 2.0 {
        delta -= code_period_samples;
    }
    delta
}

/// Convert an injected code chip phase into sample-domain code phase at a sample index.
pub fn code_phase_samples_at_sample_index(
    sample_rate_hz: f64,
    code_rate_hz: f64,
    code_length: usize,
    sample_index: u64,
    initial_code_phase_chips: f64,
) -> Result<f64, SignalError> {
    validate_sample_rate(sample_rate_hz)?;
    validate_code_rate(code_rate_hz)?;
    validate_code_phase(initial_code_phase_chips)?;
    validate_code_length(code_length)?;

    let start_s = sample_index as f64 / sample_rate_hz;
    let chip_phase =
        advance_code_phase_seconds(initial_code_phase_chips, code_rate_hz, start_s, code_length)?;
    let samples_per_chip = sample_rate_hz / code_rate_hz;
    Ok(chip_phase * samples_per_chip)
}

/// Convert an injected code chip phase into the receiver search convention.
pub fn receiver_search_code_phase_samples(
    sample_rate_hz: f64,
    code_rate_hz: f64,
    code_length: usize,
    sample_index: u64,
    initial_code_phase_chips: f64,
) -> Result<f64, SignalError> {
    let period_samples = samples_per_code(sample_rate_hz, code_rate_hz, code_length).max(1) as f64;
    let phase_samples = code_phase_samples_at_sample_index(
        sample_rate_hz,
        code_rate_hz,
        code_length,
        sample_index,
        initial_code_phase_chips,
    )?;
    Ok((period_samples - phase_samples.rem_euclid(period_samples)).rem_euclid(period_samples))
}

/// Measure wrapped code-phase error in samples over one code period.
pub fn wrapped_code_phase_error_samples(
    actual: usize,
    expected: usize,
    period_samples: usize,
) -> usize {
    let period_samples = period_samples.max(1);
    let forward = actual.abs_diff(expected);
    let wrapped = period_samples.saturating_sub(forward);
    forward.min(wrapped)
}

/// Measure wrapped code-phase error in samples over one code period for fractional estimates.
pub fn wrapped_code_phase_error_samples_f64(
    actual: f64,
    expected: f64,
    period_samples: usize,
) -> f64 {
    let period_samples = period_samples.max(1) as f64;
    let forward = (actual - expected).abs().rem_euclid(period_samples);
    let wrapped = (period_samples - forward).rem_euclid(period_samples);
    forward.min(wrapped)
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

#[cfg(test)]
mod tests {
    use super::{
        code_phase_samples_at_sample_index, epoch_start_code_phase_samples_from_receiver_phase,
        receiver_code_phase_samples_from_epoch_start_phase, receiver_search_code_phase_samples,
        wrap_code_phase_samples, wrapped_code_phase_delta_samples,
        wrapped_code_phase_error_samples, wrapped_code_phase_error_samples_f64,
    };

    #[test]
    fn receiver_phase_conversions_round_trip_one_code_period() {
        let receiver_phase = 4091.5;
        let epoch_start =
            epoch_start_code_phase_samples_from_receiver_phase(receiver_phase, 4_092);
        let recovered =
            receiver_code_phase_samples_from_epoch_start_phase(epoch_start, 4_092);

        assert!((epoch_start - 0.5).abs() < 1.0e-9, "epoch_start={epoch_start}");
        assert!((recovered - receiver_phase).abs() < 1.0e-9, "recovered={recovered}");
    }

    #[test]
    fn wrap_code_phase_samples_keeps_values_in_one_period() {
        assert!((wrap_code_phase_samples(4_093.25, 4_092) - 1.25).abs() < 1.0e-9);
        assert!((wrap_code_phase_samples(-0.5, 4_092) - 4_091.5).abs() < 1.0e-9);
    }

    #[test]
    fn wrapped_code_phase_delta_chooses_shorter_direction() {
        assert!((wrapped_code_phase_delta_samples(1.0, 4_091.0, 4_092) - 2.0).abs() < 1.0e-9);
        assert!((wrapped_code_phase_delta_samples(4_091.0, 1.0, 4_092) + 2.0).abs() < 1.0e-9);
    }

    #[test]
    fn receiver_search_code_phase_matches_receiver_search_convention() {
        let phase = receiver_search_code_phase_samples(4_092_000.0, 1_023_000.0, 1023, 0, 0.125)
            .expect("valid code phase");
        assert!((phase - 4_091.5).abs() < 1.0e-9, "phase={phase}");
    }

    #[test]
    fn code_phase_samples_at_sample_index_advances_by_elapsed_code_phase() {
        let phase =
            code_phase_samples_at_sample_index(4_092_000.0, 1_023_000.0, 1023, 2_046, 0.5)
                .expect("valid code phase");
        assert!((phase - 2_048.0).abs() < 1.0e-9, "phase={phase}");
    }

    #[test]
    fn wrapped_code_phase_error_measures_shortest_period_distance() {
        assert_eq!(wrapped_code_phase_error_samples(0, 0, 4_092), 0);
        assert_eq!(wrapped_code_phase_error_samples(1, 4_091, 4_092), 2);
        assert_eq!(wrapped_code_phase_error_samples(4_091, 1, 4_092), 2);
        assert!((wrapped_code_phase_error_samples_f64(4_091.5, 0.0, 4_092) - 0.5).abs() < 1.0e-9);
    }
}
