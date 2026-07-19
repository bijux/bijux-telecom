#![allow(missing_docs)]

//! Absolute sample-index timing helpers for code and replica generation.

use crate::error::SignalError;

/// Code and elapsed-time state for one absolute sample position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CodeSamplePosition {
    /// Elapsed seconds from the origin sample.
    pub elapsed_s: f64,
    /// Wrapped code-chip phase within one primary code period.
    pub chip_phase: f64,
    /// Whole primary-code periods completed at this sample.
    pub primary_code_period_index: usize,
}

/// Resolve wrapped code phase and primary-period state at an absolute sample index.
pub fn code_sample_position_at_index(
    initial_code_phase_chips: f64,
    sample_rate_hz: f64,
    code_rate_hz: f64,
    code_length: usize,
    sample_index: u64,
) -> Result<CodeSamplePosition, SignalError> {
    validate_sample_rate(sample_rate_hz)?;
    validate_code_rate(code_rate_hz)?;
    validate_code_phase(initial_code_phase_chips)?;
    validate_code_length(code_length)?;

    let elapsed_s = sample_index as f64 / sample_rate_hz;
    let total_chip_phase = initial_code_phase_chips + code_rate_hz * elapsed_s;
    let code_length_f64 = code_length as f64;
    let primary_code_period_index = if total_chip_phase <= 0.0 {
        0
    } else {
        (total_chip_phase / code_length_f64).floor() as usize
    };

    Ok(CodeSamplePosition {
        elapsed_s,
        chip_phase: total_chip_phase.rem_euclid(code_length_f64),
        primary_code_period_index,
    })
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

fn validate_code_length(code_length: usize) -> Result<(), SignalError> {
    if code_length == 0 {
        return Err(SignalError::EmptyCodeSequence);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::code_sample_position_at_index;

    #[test]
    fn absolute_sample_index_tracks_wrapped_phase_at_arbitrary_rate() {
        let position = code_sample_position_at_index(412.375, 4_000_000.0, 1_023_000.0, 1023, 1733)
            .expect("valid sample position");

        assert!((position.elapsed_s - 1733.0 / 4_000_000.0).abs() < 1.0e-12);
        assert!((position.chip_phase - 855.58975).abs() < 1.0e-9);
        assert_eq!(position.primary_code_period_index, 0);
    }

    #[test]
    fn negative_initial_phase_wraps_without_negative_period_index() {
        let position = code_sample_position_at_index(-0.25, 4_092_000.0, 1_023_000.0, 4092, 0)
            .expect("valid wrapped position");

        assert!((position.chip_phase - 4091.75).abs() < 1.0e-12);
        assert_eq!(position.primary_code_period_index, 0);
    }
}
