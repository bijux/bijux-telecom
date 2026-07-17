use crate::error::SignalError;
use num_complex::Complex;

/// Carrier frequency at elapsed time for a linear Doppler-rate model.
pub fn carrier_hz_at_time(
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
) -> f64 {
    initial_carrier_hz + carrier_rate_hz_per_s * elapsed_s
}

/// Carrier frequency at elapsed time for a constant carrier-jerk model.
pub fn carrier_hz_at_time_with_jerk(
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    carrier_jerk_hz_per_s2: f64,
    elapsed_s: f64,
) -> f64 {
    initial_carrier_hz
        + carrier_rate_hz_per_s * elapsed_s
        + 0.5 * carrier_jerk_hz_per_s2 * elapsed_s * elapsed_s
}

/// Carrier phase in radians at elapsed time for a linear Doppler-rate model.
pub fn carrier_phase_radians_at_time(
    initial_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
) -> f64 {
    initial_phase_radians
        + std::f64::consts::TAU
            * (initial_carrier_hz * elapsed_s + 0.5 * carrier_rate_hz_per_s * elapsed_s * elapsed_s)
}

/// Carrier phase in radians at elapsed time for a constant carrier-jerk model.
pub fn carrier_phase_radians_at_time_with_jerk(
    initial_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    carrier_jerk_hz_per_s2: f64,
    elapsed_s: f64,
) -> f64 {
    initial_phase_radians
        + std::f64::consts::TAU
            * (initial_carrier_hz * elapsed_s
                + 0.5 * carrier_rate_hz_per_s * elapsed_s * elapsed_s
                + carrier_jerk_hz_per_s2 * elapsed_s * elapsed_s * elapsed_s / 6.0)
}

/// Mix a linear-rate carrier down to baseband from a frame-relative sample origin.
pub fn wipeoff_carrier_with_linear_rate(
    samples: &[Complex<f32>],
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    sample_rate_hz: f64,
    start_sample_index: u64,
    initial_phase_radians: f64,
) -> Result<Vec<Complex<f32>>, SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !initial_carrier_hz.is_finite() || !carrier_rate_hz_per_s.is_finite() {
        return Err(SignalError::InvalidCarrierFrequency);
    }
    if !initial_phase_radians.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let mut mixed = Vec::with_capacity(samples.len());
    for (offset, sample) in samples.iter().enumerate() {
        let elapsed_s = (start_sample_index + offset as u64) as f64 / sample_rate_hz;
        let phase = carrier_phase_radians_at_time(
            initial_phase_radians,
            initial_carrier_hz,
            carrier_rate_hz_per_s,
            elapsed_s,
        );
        mixed.push(*sample * Complex::from_polar(1.0_f32, -(phase as f32)));
    }
    Ok(mixed)
}
