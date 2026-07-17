use crate::dsp::sample_timing::code_sample_position_at_index;
use crate::error::SignalError;
use num_complex::Complex;

use super::{carrier_phase_radians_at_time, ReplicaCodeModel};

/// Inputs for sampling one modulated replica at an absolute sample index.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplicaSampleIndexRequest {
    /// Receiver sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Initial code phase in chips at sample index zero.
    pub initial_code_phase_chips: f64,
    /// Initial carrier phase in radians at sample index zero.
    pub initial_carrier_phase_radians: f64,
    /// Initial carrier frequency in Hz at sample index zero.
    pub initial_carrier_hz: f64,
    /// Linear carrier frequency rate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
    /// Absolute sample index to synthesize.
    pub sample_index: u64,
    /// Data-bit polarity applied to the replica code.
    pub data_bit: i8,
    /// Complex-envelope amplitude.
    pub amplitude: f32,
}

/// Inputs for sampling one modulated replica at an elapsed time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplicaSampleTimeRequest {
    /// Initial code phase in chips at elapsed time zero.
    pub initial_code_phase_chips: f64,
    /// Initial carrier phase in radians at elapsed time zero.
    pub initial_carrier_phase_radians: f64,
    /// Initial carrier frequency in Hz at elapsed time zero.
    pub initial_carrier_hz: f64,
    /// Linear carrier frequency rate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
    /// Elapsed time in seconds.
    pub elapsed_s: f64,
    /// Data-bit polarity applied to the replica code.
    pub data_bit: i8,
    /// Complex-envelope amplitude.
    pub amplitude: f32,
}

/// Inputs for sampling a contiguous modulated replica block.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplicaBlockRequest {
    /// Receiver sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Initial code phase in chips at sample index zero.
    pub initial_code_phase_chips: f64,
    /// Initial carrier phase in radians at sample index zero.
    pub initial_carrier_phase_radians: f64,
    /// Initial carrier frequency in Hz at sample index zero.
    pub initial_carrier_hz: f64,
    /// Linear carrier frequency rate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
    /// Absolute sample index for the first synthesized sample.
    pub start_sample_index: u64,
    /// Data-bit polarity applied to the replica code.
    pub data_bit: i8,
    /// Complex-envelope amplitude.
    pub amplitude: f32,
    /// Number of samples to synthesize.
    pub sample_count: usize,
}

/// Convert C/N0 into the synthesized signal amplitude for a known complex noise power.
pub fn signal_amplitude_from_cn0_db_hz(
    cn0_db_hz: f32,
    sample_rate_hz: f64,
    complex_noise_power: f64,
) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * complex_noise_power.max(1e-12)) / sample_rate_hz.max(1e-12)).sqrt() as f32
}

/// Sample a modulated replica at one absolute sample index.
pub fn sample_modulated_replica_at_sample_index(
    model: &ReplicaCodeModel,
    request: ReplicaSampleIndexRequest,
) -> Result<Complex<f32>, SignalError> {
    let position = code_sample_position_at_index(
        request.initial_code_phase_chips,
        request.sample_rate_hz,
        model.code_rate_hz(),
        model.code_length(),
        request.sample_index,
    )?;
    let signal_value = model.sample_value(
        position.chip_phase,
        position.primary_code_period_index,
        request.data_bit,
    )?;
    let phase = carrier_phase_radians_at_time(
        request.initial_carrier_phase_radians,
        request.initial_carrier_hz,
        request.carrier_rate_hz_per_s,
        position.elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * request.amplitude)
}

/// Sample a modulated replica at one elapsed time.
pub fn sample_modulated_replica_at_time(
    model: &ReplicaCodeModel,
    request: ReplicaSampleTimeRequest,
) -> Result<Complex<f32>, SignalError> {
    let total_chip_phase =
        request.initial_code_phase_chips + model.code_rate_hz() * request.elapsed_s;
    let code_length = model.code_length().max(1) as f64;
    let primary_code_period_index =
        if total_chip_phase <= 0.0 { 0 } else { (total_chip_phase / code_length).floor() as usize };
    let code_phase = total_chip_phase.rem_euclid(code_length);
    let signal_value =
        model.sample_value(code_phase, primary_code_period_index, request.data_bit)?;
    let phase = carrier_phase_radians_at_time(
        request.initial_carrier_phase_radians,
        request.initial_carrier_hz,
        request.carrier_rate_hz_per_s,
        request.elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * request.amplitude)
}
