use crate::dsp::sample_timing::code_sample_position_at_index;
use crate::error::SignalError;
use num_complex::Complex;

use super::{carrier_phase_radians_at_time, ReplicaCodeModel};

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
    sample_rate_hz: f64,
    initial_code_phase_chips: f64,
    initial_carrier_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    sample_index: u64,
    data_bit: i8,
    amplitude: f32,
) -> Result<Complex<f32>, SignalError> {
    let position = code_sample_position_at_index(
        initial_code_phase_chips,
        sample_rate_hz,
        model.code_rate_hz(),
        model.code_length(),
        sample_index,
    )?;
    let signal_value =
        model.sample_value(position.chip_phase, position.primary_code_period_index, data_bit)?;
    let phase = carrier_phase_radians_at_time(
        initial_carrier_phase_radians,
        initial_carrier_hz,
        carrier_rate_hz_per_s,
        position.elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * amplitude)
}

/// Sample a modulated replica at one elapsed time.
pub fn sample_modulated_replica_at_time(
    model: &ReplicaCodeModel,
    initial_code_phase_chips: f64,
    initial_carrier_phase_radians: f64,
    initial_carrier_hz: f64,
    carrier_rate_hz_per_s: f64,
    elapsed_s: f64,
    data_bit: i8,
    amplitude: f32,
) -> Result<Complex<f32>, SignalError> {
    let total_chip_phase = initial_code_phase_chips + model.code_rate_hz() * elapsed_s;
    let code_length = model.code_length().max(1) as f64;
    let primary_code_period_index =
        if total_chip_phase <= 0.0 { 0 } else { (total_chip_phase / code_length).floor() as usize };
    let code_phase = total_chip_phase.rem_euclid(code_length);
    let signal_value = model.sample_value(code_phase, primary_code_period_index, data_bit)?;
    let phase = carrier_phase_radians_at_time(
        initial_carrier_phase_radians,
        initial_carrier_hz,
        carrier_rate_hz_per_s,
        elapsed_s,
    ) as f32;
    let carrier = Complex::new(phase.cos(), phase.sin());
    Ok(carrier * signal_value * amplitude)
}
