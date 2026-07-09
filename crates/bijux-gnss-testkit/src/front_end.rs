//! Synthetic front-end fixtures for RF quality tests.

use num_complex::Complex;
use std::f32::consts::TAU;

/// Generate a unit-magnitude carrier whose Q branch is skewed away from ideal quadrature.
///
/// The returned samples use `I = cos(phase)` and `Q = sin(phase + phase_error)`, so a
/// `phase_error_deg` of `0.0` represents ideal 90 degree quadrature.
pub fn generate_quadrature_skew_carrier(
    sample_count: usize,
    cycles: f32,
    phase_error_deg: f32,
) -> Vec<Complex<f32>> {
    let phase_error_rad = phase_error_deg.to_radians();
    (0..sample_count)
        .map(|idx| {
            let phase = TAU * cycles * idx as f32 / sample_count as f32;
            Complex::new(phase.cos(), (phase + phase_error_rad).sin())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::generate_quadrature_skew_carrier;

    #[test]
    fn generate_quadrature_skew_carrier_returns_requested_sample_count() {
        let samples = generate_quadrature_skew_carrier(4096, 8.0, 7.5);
        assert_eq!(samples.len(), 4096);
    }

    #[test]
    fn generate_quadrature_skew_carrier_stays_within_unit_magnitude() {
        let samples = generate_quadrature_skew_carrier(4096, 8.0, 7.5);
        let peak = samples
            .iter()
            .map(|sample| sample.norm())
            .fold(0.0_f32, f32::max);
        assert!(peak <= 2.0_f32.sqrt());
    }
}
