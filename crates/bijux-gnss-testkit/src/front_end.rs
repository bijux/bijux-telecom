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

/// Generate interleaved signed 8-bit I/Q bytes with a controlled clipped-sample count.
///
/// The first `clipped_sample_count` complex samples saturate both branches at the quantized rails.
/// Remaining samples use a fixed non-clipped pattern.
pub fn generate_clipped_iq8_bytes(sample_count: usize, clipped_sample_count: usize) -> Vec<u8> {
    assert!(clipped_sample_count <= sample_count);

    let mut bytes = Vec::with_capacity(sample_count * 2);
    for idx in 0..sample_count {
        let (i, q) = if idx < clipped_sample_count {
            (i8::MIN, i8::MAX)
        } else {
            (32_i8, -16_i8)
        };
        bytes.push(i as u8);
        bytes.push(q as u8);
    }
    bytes
}

/// Generate interleaved signed 16-bit little-endian I/Q bytes with a controlled clipped-sample count.
///
/// The first `clipped_sample_count` complex samples saturate both branches at the quantized rails.
/// Remaining samples use a fixed non-clipped pattern.
pub fn generate_clipped_iq16_le_bytes(
    sample_count: usize,
    clipped_sample_count: usize,
) -> Vec<u8> {
    assert!(clipped_sample_count <= sample_count);

    let mut bytes = Vec::with_capacity(sample_count * 4);
    for idx in 0..sample_count {
        let (i, q) = if idx < clipped_sample_count {
            (i16::MIN, i16::MAX)
        } else {
            (8_192_i16, -4_096_i16)
        };
        bytes.extend_from_slice(&i.to_le_bytes());
        bytes.extend_from_slice(&q.to_le_bytes());
    }
    bytes
}

#[cfg(test)]
mod tests {
    use super::{
        generate_clipped_iq16_le_bytes, generate_clipped_iq8_bytes,
        generate_quadrature_skew_carrier,
    };

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

    #[test]
    fn generate_clipped_iq8_bytes_marks_requested_prefix_as_saturated() {
        let bytes = generate_clipped_iq8_bytes(4, 2);
        assert_eq!(bytes, vec![0x80, 0x7f, 0x80, 0x7f, 32_u8, 240_u8, 32_u8, 240_u8]);
    }

    #[test]
    fn generate_clipped_iq16_bytes_marks_requested_prefix_as_saturated() {
        let bytes = generate_clipped_iq16_le_bytes(3, 1);
        assert_eq!(
            bytes,
            vec![
                0x00, 0x80, 0xff, 0x7f, 0x00, 0x20, 0x00, 0xf0, 0x00, 0x20, 0x00, 0xf0,
            ]
        );
    }
}
