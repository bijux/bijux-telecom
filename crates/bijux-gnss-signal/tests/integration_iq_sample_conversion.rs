#![allow(missing_docs)]

use bijux_gnss_signal::api::{iq_f32_to_samples, iq_i16_to_samples, iq_i8_to_samples};

#[test]
fn iq_i8_to_samples_normalizes_signed_pairs() {
    let samples = iq_i8_to_samples(&[-128, 127, 64, -64]);

    assert_eq!(samples.len(), 2);
    assert!((samples[0].re + 1.0).abs() < 1e-6);
    assert!((samples[0].im - 0.9921875).abs() < 1e-6);
    assert!((samples[1].re - 0.5).abs() < 1e-6);
    assert!((samples[1].im + 0.5).abs() < 1e-6);
}

#[test]
fn iq_i16_to_samples_normalizes_signed_pairs() {
    let samples = iq_i16_to_samples(&[-32768, 32767, 16384, -16384]);

    assert_eq!(samples.len(), 2);
    assert!((samples[0].re + 1.0).abs() < 1e-6);
    assert!((samples[0].im - 0.9999695).abs() < 1e-6);
    assert!((samples[1].re - 0.5).abs() < 1e-6);
    assert!((samples[1].im + 0.5).abs() < 1e-6);
}

#[test]
fn iq_i8_and_i16_conversions_agree_on_equivalent_levels() {
    let iq8 = iq_i8_to_samples(&[-128, 64, 127, -64]);
    let iq16 = iq_i16_to_samples(&[-32768, 16384, 32512, -16384]);

    assert_eq!(iq8.len(), iq16.len());
    for (left, right) in iq8.iter().zip(iq16.iter()) {
        assert!((left.re - right.re).abs() < 1e-6, "I mismatch: {left:?} vs {right:?}");
        assert!((left.im - right.im).abs() < 1e-6, "Q mismatch: {left:?} vs {right:?}");
    }
}

#[test]
fn iq_f32_to_samples_preserves_complex_pairs() {
    let samples = iq_f32_to_samples(&[-1.0, 0.75, 0.5, -0.25]);

    assert_eq!(samples.len(), 2);
    assert_eq!(samples[0].re, -1.0);
    assert_eq!(samples[0].im, 0.75);
    assert_eq!(samples[1].re, 0.5);
    assert_eq!(samples[1].im, -0.25);
}
