#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    encode_quantized_samples, iq_f32_to_samples, iq_i16_to_samples, iq_i8_to_samples,
    quantize_samples_for_storage, IqQuantization, IqSampleFormat,
};

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

#[test]
fn storage_quantization_profiles_report_expected_metadata() {
    assert_eq!(IqQuantization::Float32.sample_format(), IqSampleFormat::Cf32Le);
    assert_eq!(IqQuantization::Float32.quantization_bits(), 32);
    assert_eq!(IqQuantization::Float32.identifier(), "float32");
    assert_eq!(IqQuantization::Float32.to_string(), "float32");
    assert_eq!(IqQuantization::Bipolar1Bit.sample_format(), IqSampleFormat::Iq8);
    assert_eq!(IqQuantization::Bipolar1Bit.quantization_bits(), 1);
    assert_eq!(IqQuantization::Bipolar1Bit.identifier(), "bipolar_1bit");
    assert_eq!(IqQuantization::Signed4Bit.sample_format(), IqSampleFormat::Iq8);
    assert_eq!(IqQuantization::Signed4Bit.quantization_bits(), 4);
    assert_eq!(IqQuantization::Signed4Bit.identifier(), "signed_4bit");
    assert_eq!(IqQuantization::Signed16Bit.sample_format(), IqSampleFormat::Iq16Le);
    assert_eq!(IqQuantization::Signed16Bit.quantization_bits(), 16);
    assert_eq!(IqQuantization::Signed16Bit.identifier(), "signed_16bit");
}

#[test]
fn bipolar_sign_quantization_drives_components_to_storage_rails() {
    let samples = quantize_samples_for_storage(
        &[num_complex::Complex::new(-0.9, -0.1), num_complex::Complex::new(0.0, 0.8)],
        IqQuantization::Bipolar1Bit,
    );

    assert_eq!(samples.len(), 2);
    assert!((samples[0].re + 1.0).abs() < 1e-6);
    assert!((samples[0].im + 1.0).abs() < 1e-6);
    assert!((samples[1].re - 0.9921875).abs() < 1e-6);
    assert!((samples[1].im - 0.9921875).abs() < 1e-6);
}

#[test]
fn lower_bit_quantization_uses_coarser_receiver_visible_levels() {
    let source = [num_complex::Complex::new(0.2, -0.2)];
    let signed_2 = quantize_samples_for_storage(&source, IqQuantization::Signed2Bit);
    let signed_4 = quantize_samples_for_storage(&source, IqQuantization::Signed4Bit);
    let signed_8 = quantize_samples_for_storage(&source, IqQuantization::Signed8Bit);

    assert!(signed_2[0].re > signed_4[0].re, "{signed_2:?} vs {signed_4:?}");
    assert!(signed_4[0].re > 0.0, "{signed_4:?}");
    assert!(signed_8[0].re > 0.0, "{signed_8:?}");
    assert!((signed_2[0].re - source[0].re).abs() > (signed_4[0].re - source[0].re).abs());
    assert!((signed_4[0].re - source[0].re).abs() >= (signed_8[0].re - source[0].re).abs());
}

#[test]
fn quantized_storage_encoding_uses_declared_container_width() {
    let samples = [num_complex::Complex::new(-0.75, 0.5), num_complex::Complex::new(0.25, -0.125)];

    assert_eq!(encode_quantized_samples(&samples, IqQuantization::Bipolar1Bit).len(), 4);
    assert_eq!(encode_quantized_samples(&samples, IqQuantization::Signed4Bit).len(), 4);
    assert_eq!(encode_quantized_samples(&samples, IqQuantization::Signed16Bit).len(), 8);
    assert_eq!(encode_quantized_samples(&samples, IqQuantization::Float32).len(), 16);
}
