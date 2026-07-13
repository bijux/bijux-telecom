#![allow(missing_docs)]
#![allow(dead_code)]

use bijux_gnss_core::api::Sample;

use crate::raw_iq::IqQuantization;

/// Scale applied to i16 samples when converting to floating point.
pub const I16_SCALE: f32 = 1.0 / 32768.0;
/// Scale applied to i8 samples when converting to floating point.
pub const I8_SCALE: f32 = 1.0 / 128.0;

/// A time-stamped frame of complex baseband samples.
/// Convert interleaved i8 IQ samples into complex f32 samples.
pub fn iq_i8_to_samples(input: &[i8]) -> Vec<Sample> {
    let mut out = Vec::with_capacity(input.len() / 2);
    for chunk in input.chunks_exact(2) {
        let i = chunk[0] as f32 * I8_SCALE;
        let q = chunk[1] as f32 * I8_SCALE;
        out.push(num_complex::Complex::new(i, q));
    }
    out
}

/// Convert interleaved i16 IQ samples into complex f32 samples.
pub fn iq_i16_to_samples(input: &[i16]) -> Vec<Sample> {
    let mut out = Vec::with_capacity(input.len() / 2);
    for chunk in input.chunks_exact(2) {
        let i = chunk[0] as f32 * I16_SCALE;
        let q = chunk[1] as f32 * I16_SCALE;
        out.push(num_complex::Complex::new(i, q));
    }
    out
}

/// Convert interleaved f32 IQ samples into complex f32 samples without quantization scaling.
pub fn iq_f32_to_samples(input: &[f32]) -> Vec<Sample> {
    let mut out = Vec::with_capacity(input.len() / 2);
    for chunk in input.chunks_exact(2) {
        out.push(num_complex::Complex::new(chunk[0], chunk[1]));
    }
    out
}

/// Encode complex samples into raw-IQ bytes using one controlled quantization profile.
pub fn encode_quantized_samples(samples: &[Sample], quantization: IqQuantization) -> Vec<u8> {
    match quantization {
        IqQuantization::Float32 => encode_cf32_le_bytes(samples),
        IqQuantization::Bipolar1Bit
        | IqQuantization::Signed2Bit
        | IqQuantization::Signed4Bit
        | IqQuantization::Signed8Bit => encode_i8_bytes(samples, quantization),
        IqQuantization::Signed16Bit => encode_i16_le_bytes(samples, quantization),
    }
}

/// Quantize complex samples through one storage profile and return the receiver-visible samples.
pub fn quantize_samples_for_storage(
    samples: &[Sample],
    quantization: IqQuantization,
) -> Vec<Sample> {
    match quantization {
        IqQuantization::Float32 => samples.to_vec(),
        IqQuantization::Bipolar1Bit
        | IqQuantization::Signed2Bit
        | IqQuantization::Signed4Bit
        | IqQuantization::Signed8Bit => {
            let quantized = encode_i8_components(samples, quantization);
            iq_i8_to_samples(&quantized)
        }
        IqQuantization::Signed16Bit => {
            let quantized = encode_i16_components(samples, quantization);
            iq_i16_to_samples(&quantized)
        }
    }
}

fn encode_cf32_le_bytes(samples: &[Sample]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 8);
    for sample in samples {
        encoded.extend_from_slice(&sample.re.to_le_bytes());
        encoded.extend_from_slice(&sample.im.to_le_bytes());
    }
    encoded
}

fn encode_i8_bytes(samples: &[Sample], quantization: IqQuantization) -> Vec<u8> {
    encode_i8_components(samples, quantization)
        .into_iter()
        .map(|component| component as u8)
        .collect()
}

fn encode_i16_le_bytes(samples: &[Sample], quantization: IqQuantization) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 4);
    for component in encode_i16_components(samples, quantization) {
        encoded.extend_from_slice(&component.to_le_bytes());
    }
    encoded
}

fn encode_i8_components(samples: &[Sample], quantization: IqQuantization) -> Vec<i8> {
    let mut encoded = Vec::with_capacity(samples.len() * 2);
    for sample in samples {
        encoded.push(quantize_to_i8(sample.re, quantization));
        encoded.push(quantize_to_i8(sample.im, quantization));
    }
    encoded
}

fn encode_i16_components(samples: &[Sample], quantization: IqQuantization) -> Vec<i16> {
    let mut encoded = Vec::with_capacity(samples.len() * 2);
    for sample in samples {
        encoded.push(quantize_to_i16(sample.re, quantization));
        encoded.push(quantize_to_i16(sample.im, quantization));
    }
    encoded
}

fn quantize_to_i8(value: f32, quantization: IqQuantization) -> i8 {
    let quantized = quantized_level(value, quantization);
    (quantized * 128.0).round().clamp(-128.0, 127.0) as i8
}

fn quantize_to_i16(value: f32, quantization: IqQuantization) -> i16 {
    let quantized = quantized_level(value, quantization);
    (quantized * 32768.0).round().clamp(-32768.0, 32767.0) as i16
}

fn quantized_level(value: f32, quantization: IqQuantization) -> f32 {
    let clamped = if value.is_finite() { value.clamp(-1.0, 1.0) } else { 0.0 };
    match quantization {
        IqQuantization::Float32 => value,
        IqQuantization::Bipolar1Bit => {
            if clamped.is_sign_negative() {
                -1.0
            } else {
                1.0
            }
        }
        IqQuantization::Signed2Bit
        | IqQuantization::Signed4Bit
        | IqQuantization::Signed8Bit
        | IqQuantization::Signed16Bit => {
            let levels = (1usize << quantization.quantization_bits()) - 1;
            let normalized = (clamped + 1.0) * 0.5;
            let index = (normalized * levels as f32).round().clamp(0.0, levels as f32);
            -1.0 + 2.0 * (index / levels as f32)
        }
    }
}
