#![allow(missing_docs)]
#![allow(dead_code)]

use bijux_gnss_core::api::Sample;

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
