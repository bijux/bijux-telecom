#![allow(missing_docs)]

//! Signal processing utilities.

/// Compute the number of samples per C/A code period (1 ms) given sampling
/// frequency and code parameters.
pub fn samples_per_code(
    sampling_freq_hz: f64,
    code_freq_basis_hz: f64,
    code_length: usize,
) -> usize {
    (sampling_freq_hz / (code_freq_basis_hz / code_length as f64)).round() as usize
}
