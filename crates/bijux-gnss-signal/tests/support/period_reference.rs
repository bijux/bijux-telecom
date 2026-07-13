#![allow(dead_code, missing_docs)]

use sha2::{Digest, Sha256};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceWindow {
    pub start: usize,
    pub bits: String,
}

pub fn logical_bits_from_bipolar(code: &[i8], label: &str) -> String {
    code.iter()
        .map(|chip| match chip {
            1 => '0',
            -1 => '1',
            _ => panic!("{label} must be bipolar, found {chip}"),
        })
        .collect()
}

pub fn assert_code_period_reference(
    logical_bits: &str,
    expected_length: usize,
    expected_hash: &str,
    windows: &[ReferenceWindow],
    label: &str,
) {
    assert_eq!(logical_bits.len(), expected_length, "{label} length mismatch");
    assert_eq!(sha256_hex(logical_bits), expected_hash, "{label} fingerprint mismatch");
    for window in windows {
        assert_eq!(
            wrapped_window_bits(logical_bits, window.start, window.bits.len()),
            window.bits,
            "{label} window mismatch at chip {}",
            window.start
        );
    }
}

pub fn assert_period_repetition(code: &[i8], period_length: usize, label: &str) {
    let repeated = code.iter().copied().cycle().take(period_length * 2).collect::<Vec<_>>();
    let (first_period, second_period) = repeated.split_at(period_length);
    assert_eq!(first_period, second_period, "{label} period repetition mismatch");
}

pub fn wrapped_window_bits(logical_bits: &str, start: usize, length: usize) -> String {
    if start + length <= logical_bits.len() {
        return logical_bits[start..start + length].to_owned();
    }
    let tail = &logical_bits[start..];
    let head = &logical_bits[..length - tail.len()];
    format!("{tail}{head}")
}

pub fn sha256_hex(payload: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(payload.as_bytes());
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}
