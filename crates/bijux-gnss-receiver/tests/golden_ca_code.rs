#![allow(missing_docs)]
use bijux_gnss_signal::{generate_ca_code, Prn};

#[test]
fn ca_code_length_is_1023() {
    let code = generate_ca_code(Prn(1));
    assert_eq!(code.len(), 1023);
}

#[test]
fn ca_code_chips_are_pm_one() {
    let code = generate_ca_code(Prn(1));
    assert!(code.iter().all(|&c| c == 1 || c == -1));
}

#[test]
fn ca_code_reference_hashes_match() {
    let prn1 = generate_ca_code(Prn(1));
    let prn2 = generate_ca_code(Prn(2));

    let h1 = fnv1a_hash(&prn1);
    let h2 = fnv1a_hash(&prn2);

    assert_eq!(h1, 0x7D4AF8CC, "PRN1 hash mismatch");
    assert_eq!(h2, 0xEE28140C, "PRN2 hash mismatch");
}

fn fnv1a_hash(code: &[i8]) -> u32 {
    let mut hash: u32 = 0x811C9DC5;
    for &chip in code {
        let byte = if chip > 0 { 1u8 } else { 0u8 };
        hash ^= byte as u32;
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash
}
