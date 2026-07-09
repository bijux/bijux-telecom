#![allow(missing_docs)]
use bijux_gnss_signal::api::{generate_ca_code, Prn};

#[test]
fn ca_code_length_is_1023() {
    let code = generate_ca_code(Prn(1)).expect("valid PRN");
    assert_eq!(code.len(), 1023);
}

#[test]
fn ca_code_chips_are_pm_one() {
    let code = generate_ca_code(Prn(1)).expect("valid PRN");
    assert!(code.iter().all(|&c| c == 1 || c == -1));
}

#[test]
fn ca_code_reference_hashes_match_all_prns() {
    for (prn, expected_hash) in EXPECTED_CA_CODE_HASHES {
        let code = generate_ca_code(Prn(prn)).expect("valid PRN");
        let actual_hash = fnv1a_hash(&code);
        assert_eq!(actual_hash, expected_hash, "PRN {prn} hash mismatch");
    }
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

const EXPECTED_CA_CODE_HASHES: [(u8, u32); 32] = [
    (1, 0x7D4AF8CC),
    (2, 0xEE28140C),
    (3, 0x2F9DEFEC),
    (4, 0xB0E956DC),
    (5, 0x2C1E595C),
    (6, 0xAC6129EC),
    (7, 0xC0BEA046),
    (8, 0xA0B95FFE),
    (9, 0xEE465D76),
    (10, 0x9B5FCCA6),
    (11, 0x2F84162E),
    (12, 0xE90A02BE),
    (13, 0x5D3966E6),
    (14, 0x582EAD0E),
    (15, 0x8FEE04C6),
    (16, 0x1195C1AE),
    (17, 0x10957BA6),
    (18, 0xADA1E72E),
    (19, 0x09C011F6),
    (20, 0x94736D9E),
    (21, 0x70CFAD46),
    (22, 0x577EE89E),
    (23, 0x9D2F9EE4),
    (24, 0x15DA5344),
    (25, 0x95048044),
    (26, 0xB8073284),
    (27, 0xF323FEF4),
    (28, 0x961E6764),
    (29, 0x91C617AE),
    (30, 0x10574B56),
    (31, 0x1728E6FE),
    (32, 0x4B308B16),
];
