#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

const GLONASS_STRING_BITS: usize = 85;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassStringParitySummary {
    pub corrected_bit_index: Option<u8>,
    pub syndrome: u8,
    pub overall_parity_error: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassNavigationString {
    pub string_number: u8,
    pub bits: Vec<u8>,
    pub parity: GlonassStringParitySummary,
}

impl GlonassNavigationString {
    pub fn bit(&self, position: usize) -> u8 {
        self.bits[GLONASS_STRING_BITS - position]
    }

    pub fn unsigned_bits(&self, low: usize, high: usize) -> u64 {
        let mut value = 0_u64;
        for position in (low..=high).rev() {
            value = (value << 1) | u64::from(self.bit(position));
        }
        value
    }

    pub fn sign_magnitude(&self, low: usize, high: usize, lsb: f64) -> f64 {
        let raw = self.unsigned_bits(low, high);
        let width = high - low + 1;
        let sign_mask = 1_u64 << (width - 1);
        let magnitude_mask = sign_mask - 1;
        let magnitude = (raw & magnitude_mask) as f64 * lsb;
        if raw & sign_mask == 0 { magnitude } else { -magnitude }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GlonassNavigationStringRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    UnrecoverableParity,
    InvalidStringNumber,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassNavigationStringRejection {
    pub reason: GlonassNavigationStringRejectionReason,
    pub bit_index: Option<usize>,
    pub string_number: Option<u8>,
}

pub fn decode_glonass_navigation_string(
    bits: &[u8],
) -> Result<GlonassNavigationString, GlonassNavigationStringRejection> {
    let mut normalized = normalize_bits(bits)?;
    let (syndrome, overall_parity_error) = parity_state(&normalized);
    let mut corrected_bit_index = None;

    match (syndrome, overall_parity_error) {
        (0, false) => {}
        (0, true) => {
            flip_bit(&mut normalized, 8);
            corrected_bit_index = Some(8);
        }
        (syndrome, true) => {
            let corrected = correction_index(syndrome);
            if corrected == 0 || corrected > GLONASS_STRING_BITS {
                return Err(GlonassNavigationStringRejection {
                    reason: GlonassNavigationStringRejectionReason::UnrecoverableParity,
                    bit_index: None,
                    string_number: None,
                });
            }
            flip_bit(&mut normalized, corrected);
            corrected_bit_index = Some(corrected as u8);
        }
        (_, false) => {
            return Err(GlonassNavigationStringRejection {
                reason: GlonassNavigationStringRejectionReason::UnrecoverableParity,
                bit_index: None,
                string_number: None,
            });
        }
    }

    let decoded = GlonassNavigationString {
        string_number: decode_string_number(&normalized),
        bits: normalized.to_vec(),
        parity: GlonassStringParitySummary { corrected_bit_index, syndrome, overall_parity_error },
    };
    if !(1..=15).contains(&decoded.string_number) {
        return Err(GlonassNavigationStringRejection {
            reason: GlonassNavigationStringRejectionReason::InvalidStringNumber,
            bit_index: None,
            string_number: Some(decoded.string_number),
        });
    }
    Ok(decoded)
}

fn normalize_bits(
    bits: &[u8],
) -> Result<[u8; GLONASS_STRING_BITS], GlonassNavigationStringRejection> {
    if bits.len() != GLONASS_STRING_BITS {
        return Err(GlonassNavigationStringRejection {
            reason: GlonassNavigationStringRejectionReason::InvalidBitCount,
            bit_index: None,
            string_number: None,
        });
    }
    let mut normalized = [0_u8; GLONASS_STRING_BITS];
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(GlonassNavigationStringRejection {
                reason: GlonassNavigationStringRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                string_number: None,
            });
        }
        normalized[idx] = bit;
    }
    Ok(normalized)
}

fn parity_state(bits: &[u8; GLONASS_STRING_BITS]) -> (u8, bool) {
    let checksums = [
        checksum_c1(bits),
        checksum_c2(bits),
        checksum_c3(bits),
        checksum_c4(bits),
        checksum_c5(bits),
        checksum_c6(bits),
        checksum_c7(bits),
    ];
    let syndrome = checksums
        .iter()
        .enumerate()
        .fold(0_u8, |acc, (idx, value)| acc | (*value << idx));
    (syndrome, checksum_overall(bits) == 1)
}

fn checksum_c1(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 1) ^ xor_positions(
        bits,
        &[
            9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45,
            47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
        ],
    )
}

fn checksum_c2(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 2) ^ xor_positions(
        bits,
        &[
            9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45,
            48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84,
        ],
    )
}

fn checksum_c3(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 3) ^ xor_ranges(bits, &[(10, 12), (16, 19), (23, 26), (31, 34), (38, 41), (46, 49), (54, 57), (62, 65), (69, 72), (77, 80), (85, 85)])
}

fn checksum_c4(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 4) ^ xor_ranges(bits, &[(13, 19), (27, 34), (42, 49), (58, 65), (73, 80)])
}

fn checksum_c5(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 5) ^ xor_ranges(bits, &[(20, 34), (50, 65), (81, 85)])
}

fn checksum_c6(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 6) ^ xor_range(bits, 35, 85)
}

fn checksum_c7(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 7) ^ xor_range(bits, 66, 85)
}

fn checksum_overall(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    xor_range(bits, 1, 85)
}

fn correction_index(syndrome: u8) -> usize {
    let highest_set = 8 - syndrome.leading_zeros() as usize;
    usize::from(syndrome) + 8 - highest_set
}

fn decode_string_number(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    unsigned_bits(bits, 81, 84) as u8
}

fn bit(bits: &[u8; GLONASS_STRING_BITS], position: usize) -> u8 {
    bits[GLONASS_STRING_BITS - position]
}

fn flip_bit(bits: &mut [u8; GLONASS_STRING_BITS], position: usize) {
    let idx = GLONASS_STRING_BITS - position;
    bits[idx] ^= 1;
}

fn xor_positions(bits: &[u8; GLONASS_STRING_BITS], positions: &[usize]) -> u8 {
    positions.iter().fold(0_u8, |acc, position| acc ^ bit(bits, *position))
}

fn xor_range(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u8 {
    (low..=high).fold(0_u8, |acc, position| acc ^ bit(bits, position))
}

fn xor_ranges(bits: &[u8; GLONASS_STRING_BITS], ranges: &[(usize, usize)]) -> u8 {
    ranges.iter().fold(0_u8, |acc, (low, high)| acc ^ xor_range(bits, *low, *high))
}

fn unsigned_bits(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u64 {
    let mut value = 0_u64;
    for position in (low..=high).rev() {
        value = (value << 1) | u64::from(bit(bits, position));
    }
    value
}

#[cfg(test)]
mod tests {
    use super::{
        checksum_c1, checksum_c2, checksum_c3, checksum_c4, checksum_c5, checksum_c6,
        checksum_c7, decode_glonass_navigation_string, flip_bit, GlonassNavigationStringRejectionReason,
        GLONASS_STRING_BITS,
    };

    #[test]
    fn navigation_string_decodes_string_number_and_payload_bits() {
        let bits = encoded_string(3, &[(65, 1), (64, 1), (20, 1)]);
        let decoded = decode_glonass_navigation_string(&bits).expect("decoded string");

        assert_eq!(decoded.string_number, 3);
        assert_eq!(decoded.bit(65), 1);
        assert_eq!(decoded.bit(64), 1);
        assert_eq!(decoded.bit(20), 1);
        assert_eq!(decoded.parity.corrected_bit_index, None);
    }

    #[test]
    fn navigation_string_corrects_single_data_bit_error() {
        let mut bits = encoded_string(7, &[(65, 1), (30, 1), (22, 1)]);
        flip_bit(&mut bits, 30);

        let decoded = decode_glonass_navigation_string(&bits).expect("single-bit correction");

        assert_eq!(decoded.string_number, 7);
        assert_eq!(decoded.parity.corrected_bit_index, Some(30));
        assert_eq!(decoded.bit(30), 1);
    }

    #[test]
    fn navigation_string_extracts_unsigned_and_signed_fields() {
        let mut bits = encoded_string(4, &[]);
        set_unsigned_bits(&mut bits, 9, 13, 0b1_0110);
        set_sign_magnitude_bits(&mut bits, 20, 24, -6);
        apply_check_bits(&mut bits);

        let decoded = decode_glonass_navigation_string(&bits).expect("decoded field string");

        assert_eq!(decoded.unsigned_bits(9, 13), 0b1_0110);
        assert_eq!(decoded.sign_magnitude(20, 24, 0.5), -3.0);
    }

    #[test]
    fn navigation_string_rejects_multiple_bit_errors() {
        let mut bits = encoded_string(12, &[(65, 1), (55, 1)]);
        flip_bit(&mut bits, 30);
        flip_bit(&mut bits, 31);

        let rejection = decode_glonass_navigation_string(&bits).expect_err("multiple-bit rejection");

        assert_eq!(rejection.reason, GlonassNavigationStringRejectionReason::UnrecoverableParity);
    }

    fn encoded_string(string_number: u8, ones: &[(usize, u8)]) -> [u8; GLONASS_STRING_BITS] {
        let mut bits = [0_u8; GLONASS_STRING_BITS];
        set_unsigned_bits(&mut bits, 81, 84, u64::from(string_number));
        for (position, value) in ones {
            set_bit(&mut bits, *position, *value);
        }
        apply_check_bits(&mut bits);
        bits
    }

    fn apply_check_bits(bits: &mut [u8; GLONASS_STRING_BITS]) {
        for position in 1..=8 {
            set_bit(bits, position, 0);
        }
        set_bit(bits, 1, checksum_c1(bits));
        set_bit(bits, 2, checksum_c2(bits));
        set_bit(bits, 3, checksum_c3(bits));
        set_bit(bits, 4, checksum_c4(bits));
        set_bit(bits, 5, checksum_c5(bits));
        set_bit(bits, 6, checksum_c6(bits));
        set_bit(bits, 7, checksum_c7(bits));
        set_bit(bits, 8, xor_range(bits, 1, 85));
    }

    fn set_unsigned_bits(bits: &mut [u8; GLONASS_STRING_BITS], low: usize, high: usize, value: u64) {
        for (offset, position) in (low..=high).enumerate() {
            let bit_value = ((value >> offset) & 1) as u8;
            set_bit(bits, position, bit_value);
        }
    }

    fn set_sign_magnitude_bits(
        bits: &mut [u8; GLONASS_STRING_BITS],
        low: usize,
        high: usize,
        signed_value: i64,
    ) {
        let width = high - low + 1;
        let magnitude_mask = (1_u64 << (width - 1)) - 1;
        let sign_bit = u64::from(signed_value.is_negative());
        let magnitude = signed_value.unsigned_abs() & magnitude_mask;
        set_unsigned_bits(bits, low, high, magnitude | (sign_bit << (width - 1)));
    }

    fn set_bit(bits: &mut [u8; GLONASS_STRING_BITS], position: usize, value: u8) {
        bits[GLONASS_STRING_BITS - position] = value;
    }

    fn xor_range(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u8 {
        (low..=high).fold(0_u8, |acc, position| acc ^ bits[GLONASS_STRING_BITS - position])
    }
}
