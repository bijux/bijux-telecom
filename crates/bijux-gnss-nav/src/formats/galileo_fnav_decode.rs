#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

const GALILEO_FNAV_PAGE_BITS: usize = 244;
const GALILEO_FNAV_PAYLOAD_BITS: usize = 214;
const GALILEO_FNAV_CRC_BITS: usize = 24;
const GALILEO_FNAV_TAIL_BITS: usize = 6;
const CRC24Q_POLY: u32 = 0x18_64CFB;
const CRC24Q_MASK: u32 = 0xFF_FFFF;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavPage {
    pub page_type: u8,
    pub bits: Vec<u8>,
    pub transmitted_crc: u32,
    pub computed_crc: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GalileoFnavPageRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    NonZeroTail,
    CrcMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavPageRejection {
    pub reason: GalileoFnavPageRejectionReason,
    pub bit_index: Option<usize>,
    pub page_type: Option<u8>,
    pub transmitted_crc: Option<u32>,
    pub computed_crc: Option<u32>,
}

impl GalileoFnavPage {
    pub fn unsigned_bits(&self, start: usize, len: usize) -> u64 {
        unsigned_bits(&self.bits, start, len)
    }

    pub fn signed_bits(&self, start: usize, len: usize) -> i64 {
        signed_from_unsigned(self.unsigned_bits(start, len), len)
    }
}

pub fn decode_galileo_fnav_page(bits: &[u8]) -> Result<GalileoFnavPage, GalileoFnavPageRejection> {
    let normalized = normalize_fnav_bits(bits)?;
    let page_type = unsigned_bits(&normalized, 1, 6) as u8;

    let tail_start = GALILEO_FNAV_PAGE_BITS - GALILEO_FNAV_TAIL_BITS + 1;
    for position in tail_start..=GALILEO_FNAV_PAGE_BITS {
        if normalized[position - 1] != 0 {
            return Err(GalileoFnavPageRejection {
                reason: GalileoFnavPageRejectionReason::NonZeroTail,
                bit_index: Some(position - 1),
                page_type: Some(page_type),
                transmitted_crc: None,
                computed_crc: None,
            });
        }
    }

    let transmitted_crc =
        unsigned_bits(&normalized, GALILEO_FNAV_PAYLOAD_BITS + 1, GALILEO_FNAV_CRC_BITS) as u32;
    let computed_crc = fnav_crc24q(&normalized[..GALILEO_FNAV_PAYLOAD_BITS]);
    if transmitted_crc != computed_crc {
        return Err(GalileoFnavPageRejection {
            reason: GalileoFnavPageRejectionReason::CrcMismatch,
            bit_index: None,
            page_type: Some(page_type),
            transmitted_crc: Some(transmitted_crc),
            computed_crc: Some(computed_crc),
        });
    }

    Ok(GalileoFnavPage { page_type, bits: normalized, transmitted_crc, computed_crc })
}

fn normalize_fnav_bits(bits: &[u8]) -> Result<Vec<u8>, GalileoFnavPageRejection> {
    if bits.len() != GALILEO_FNAV_PAGE_BITS {
        return Err(GalileoFnavPageRejection {
            reason: GalileoFnavPageRejectionReason::InvalidBitCount,
            bit_index: None,
            page_type: None,
            transmitted_crc: None,
            computed_crc: None,
        });
    }

    let mut normalized = Vec::with_capacity(GALILEO_FNAV_PAGE_BITS);
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(GalileoFnavPageRejection {
                reason: GalileoFnavPageRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                page_type: None,
                transmitted_crc: None,
                computed_crc: None,
            });
        }
        normalized.push(bit);
    }
    Ok(normalized)
}

fn fnav_crc24q(bits: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for bit in bits {
        let top = ((crc >> 23) & 1) as u8;
        crc = ((crc << 1) & CRC24Q_MASK) | u32::from(*bit);
        if (top ^ *bit) != 0 {
            crc ^= CRC24Q_POLY & CRC24Q_MASK;
        }
    }
    crc & CRC24Q_MASK
}

fn unsigned_bits(bits: &[u8], start: usize, len: usize) -> u64 {
    let mut value = 0_u64;
    for bit in &bits[start - 1..start - 1 + len] {
        value = (value << 1) | u64::from(*bit);
    }
    value
}

fn signed_from_unsigned(value: u64, bits: usize) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

#[cfg(test)]
mod tests {
    use super::{
        decode_galileo_fnav_page, fnav_crc24q, GalileoFnavPageRejectionReason,
        GALILEO_FNAV_CRC_BITS, GALILEO_FNAV_PAGE_BITS, GALILEO_FNAV_PAYLOAD_BITS,
        GALILEO_FNAV_TAIL_BITS,
    };

    fn set_bits(bits: &mut [u8], start: usize, len: usize, value: u64) {
        for offset in 0..len {
            let shift = len - offset - 1;
            bits[start + offset - 1] = ((value >> shift) & 1) as u8;
        }
    }

    fn apply_crc(bits: &mut [u8]) {
        let crc = fnav_crc24q(&bits[..GALILEO_FNAV_PAYLOAD_BITS]);
        set_bits(bits, GALILEO_FNAV_PAYLOAD_BITS + 1, GALILEO_FNAV_CRC_BITS, u64::from(crc));
    }

    fn valid_page_bits(page_type: u8) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, u64::from(page_type));
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 14, 2_345);
        apply_crc(&mut bits);
        bits
    }

    #[test]
    fn fnav_page_decodes_type_and_crc() {
        let bits = valid_page_bits(1);

        let page = decode_galileo_fnav_page(&bits).expect("valid F/NAV page");

        assert_eq!(page.page_type, 1);
        assert_eq!(page.transmitted_crc, page.computed_crc);
        assert_eq!(page.unsigned_bits(7, 10), 0x155);
    }

    #[test]
    fn fnav_page_rejects_corrupted_payload() {
        let mut bits = valid_page_bits(2);
        bits[30] ^= 1;

        let rejection = decode_galileo_fnav_page(&bits).expect_err("CRC rejection");

        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::CrcMismatch);
        assert_eq!(rejection.page_type, Some(2));
        assert_ne!(rejection.transmitted_crc, rejection.computed_crc);
    }

    #[test]
    fn fnav_page_rejects_invalid_shape_before_crc() {
        let short_bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS - 1];
        let rejection = decode_galileo_fnav_page(&short_bits).expect_err("bit count rejection");
        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::InvalidBitCount);

        let mut non_binary = valid_page_bits(1);
        non_binary[10] = 2;
        let rejection = decode_galileo_fnav_page(&non_binary).expect_err("binary rejection");
        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::NonBinaryBit);
        assert_eq!(rejection.bit_index, Some(10));
    }

    #[test]
    fn fnav_page_rejects_nonzero_tail_bits() {
        let mut bits = valid_page_bits(1);
        bits[GALILEO_FNAV_PAGE_BITS - 1] = 1;

        let rejection = decode_galileo_fnav_page(&bits).expect_err("tail rejection");

        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::NonZeroTail);
        assert_eq!(rejection.bit_index, Some(GALILEO_FNAV_PAGE_BITS - 1));
    }

    #[test]
    fn fnav_page_tail_boundary_matches_layout() {
        assert_eq!(
            GALILEO_FNAV_PAYLOAD_BITS + GALILEO_FNAV_CRC_BITS + GALILEO_FNAV_TAIL_BITS,
            GALILEO_FNAV_PAGE_BITS
        );
    }
}
