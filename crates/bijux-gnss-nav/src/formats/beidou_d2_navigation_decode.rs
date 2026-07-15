#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

const BEIDOU_D2_PAGE_BITS: usize = 300;
const BEIDOU_D2_PREAMBLE: u16 = 0b111_0001_0010;
const BEIDOU_D2_SUBFRAME_ID: u8 = 1;
const BCH_GENERATOR: u16 = 0b1_0011;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2ParitySummary {
    pub corrected_blocks: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2Page {
    pub page_number: u8,
    pub sow_s: u32,
    pub bits: Vec<u8>,
    pub parity: BeidouD2ParitySummary,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BeidouD2PageRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    InvalidPreamble,
    InvalidSubframeId,
    InvalidPageNumber,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2PageRejection {
    pub reason: BeidouD2PageRejectionReason,
    pub bit_index: Option<usize>,
    pub subframe_id: Option<u8>,
    pub page_number: Option<u8>,
}

impl BeidouD2Page {
    pub fn unsigned_bits(&self, start: usize, len: usize) -> u64 {
        unsigned_bits(&self.bits, start, len)
    }

    pub fn signed_bits(&self, start: usize, len: usize) -> i64 {
        signed_bits(&self.bits, start, len)
    }

    pub fn concat_unsigned_bits(&self, parts: &[(usize, usize)]) -> u64 {
        concat_unsigned_bits(&self.bits, parts)
    }

    pub fn concat_signed_bits(&self, parts: &[(usize, usize)]) -> i64 {
        concat_signed_bits(&self.bits, parts)
    }
}

pub fn decode_beidou_d2_page(bits: &[u8]) -> Result<BeidouD2Page, BeidouD2PageRejection> {
    let normalized = normalize_bits(bits)?;
    let corrected = correct_bch_blocks(normalized);
    validate_header(&corrected.bits)?;
    let page_number = unsigned_bits(&corrected.bits, 43, 4) as u8;
    if !(1..=10).contains(&page_number) {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidPageNumber,
            bit_index: None,
            subframe_id: Some(subframe_id(&corrected.bits)),
            page_number: Some(page_number),
        });
    }
    Ok(BeidouD2Page {
        page_number,
        sow_s: sow_s(&corrected.bits),
        bits: corrected.bits.to_vec(),
        parity: BeidouD2ParitySummary { corrected_blocks: corrected.corrected_blocks },
    })
}

fn normalize_bits(bits: &[u8]) -> Result<[u8; BEIDOU_D2_PAGE_BITS], BeidouD2PageRejection> {
    if bits.len() != BEIDOU_D2_PAGE_BITS {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidBitCount,
            bit_index: None,
            subframe_id: None,
            page_number: None,
        });
    }

    let mut normalized = [0_u8; BEIDOU_D2_PAGE_BITS];
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(BeidouD2PageRejection {
                reason: BeidouD2PageRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                subframe_id: None,
                page_number: None,
            });
        }
        normalized[idx] = bit;
    }
    Ok(normalized)
}

#[derive(Debug, Clone, Copy)]
struct CorrectedD2Bits {
    bits: [u8; BEIDOU_D2_PAGE_BITS],
    corrected_blocks: u8,
}

fn correct_bch_blocks(mut bits: [u8; BEIDOU_D2_PAGE_BITS]) -> CorrectedD2Bits {
    let mut corrected_blocks = 0_u8;
    if correct_bch_codeword(&mut bits, 16).is_some() {
        corrected_blocks += 1;
    }
    for word_index in 1..10 {
        let start = word_index * 30 + 1;
        if correct_bch_split_codeword(&mut bits, start, start + 22).is_some() {
            corrected_blocks += 1;
        }
        if correct_bch_split_codeword(&mut bits, start + 11, start + 26).is_some() {
            corrected_blocks += 1;
        }
    }
    CorrectedD2Bits { bits, corrected_blocks }
}

fn correct_bch_codeword(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize) -> Option<usize> {
    let mut codeword = unsigned_bits(bits, start, 15) as u16;
    let syndrome = bch_syndrome(codeword);
    if syndrome == 0 {
        return None;
    }
    for offset in 0..15 {
        let mask = 1_u16 << (14 - offset);
        if bch_syndrome(codeword ^ mask) == 0 {
            codeword ^= mask;
            set_unsigned_bits(bits, start, 15, u64::from(codeword));
            return Some(start + offset);
        }
    }
    None
}

fn correct_bch_split_codeword(
    bits: &mut [u8; BEIDOU_D2_PAGE_BITS],
    info_start: usize,
    parity_start: usize,
) -> Option<usize> {
    let mut codeword = ((unsigned_bits(bits, info_start, 11) as u16) << 4)
        | unsigned_bits(bits, parity_start, 4) as u16;
    let syndrome = bch_syndrome(codeword);
    if syndrome == 0 {
        return None;
    }
    for offset in 0..15 {
        let mask = 1_u16 << (14 - offset);
        if bch_syndrome(codeword ^ mask) == 0 {
            codeword ^= mask;
            set_unsigned_bits(bits, info_start, 11, u64::from((codeword >> 4) & 0x07FF));
            set_unsigned_bits(bits, parity_start, 4, u64::from(codeword & 0x000F));
            return Some(if offset < 11 {
                info_start + offset
            } else {
                parity_start + offset - 11
            });
        }
    }
    None
}

fn bch_syndrome(codeword: u16) -> u16 {
    let mut value = codeword;
    for degree in (4..=14).rev() {
        if (value & (1_u16 << degree)) != 0 {
            value ^= BCH_GENERATOR << (degree - 4);
        }
    }
    value & 0x0F
}

fn validate_header(bits: &[u8; BEIDOU_D2_PAGE_BITS]) -> Result<(), BeidouD2PageRejection> {
    if unsigned_bits(bits, 1, 11) as u16 != BEIDOU_D2_PREAMBLE {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidPreamble,
            bit_index: Some(0),
            subframe_id: Some(subframe_id(bits)),
            page_number: Some(unsigned_bits(bits, 43, 4) as u8),
        });
    }
    let actual_subframe_id = subframe_id(bits);
    if actual_subframe_id != BEIDOU_D2_SUBFRAME_ID {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidSubframeId,
            bit_index: None,
            subframe_id: Some(actual_subframe_id),
            page_number: Some(unsigned_bits(bits, 43, 4) as u8),
        });
    }
    Ok(())
}

fn subframe_id(bits: &[u8]) -> u8 {
    unsigned_bits(bits, 16, 3) as u8
}

fn sow_s(bits: &[u8]) -> u32 {
    concat_unsigned_bits(bits, &[(19, 8), (31, 12)]) as u32
}

fn bit(bits: &[u8], position: usize) -> u8 {
    bits[position - 1]
}

fn unsigned_bits(bits: &[u8], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0_u64, |acc, position| (acc << 1) | u64::from(bit(bits, position)))
}

fn signed_bits(bits: &[u8], start: usize, len: usize) -> i64 {
    signed_from_unsigned(unsigned_bits(bits, start, len), len)
}

fn concat_unsigned_bits(bits: &[u8], parts: &[(usize, usize)]) -> u64 {
    parts.iter().fold(0_u64, |acc, (start, len)| (acc << len) | unsigned_bits(bits, *start, *len))
}

fn concat_signed_bits(bits: &[u8], parts: &[(usize, usize)]) -> i64 {
    let total_bits = parts.iter().map(|(_, len)| *len).sum();
    signed_from_unsigned(concat_unsigned_bits(bits, parts), total_bits)
}

fn signed_from_unsigned(value: u64, bits: usize) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

fn set_unsigned_bits(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize, len: usize, value: u64) {
    for offset in 0..len {
        let shift = len - offset - 1;
        bits[start + offset - 1] = ((value >> shift) & 1) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        bch_syndrome, decode_beidou_d2_page, set_unsigned_bits, BeidouD2PageRejectionReason,
        BEIDOU_D2_PAGE_BITS, BEIDOU_D2_PREAMBLE,
    };

    fn set_common_header(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], page_number: u8, sow_s: u32) {
        set_unsigned_bits(bits, 1, 11, u64::from(BEIDOU_D2_PREAMBLE));
        set_unsigned_bits(bits, 16, 3, 1);
        set_unsigned_bits(bits, 19, 8, u64::from(sow_s >> 12));
        set_unsigned_bits(bits, 31, 12, u64::from(sow_s & 0x0FFF));
        set_unsigned_bits(bits, 43, 4, u64::from(page_number));
    }

    fn apply_bch_parity(bits: &mut [u8; BEIDOU_D2_PAGE_BITS]) {
        set_codeword_parity(bits, 16);
        for word_index in 1..10 {
            let start = word_index * 30 + 1;
            set_split_codeword_parity(bits, start, start + 22);
            set_split_codeword_parity(bits, start + 11, start + 26);
        }
    }

    fn set_codeword_parity(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize) {
        set_unsigned_bits(bits, start + 11, 4, 0);
        let codeword_without_parity = super::unsigned_bits(bits, start, 15) as u16;
        let parity = bch_syndrome(codeword_without_parity);
        set_unsigned_bits(bits, start + 11, 4, u64::from(parity));
    }

    fn set_split_codeword_parity(
        bits: &mut [u8; BEIDOU_D2_PAGE_BITS],
        info_start: usize,
        parity_start: usize,
    ) {
        set_unsigned_bits(bits, parity_start, 4, 0);
        let codeword_without_parity = ((super::unsigned_bits(bits, info_start, 11) as u16) << 4)
            | super::unsigned_bits(bits, parity_start, 4) as u16;
        let parity = bch_syndrome(codeword_without_parity);
        set_unsigned_bits(bits, parity_start, 4, u64::from(parity));
    }

    fn valid_page_bits(page_number: u8) -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, page_number, 345_678);
        set_unsigned_bits(&mut bits, 47, 6, 0x15);
        set_unsigned_bits(&mut bits, 61, 11, 0x321);
        apply_bch_parity(&mut bits);
        bits
    }

    #[test]
    fn d2_page_decodes_header_page_number_and_sow() {
        let page = decode_beidou_d2_page(&valid_page_bits(4)).expect("valid D2 page");

        assert_eq!(page.page_number, 4);
        assert_eq!(page.sow_s, 345_678);
        assert_eq!(page.parity.corrected_blocks, 0);
        assert_eq!(page.unsigned_bits(47, 6), 0x15);
        assert_eq!(page.unsigned_bits(61, 11), 0x321);
    }

    #[test]
    fn d2_page_corrects_single_bch_bit_error() {
        let mut bits = valid_page_bits(2);
        bits[60] ^= 1;

        let page = decode_beidou_d2_page(&bits).expect("correctable D2 page");

        assert_eq!(page.page_number, 2);
        assert_eq!(page.parity.corrected_blocks, 1);
        assert_eq!(page.unsigned_bits(61, 11), 0x321);
    }

    #[test]
    fn d2_page_rejects_invalid_shape_and_header() {
        let short = vec![0_u8; BEIDOU_D2_PAGE_BITS - 1];
        let rejection = decode_beidou_d2_page(&short).expect_err("bit count rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidBitCount);

        let mut non_binary = valid_page_bits(1);
        non_binary[42] = 2;
        let rejection = decode_beidou_d2_page(&non_binary).expect_err("binary rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::NonBinaryBit);
        assert_eq!(rejection.bit_index, Some(42));

        let mut bad_subframe = valid_page_bits(1);
        set_unsigned_bits(&mut bad_subframe, 16, 3, 2);
        apply_bch_parity(&mut bad_subframe);
        let rejection = decode_beidou_d2_page(&bad_subframe).expect_err("subframe rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidSubframeId);
    }

    #[test]
    fn d2_page_rejects_page_numbers_outside_subcommutated_frame() {
        let mut bits = valid_page_bits(1);
        set_unsigned_bits(&mut bits, 43, 4, 0);
        apply_bch_parity(&mut bits);

        let rejection = decode_beidou_d2_page(&bits).expect_err("page rejection");

        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidPageNumber);
        assert_eq!(rejection.page_number, Some(0));
    }
}
