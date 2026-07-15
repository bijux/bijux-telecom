#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

const GPS_CNAV_BITS: usize = 300;
const GPS_CNAV_DATA_BITS: usize = 276;
const GPS_CNAV_CRC_BITS: usize = 24;
const GPS_CNAV_PREAMBLE: u8 = 0b1000_1011;
const CRC24Q_POLY: u32 = 0x18_64CFB;
const CRC24Q_MASK: u32 = 0xFF_FFFF;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsCnavCommon {
    pub preamble: u8,
    pub prn: u8,
    pub message_type: u8,
    pub tow_count: u32,
    pub alert: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsCnavMessage {
    pub common: GpsCnavCommon,
    pub bits: Vec<u8>,
    pub transmitted_crc: u32,
    pub computed_crc: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GpsCnavMessageRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    InvalidPreamble,
    InvalidMessageType,
    CrcMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsCnavMessageRejection {
    pub reason: GpsCnavMessageRejectionReason,
    pub bit_index: Option<usize>,
    pub preamble: Option<u8>,
    pub message_type: Option<u8>,
    pub transmitted_crc: Option<u32>,
    pub computed_crc: Option<u32>,
}

impl GpsCnavMessage {
    pub fn bit(&self, position: usize) -> u8 {
        self.bits[position - 1]
    }

    pub fn unsigned_bits(&self, start: usize, len: usize) -> u64 {
        unsigned_bits(&self.bits, start, len)
    }

    pub fn signed_bits(&self, start: usize, len: usize) -> i64 {
        signed_from_unsigned(self.unsigned_bits(start, len), len)
    }
}

pub fn decode_gps_cnav_message(bits: &[u8]) -> Result<GpsCnavMessage, GpsCnavMessageRejection> {
    let normalized = normalize_cnav_bits(bits)?;
    let preamble = unsigned_bits(&normalized, 1, 8) as u8;
    if preamble != GPS_CNAV_PREAMBLE {
        return Err(GpsCnavMessageRejection {
            reason: GpsCnavMessageRejectionReason::InvalidPreamble,
            bit_index: None,
            preamble: Some(preamble),
            message_type: None,
            transmitted_crc: None,
            computed_crc: None,
        });
    }

    let common = GpsCnavCommon {
        preamble,
        prn: unsigned_bits(&normalized, 9, 6) as u8,
        message_type: unsigned_bits(&normalized, 15, 6) as u8,
        tow_count: unsigned_bits(&normalized, 21, 17) as u32,
        alert: unsigned_bits(&normalized, 38, 1) != 0,
    };
    if common.message_type > 63 {
        return Err(GpsCnavMessageRejection {
            reason: GpsCnavMessageRejectionReason::InvalidMessageType,
            bit_index: None,
            preamble: Some(preamble),
            message_type: Some(common.message_type),
            transmitted_crc: None,
            computed_crc: None,
        });
    }

    let transmitted_crc =
        unsigned_bits(&normalized, GPS_CNAV_DATA_BITS + 1, GPS_CNAV_CRC_BITS) as u32;
    let computed_crc = cnav_crc24q(&normalized[..GPS_CNAV_DATA_BITS]);
    if transmitted_crc != computed_crc {
        return Err(GpsCnavMessageRejection {
            reason: GpsCnavMessageRejectionReason::CrcMismatch,
            bit_index: None,
            preamble: Some(preamble),
            message_type: Some(common.message_type),
            transmitted_crc: Some(transmitted_crc),
            computed_crc: Some(computed_crc),
        });
    }

    Ok(GpsCnavMessage { common, bits: normalized, transmitted_crc, computed_crc })
}

pub fn decode_gps_cnav_message_hex(hex: &str) -> Result<GpsCnavMessage, GpsCnavMessageRejection> {
    let Ok(bytes) = hex::decode(hex) else {
        return Err(GpsCnavMessageRejection {
            reason: GpsCnavMessageRejectionReason::InvalidBitCount,
            bit_index: None,
            preamble: None,
            message_type: None,
            transmitted_crc: None,
            computed_crc: None,
        });
    };
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for byte in bytes {
        for shift in (0..8).rev() {
            bits.push((byte >> shift) & 1);
        }
    }
    decode_gps_cnav_message(&bits)
}

fn normalize_cnav_bits(bits: &[u8]) -> Result<Vec<u8>, GpsCnavMessageRejection> {
    if bits.len() != GPS_CNAV_BITS {
        return Err(GpsCnavMessageRejection {
            reason: GpsCnavMessageRejectionReason::InvalidBitCount,
            bit_index: None,
            preamble: None,
            message_type: None,
            transmitted_crc: None,
            computed_crc: None,
        });
    }
    let mut normalized = Vec::with_capacity(GPS_CNAV_BITS);
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(GpsCnavMessageRejection {
                reason: GpsCnavMessageRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                preamble: None,
                message_type: None,
                transmitted_crc: None,
                computed_crc: None,
            });
        }
        normalized.push(bit);
    }
    Ok(normalized)
}

fn cnav_crc24q(bits: &[u8]) -> u32 {
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
        cnav_crc24q, decode_gps_cnav_message, GpsCnavMessageRejectionReason, GPS_CNAV_BITS,
        GPS_CNAV_CRC_BITS, GPS_CNAV_DATA_BITS, GPS_CNAV_PREAMBLE,
    };

    fn set_bits(bits: &mut [u8], start: usize, len: usize, value: u64) {
        for offset in 0..len {
            let shift = len - offset - 1;
            bits[start + offset - 1] = ((value >> shift) & 1) as u8;
        }
    }

    fn valid_message(message_type: u8) -> Vec<u8> {
        let mut bits = vec![0_u8; GPS_CNAV_BITS];
        set_bits(&mut bits, 1, 8, u64::from(GPS_CNAV_PREAMBLE));
        set_bits(&mut bits, 9, 6, 17);
        set_bits(&mut bits, 15, 6, u64::from(message_type));
        set_bits(&mut bits, 21, 17, 12_345);
        set_bits(&mut bits, 38, 1, 1);
        let crc = cnav_crc24q(&bits[..GPS_CNAV_DATA_BITS]);
        set_bits(&mut bits, GPS_CNAV_DATA_BITS + 1, GPS_CNAV_CRC_BITS, u64::from(crc));
        bits
    }

    #[test]
    fn cnav_message_decodes_common_header_and_crc() {
        let bits = valid_message(10);

        let message = decode_gps_cnav_message(&bits).expect("valid CNAV message");

        assert_eq!(message.common.preamble, GPS_CNAV_PREAMBLE);
        assert_eq!(message.common.prn, 17);
        assert_eq!(message.common.message_type, 10);
        assert_eq!(message.common.tow_count, 12_345);
        assert!(message.common.alert);
        assert_eq!(message.transmitted_crc, message.computed_crc);
    }

    #[test]
    fn cnav_message_rejects_bad_crc() {
        let mut bits = valid_message(10);
        bits[120] ^= 1;

        let rejection = decode_gps_cnav_message(&bits).expect_err("CRC rejection");

        assert_eq!(rejection.reason, GpsCnavMessageRejectionReason::CrcMismatch);
        assert_ne!(rejection.transmitted_crc, rejection.computed_crc);
    }

    #[test]
    fn cnav_message_rejects_invalid_shape_before_crc() {
        let mut bits = valid_message(10);
        bits[0] = 0;
        let rejection = decode_gps_cnav_message(&bits).expect_err("preamble rejection");
        assert_eq!(rejection.reason, GpsCnavMessageRejectionReason::InvalidPreamble);

        let rejection =
            decode_gps_cnav_message(&bits[..GPS_CNAV_BITS - 1]).expect_err("bit count rejection");
        assert_eq!(rejection.reason, GpsCnavMessageRejectionReason::InvalidBitCount);

        let mut bits = valid_message(10);
        bits[4] = 2;
        let rejection = decode_gps_cnav_message(&bits).expect_err("binary rejection");
        assert_eq!(rejection.reason, GpsCnavMessageRejectionReason::NonBinaryBit);
        assert_eq!(rejection.bit_index, Some(4));
    }
}
