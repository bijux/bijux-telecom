#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use crate::orbits::galileo::{
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSystemTime,
};

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
pub struct GalileoFnavSignalStatus {
    pub e5a_signal_health: u8,
    pub e5a_data_valid: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavClockCorrection {
    pub t0c_s: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub bgd_e1_e5a_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavClockStatusPage {
    pub svid: u8,
    pub iodnav: u16,
    pub sisa_e1_e5a: u8,
    pub clock: GalileoFnavClockCorrection,
    pub ionosphere: GalileoIonosphericCorrection,
    pub signal_status: GalileoFnavSignalStatus,
    pub gst: GalileoSystemTime,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavKeplerianPage {
    pub iodnav: u16,
    pub m0: f64,
    pub omegadot: f64,
    pub e: f64,
    pub sqrt_a: f64,
    pub omega0: f64,
    pub idot: f64,
    pub gst: GalileoSystemTime,
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

pub fn decode_galileo_fnav_clock_status_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavClockStatusPage> {
    (page.page_type == 1).then(|| GalileoFnavClockStatusPage {
        svid: page.unsigned_bits(7, 6) as u8,
        iodnav: page.unsigned_bits(13, 10) as u16,
        clock: GalileoFnavClockCorrection {
            t0c_s: page.unsigned_bits(23, 14) as f64 * 60.0,
            af0: page.signed_bits(37, 31) as f64 * 2f64.powi(-34),
            af1: page.signed_bits(68, 21) as f64 * 2f64.powi(-46),
            af2: page.signed_bits(89, 6) as f64 * 2f64.powi(-59),
            bgd_e1_e5a_s: page.signed_bits(144, 10) as f64 * 2f64.powi(-32),
        },
        sisa_e1_e5a: page.unsigned_bits(95, 8) as u8,
        ionosphere: GalileoIonosphericCorrection {
            ai0: page.unsigned_bits(103, 11) as f64 * 2f64.powi(-2),
            ai1: page.signed_bits(114, 11) as f64 * 2f64.powi(-8),
            ai2: page.signed_bits(125, 14) as f64 * 2f64.powi(-15),
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: page.unsigned_bits(139, 1) != 0,
                region_2: page.unsigned_bits(140, 1) != 0,
                region_3: page.unsigned_bits(141, 1) != 0,
                region_4: page.unsigned_bits(142, 1) != 0,
                region_5: page.unsigned_bits(143, 1) != 0,
            },
        },
        signal_status: GalileoFnavSignalStatus {
            e5a_signal_health: page.unsigned_bits(154, 2) as u8,
            e5a_data_valid: page.unsigned_bits(188, 1) == 0,
        },
        gst: GalileoSystemTime {
            week: page.unsigned_bits(156, 12) as u16,
            tow_s: page.unsigned_bits(168, 20) as u32,
        },
    })
}

pub fn decode_galileo_fnav_keplerian_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavKeplerianPage> {
    (page.page_type == 2).then(|| GalileoFnavKeplerianPage {
        iodnav: page.unsigned_bits(7, 10) as u16,
        m0: page.signed_bits(17, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        omegadot: page.signed_bits(49, 24) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        e: page.unsigned_bits(73, 32) as f64 * 2f64.powi(-33),
        sqrt_a: page.unsigned_bits(105, 32) as f64 * 2f64.powi(-19),
        omega0: page.signed_bits(137, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        idot: page.signed_bits(169, 14) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        gst: GalileoSystemTime {
            week: page.unsigned_bits(183, 12) as u16,
            tow_s: page.unsigned_bits(195, 20) as u32,
        },
    })
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

    fn encode_signed(value: i64, bits: usize) -> u64 {
        let mask = (1_u64 << bits) - 1;
        value as u64 & mask
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

    #[test]
    fn clock_status_page_decodes_e5a_clock_and_status_fields() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 1);
        set_bits(&mut bits, 7, 6, 19);
        set_bits(&mut bits, 13, 10, 0x1A5);
        set_bits(&mut bits, 23, 14, 1_111);
        set_bits(&mut bits, 37, 31, encode_signed(-0x1ABCD_i64, 31));
        set_bits(&mut bits, 68, 21, encode_signed(0x01234_i64, 21));
        set_bits(&mut bits, 89, 6, encode_signed(-0x09_i64, 6));
        set_bits(&mut bits, 95, 8, 77);
        set_bits(&mut bits, 103, 11, 211);
        set_bits(&mut bits, 114, 11, encode_signed(-87_i64, 11));
        set_bits(&mut bits, 125, 14, encode_signed(0x00A5_i64, 14));
        set_bits(&mut bits, 139, 1, 1);
        set_bits(&mut bits, 140, 1, 1);
        set_bits(&mut bits, 142, 1, 1);
        set_bits(&mut bits, 144, 10, encode_signed(-12_i64, 10));
        set_bits(&mut bits, 154, 2, 2);
        set_bits(&mut bits, 156, 12, 2_222);
        set_bits(&mut bits, 168, 20, 456_789);
        set_bits(&mut bits, 188, 1, 0);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 1 page");

        let status = super::decode_galileo_fnav_clock_status_page(&page).expect("type 1 payload");

        assert_eq!(status.svid, 19);
        assert_eq!(status.iodnav, 0x1A5);
        assert_eq!(status.sisa_e1_e5a, 77);
        assert_eq!(status.clock.t0c_s, 1_111.0 * 60.0);
        assert_eq!(status.clock.af0, -0x1ABCD_i64 as f64 * 2f64.powi(-34));
        assert_eq!(status.clock.af1, 0x01234_i64 as f64 * 2f64.powi(-46));
        assert_eq!(status.clock.af2, -0x09_i64 as f64 * 2f64.powi(-59));
        assert_eq!(status.clock.bgd_e1_e5a_s, -12.0 * 2f64.powi(-32));
        assert_eq!(status.ionosphere.ai0, 211.0 * 2f64.powi(-2));
        assert_eq!(status.ionosphere.ai1, -87.0 * 2f64.powi(-8));
        assert_eq!(status.ionosphere.ai2, 0x00A5_i64 as f64 * 2f64.powi(-15));
        assert!(status.ionosphere.disturbance_flags.region_1);
        assert!(status.ionosphere.disturbance_flags.region_2);
        assert!(!status.ionosphere.disturbance_flags.region_3);
        assert!(status.ionosphere.disturbance_flags.region_4);
        assert!(!status.ionosphere.disturbance_flags.region_5);
        assert_eq!(status.signal_status.e5a_signal_health, 2);
        assert!(status.signal_status.e5a_data_valid);
        assert_eq!(status.gst.week, 2_222);
        assert_eq!(status.gst.tow_s, 456_789);
    }

    #[test]
    fn keplerian_page_decodes_orbit_shape_and_gst_fields() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 2);
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 32, encode_signed(-0x1020_304_i64, 32));
        set_bits(&mut bits, 49, 24, encode_signed(-0x04_321_i64, 24));
        set_bits(&mut bits, 73, 32, 0x0123_4567);
        set_bits(&mut bits, 105, 32, 0x0987_6543);
        set_bits(&mut bits, 137, 32, encode_signed(0x1020_3040_i64, 32));
        set_bits(&mut bits, 169, 14, encode_signed(-0x03A5_i64, 14));
        set_bits(&mut bits, 183, 12, 2_223);
        set_bits(&mut bits, 195, 20, 456_799);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 2 page");

        let keplerian = super::decode_galileo_fnav_keplerian_page(&page).expect("type 2 payload");

        assert_eq!(keplerian.iodnav, 0x155);
        assert_eq!(keplerian.m0, -0x1020_304_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI);
        assert_eq!(
            keplerian.omegadot,
            -0x04_321_i64 as f64 * 2f64.powi(-43) * std::f64::consts::PI
        );
        assert_eq!(keplerian.e, 0x0123_4567_u64 as f64 * 2f64.powi(-33));
        assert_eq!(keplerian.sqrt_a, 0x0987_6543_u64 as f64 * 2f64.powi(-19));
        assert_eq!(
            keplerian.omega0,
            0x1020_3040_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI
        );
        assert_eq!(keplerian.idot, -0x03A5_i64 as f64 * 2f64.powi(-43) * std::f64::consts::PI);
        assert_eq!(keplerian.gst.week, 2_223);
        assert_eq!(keplerian.gst.tow_s, 456_799);
    }
}
