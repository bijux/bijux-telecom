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
pub struct GpsCnavSignalHealth {
    pub l1_unhealthy: bool,
    pub l2_unhealthy: bool,
    pub l5_unhealthy: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavEphemerisMessage {
    pub common: GpsCnavCommon,
    pub week: u16,
    pub signal_health: GpsCnavSignalHealth,
    pub uraed_index: u8,
    pub top_s: f64,
    pub delta_a_m: f64,
    pub a_dot_mps: f64,
    pub delta_n0_rad_per_s: f64,
    pub delta_n0_dot_rad_per_s2: f64,
    pub m0_rad: f64,
    pub e: f64,
    pub w_rad: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavOrbitMessage {
    pub common: GpsCnavCommon,
    pub toe_s: f64,
    pub omega0_rad: f64,
    pub delta_omegadot_rad_per_s: f64,
    pub i0_rad: f64,
    pub idot_rad_per_s: f64,
    pub cis_rad: f64,
    pub cic_rad: f64,
    pub crs_m: f64,
    pub crc_m: f64,
    pub cus_rad: f64,
    pub cuc_rad: f64,
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

pub fn decode_gps_cnav_ephemeris_message(
    message: &GpsCnavMessage,
) -> Option<GpsCnavEphemerisMessage> {
    if message.common.message_type != 10 {
        return None;
    }

    let health = message.unsigned_bits(52, 3) as u8;
    Some(GpsCnavEphemerisMessage {
        common: message.common,
        week: message.unsigned_bits(39, 13) as u16,
        signal_health: GpsCnavSignalHealth {
            l1_unhealthy: (health & 0b100) != 0,
            l2_unhealthy: (health & 0b010) != 0,
            l5_unhealthy: (health & 0b001) != 0,
        },
        uraed_index: message.unsigned_bits(55, 5) as u8,
        top_s: message.unsigned_bits(60, 11) as f64 * 300.0,
        delta_a_m: message.signed_bits(71, 26) as f64 * 2f64.powi(-9),
        a_dot_mps: message.signed_bits(97, 25) as f64 * 2f64.powi(-21),
        delta_n0_rad_per_s: message.signed_bits(122, 17) as f64
            * 2f64.powi(-44)
            * std::f64::consts::PI,
        delta_n0_dot_rad_per_s2: message.signed_bits(139, 23) as f64
            * 2f64.powi(-57)
            * std::f64::consts::PI,
        m0_rad: message.signed_bits(162, 33) as f64 * 2f64.powi(-32) * std::f64::consts::PI,
        e: message.unsigned_bits(195, 33) as f64 * 2f64.powi(-34),
        w_rad: message.signed_bits(228, 33) as f64 * 2f64.powi(-32) * std::f64::consts::PI,
    })
}

pub fn decode_gps_cnav_orbit_message(message: &GpsCnavMessage) -> Option<GpsCnavOrbitMessage> {
    if message.common.message_type != 11 {
        return None;
    }

    Some(GpsCnavOrbitMessage {
        common: message.common,
        toe_s: message.unsigned_bits(39, 11) as f64 * 300.0,
        omega0_rad: message.signed_bits(50, 33) as f64 * 2f64.powi(-32) * std::f64::consts::PI,
        delta_omegadot_rad_per_s: message.signed_bits(83, 17) as f64
            * 2f64.powi(-44)
            * std::f64::consts::PI,
        i0_rad: message.signed_bits(100, 33) as f64 * 2f64.powi(-32) * std::f64::consts::PI,
        idot_rad_per_s: message.signed_bits(133, 15) as f64 * 2f64.powi(-44) * std::f64::consts::PI,
        cis_rad: message.signed_bits(148, 16) as f64 * 2f64.powi(-30),
        cic_rad: message.signed_bits(164, 16) as f64 * 2f64.powi(-30),
        crs_m: message.signed_bits(180, 24) as f64 * 2f64.powi(-8),
        crc_m: message.signed_bits(204, 24) as f64 * 2f64.powi(-8),
        cus_rad: message.signed_bits(228, 21) as f64 * 2f64.powi(-30),
        cuc_rad: message.signed_bits(249, 21) as f64 * 2f64.powi(-30),
    })
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
        cnav_crc24q, decode_gps_cnav_ephemeris_message, decode_gps_cnav_message,
        decode_gps_cnav_orbit_message, GpsCnavMessageRejectionReason, GPS_CNAV_BITS,
        GPS_CNAV_CRC_BITS, GPS_CNAV_DATA_BITS, GPS_CNAV_PREAMBLE,
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
        let crc = cnav_crc24q(&bits[..GPS_CNAV_DATA_BITS]);
        set_bits(bits, GPS_CNAV_DATA_BITS + 1, GPS_CNAV_CRC_BITS, u64::from(crc));
    }

    fn valid_message(message_type: u8) -> Vec<u8> {
        let mut bits = vec![0_u8; GPS_CNAV_BITS];
        set_bits(&mut bits, 1, 8, u64::from(GPS_CNAV_PREAMBLE));
        set_bits(&mut bits, 9, 6, 17);
        set_bits(&mut bits, 15, 6, u64::from(message_type));
        set_bits(&mut bits, 21, 17, 12_345);
        set_bits(&mut bits, 38, 1, 1);
        apply_crc(&mut bits);
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

    #[test]
    fn cnav_ephemeris_message_decodes_type_10_fields() {
        let week = 4_321_u64;
        let health = 0b101_u64;
        let uraed = 17_u64;
        let top_raw = 1_234_u64;
        let delta_a_raw = -123_456_i64;
        let a_dot_raw = 654_321_i64;
        let delta_n0_raw = -16_321_i64;
        let delta_n0_dot_raw = 2_001_i64;
        let m0_raw = -0x1234_5678_i64;
        let e_raw = 0x1234_5678_u64;
        let w_raw = 0x1020_3040_i64;
        let mut bits = valid_message(10);
        set_bits(&mut bits, 39, 13, week);
        set_bits(&mut bits, 52, 3, health);
        set_bits(&mut bits, 55, 5, uraed);
        set_bits(&mut bits, 60, 11, top_raw);
        set_bits(&mut bits, 71, 26, encode_signed(delta_a_raw, 26));
        set_bits(&mut bits, 97, 25, encode_signed(a_dot_raw, 25));
        set_bits(&mut bits, 122, 17, encode_signed(delta_n0_raw, 17));
        set_bits(&mut bits, 139, 23, encode_signed(delta_n0_dot_raw, 23));
        set_bits(&mut bits, 162, 33, encode_signed(m0_raw, 33));
        set_bits(&mut bits, 195, 33, e_raw);
        set_bits(&mut bits, 228, 33, encode_signed(w_raw, 33));
        apply_crc(&mut bits);

        let message = decode_gps_cnav_message(&bits).expect("valid CNAV message");
        let ephemeris = decode_gps_cnav_ephemeris_message(&message).expect("type 10 message");

        assert_eq!(ephemeris.week, week as u16);
        assert!(ephemeris.signal_health.l1_unhealthy);
        assert!(!ephemeris.signal_health.l2_unhealthy);
        assert!(ephemeris.signal_health.l5_unhealthy);
        assert_eq!(ephemeris.uraed_index, uraed as u8);
        assert_eq!(ephemeris.top_s, top_raw as f64 * 300.0);
        assert!((ephemeris.delta_a_m - delta_a_raw as f64 * 2f64.powi(-9)).abs() < f64::EPSILON);
        assert!((ephemeris.a_dot_mps - a_dot_raw as f64 * 2f64.powi(-21)).abs() < f64::EPSILON);
        assert!(
            (ephemeris.delta_n0_rad_per_s
                - delta_n0_raw as f64 * 2f64.powi(-44) * std::f64::consts::PI)
                .abs()
                < f64::EPSILON
        );
        assert!(
            (ephemeris.delta_n0_dot_rad_per_s2
                - delta_n0_dot_raw as f64 * 2f64.powi(-57) * std::f64::consts::PI)
                .abs()
                < f64::EPSILON
        );
        assert!(
            (ephemeris.m0_rad - m0_raw as f64 * 2f64.powi(-32) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert_eq!(ephemeris.e, e_raw as f64 * 2f64.powi(-34));
        assert!(
            (ephemeris.w_rad - w_raw as f64 * 2f64.powi(-32) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
    }

    #[test]
    fn cnav_orbit_message_decodes_type_11_fields() {
        let toe_raw = 1_111_u64;
        let omega0_raw = -0x0102_0304_i64;
        let delta_omegadot_raw = 12_345_i64;
        let i0_raw = 0x0123_4567_i64;
        let idot_raw = -2_001_i64;
        let cis_raw = -432_i64;
        let cic_raw = 543_i64;
        let crs_raw = -654_321_i64;
        let crc_raw = 765_432_i64;
        let cus_raw = -12_345_i64;
        let cuc_raw = 23_456_i64;
        let mut bits = valid_message(11);
        set_bits(&mut bits, 39, 11, toe_raw);
        set_bits(&mut bits, 50, 33, encode_signed(omega0_raw, 33));
        set_bits(&mut bits, 83, 17, encode_signed(delta_omegadot_raw, 17));
        set_bits(&mut bits, 100, 33, encode_signed(i0_raw, 33));
        set_bits(&mut bits, 133, 15, encode_signed(idot_raw, 15));
        set_bits(&mut bits, 148, 16, encode_signed(cis_raw, 16));
        set_bits(&mut bits, 164, 16, encode_signed(cic_raw, 16));
        set_bits(&mut bits, 180, 24, encode_signed(crs_raw, 24));
        set_bits(&mut bits, 204, 24, encode_signed(crc_raw, 24));
        set_bits(&mut bits, 228, 21, encode_signed(cus_raw, 21));
        set_bits(&mut bits, 249, 21, encode_signed(cuc_raw, 21));
        apply_crc(&mut bits);

        let message = decode_gps_cnav_message(&bits).expect("valid CNAV message");
        let orbit = decode_gps_cnav_orbit_message(&message).expect("type 11 message");

        assert_eq!(orbit.toe_s, toe_raw as f64 * 300.0);
        assert!(
            (orbit.omega0_rad - omega0_raw as f64 * 2f64.powi(-32) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert!(
            (orbit.delta_omegadot_rad_per_s
                - delta_omegadot_raw as f64 * 2f64.powi(-44) * std::f64::consts::PI)
                .abs()
                < f64::EPSILON
        );
        assert!(
            (orbit.i0_rad - i0_raw as f64 * 2f64.powi(-32) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert!(
            (orbit.idot_rad_per_s - idot_raw as f64 * 2f64.powi(-44) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert_eq!(orbit.cis_rad, cis_raw as f64 * 2f64.powi(-30));
        assert_eq!(orbit.cic_rad, cic_raw as f64 * 2f64.powi(-30));
        assert_eq!(orbit.crs_m, crs_raw as f64 * 2f64.powi(-8));
        assert_eq!(orbit.crc_m, crc_raw as f64 * 2f64.powi(-8));
        assert_eq!(orbit.cus_rad, cus_raw as f64 * 2f64.powi(-30));
        assert_eq!(orbit.cuc_rad, cuc_raw as f64 * 2f64.powi(-30));
    }
}
