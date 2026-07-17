#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, SatId};

const GPS_CNAV_BITS: usize = 300;
const GPS_CNAV_DATA_BITS: usize = 276;
const GPS_CNAV_CRC_BITS: usize = 24;
const GPS_CNAV_PREAMBLE: u8 = 0b1000_1011;
const CRC24Q_POLY: u32 = 0x18_64CFB;
const CRC24Q_MASK: u32 = 0xFF_FFFF;
const GPS_CNAV_SEMI_MAJOR_AXIS_REFERENCE_M: f64 = 26_559_710.0;
const GPS_CNAV_OMEGADOT_REFERENCE_RAD_PER_S: f64 = -2.6e-9 * std::f64::consts::PI;

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
pub struct GpsCnavNonElevationAccuracy {
    pub uraned0_index: u8,
    pub uraned1_index: u8,
    pub uraned2_index: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavClockCorrection {
    pub toc_s: f64,
    pub af0_s: f64,
    pub af1_s_per_s: f64,
    pub af2_s_per_s2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavGroupDelayCorrection {
    pub tgd_s: f64,
    pub isc_l1ca_s: f64,
    pub isc_l2c_s: f64,
    pub isc_l5i5_s: f64,
    pub isc_l5q5_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavIonosphericCorrection {
    pub alpha0: f64,
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub beta0: f64,
    pub beta1: f64,
    pub beta2: f64,
    pub beta3: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavClockCorrectionMessage {
    pub common: GpsCnavCommon,
    pub top_s: f64,
    pub accuracy: GpsCnavNonElevationAccuracy,
    pub clock: GpsCnavClockCorrection,
    pub group_delay: GpsCnavGroupDelayCorrection,
    pub ionosphere: GpsCnavIonosphericCorrection,
    pub propagation_week_modulo_256: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavEphemeris {
    pub week: u16,
    pub top_s: f64,
    pub toe_s: f64,
    pub semi_major_axis_m: f64,
    pub a_dot_mps: f64,
    pub delta_n0_rad_per_s: f64,
    pub delta_n0_dot_rad_per_s2: f64,
    pub m0_rad: f64,
    pub e: f64,
    pub w_rad: f64,
    pub omega0_rad: f64,
    pub omegadot_rad_per_s: f64,
    pub i0_rad: f64,
    pub idot_rad_per_s: f64,
    pub cis_rad: f64,
    pub cic_rad: f64,
    pub crs_m: f64,
    pub crc_m: f64,
    pub cus_rad: f64,
    pub cuc_rad: f64,
    pub uraed_index: u8,
    pub signal_health: GpsCnavSignalHealth,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsCnavBroadcastNavigationData {
    pub sat: SatId,
    pub ephemeris: GpsCnavEphemeris,
    pub clock: GpsCnavClockCorrection,
    pub accuracy: GpsCnavNonElevationAccuracy,
    pub group_delay: GpsCnavGroupDelayCorrection,
    pub ionosphere: GpsCnavIonosphericCorrection,
    pub propagation_week_modulo_256: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GpsCnavNavigationRejectionReason {
    MixedPrn,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsCnavNavigationRejection {
    pub reason: GpsCnavNavigationRejectionReason,
    pub expected_prn: Option<u8>,
    pub incoming_prn: Option<u8>,
    pub message_type: Option<u8>,
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

pub fn decode_gps_cnav_clock_correction_message(
    message: &GpsCnavMessage,
) -> Option<GpsCnavClockCorrectionMessage> {
    if message.common.message_type != 30 {
        return None;
    }

    Some(GpsCnavClockCorrectionMessage {
        common: message.common,
        top_s: message.unsigned_bits(39, 11) as f64 * 300.0,
        accuracy: GpsCnavNonElevationAccuracy {
            uraned0_index: message.unsigned_bits(50, 5) as u8,
            uraned1_index: message.unsigned_bits(55, 3) as u8,
            uraned2_index: message.unsigned_bits(58, 3) as u8,
        },
        clock: GpsCnavClockCorrection {
            toc_s: message.unsigned_bits(61, 11) as f64 * 300.0,
            af2_s_per_s2: message.signed_bits(72, 10) as f64 * 2f64.powi(-60),
            af1_s_per_s: message.signed_bits(82, 20) as f64 * 2f64.powi(-48),
            af0_s: message.signed_bits(102, 26) as f64 * 2f64.powi(-35),
        },
        group_delay: GpsCnavGroupDelayCorrection {
            tgd_s: message.signed_bits(128, 13) as f64 * 2f64.powi(-35),
            isc_l1ca_s: message.signed_bits(141, 13) as f64 * 2f64.powi(-35),
            isc_l2c_s: message.signed_bits(154, 13) as f64 * 2f64.powi(-35),
            isc_l5i5_s: message.signed_bits(167, 13) as f64 * 2f64.powi(-35),
            isc_l5q5_s: message.signed_bits(180, 13) as f64 * 2f64.powi(-35),
        },
        ionosphere: GpsCnavIonosphericCorrection {
            alpha0: message.signed_bits(193, 8) as f64 * 2f64.powi(-30),
            alpha1: message.signed_bits(201, 8) as f64 * 2f64.powi(-27),
            alpha2: message.signed_bits(209, 8) as f64 * 2f64.powi(-24),
            alpha3: message.signed_bits(217, 8) as f64 * 2f64.powi(-24),
            beta0: message.signed_bits(225, 8) as f64 * 2f64.powi(11),
            beta1: message.signed_bits(233, 8) as f64 * 2f64.powi(14),
            beta2: message.signed_bits(241, 8) as f64 * 2f64.powi(16),
            beta3: message.signed_bits(249, 8) as f64 * 2f64.powi(16),
        },
        propagation_week_modulo_256: message.unsigned_bits(257, 8) as u8,
    })
}

pub fn decode_gps_cnav_broadcast_navigation(
    messages: &[GpsCnavMessage],
) -> Result<Option<GpsCnavBroadcastNavigationData>, GpsCnavNavigationRejection> {
    let mut prn = None;
    let mut ephemeris_message = None;
    let mut orbit_message = None;
    let mut clock_message = None;

    for message in messages {
        if let Some(expected_prn) = prn {
            if expected_prn != message.common.prn {
                return Err(GpsCnavNavigationRejection {
                    reason: GpsCnavNavigationRejectionReason::MixedPrn,
                    expected_prn: Some(expected_prn),
                    incoming_prn: Some(message.common.prn),
                    message_type: Some(message.common.message_type),
                });
            }
        } else {
            prn = Some(message.common.prn);
        }

        match message.common.message_type {
            10 => ephemeris_message = decode_gps_cnav_ephemeris_message(message),
            11 => orbit_message = decode_gps_cnav_orbit_message(message),
            30 => clock_message = decode_gps_cnav_clock_correction_message(message),
            _ => {}
        }
    }

    let (Some(prn), Some(ephemeris), Some(orbit), Some(clock)) =
        (prn, ephemeris_message, orbit_message, clock_message)
    else {
        return Ok(None);
    };

    let sat = SatId { constellation: Constellation::Gps, prn };
    Ok(Some(GpsCnavBroadcastNavigationData {
        sat,
        ephemeris: GpsCnavEphemeris {
            week: ephemeris.week,
            top_s: ephemeris.top_s,
            toe_s: orbit.toe_s,
            semi_major_axis_m: GPS_CNAV_SEMI_MAJOR_AXIS_REFERENCE_M + ephemeris.delta_a_m,
            a_dot_mps: ephemeris.a_dot_mps,
            delta_n0_rad_per_s: ephemeris.delta_n0_rad_per_s,
            delta_n0_dot_rad_per_s2: ephemeris.delta_n0_dot_rad_per_s2,
            m0_rad: ephemeris.m0_rad,
            e: ephemeris.e,
            w_rad: ephemeris.w_rad,
            omega0_rad: orbit.omega0_rad,
            omegadot_rad_per_s: GPS_CNAV_OMEGADOT_REFERENCE_RAD_PER_S
                + orbit.delta_omegadot_rad_per_s,
            i0_rad: orbit.i0_rad,
            idot_rad_per_s: orbit.idot_rad_per_s,
            cis_rad: orbit.cis_rad,
            cic_rad: orbit.cic_rad,
            crs_m: orbit.crs_m,
            crc_m: orbit.crc_m,
            cus_rad: orbit.cus_rad,
            cuc_rad: orbit.cuc_rad,
            uraed_index: ephemeris.uraed_index,
            signal_health: ephemeris.signal_health,
        },
        clock: clock.clock,
        accuracy: clock.accuracy,
        group_delay: clock.group_delay,
        ionosphere: clock.ionosphere,
        propagation_week_modulo_256: clock.propagation_week_modulo_256,
    }))
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
        cnav_crc24q, decode_gps_cnav_broadcast_navigation,
        decode_gps_cnav_clock_correction_message, decode_gps_cnav_ephemeris_message,
        decode_gps_cnav_message, decode_gps_cnav_orbit_message, GpsCnavMessageRejectionReason,
        GpsCnavNavigationRejectionReason, GPS_CNAV_BITS, GPS_CNAV_CRC_BITS, GPS_CNAV_DATA_BITS,
        GPS_CNAV_PREAMBLE,
    };
    use bijux_gnss_core::api::Constellation;

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

    fn set_prn(bits: &mut [u8], prn: u8) {
        set_bits(bits, 9, 6, u64::from(prn));
        apply_crc(bits);
    }

    fn valid_ephemeris_message_bits() -> Vec<u8> {
        let mut bits = valid_message(10);
        set_bits(&mut bits, 39, 13, 4_321);
        set_bits(&mut bits, 52, 3, 0b101);
        set_bits(&mut bits, 55, 5, 17);
        set_bits(&mut bits, 60, 11, 1_234);
        set_bits(&mut bits, 71, 26, encode_signed(-123_456, 26));
        set_bits(&mut bits, 97, 25, encode_signed(654_321, 25));
        set_bits(&mut bits, 122, 17, encode_signed(-16_321, 17));
        set_bits(&mut bits, 139, 23, encode_signed(2_001, 23));
        set_bits(&mut bits, 162, 33, encode_signed(-0x1234_5678, 33));
        set_bits(&mut bits, 195, 33, 0x1234_5678);
        set_bits(&mut bits, 228, 33, encode_signed(0x1020_3040, 33));
        apply_crc(&mut bits);
        bits
    }

    fn valid_orbit_message_bits() -> Vec<u8> {
        let mut bits = valid_message(11);
        set_bits(&mut bits, 39, 11, 1_111);
        set_bits(&mut bits, 50, 33, encode_signed(-0x0102_0304, 33));
        set_bits(&mut bits, 83, 17, encode_signed(12_345, 17));
        set_bits(&mut bits, 100, 33, encode_signed(0x0123_4567, 33));
        set_bits(&mut bits, 133, 15, encode_signed(-2_001, 15));
        set_bits(&mut bits, 148, 16, encode_signed(-432, 16));
        set_bits(&mut bits, 164, 16, encode_signed(543, 16));
        set_bits(&mut bits, 180, 24, encode_signed(-654_321, 24));
        set_bits(&mut bits, 204, 24, encode_signed(765_432, 24));
        set_bits(&mut bits, 228, 21, encode_signed(-12_345, 21));
        set_bits(&mut bits, 249, 21, encode_signed(23_456, 21));
        apply_crc(&mut bits);
        bits
    }

    fn valid_clock_message_bits() -> Vec<u8> {
        let mut bits = valid_message(30);
        set_bits(&mut bits, 39, 11, 1_010);
        set_bits(&mut bits, 50, 5, 17);
        set_bits(&mut bits, 55, 3, 3);
        set_bits(&mut bits, 58, 3, 5);
        set_bits(&mut bits, 61, 11, 1_111);
        set_bits(&mut bits, 72, 10, encode_signed(-123, 10));
        set_bits(&mut bits, 82, 20, encode_signed(45_678, 20));
        set_bits(&mut bits, 102, 26, encode_signed(-2_345_678, 26));
        set_bits(&mut bits, 128, 13, encode_signed(-111, 13));
        set_bits(&mut bits, 141, 13, encode_signed(222, 13));
        set_bits(&mut bits, 154, 13, encode_signed(-333, 13));
        set_bits(&mut bits, 167, 13, encode_signed(444, 13));
        set_bits(&mut bits, 180, 13, encode_signed(-555, 13));
        for (idx, value) in [-7_i64, 8, -9, 10, 11, -12, 13, -14].iter().enumerate() {
            set_bits(&mut bits, 193 + idx * 8, 8, encode_signed(*value, 8));
        }
        set_bits(&mut bits, 257, 8, 211);
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
        let uraed = 17_u64;
        let top_raw = 1_234_u64;
        let delta_a_raw = -123_456_i64;
        let a_dot_raw = 654_321_i64;
        let delta_n0_raw = -16_321_i64;
        let delta_n0_dot_raw = 2_001_i64;
        let m0_raw = -0x1234_5678_i64;
        let e_raw = 0x1234_5678_u64;
        let w_raw = 0x1020_3040_i64;
        let bits = valid_ephemeris_message_bits();

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
        let bits = valid_orbit_message_bits();

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

    #[test]
    fn cnav_clock_correction_message_decodes_type_30_fields() {
        let top_raw = 1_010_u64;
        let toc_raw = 1_111_u64;
        let af2_raw = -123_i64;
        let af1_raw = 45_678_i64;
        let af0_raw = -2_345_678_i64;
        let tgd_raw = -111_i64;
        let isc_l1ca_raw = 222_i64;
        let isc_l2c_raw = -333_i64;
        let isc_l5i5_raw = 444_i64;
        let isc_l5q5_raw = -555_i64;
        let iono_raw = [-7_i64, 8, -9, 10, 11, -12, 13, -14];
        let bits = valid_clock_message_bits();

        let message = decode_gps_cnav_message(&bits).expect("valid CNAV message");
        let correction =
            decode_gps_cnav_clock_correction_message(&message).expect("type 30 message");

        assert_eq!(correction.top_s, top_raw as f64 * 300.0);
        assert_eq!(correction.accuracy.uraned0_index, 17);
        assert_eq!(correction.accuracy.uraned1_index, 3);
        assert_eq!(correction.accuracy.uraned2_index, 5);
        assert_eq!(correction.clock.toc_s, toc_raw as f64 * 300.0);
        assert_eq!(correction.clock.af2_s_per_s2, af2_raw as f64 * 2f64.powi(-60));
        assert_eq!(correction.clock.af1_s_per_s, af1_raw as f64 * 2f64.powi(-48));
        assert_eq!(correction.clock.af0_s, af0_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.group_delay.tgd_s, tgd_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.group_delay.isc_l1ca_s, isc_l1ca_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.group_delay.isc_l2c_s, isc_l2c_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.group_delay.isc_l5i5_s, isc_l5i5_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.group_delay.isc_l5q5_s, isc_l5q5_raw as f64 * 2f64.powi(-35));
        assert_eq!(correction.ionosphere.alpha0, iono_raw[0] as f64 * 2f64.powi(-30));
        assert_eq!(correction.ionosphere.alpha1, iono_raw[1] as f64 * 2f64.powi(-27));
        assert_eq!(correction.ionosphere.alpha2, iono_raw[2] as f64 * 2f64.powi(-24));
        assert_eq!(correction.ionosphere.alpha3, iono_raw[3] as f64 * 2f64.powi(-24));
        assert_eq!(correction.ionosphere.beta0, iono_raw[4] as f64 * 2f64.powi(11));
        assert_eq!(correction.ionosphere.beta1, iono_raw[5] as f64 * 2f64.powi(14));
        assert_eq!(correction.ionosphere.beta2, iono_raw[6] as f64 * 2f64.powi(16));
        assert_eq!(correction.ionosphere.beta3, iono_raw[7] as f64 * 2f64.powi(16));
        assert_eq!(correction.propagation_week_modulo_256, 211);
    }

    #[test]
    fn cnav_broadcast_navigation_assembles_valid_message_set() {
        let messages = [
            decode_gps_cnav_message(&valid_ephemeris_message_bits()).expect("type 10"),
            decode_gps_cnav_message(&valid_orbit_message_bits()).expect("type 11"),
            decode_gps_cnav_message(&valid_clock_message_bits()).expect("type 30"),
        ];

        let navigation = decode_gps_cnav_broadcast_navigation(&messages)
            .expect("navigation assembly")
            .expect("complete navigation");

        assert_eq!(navigation.sat.constellation, Constellation::Gps);
        assert_eq!(navigation.sat.prn, 17);
        assert_eq!(navigation.ephemeris.week, 4_321);
        assert_eq!(navigation.ephemeris.top_s, 1_234.0 * 300.0);
        assert_eq!(navigation.ephemeris.toe_s, 1_111.0 * 300.0);
        assert!(
            (navigation.ephemeris.semi_major_axis_m - (26_559_710.0 - 123_456.0 * 2f64.powi(-9)))
                .abs()
                < f64::EPSILON
        );
        assert!(navigation.ephemeris.signal_health.l1_unhealthy);
        assert_eq!(navigation.accuracy.uraned0_index, 17);
        assert_eq!(navigation.clock.toc_s, 1_111.0 * 300.0);
        assert_eq!(navigation.group_delay.isc_l2c_s, -333.0 * 2f64.powi(-35));
        assert_eq!(navigation.ionosphere.beta3, -14.0 * 2f64.powi(16));
        assert_eq!(navigation.propagation_week_modulo_256, 211);
    }

    #[test]
    fn cnav_broadcast_navigation_waits_for_complete_message_set() {
        let messages = [
            decode_gps_cnav_message(&valid_ephemeris_message_bits()).expect("type 10"),
            decode_gps_cnav_message(&valid_orbit_message_bits()).expect("type 11"),
        ];

        let navigation =
            decode_gps_cnav_broadcast_navigation(&messages).expect("incomplete navigation");

        assert!(navigation.is_none());
    }

    #[test]
    fn cnav_broadcast_navigation_rejects_mixed_prn_messages() {
        let ephemeris = valid_ephemeris_message_bits();
        let mut orbit = valid_orbit_message_bits();
        set_prn(&mut orbit, 18);
        let messages = [
            decode_gps_cnav_message(&ephemeris).expect("type 10"),
            decode_gps_cnav_message(&orbit).expect("type 11"),
            decode_gps_cnav_message(&valid_clock_message_bits()).expect("type 30"),
        ];

        let rejection =
            decode_gps_cnav_broadcast_navigation(&messages).expect_err("mixed PRN rejection");

        assert_eq!(rejection.reason, GpsCnavNavigationRejectionReason::MixedPrn);
        assert_eq!(rejection.expected_prn, Some(17));
        assert_eq!(rejection.incoming_prn, Some(18));
        assert_eq!(rejection.message_type, Some(11));
    }
}
