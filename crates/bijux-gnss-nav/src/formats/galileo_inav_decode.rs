#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use crate::orbits::galileo::{
    GalileoClockCorrection, GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags,
    GalileoSignalHealth, GalileoSystemTime,
};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GalileoInavWord {
    Ephemeris1(GalileoInavEphemeris1Word),
    Ephemeris2(GalileoInavEphemeris2Word),
    Ephemeris3(GalileoInavEphemeris3Word),
    Clock(GalileoInavClockWord),
    Status(GalileoInavStatusWord),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoInavEphemeris1Word {
    pub iodnav: u16,
    pub toe_s: f64,
    pub m0: f64,
    pub e: f64,
    pub sqrt_a: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoInavEphemeris2Word {
    pub iodnav: u16,
    pub omega0: f64,
    pub i0: f64,
    pub w: f64,
    pub idot: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoInavEphemeris3Word {
    pub iodnav: u16,
    pub omegadot: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub sisa_e1_e5b: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoInavClockWord {
    pub iodnav: u16,
    pub svid: u8,
    pub cic: f64,
    pub cis: f64,
    pub clock: GalileoClockCorrection,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoInavStatusWord {
    pub ionosphere: GalileoIonosphericCorrection,
    pub signal_health: GalileoSignalHealth,
    pub gst: GalileoSystemTime,
    pub bgd_e1_e5a_s: f64,
    pub bgd_e1_e5b_s: f64,
}

pub fn decode_galileo_inav_word(payload: u128) -> Option<GalileoInavWord> {
    match inav_word_type(payload) {
        1 => Some(GalileoInavWord::Ephemeris1(decode_galileo_inav_ephemeris_1_word(payload)?)),
        2 => Some(GalileoInavWord::Ephemeris2(decode_galileo_inav_ephemeris_2_word(payload)?)),
        3 => Some(GalileoInavWord::Ephemeris3(decode_galileo_inav_ephemeris_3_word(payload)?)),
        4 => Some(GalileoInavWord::Clock(decode_galileo_inav_clock_word(payload)?)),
        5 => Some(GalileoInavWord::Status(decode_galileo_inav_status_word(payload)?)),
        _ => None,
    }
}

pub fn decode_galileo_inav_word_bytes(bytes: &[u8; 16]) -> Option<GalileoInavWord> {
    decode_galileo_inav_word(u128::from_be_bytes(*bytes))
}

pub fn decode_galileo_inav_word_hex(hex: &str) -> Option<GalileoInavWord> {
    let bytes = hex::decode(hex).ok()?;
    let payload = bytes.try_into().ok().map(u128::from_be_bytes)?;
    decode_galileo_inav_word(payload)
}

pub fn decode_galileo_inav_ephemeris_1_word(payload: u128) -> Option<GalileoInavEphemeris1Word> {
    (inav_word_type(payload) == 1).then(|| GalileoInavEphemeris1Word {
        iodnav: get_bits(payload, 7, 10) as u16,
        toe_s: get_bits(payload, 17, 14) as f64 * 60.0,
        m0: signed(get_bits(payload, 31, 32), 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        e: get_bits(payload, 63, 32) as f64 * 2f64.powi(-33),
        sqrt_a: get_bits(payload, 95, 32) as f64 * 2f64.powi(-19),
    })
}

pub fn decode_galileo_inav_ephemeris_2_word(payload: u128) -> Option<GalileoInavEphemeris2Word> {
    (inav_word_type(payload) == 2).then(|| GalileoInavEphemeris2Word {
        iodnav: get_bits(payload, 7, 10) as u16,
        omega0: signed(get_bits(payload, 17, 32), 32) as f64
            * 2f64.powi(-31)
            * std::f64::consts::PI,
        i0: signed(get_bits(payload, 49, 32), 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        w: signed(get_bits(payload, 81, 32), 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        idot: signed(get_bits(payload, 113, 14), 14) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
    })
}

pub fn decode_galileo_inav_ephemeris_3_word(payload: u128) -> Option<GalileoInavEphemeris3Word> {
    (inav_word_type(payload) == 3).then(|| GalileoInavEphemeris3Word {
        iodnav: get_bits(payload, 7, 10) as u16,
        omegadot: signed(get_bits(payload, 17, 24), 24) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
        delta_n: signed(get_bits(payload, 41, 16), 16) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
        cuc: signed(get_bits(payload, 57, 16), 16) as f64 * 2f64.powi(-29),
        cus: signed(get_bits(payload, 73, 16), 16) as f64 * 2f64.powi(-29),
        crc: signed(get_bits(payload, 89, 16), 16) as f64 * 2f64.powi(-5),
        crs: signed(get_bits(payload, 105, 16), 16) as f64 * 2f64.powi(-5),
        sisa_e1_e5b: get_bits(payload, 121, 8) as u8,
    })
}

pub fn decode_galileo_inav_clock_word(payload: u128) -> Option<GalileoInavClockWord> {
    (inav_word_type(payload) == 4).then(|| GalileoInavClockWord {
        iodnav: get_bits(payload, 7, 10) as u16,
        svid: get_bits(payload, 17, 6) as u8,
        cic: signed(get_bits(payload, 23, 16), 16) as f64 * 2f64.powi(-29),
        cis: signed(get_bits(payload, 39, 16), 16) as f64 * 2f64.powi(-29),
        clock: GalileoClockCorrection {
            t0c_s: get_bits(payload, 55, 14) as f64 * 60.0,
            af0: signed(get_bits(payload, 69, 31), 31) as f64 * 2f64.powi(-34),
            af1: signed(get_bits(payload, 100, 21), 21) as f64 * 2f64.powi(-46),
            af2: signed(get_bits(payload, 121, 6), 6) as f64 * 2f64.powi(-59),
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
    })
}

pub fn decode_galileo_inav_status_word(payload: u128) -> Option<GalileoInavStatusWord> {
    (inav_word_type(payload) == 5).then(|| GalileoInavStatusWord {
        ionosphere: GalileoIonosphericCorrection {
            ai0: get_bits(payload, 7, 11) as f64 * 2f64.powi(-2),
            ai1: signed(get_bits(payload, 18, 11), 11) as f64 * 2f64.powi(-8),
            ai2: signed(get_bits(payload, 29, 14), 14) as f64 * 2f64.powi(-15),
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: get_bits(payload, 43, 1) != 0,
                region_2: get_bits(payload, 44, 1) != 0,
                region_3: get_bits(payload, 45, 1) != 0,
                region_4: get_bits(payload, 46, 1) != 0,
                region_5: get_bits(payload, 47, 1) != 0,
            },
        },
        signal_health: GalileoSignalHealth {
            e5b_signal_health: get_bits(payload, 68, 2) as u8,
            e1b_signal_health: get_bits(payload, 70, 2) as u8,
            e5b_data_valid: get_bits(payload, 72, 1) == 0,
            e1b_data_valid: get_bits(payload, 73, 1) == 0,
        },
        gst: GalileoSystemTime {
            week: get_bits(payload, 74, 12) as u16,
            tow_s: get_bits(payload, 86, 20) as u32,
        },
        bgd_e1_e5a_s: signed(get_bits(payload, 48, 10), 10) as f64 * 2f64.powi(-32),
        bgd_e1_e5b_s: signed(get_bits(payload, 58, 10), 10) as f64 * 2f64.powi(-32),
    })
}

pub fn inav_word_type(payload: u128) -> u8 {
    get_bits(payload, 1, 6) as u8
}

fn get_bits(payload: u128, start: usize, len: usize) -> u128 {
    let shift = 128 - (start - 1) - len;
    (payload >> shift) & ((1_u128 << len) - 1)
}

fn signed(value: u128, bits: usize) -> i64 {
    let shift = 128 - bits;
    ((value << shift) as i128 >> shift) as i64
}
