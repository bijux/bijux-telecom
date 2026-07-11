#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::galileo::{
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime,
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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GalileoInavBatchRejectionReason {
    DecodeFailure,
    IodnavMismatch,
    SatelliteIdMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoInavBatchRejection {
    pub word_type: u8,
    pub reason: GalileoInavBatchRejectionReason,
    pub existing_iodnav: Option<u16>,
    pub incoming_iodnav: Option<u16>,
    pub existing_svid: Option<u8>,
    pub incoming_svid: Option<u8>,
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

pub fn decode_galileo_broadcast_navigation_data(
    words: &[GalileoInavWord],
) -> Result<Option<GalileoBroadcastNavigationData>, GalileoInavBatchRejection> {
    let mut builder = GalileoInavBatchBuilder::default();
    for word in words {
        builder.merge(word)?;
    }
    Ok(builder.try_build())
}

pub fn decode_galileo_broadcast_navigation_data_payloads(
    payloads: &[u128],
) -> Result<Option<GalileoBroadcastNavigationData>, GalileoInavBatchRejection> {
    let words = payloads
        .iter()
        .copied()
        .map(decode_galileo_inav_word)
        .collect::<Option<Vec<_>>>()
        .ok_or(GalileoInavBatchRejection {
            word_type: 0,
            reason: GalileoInavBatchRejectionReason::DecodeFailure,
            existing_iodnav: None,
            incoming_iodnav: None,
            existing_svid: None,
            incoming_svid: None,
        })?;
    decode_galileo_broadcast_navigation_data(&words)
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

#[derive(Debug, Default, Clone)]
struct GalileoInavBatchBuilder {
    iodnav: Option<u16>,
    svid: Option<u8>,
    toe_s: Option<f64>,
    m0: Option<f64>,
    e: Option<f64>,
    sqrt_a: Option<f64>,
    omega0: Option<f64>,
    i0: Option<f64>,
    w: Option<f64>,
    idot: Option<f64>,
    omegadot: Option<f64>,
    delta_n: Option<f64>,
    cuc: Option<f64>,
    cus: Option<f64>,
    crc: Option<f64>,
    crs: Option<f64>,
    cic: Option<f64>,
    cis: Option<f64>,
    sisa_e1_e5b: Option<u8>,
    clock: Option<GalileoClockCorrection>,
    signal_health: Option<GalileoSignalHealth>,
    gst: Option<GalileoSystemTime>,
    ionosphere: Option<GalileoIonosphericCorrection>,
    bgd_e1_e5a_s: Option<f64>,
    bgd_e1_e5b_s: Option<f64>,
}

impl GalileoInavBatchBuilder {
    fn merge(&mut self, word: &GalileoInavWord) -> Result<(), GalileoInavBatchRejection> {
        self.check_consistency(word)?;

        match word {
            GalileoInavWord::Ephemeris1(word) => {
                self.iodnav = Some(word.iodnav);
                self.toe_s = Some(word.toe_s);
                self.m0 = Some(word.m0);
                self.e = Some(word.e);
                self.sqrt_a = Some(word.sqrt_a);
            }
            GalileoInavWord::Ephemeris2(word) => {
                self.iodnav = Some(word.iodnav);
                self.omega0 = Some(word.omega0);
                self.i0 = Some(word.i0);
                self.w = Some(word.w);
                self.idot = Some(word.idot);
            }
            GalileoInavWord::Ephemeris3(word) => {
                self.iodnav = Some(word.iodnav);
                self.omegadot = Some(word.omegadot);
                self.delta_n = Some(word.delta_n);
                self.cuc = Some(word.cuc);
                self.cus = Some(word.cus);
                self.crc = Some(word.crc);
                self.crs = Some(word.crs);
                self.sisa_e1_e5b = Some(word.sisa_e1_e5b);
            }
            GalileoInavWord::Clock(word) => {
                self.iodnav = Some(word.iodnav);
                self.svid = Some(word.svid);
                self.cic = Some(word.cic);
                self.cis = Some(word.cis);
                self.clock = Some(word.clock);
            }
            GalileoInavWord::Status(word) => {
                self.signal_health = Some(word.signal_health);
                self.gst = Some(word.gst);
                self.ionosphere = Some(word.ionosphere.clone());
                self.bgd_e1_e5a_s = Some(word.bgd_e1_e5a_s);
                self.bgd_e1_e5b_s = Some(word.bgd_e1_e5b_s);
            }
        }

        Ok(())
    }

    fn try_build(&self) -> Option<GalileoBroadcastNavigationData> {
        let iodnav = self.iodnav?;
        let svid = self.svid?;
        let sat = SatId { constellation: Constellation::Galileo, prn: svid };
        let bgd_e1_e5a_s = self.bgd_e1_e5a_s?;
        let bgd_e1_e5b_s = self.bgd_e1_e5b_s?;
        let mut clock = self.clock?;
        clock.bgd_e1_e5a_s = bgd_e1_e5a_s;
        clock.bgd_e1_e5b_s = bgd_e1_e5b_s;

        Some(GalileoBroadcastNavigationData {
            sat,
            iodnav,
            gst: self.gst?,
            sisa_e1_e5b: self.sisa_e1_e5b?,
            signal_health: self.signal_health?,
            clock,
            ephemeris: GalileoEphemeris {
                sat,
                iodnav,
                toe_s: self.toe_s?,
                sqrt_a: self.sqrt_a?,
                e: self.e?,
                i0: self.i0?,
                idot: self.idot?,
                omega0: self.omega0?,
                omegadot: self.omegadot?,
                w: self.w?,
                m0: self.m0?,
                delta_n: self.delta_n?,
                cuc: self.cuc?,
                cus: self.cus?,
                crc: self.crc?,
                crs: self.crs?,
                cic: self.cic?,
                cis: self.cis?,
            },
            ionosphere: self.ionosphere.clone()?,
        })
    }

    fn check_consistency(&self, word: &GalileoInavWord) -> Result<(), GalileoInavBatchRejection> {
        if let Some(incoming_iodnav) = incoming_iodnav(word) {
            if let Some(existing_iodnav) = self.iodnav {
                if existing_iodnav != incoming_iodnav {
                    return Err(GalileoInavBatchRejection {
                        word_type: incoming_word_type(word),
                        reason: GalileoInavBatchRejectionReason::IodnavMismatch,
                        existing_iodnav: Some(existing_iodnav),
                        incoming_iodnav: Some(incoming_iodnav),
                        existing_svid: self.svid,
                        incoming_svid: incoming_svid(word),
                    });
                }
            }
        }

        if let Some(incoming_svid) = incoming_svid(word) {
            if let Some(existing_svid) = self.svid {
                if existing_svid != incoming_svid {
                    return Err(GalileoInavBatchRejection {
                        word_type: incoming_word_type(word),
                        reason: GalileoInavBatchRejectionReason::SatelliteIdMismatch,
                        existing_iodnav: self.iodnav,
                        incoming_iodnav: incoming_iodnav(word),
                        existing_svid: Some(existing_svid),
                        incoming_svid: Some(incoming_svid),
                    });
                }
            }
        }

        Ok(())
    }
}

fn incoming_iodnav(word: &GalileoInavWord) -> Option<u16> {
    match word {
        GalileoInavWord::Ephemeris1(word) => Some(word.iodnav),
        GalileoInavWord::Ephemeris2(word) => Some(word.iodnav),
        GalileoInavWord::Ephemeris3(word) => Some(word.iodnav),
        GalileoInavWord::Clock(word) => Some(word.iodnav),
        GalileoInavWord::Status(_) => None,
    }
}

fn incoming_svid(word: &GalileoInavWord) -> Option<u8> {
    match word {
        GalileoInavWord::Clock(word) => Some(word.svid),
        _ => None,
    }
}

fn incoming_word_type(word: &GalileoInavWord) -> u8 {
    match word {
        GalileoInavWord::Ephemeris1(_) => 1,
        GalileoInavWord::Ephemeris2(_) => 2,
        GalileoInavWord::Ephemeris3(_) => 3,
        GalileoInavWord::Clock(_) => 4,
        GalileoInavWord::Status(_) => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        decode_galileo_broadcast_navigation_data, decode_galileo_broadcast_navigation_data_payloads,
        decode_galileo_inav_clock_word, decode_galileo_inav_ephemeris_1_word,
        decode_galileo_inav_ephemeris_2_word, decode_galileo_inav_ephemeris_3_word,
        decode_galileo_inav_status_word, decode_galileo_inav_word, GalileoInavBatchRejectionReason,
    };
    use bijux_gnss_core::api::Constellation;

    fn set_bits(payload: &mut u128, start: usize, len: usize, value: u128) {
        let shift = 128 - (start - 1) - len;
        let mask = ((1_u128 << len) - 1) << shift;
        *payload &= !mask;
        *payload |= (value << shift) & mask;
    }

    fn encode_signed(value: i64, bits: usize) -> u128 {
        let mask = (1_u128 << bits) - 1;
        (value as u128) & mask
    }

    fn sample_batch_payloads() -> [u128; 5] {
        let iodnav = 0x1A5_u16;
        let mut word_1 = 0_u128;
        let mut word_2 = 0_u128;
        let mut word_3 = 0_u128;
        let mut word_4 = 0_u128;
        let mut word_5 = 0_u128;

        set_bits(&mut word_1, 1, 6, 1);
        set_bits(&mut word_1, 7, 10, iodnav as u128);
        set_bits(&mut word_1, 17, 14, 2_345);
        set_bits(&mut word_1, 31, 32, encode_signed(-0x1020_304_i64, 32));
        set_bits(&mut word_1, 63, 32, 0x0123_4567);
        set_bits(&mut word_1, 95, 32, 0x0987_6543);

        set_bits(&mut word_2, 1, 6, 2);
        set_bits(&mut word_2, 7, 10, iodnav as u128);
        set_bits(&mut word_2, 17, 32, encode_signed(0x1020_3040_i64, 32));
        set_bits(&mut word_2, 49, 32, encode_signed(-0x1234_567_i64, 32));
        set_bits(&mut word_2, 81, 32, encode_signed(0x2345_6789_i64, 32));
        set_bits(&mut word_2, 113, 14, encode_signed(-0x03A5_i64, 14));

        set_bits(&mut word_3, 1, 6, 3);
        set_bits(&mut word_3, 7, 10, iodnav as u128);
        set_bits(&mut word_3, 17, 24, encode_signed(-0x04_321_i64, 24));
        set_bits(&mut word_3, 41, 16, encode_signed(0x0F0F_i64, 16));
        set_bits(&mut word_3, 57, 16, encode_signed(-321_i64, 16));
        set_bits(&mut word_3, 73, 16, encode_signed(654_i64, 16));
        set_bits(&mut word_3, 89, 16, encode_signed(1_111_i64, 16));
        set_bits(&mut word_3, 105, 16, encode_signed(-2_222_i64, 16));
        set_bits(&mut word_3, 121, 8, 77);

        set_bits(&mut word_4, 1, 6, 4);
        set_bits(&mut word_4, 7, 10, iodnav as u128);
        set_bits(&mut word_4, 17, 6, 19);
        set_bits(&mut word_4, 23, 16, encode_signed(-123_i64, 16));
        set_bits(&mut word_4, 39, 16, encode_signed(456_i64, 16));
        set_bits(&mut word_4, 55, 14, 1_111);
        set_bits(&mut word_4, 69, 31, encode_signed(-0x1ABCD_i64, 31));
        set_bits(&mut word_4, 100, 21, encode_signed(0x01234_i64, 21));
        set_bits(&mut word_4, 121, 6, encode_signed(-0x09_i64, 6));

        set_bits(&mut word_5, 1, 6, 5);
        set_bits(&mut word_5, 7, 11, 211);
        set_bits(&mut word_5, 18, 11, encode_signed(-87_i64, 11));
        set_bits(&mut word_5, 29, 14, encode_signed(0x00A5_i64, 14));
        set_bits(&mut word_5, 43, 1, 1);
        set_bits(&mut word_5, 44, 1, 1);
        set_bits(&mut word_5, 46, 1, 1);
        set_bits(&mut word_5, 48, 10, encode_signed(-12_i64, 10));
        set_bits(&mut word_5, 58, 10, encode_signed(23_i64, 10));
        set_bits(&mut word_5, 68, 2, 0);
        set_bits(&mut word_5, 70, 2, 2);
        set_bits(&mut word_5, 72, 1, 0);
        set_bits(&mut word_5, 73, 1, 0);
        set_bits(&mut word_5, 74, 12, 2_222);
        set_bits(&mut word_5, 86, 20, 456_789);

        [word_1, word_2, word_3, word_4, word_5]
    }

    #[test]
    fn ephemeris_word_1_decodes_toe_and_kepler_terms() {
        let mut payload = 0_u128;
        let iodnav = 0x1A5_u16;
        let toe_raw = 4_321_u16;
        let m0_raw = -0x1234_567_i64;
        let e_raw = 0x2345_6789_u32;
        let sqrt_a_raw = 0x1234_5678_u32;

        set_bits(&mut payload, 1, 6, 1);
        set_bits(&mut payload, 7, 10, iodnav as u128);
        set_bits(&mut payload, 17, 14, toe_raw as u128);
        set_bits(&mut payload, 31, 32, encode_signed(m0_raw, 32));
        set_bits(&mut payload, 63, 32, e_raw as u128);
        set_bits(&mut payload, 95, 32, sqrt_a_raw as u128);

        let word = decode_galileo_inav_ephemeris_1_word(payload).expect("word type 1");

        assert_eq!(word.iodnav, iodnav);
        assert!((word.toe_s - toe_raw as f64 * 60.0).abs() < f64::EPSILON);
        assert!((word.m0 - m0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < 1e-12);
        assert!((word.e - e_raw as f64 * 2f64.powi(-33)).abs() < f64::EPSILON);
        assert!((word.sqrt_a - sqrt_a_raw as f64 * 2f64.powi(-19)).abs() < f64::EPSILON);
    }

    #[test]
    fn ephemeris_words_2_and_3_decode_orbit_corrections() {
        let mut word_2 = 0_u128;
        let mut word_3 = 0_u128;

        let iodnav = 0x155_u16;
        let omega0_raw = 0x1234_5678_u32 as i64;
        let i0_raw = -0x1111_222_i64;
        let w_raw = 0x3141_5926_u32 as i64;
        let idot_raw = -0x0A55_i64;
        let omegadot_raw = -0x54_321_i64;
        let delta_n_raw = 0x1234_i64;
        let cuc_raw = -777_i64;
        let cus_raw = 888_i64;
        let crc_raw = 1_234_i64;
        let crs_raw = -1_111_i64;
        let sisa_e1_e5b = 0xAB_u8;

        set_bits(&mut word_2, 1, 6, 2);
        set_bits(&mut word_2, 7, 10, iodnav as u128);
        set_bits(&mut word_2, 17, 32, encode_signed(omega0_raw, 32));
        set_bits(&mut word_2, 49, 32, encode_signed(i0_raw, 32));
        set_bits(&mut word_2, 81, 32, encode_signed(w_raw, 32));
        set_bits(&mut word_2, 113, 14, encode_signed(idot_raw, 14));

        set_bits(&mut word_3, 1, 6, 3);
        set_bits(&mut word_3, 7, 10, iodnav as u128);
        set_bits(&mut word_3, 17, 24, encode_signed(omegadot_raw, 24));
        set_bits(&mut word_3, 41, 16, encode_signed(delta_n_raw, 16));
        set_bits(&mut word_3, 57, 16, encode_signed(cuc_raw, 16));
        set_bits(&mut word_3, 73, 16, encode_signed(cus_raw, 16));
        set_bits(&mut word_3, 89, 16, encode_signed(crc_raw, 16));
        set_bits(&mut word_3, 105, 16, encode_signed(crs_raw, 16));
        set_bits(&mut word_3, 121, 8, sisa_e1_e5b as u128);

        let word_2 = decode_galileo_inav_ephemeris_2_word(word_2).expect("word type 2");
        let word_3 = decode_galileo_inav_ephemeris_3_word(word_3).expect("word type 3");

        assert_eq!(word_2.iodnav, iodnav);
        assert!(
            (word_2.omega0 - omega0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs()
                < 1e-12
        );
        assert!((word_2.i0 - i0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < 1e-12);
        assert!((word_2.w - w_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < 1e-12);
        assert!(
            (word_2.idot - idot_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < 1e-15
        );

        assert_eq!(word_3.iodnav, iodnav);
        assert!(
            (word_3.omegadot - omegadot_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < 1e-15
        );
        assert!(
            (word_3.delta_n - delta_n_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < 1e-15
        );
        assert!((word_3.cuc - cuc_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((word_3.cus - cus_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((word_3.crc - crc_raw as f64 * 2f64.powi(-5)).abs() < f64::EPSILON);
        assert!((word_3.crs - crs_raw as f64 * 2f64.powi(-5)).abs() < f64::EPSILON);
        assert_eq!(word_3.sisa_e1_e5b, sisa_e1_e5b);
    }

    #[test]
    fn clock_word_decodes_satellite_id_and_clock_terms() {
        let mut payload = 0_u128;
        let iodnav = 0x12C_u16;
        let svid = 24_u8;
        let cic_raw = -432_i64;
        let cis_raw = 321_i64;
        let t0c_raw = 1_234_u16;
        let af0_raw = -0x12_345_i64;
        let af1_raw = 0x1A_55_u32 as i64;
        let af2_raw = -0x11_i64;

        set_bits(&mut payload, 1, 6, 4);
        set_bits(&mut payload, 7, 10, iodnav as u128);
        set_bits(&mut payload, 17, 6, svid as u128);
        set_bits(&mut payload, 23, 16, encode_signed(cic_raw, 16));
        set_bits(&mut payload, 39, 16, encode_signed(cis_raw, 16));
        set_bits(&mut payload, 55, 14, t0c_raw as u128);
        set_bits(&mut payload, 69, 31, encode_signed(af0_raw, 31));
        set_bits(&mut payload, 100, 21, encode_signed(af1_raw, 21));
        set_bits(&mut payload, 121, 6, encode_signed(af2_raw, 6));

        let word = decode_galileo_inav_clock_word(payload).expect("word type 4");

        assert_eq!(word.iodnav, iodnav);
        assert_eq!(word.svid, svid);
        assert!((word.cic - cic_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((word.cis - cis_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((word.clock.t0c_s - t0c_raw as f64 * 60.0).abs() < f64::EPSILON);
        assert!((word.clock.af0 - af0_raw as f64 * 2f64.powi(-34)).abs() < f64::EPSILON);
        assert!((word.clock.af1 - af1_raw as f64 * 2f64.powi(-46)).abs() < f64::EPSILON);
        assert!((word.clock.af2 - af2_raw as f64 * 2f64.powi(-59)).abs() < f64::EPSILON);
    }

    #[test]
    fn status_word_decodes_gst_health_and_group_delay() {
        let mut payload = 0_u128;
        let ai0_raw = 321_u16;
        let ai1_raw = -211_i64;
        let ai2_raw = 0x123_i64;
        let bgd_e1_e5a_raw = -44_i64;
        let bgd_e1_e5b_raw = 55_i64;
        let week = 2_048_u16;
        let tow = 345_678_u32;

        set_bits(&mut payload, 1, 6, 5);
        set_bits(&mut payload, 7, 11, ai0_raw as u128);
        set_bits(&mut payload, 18, 11, encode_signed(ai1_raw, 11));
        set_bits(&mut payload, 29, 14, encode_signed(ai2_raw, 14));
        set_bits(&mut payload, 43, 1, 1);
        set_bits(&mut payload, 45, 1, 1);
        set_bits(&mut payload, 47, 1, 1);
        set_bits(&mut payload, 48, 10, encode_signed(bgd_e1_e5a_raw, 10));
        set_bits(&mut payload, 58, 10, encode_signed(bgd_e1_e5b_raw, 10));
        set_bits(&mut payload, 68, 2, 2);
        set_bits(&mut payload, 70, 2, 1);
        set_bits(&mut payload, 72, 1, 0);
        set_bits(&mut payload, 73, 1, 1);
        set_bits(&mut payload, 74, 12, week as u128);
        set_bits(&mut payload, 86, 20, tow as u128);

        let word = decode_galileo_inav_status_word(payload).expect("word type 5");

        assert!((word.ionosphere.ai0 - ai0_raw as f64 * 2f64.powi(-2)).abs() < f64::EPSILON);
        assert!((word.ionosphere.ai1 - ai1_raw as f64 * 2f64.powi(-8)).abs() < f64::EPSILON);
        assert!((word.ionosphere.ai2 - ai2_raw as f64 * 2f64.powi(-15)).abs() < f64::EPSILON);
        assert!(word.ionosphere.disturbance_flags.region_1);
        assert!(!word.ionosphere.disturbance_flags.region_2);
        assert!(word.ionosphere.disturbance_flags.region_3);
        assert!(!word.ionosphere.disturbance_flags.region_4);
        assert!(word.ionosphere.disturbance_flags.region_5);
        assert!((word.bgd_e1_e5a_s - bgd_e1_e5a_raw as f64 * 2f64.powi(-32)).abs() < f64::EPSILON);
        assert!((word.bgd_e1_e5b_s - bgd_e1_e5b_raw as f64 * 2f64.powi(-32)).abs() < f64::EPSILON);
        assert_eq!(word.signal_health.e5b_signal_health, 2);
        assert_eq!(word.signal_health.e1b_signal_health, 1);
        assert!(word.signal_health.e5b_data_valid);
        assert!(!word.signal_health.e1b_data_valid);
        assert_eq!(word.gst.week, week);
        assert_eq!(word.gst.tow_s, tow);
    }

    #[test]
    fn assembled_batch_exposes_time_satellite_health_ephemeris_and_clock_fields() {
        let words = sample_batch_payloads()
            .into_iter()
            .map(|payload| decode_galileo_inav_word(payload).expect("decoded word"))
            .collect::<Vec<_>>();

        let nav = decode_galileo_broadcast_navigation_data(&words)
            .expect("consistent batch")
            .expect("complete batch");

        assert_eq!(nav.sat.constellation, Constellation::Galileo);
        assert_eq!(nav.sat.prn, 19);
        assert_eq!(nav.iodnav, 0x1A5);
        assert_eq!(nav.gst.week, 2_222);
        assert_eq!(nav.gst.tow_s, 456_789);
        assert_eq!(nav.signal_health.e5b_signal_health, 0);
        assert_eq!(nav.signal_health.e1b_signal_health, 2);
        assert!(nav.signal_health.e5b_data_valid);
        assert!(nav.signal_health.e1b_data_valid);
        assert_eq!(nav.sisa_e1_e5b, 77);
        assert_eq!(nav.ephemeris.sat, nav.sat);
        assert_eq!(nav.ephemeris.iodnav, nav.iodnav);
        assert!((nav.clock.bgd_e1_e5a_s - (-12_i64) as f64 * 2f64.powi(-32)).abs() < f64::EPSILON);
        assert!((nav.clock.bgd_e1_e5b_s - 23_f64 * 2f64.powi(-32)).abs() < f64::EPSILON);
    }

    #[test]
    fn payload_batch_decoder_rejects_mixed_issue_numbers() {
        let mut payloads = sample_batch_payloads();
        set_bits(&mut payloads[2], 7, 10, 0x155);

        let rejection = decode_galileo_broadcast_navigation_data_payloads(&payloads)
            .expect_err("mixed IODnav batch");

        assert_eq!(rejection.reason, GalileoInavBatchRejectionReason::IodnavMismatch);
        assert_eq!(rejection.word_type, 3);
        assert_eq!(rejection.existing_iodnav, Some(0x1A5));
        assert_eq!(rejection.incoming_iodnav, Some(0x155));
    }

    #[test]
    fn payload_batch_decoder_reports_decode_failures() {
        let mut payloads = sample_batch_payloads();
        set_bits(&mut payloads[4], 1, 6, 63);

        let rejection = decode_galileo_broadcast_navigation_data_payloads(&payloads)
            .expect_err("unsupported word type");

        assert_eq!(rejection.reason, GalileoInavBatchRejectionReason::DecodeFailure);
        assert_eq!(rejection.word_type, 0);
    }
}
