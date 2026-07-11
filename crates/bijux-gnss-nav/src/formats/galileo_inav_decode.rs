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
