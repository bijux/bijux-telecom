#![allow(missing_docs)]
#![allow(dead_code)]

use crate::formats::lnav_bits::GpsWord;
use crate::orbits::gps::GpsEphemeris;
use bijux_gnss_core::api::{Constellation, SatId};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaLnavSubframe1Clock {
    pub week: u16,
    pub iodc: u16,
    pub sv_health: u8,
    pub toc_s: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaLnavSubframe2Orbit {
    pub iode: u8,
    pub crs: f64,
    pub delta_n: f64,
    pub m0: f64,
    pub cuc: f64,
    pub e: f64,
    pub cus: f64,
    pub sqrt_a: f64,
    pub toe_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaLnavSubframe3Orbit {
    pub iode: u8,
    pub cic: f64,
    pub omega0: f64,
    pub cis: f64,
    pub i0: f64,
    pub crc: f64,
    pub w: f64,
    pub omegadot: f64,
    pub idot: f64,
}

pub fn decode_subframe1_clock(words: &[GpsWord]) -> Option<GpsL1CaLnavSubframe1Clock> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let week = get_bits(w3, 1, 10) as u16;
    let sv_health = get_bits(w3, 17, 6) as u8;
    let iodc_msb = get_bits(w3, 23, 2) as u16;
    let iodc_lsb = get_bits(w8, 1, 8) as u16;
    let iodc = (iodc_msb << 8) | iodc_lsb;
    let tgd = signed(get_bits(w7, 17, 8), 8) as f64 * 2f64.powi(-31);
    let toc_s = get_bits(w8, 9, 16) as f64 * 16.0;
    let af2 = signed(get_bits(w9, 1, 8), 8) as f64 * 2f64.powi(-55);
    let af1 = signed(get_bits(w9, 9, 16), 16) as f64 * 2f64.powi(-43);
    let af0 = signed(get_bits(w10, 1, 22), 22) as f64 * 2f64.powi(-31);

    Some(GpsL1CaLnavSubframe1Clock { week, iodc, sv_health, toc_s, af0, af1, af2, tgd })
}

pub fn parse_subframe1(words: &[GpsWord]) -> Option<EphemerisPart> {
    let clock = decode_subframe1_clock(words)?;

    Some(EphemerisPart {
        iodc: Some(clock.iodc),
        iode: None,
        week: Some(clock.week),
        sv_health: Some(clock.sv_health),
        toe_s: None,
        toc_s: Some(clock.toc_s),
        sqrt_a: None,
        e: None,
        i0: None,
        idot: None,
        omega0: None,
        omegadot: None,
        w: None,
        m0: None,
        delta_n: None,
        cuc: None,
        cus: None,
        crc: None,
        crs: None,
        cic: None,
        cis: None,
        af0: Some(clock.af0),
        af1: Some(clock.af1),
        af2: Some(clock.af2),
        tgd: Some(clock.tgd),
    })
}

pub fn parse_subframe2(words: &[GpsWord]) -> Option<EphemerisPart> {
    let orbit = decode_subframe2_orbit(words)?;

    Some(EphemerisPart {
        iodc: None,
        iode: Some(orbit.iode),
        week: None,
        sv_health: None,
        toe_s: Some(orbit.toe_s),
        toc_s: None,
        sqrt_a: Some(orbit.sqrt_a),
        e: Some(orbit.e),
        i0: None,
        idot: None,
        omega0: None,
        omegadot: None,
        w: None,
        m0: Some(orbit.m0),
        delta_n: Some(orbit.delta_n),
        cuc: Some(orbit.cuc),
        cus: Some(orbit.cus),
        crc: None,
        crs: Some(orbit.crs),
        cic: None,
        cis: None,
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

pub fn decode_subframe2_orbit(words: &[GpsWord]) -> Option<GpsL1CaLnavSubframe2Orbit> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w4 = words[3].data;
    let w5 = words[4].data;
    let w6 = words[5].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let iode = get_bits(w3, 1, 8) as u8;
    let crs = signed(get_bits(w3, 9, 16), 16) as f64 * 2f64.powi(-5);
    let delta_n = signed(get_bits(w4, 1, 16), 16) as f64 * 2f64.powi(-43) * std::f64::consts::PI;
    let m0 = signed((get_bits(w4, 17, 8) << 24) | get_bits(w5, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let cuc = signed(get_bits(w6, 1, 16), 16) as f64 * 2f64.powi(-29);
    let e = (get_bits(w6, 17, 8) << 24 | get_bits(w7, 1, 24)) as f64 * 2f64.powi(-33);
    let cus = signed(get_bits(w8, 1, 16), 16) as f64 * 2f64.powi(-29);
    let sqrt_a = (get_bits(w8, 17, 8) << 24 | get_bits(w9, 1, 24)) as f64 * 2f64.powi(-19);
    let toe = get_bits(w10, 1, 16) as f64 * 16.0;

    Some(GpsL1CaLnavSubframe2Orbit {
        iode,
        crs,
        delta_n,
        m0,
        cuc,
        e,
        cus,
        sqrt_a,
        toe_s: toe,
    })
}

pub fn parse_subframe3(words: &[GpsWord]) -> Option<EphemerisPart> {
    let orbit = decode_subframe3_orbit(words)?;

    Some(EphemerisPart {
        iodc: None,
        iode: Some(orbit.iode),
        week: None,
        sv_health: None,
        toe_s: None,
        toc_s: None,
        sqrt_a: None,
        e: None,
        i0: Some(orbit.i0),
        idot: Some(orbit.idot),
        omega0: Some(orbit.omega0),
        omegadot: Some(orbit.omegadot),
        w: Some(orbit.w),
        m0: None,
        delta_n: None,
        cuc: None,
        cus: None,
        crc: Some(orbit.crc),
        crs: None,
        cic: Some(orbit.cic),
        cis: Some(orbit.cis),
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

pub fn decode_subframe3_orbit(words: &[GpsWord]) -> Option<GpsL1CaLnavSubframe3Orbit> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w4 = words[3].data;
    let w5 = words[4].data;
    let w6 = words[5].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let cic = signed(get_bits(w3, 1, 16), 16) as f64 * 2f64.powi(-29);
    let omega0 = signed((get_bits(w3, 17, 8) << 24) | get_bits(w4, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let cis = signed(get_bits(w5, 1, 16), 16) as f64 * 2f64.powi(-29);
    let i0 = signed((get_bits(w5, 17, 8) << 24) | get_bits(w6, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let crc = signed(get_bits(w7, 1, 16), 16) as f64 * 2f64.powi(-5);
    let w = signed((get_bits(w7, 17, 8) << 24) | get_bits(w8, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let omegadot = signed(get_bits(w9, 1, 24), 24) as f64 * 2f64.powi(-43) * std::f64::consts::PI;
    let iode = get_bits(w10, 1, 8) as u8;
    let idot = signed(get_bits(w10, 9, 14), 14) as f64 * 2f64.powi(-43) * std::f64::consts::PI;

    Some(GpsL1CaLnavSubframe3Orbit {
        iode,
        cic,
        omega0,
        cis,
        i0,
        crc,
        w,
        omegadot,
        idot,
    })
}

pub fn get_bits(data: u32, start: usize, len: usize) -> u32 {
    let shift = 24 - (start - 1) - len;
    (data >> shift) & ((1_u32 << len) - 1)
}

fn signed(value: u32, bits: usize) -> i32 {
    let shift = 32 - bits;
    ((value << shift) as i32) >> shift
}

#[derive(Debug, Default, Clone)]
pub struct EphemerisPart {
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
    sv_health: Option<u8>,
    toe_s: Option<f64>,
    toc_s: Option<f64>,
    sqrt_a: Option<f64>,
    e: Option<f64>,
    i0: Option<f64>,
    idot: Option<f64>,
    omega0: Option<f64>,
    omegadot: Option<f64>,
    w: Option<f64>,
    m0: Option<f64>,
    delta_n: Option<f64>,
    cuc: Option<f64>,
    cus: Option<f64>,
    crc: Option<f64>,
    crs: Option<f64>,
    cic: Option<f64>,
    cis: Option<f64>,
    af0: Option<f64>,
    af1: Option<f64>,
    af2: Option<f64>,
    tgd: Option<f64>,
}

#[derive(Debug, Default, Clone)]
pub struct EphemerisBuilder {
    prn: u8,
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
    sv_health: Option<u8>,
    toe_s: Option<f64>,
    toc_s: Option<f64>,
    sqrt_a: Option<f64>,
    e: Option<f64>,
    i0: Option<f64>,
    idot: Option<f64>,
    omega0: Option<f64>,
    omegadot: Option<f64>,
    w: Option<f64>,
    m0: Option<f64>,
    delta_n: Option<f64>,
    cuc: Option<f64>,
    cus: Option<f64>,
    crc: Option<f64>,
    crs: Option<f64>,
    cic: Option<f64>,
    cis: Option<f64>,
    af0: Option<f64>,
    af1: Option<f64>,
    af2: Option<f64>,
    tgd: Option<f64>,
}

impl EphemerisBuilder {
    pub fn merge(&mut self, part: EphemerisPart) {
        if let Some(value) = part.iodc {
            self.iodc = Some(value);
        }
        if let Some(value) = part.iode {
            self.iode = Some(value);
        }
        if let Some(value) = part.week {
            self.week = Some(value);
        }
        if let Some(value) = part.sv_health {
            self.sv_health = Some(value);
        }
        if let Some(value) = part.toe_s {
            self.toe_s = Some(value);
        }
        if let Some(value) = part.toc_s {
            self.toc_s = Some(value);
        }
        if let Some(value) = part.sqrt_a {
            self.sqrt_a = Some(value);
        }
        if let Some(value) = part.e {
            self.e = Some(value);
        }
        if let Some(value) = part.i0 {
            self.i0 = Some(value);
        }
        if let Some(value) = part.idot {
            self.idot = Some(value);
        }
        if let Some(value) = part.omega0 {
            self.omega0 = Some(value);
        }
        if let Some(value) = part.omegadot {
            self.omegadot = Some(value);
        }
        if let Some(value) = part.w {
            self.w = Some(value);
        }
        if let Some(value) = part.m0 {
            self.m0 = Some(value);
        }
        if let Some(value) = part.delta_n {
            self.delta_n = Some(value);
        }
        if let Some(value) = part.cuc {
            self.cuc = Some(value);
        }
        if let Some(value) = part.cus {
            self.cus = Some(value);
        }
        if let Some(value) = part.crc {
            self.crc = Some(value);
        }
        if let Some(value) = part.crs {
            self.crs = Some(value);
        }
        if let Some(value) = part.cic {
            self.cic = Some(value);
        }
        if let Some(value) = part.cis {
            self.cis = Some(value);
        }
        if let Some(value) = part.af0 {
            self.af0 = Some(value);
        }
        if let Some(value) = part.af1 {
            self.af1 = Some(value);
        }
        if let Some(value) = part.af2 {
            self.af2 = Some(value);
        }
        if let Some(value) = part.tgd {
            self.tgd = Some(value);
        }
    }

    pub fn try_build(&self) -> Option<GpsEphemeris> {
        Some(GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: self.prn },
            iodc: self.iodc?,
            iode: self.iode?,
            week: self.week?,
            sv_health: self.sv_health?,
            toe_s: self.toe_s?,
            toc_s: self.toc_s?,
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
            af0: self.af0?,
            af1: self.af1?,
            af2: self.af2?,
            tgd: self.tgd?,
        })
    }
}

pub fn decode_subframe_hex(hex: &str) -> Option<Vec<u32>> {
    let bytes = hex::decode(hex).ok()?;
    if bytes.len() != 30 {
        return None;
    }
    let mut bits = Vec::with_capacity(240);
    for byte in bytes {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    let mut words = Vec::new();
    for w in 0..10 {
        let mut data = 0_u32;
        for b in 0..24 {
            let bit = bits[w * 24 + b];
            data = (data << 1) | bit as u32;
        }
        words.push(data);
    }
    Some(words)
}

pub fn decode_rawephem_hex(prn: u8, sub1: &str, sub2: &str, sub3: &str) -> Option<GpsEphemeris> {
    let w1 = decode_subframe_hex(sub1)?;
    let w2 = decode_subframe_hex(sub2)?;
    let w3 = decode_subframe_hex(sub3)?;
    let mut builder = EphemerisBuilder { prn, ..Default::default() };
    let words1: Vec<GpsWord> = w1
        .into_iter()
        .map(|data| GpsWord { data, parity_ok: true, d29_star: 0, d30_star: 0 })
        .collect();
    let words2: Vec<GpsWord> = w2
        .into_iter()
        .map(|data| GpsWord { data, parity_ok: true, d29_star: 0, d30_star: 0 })
        .collect();
    let words3: Vec<GpsWord> = w3
        .into_iter()
        .map(|data| GpsWord { data, parity_ok: true, d29_star: 0, d30_star: 0 })
        .collect();
    builder.merge(parse_subframe1(&words1)?);
    builder.merge(parse_subframe2(&words2)?);
    builder.merge(parse_subframe3(&words3)?);
    builder.try_build()
}

#[cfg(test)]
mod tests {
    use super::{decode_subframe1_clock, decode_subframe2_orbit, decode_subframe3_orbit, signed};
    use crate::formats::lnav_bits::GpsWord;

    fn set_bits(data: &mut u32, start: usize, len: usize, value: u32) {
        let shift = 24 - (start - 1) - len;
        let mask = ((1_u32 << len) - 1) << shift;
        *data &= !mask;
        *data |= (value << shift) & mask;
    }

    fn encode_signed(value: i32, bits: usize) -> u32 {
        let mask = (1_u32 << bits) - 1;
        (value as u32) & mask
    }

    #[test]
    fn signed_decodes_32_bit_negative_values_without_overflow() {
        assert_eq!(signed(0x8000_0000, 32), i32::MIN);
        assert_eq!(signed(0xFFFF_FFFF, 32), -1);
    }

    #[test]
    fn subframe_1_clock_decodes_week_health_and_clock_terms() {
        let week = 987_u16;
        let sv_health = 0b10_1101_u8;
        let iodc = 0x2AB_u16;
        let tgd_raw = -20_i32;
        let toc_raw = 21_600_u32;
        let af2_raw = -12_i32;
        let af1_raw = 3_210_i32;
        let af0_raw = -123_456_i32;

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 10, week as u32);
        set_bits(&mut w3, 17, 6, sv_health as u32);
        set_bits(&mut w3, 23, 2, ((iodc >> 8) & 0b11) as u32);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 17, 8, encode_signed(tgd_raw, 8));

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 8, (iodc & 0xFF) as u32);
        set_bits(&mut w8, 9, 16, toc_raw);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 8, encode_signed(af2_raw, 8));
        set_bits(&mut w9, 9, 16, encode_signed(af1_raw, 16));

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 22, encode_signed(af0_raw, 22));

        let words = vec![
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w3, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w7, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w8, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w9, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w10, parity_ok: true, d29_star: 0, d30_star: 0 },
        ];

        let clock = decode_subframe1_clock(&words).expect("subframe 1 clock");

        assert_eq!(clock.week, week);
        assert_eq!(clock.sv_health, sv_health);
        assert_eq!(clock.iodc, iodc);
        assert!((clock.tgd - tgd_raw as f64 * 2f64.powi(-31)).abs() < f64::EPSILON);
        assert!((clock.toc_s - toc_raw as f64 * 16.0).abs() < f64::EPSILON);
        assert!((clock.af2 - af2_raw as f64 * 2f64.powi(-55)).abs() < f64::EPSILON);
        assert!((clock.af1 - af1_raw as f64 * 2f64.powi(-43)).abs() < f64::EPSILON);
        assert!((clock.af0 - af0_raw as f64 * 2f64.powi(-31)).abs() < f64::EPSILON);
    }

    #[test]
    fn subframe_2_orbit_decodes_ephemeris_fields() {
        let iode = 0xA5_u8;
        let crs_raw = -512_i32;
        let delta_n_raw = 1234_i32;
        let m0_raw = -0x1234_5678_i32;
        let cuc_raw = -777_i32;
        let e_raw = 0x0123_4567_u32;
        let cus_raw = 911_i32;
        let sqrt_a_raw = 0x0056_789A_u32;
        let toe_raw = 21_600_u32;

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 8, iode as u32);
        set_bits(&mut w3, 9, 16, encode_signed(crs_raw, 16));

        let mut w4 = 0_u32;
        set_bits(&mut w4, 1, 16, encode_signed(delta_n_raw, 16));
        set_bits(&mut w4, 17, 8, ((m0_raw as u32) >> 24) & 0xFF);

        let mut w5 = 0_u32;
        set_bits(&mut w5, 1, 24, (m0_raw as u32) & 0xFF_FFFF);

        let mut w6 = 0_u32;
        set_bits(&mut w6, 1, 16, encode_signed(cuc_raw, 16));
        set_bits(&mut w6, 17, 8, (e_raw >> 24) & 0xFF);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 1, 24, e_raw & 0xFF_FFFF);

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 16, encode_signed(cus_raw, 16));
        set_bits(&mut w8, 17, 8, (sqrt_a_raw >> 24) & 0xFF);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 24, sqrt_a_raw & 0xFF_FFFF);

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 16, toe_raw);

        let words = vec![
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w3, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w4, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w5, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w6, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w7, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w8, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w9, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w10, parity_ok: true, d29_star: 0, d30_star: 0 },
        ];

        let orbit = decode_subframe2_orbit(&words).expect("subframe 2 orbit");

        assert_eq!(orbit.iode, iode);
        assert!((orbit.crs - crs_raw as f64 * 2f64.powi(-5)).abs() < f64::EPSILON);
        assert!(
            (orbit.delta_n - delta_n_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert!((orbit.m0 - m0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < f64::EPSILON);
        assert!((orbit.cuc - cuc_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((orbit.e - e_raw as f64 * 2f64.powi(-33)).abs() < f64::EPSILON);
        assert!((orbit.cus - cus_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((orbit.sqrt_a - sqrt_a_raw as f64 * 2f64.powi(-19)).abs() < f64::EPSILON);
        assert!((orbit.toe_s - toe_raw as f64 * 16.0).abs() < f64::EPSILON);
    }

    #[test]
    fn subframe_3_orbit_decodes_ephemeris_fields() {
        let iode = 0x5A_u8;
        let cic_raw = -321_i32;
        let omega0_raw = 0x2345_6789_u32 as i32;
        let cis_raw = 654_i32;
        let i0_raw = -0x1234_0000_i32;
        let crc_raw = 2047_i32;
        let w_raw = 0x1112_1314_u32 as i32;
        let omegadot_raw = -0x34567_i32;
        let idot_raw = 0x1234_i32;

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 16, encode_signed(cic_raw, 16));
        set_bits(&mut w3, 17, 8, ((omega0_raw as u32) >> 24) & 0xFF);

        let mut w4 = 0_u32;
        set_bits(&mut w4, 1, 24, (omega0_raw as u32) & 0xFF_FFFF);

        let mut w5 = 0_u32;
        set_bits(&mut w5, 1, 16, encode_signed(cis_raw, 16));
        set_bits(&mut w5, 17, 8, ((i0_raw as u32) >> 24) & 0xFF);

        let mut w6 = 0_u32;
        set_bits(&mut w6, 1, 24, (i0_raw as u32) & 0xFF_FFFF);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 1, 16, encode_signed(crc_raw, 16));
        set_bits(&mut w7, 17, 8, ((w_raw as u32) >> 24) & 0xFF);

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 24, (w_raw as u32) & 0xFF_FFFF);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 24, encode_signed(omegadot_raw, 24));

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 8, iode as u32);
        set_bits(&mut w10, 9, 14, encode_signed(idot_raw, 14));

        let words = vec![
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: 0, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w3, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w4, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w5, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w6, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w7, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w8, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w9, parity_ok: true, d29_star: 0, d30_star: 0 },
            GpsWord { data: w10, parity_ok: true, d29_star: 0, d30_star: 0 },
        ];

        let orbit = decode_subframe3_orbit(&words).expect("subframe 3 orbit");

        assert_eq!(orbit.iode, iode);
        assert!((orbit.cic - cic_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!(
            (orbit.omega0 - omega0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert!((orbit.cis - cis_raw as f64 * 2f64.powi(-29)).abs() < f64::EPSILON);
        assert!((orbit.i0 - i0_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < f64::EPSILON);
        assert!((orbit.crc - crc_raw as f64 * 2f64.powi(-5)).abs() < f64::EPSILON);
        assert!((orbit.w - w_raw as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs() < f64::EPSILON);
        assert!(
            (orbit.omegadot - omegadot_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < f64::EPSILON
        );
        assert!((orbit.idot - idot_raw as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs() < f64::EPSILON);
    }
}
