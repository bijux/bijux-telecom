//! GPS L1 C/A LNAV decoding and broadcast ephemeris utilities.

#![deny(clippy::unwrap_used)]

use serde::{Deserialize, Serialize};

const GPS_L1CA_PREAMBLE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
const OMEGA_E_DOT: f64 = 7.292_115_146_7e-5;
const MU: f64 = 3.986_005e14;
const RELATIVISTIC_F: f64 = -4.442_807_633e-10;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsEphemeris {
    pub prn: u8,
    pub iodc: u16,
    pub iode: u8,
    pub week: u16,
    pub toe_s: f64,
    pub toc_s: f64,
    pub sqrt_a: f64,
    pub e: f64,
    pub i0: f64,
    pub idot: f64,
    pub omega0: f64,
    pub omegadot: f64,
    pub w: f64,
    pub m0: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
    pub relativistic_s: f64,
}

#[derive(Debug, Clone)]
pub struct BitSyncResult {
    pub bit_start_ms: usize,
    pub bits: Vec<i8>,
}

pub fn bit_sync_from_prompt(prompt_i: &[f32]) -> BitSyncResult {
    let mut best_offset = 0;
    let mut best_metric = f64::MIN;
    for offset in 0..20 {
        let mut metric = 0.0_f64;
        let mut idx = offset;
        while idx + 20 <= prompt_i.len() {
            let sum: f64 = prompt_i[idx..idx + 20].iter().map(|v| *v as f64).sum();
            metric += sum.abs();
            idx += 20;
        }
        if metric > best_metric {
            best_metric = metric;
            best_offset = offset;
        }
    }

    let mut bits = Vec::new();
    let mut idx = best_offset;
    while idx + 20 <= prompt_i.len() {
        let sum: f64 = prompt_i[idx..idx + 20].iter().map(|v| *v as f64).sum();
        bits.push(if sum >= 0.0 { 1 } else { -1 });
        idx += 20;
    }

    BitSyncResult {
        bit_start_ms: best_offset,
        bits,
    }
}

pub fn find_preamble(bits: &[i8]) -> Option<(usize, bool)> {
    if bits.len() < GPS_L1CA_PREAMBLE.len() {
        return None;
    }
    for i in 0..=bits.len() - GPS_L1CA_PREAMBLE.len() {
        let mut matched = true;
        let mut matched_inverted = true;
        for (j, &p) in GPS_L1CA_PREAMBLE.iter().enumerate() {
            let bit = if bits[i + j] > 0 { 1 } else { 0 };
            if bit != p {
                matched = false;
            }
            if (1 - bit) != p {
                matched_inverted = false;
            }
            if !matched && !matched_inverted {
                break;
            }
        }
        if matched {
            return Some((i, false));
        }
        if matched_inverted {
            return Some((i, true));
        }
    }
    None
}

#[derive(Debug, Clone)]
pub struct GpsWord {
    pub data: u32,
    pub parity_ok: bool,
    pub d29_star: u8,
    pub d30_star: u8,
}

pub fn decode_words(bits: &[i8]) -> Vec<GpsWord> {
    let mut words = Vec::new();
    let mut prev_d29 = 0_u8;
    let mut prev_d30 = 0_u8;
    let mut idx = 0;
    while idx + 30 <= bits.len() {
        let mut raw_bits = [0_u8; 30];
        for j in 0..30 {
            raw_bits[j] = if bits[idx + j] > 0 { 1 } else { 0 };
        }
        let inverted = prev_d30 == 1;
        if inverted {
            for bit in raw_bits.iter_mut() {
                *bit = 1 - *bit;
            }
        }

        let mut data = 0_u32;
        for &bit in raw_bits.iter().take(24) {
            data = (data << 1) | bit as u32;
        }

        let (p1, p2, p3, p4, p5, p6) = compute_parity(&raw_bits[..24], prev_d29, prev_d30);
        let parity_ok = p1 == raw_bits[24]
            && p2 == raw_bits[25]
            && p3 == raw_bits[26]
            && p4 == raw_bits[27]
            && p5 == raw_bits[28]
            && p6 == raw_bits[29];

        prev_d29 = raw_bits[28];
        prev_d30 = raw_bits[29];

        words.push(GpsWord {
            data,
            parity_ok,
            d29_star: prev_d29,
            d30_star: prev_d30,
        });

        idx += 30;
    }
    words
}

fn compute_parity(data_bits: &[u8], d29_star: u8, d30_star: u8) -> (u8, u8, u8, u8, u8, u8) {
    let d = |i: usize| -> u8 { data_bits[i - 1] };
    let p1 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(10)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(23)
        ^ d(24)
        ^ d29_star;
    let p2 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(24)
        ^ d29_star
        ^ d30_star;
    let p3 = d(1)
        ^ d(3)
        ^ d(4)
        ^ d(5)
        ^ d(7)
        ^ d(8)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d29_star
        ^ d30_star;
    let p4 = d(1)
        ^ d(2)
        ^ d(4)
        ^ d(5)
        ^ d(6)
        ^ d(8)
        ^ d(9)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(21)
        ^ d(23)
        ^ d30_star;
    let p5 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(7)
        ^ d(9)
        ^ d(10)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(22)
        ^ d(24)
        ^ d29_star;
    let p6 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(8)
        ^ d(10)
        ^ d(11)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d(23)
        ^ d(24)
        ^ d30_star;
    (p1, p2, p3, p4, p5, p6)
}

#[derive(Debug, Clone)]
pub struct SubframeInfo {
    pub tow_s: f64,
    pub subframe_id: u8,
    pub parity_ok_count: usize,
}

#[derive(Debug, Clone)]
pub struct LnavDecodeStats {
    pub preamble_hits: usize,
    pub parity_pass_rate: f64,
}

pub fn decode_subframes(bits: &[i8]) -> (Vec<GpsEphemeris>, LnavDecodeStats) {
    let mut ephemerides = Vec::new();
    let mut preamble_hits = 0;
    let mut parity_ok = 0;
    let mut parity_total = 0;
    let mut builder = EphemerisBuilder::default();

    let mut idx = 0;
    while idx + 300 <= bits.len() {
        let chunk = &bits[idx..idx + 300];
        if let Some((preamble_idx, inverted)) = find_preamble(chunk) {
            if preamble_idx != 0 {
                idx += preamble_idx;
                continue;
            }
            preamble_hits += 1;
            let mut chunk_bits = chunk.to_vec();
            if inverted {
                for b in chunk_bits.iter_mut() {
                    *b *= -1;
                }
            }
            let words = decode_words(&chunk_bits);
            for w in &words {
                parity_total += 1;
                if w.parity_ok {
                    parity_ok += 1;
                }
            }
            if words.len() >= 10 {
                let info = parse_how(&words[1]);
                if info.subframe_id == 1 || info.subframe_id == 2 || info.subframe_id == 3 {
                    if let Some(part) = parse_ephemeris(&words, info.subframe_id) {
                        builder.merge(part);
                        if let Some(eph) = builder.try_build() {
                            ephemerides.push(eph);
                            builder = EphemerisBuilder::default();
                        }
                    }
                }
            }
        }
        idx += 300;
    }

    let parity_pass_rate = if parity_total > 0 {
        parity_ok as f64 / parity_total as f64
    } else {
        0.0
    };

    (
        ephemerides,
        LnavDecodeStats {
            preamble_hits,
            parity_pass_rate,
        },
    )
}

fn parse_how(word: &GpsWord) -> SubframeInfo {
    let tow = get_bits(word.data, 1, 17) as f64 * 6.0;
    let subframe_id = get_bits(word.data, 19, 3) as u8;
    SubframeInfo {
        tow_s: tow,
        subframe_id,
        parity_ok_count: if word.parity_ok { 1 } else { 0 },
    }
}

fn parse_ephemeris(words: &[GpsWord], subframe_id: u8) -> Option<EphemerisPart> {
    match subframe_id {
        1 => parse_subframe1(words),
        2 => parse_subframe2(words),
        3 => parse_subframe3(words),
        _ => None,
    }
}

fn parse_subframe1(words: &[GpsWord]) -> Option<EphemerisPart> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let week = get_bits(w3, 1, 10) as u16;
    let iodc_msb = get_bits(w3, 23, 2) as u16;
    let iodc_lsb = get_bits(w8, 1, 8) as u16;
    let iodc = (iodc_msb << 8) | iodc_lsb;
    let tgd = signed(get_bits(w7, 17, 8), 8) as f64 * 2f64.powi(-31);
    let toc = get_bits(w8, 9, 16) as f64 * 16.0;
    let af2 = signed(get_bits(w9, 1, 8), 8) as f64 * 2f64.powi(-55);
    let af1 = signed(get_bits(w9, 9, 16), 16) as f64 * 2f64.powi(-43);
    let af0 = signed(get_bits(w10, 1, 22), 22) as f64 * 2f64.powi(-31);

    Some(EphemerisPart {
        iodc: Some(iodc),
        iode: None,
        week: Some(week),
        toe_s: None,
        toc_s: Some(toc),
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
        af0: Some(af0),
        af1: Some(af1),
        af2: Some(af2),
        tgd: Some(tgd),
    })
}

fn parse_subframe2(words: &[GpsWord]) -> Option<EphemerisPart> {
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

    Some(EphemerisPart {
        iodc: None,
        iode: Some(iode),
        week: None,
        toe_s: Some(toe),
        toc_s: None,
        sqrt_a: Some(sqrt_a),
        e: Some(e),
        i0: None,
        idot: None,
        omega0: None,
        omegadot: None,
        w: None,
        m0: Some(m0),
        delta_n: Some(delta_n),
        cuc: Some(cuc),
        cus: Some(cus),
        crc: None,
        crs: Some(crs),
        cic: None,
        cis: None,
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

fn parse_subframe3(words: &[GpsWord]) -> Option<EphemerisPart> {
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

    Some(EphemerisPart {
        iodc: None,
        iode: Some(iode),
        week: None,
        toe_s: None,
        toc_s: None,
        sqrt_a: None,
        e: None,
        i0: Some(i0),
        idot: Some(idot),
        omega0: Some(omega0),
        omegadot: Some(omegadot),
        w: Some(w),
        m0: None,
        delta_n: None,
        cuc: None,
        cus: None,
        crc: Some(crc),
        crs: None,
        cic: Some(cic),
        cis: Some(cis),
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

fn get_bits(data: u32, start: usize, len: usize) -> u32 {
    let shift = 24 - (start - 1) - len;
    (data >> shift) & ((1_u32 << len) - 1)
}

fn signed(value: u32, bits: usize) -> i32 {
    let sign_bit = 1_u32 << (bits - 1);
    if value & sign_bit != 0 {
        (value as i32) - (1_i32 << bits)
    } else {
        value as i32
    }
}

#[derive(Debug, Default, Clone)]
struct EphemerisPart {
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
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
struct EphemerisBuilder {
    prn: u8,
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
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
    fn merge(&mut self, part: EphemerisPart) {
        if let Some(value) = part.iodc {
            self.iodc = Some(value);
        }
        if let Some(value) = part.iode {
            self.iode = Some(value);
        }
        if let Some(value) = part.week {
            self.week = Some(value);
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

    fn try_build(&self) -> Option<GpsEphemeris> {
        Some(GpsEphemeris {
            prn: self.prn,
            iodc: self.iodc?,
            iode: self.iode?,
            week: self.week?,
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
    let mut builder = EphemerisBuilder {
        prn,
        ..Default::default()
    };
    let words1: Vec<GpsWord> = w1
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    let words2: Vec<GpsWord> = w2
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    let words3: Vec<GpsWord> = w3
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    builder.merge(parse_subframe1(&words1)?);
    builder.merge(parse_subframe2(&words2)?);
    builder.merge(parse_subframe3(&words3)?);
    builder.try_build()
}

pub fn sat_state_gps_l1ca(eph: &GpsEphemeris, t_tx_s: f64, tau_s: f64) -> GpsSatState {
    let a = eph.sqrt_a * eph.sqrt_a;
    let n0 = (MU / (a * a * a)).sqrt();
    let n = n0 + eph.delta_n;
    let mut tk = t_tx_s - eph.toe_s;
    tk = wrap_time(tk);

    let m = eph.m0 + n * tk;
    let (_e_anom, sin_e, cos_e) = solve_kepler(m, eph.e);
    let v = (1.0 - eph.e * eph.e).sqrt() * sin_e;
    let v = v.atan2(cos_e - eph.e);
    let phi = v + eph.w;
    let sin2phi = (2.0 * phi).sin();
    let cos2phi = (2.0 * phi).cos();
    let u = phi + eph.cuc * cos2phi + eph.cus * sin2phi;
    let r = a * (1.0 - eph.e * cos_e) + eph.crc * cos2phi + eph.crs * sin2phi;
    let i = eph.i0 + eph.idot * tk + eph.cic * cos2phi + eph.cis * sin2phi;

    let x_orb = r * u.cos();
    let y_orb = r * u.sin();

    let omega = eph.omega0 + (eph.omegadot - OMEGA_E_DOT) * tk - OMEGA_E_DOT * eph.toe_s;

    let cos_omega = omega.cos();
    let sin_omega = omega.sin();
    let cos_i = i.cos();
    let sin_i = i.sin();

    let mut x = x_orb * cos_omega - y_orb * cos_i * sin_omega;
    let mut y = x_orb * sin_omega + y_orb * cos_i * cos_omega;
    let z = y_orb * sin_i;

    let rot = OMEGA_E_DOT * tau_s;
    if rot.abs() > 0.0 {
        let cos_rot = rot.cos();
        let sin_rot = rot.sin();
        let xr = cos_rot * x + sin_rot * y;
        let yr = -sin_rot * x + cos_rot * y;
        x = xr;
        y = yr;
    }

    let dt = wrap_time(t_tx_s - eph.toc_s);
    let relativistic = RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
    let clock_bias = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + relativistic - eph.tgd;
    let clock_drift = eph.af1 + 2.0 * eph.af2 * dt;

    GpsSatState {
        x_m: x,
        y_m: y,
        z_m: z,
        clock_bias_s: clock_bias,
        clock_drift_s: clock_drift,
        relativistic_s: relativistic,
    }
}

pub fn solve_kepler(m: f64, e: f64) -> (f64, f64, f64) {
    let mut e_anom = m;
    for _ in 0..12 {
        let f = e_anom - e * e_anom.sin() - m;
        let f_prime = 1.0 - e * e_anom.cos();
        let step = f / f_prime;
        e_anom -= step;
        if step.abs() < 1e-12 {
            break;
        }
    }
    if !e_anom.is_finite() {
        e_anom = m;
    }
    (e_anom, e_anom.sin(), e_anom.cos())
}

fn wrap_time(mut t: f64) -> f64 {
    let half = 302_400.0;
    while t > half {
        t -= 604_800.0;
    }
    while t < -half {
        t += 604_800.0;
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_word(data: u32, prev_d29: u8, prev_d30: u8) -> [u8; 30] {
        let mut bits = [0_u8; 30];
        for (i, bit) in bits.iter_mut().enumerate().take(24) {
            let shift = 23 - i;
            *bit = ((data >> shift) & 1) as u8;
        }
        let (p1, p2, p3, p4, p5, p6) = compute_parity(&bits[..24], prev_d29, prev_d30);
        bits[24] = p1;
        bits[25] = p2;
        bits[26] = p3;
        bits[27] = p4;
        bits[28] = p5;
        bits[29] = p6;
        bits
    }

    #[test]
    fn parity_roundtrip() {
        let data = 0xABCDE;
        let bits = encode_word(data, 0, 0);
        let mut signed = Vec::new();
        for b in bits {
            signed.push(if b == 1 { 1 } else { -1 });
        }
        let words = decode_words(&signed);
        assert_eq!(words.len(), 1);
        assert!(words[0].parity_ok);
        assert_eq!(words[0].data, data);
    }

    #[test]
    fn kepler_solution_close_for_circular() {
        let m = 1.0;
        let e = 0.0;
        let (e_anom, _, _) = solve_kepler(m, e);
        assert!((e_anom - m).abs() < 1e-10);
    }

    #[test]
    fn sat_state_basic() {
        let eph = GpsEphemeris {
            prn: 1,
            iodc: 0,
            iode: 0,
            week: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.0,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let state = sat_state_gps_l1ca(&eph, 0.0, 0.0);
        let radius = (state.x_m * state.x_m + state.y_m * state.y_m + state.z_m * state.z_m).sqrt();
        assert!((radius - 26_560_000.0).abs() < 5_000_000.0);
    }

    #[test]
    fn bit_sync_detects_offset() {
        let mut prompt = vec![1.0_f32; 5];
        prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        let result = bit_sync_from_prompt(&prompt);
        assert_eq!(result.bit_start_ms, 5);
        assert_eq!(result.bits.len(), 2);
        assert_eq!(result.bits[0], -1);
        assert_eq!(result.bits[1], 1);
    }

    #[test]
    fn relativistic_term_nonzero_for_eccentric() {
        let eph = GpsEphemeris {
            prn: 1,
            iodc: 0,
            iode: 0,
            week: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.1,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.1,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let state = sat_state_gps_l1ca(&eph, 1000.0, 0.0);
        assert!(state.relativistic_s.abs() > 0.0);
    }
}
