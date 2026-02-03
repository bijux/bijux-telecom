#![allow(missing_docs)]
#![allow(dead_code)]

use crate::formats::lnav_decode::{
    get_bits, parse_subframe1, parse_subframe2, parse_subframe3, EphemerisBuilder, EphemerisPart,
};
use crate::orbits::gps::GpsEphemeris;

const GPS_L1CA_PREAMBLE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];

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

pub fn compute_parity(data_bits: &[u8], d29_star: u8, d30_star: u8) -> (u8, u8, u8, u8, u8, u8) {
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
