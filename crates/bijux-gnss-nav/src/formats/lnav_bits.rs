#![allow(missing_docs)]
#![allow(dead_code)]

use serde::{Deserialize, Serialize};

use crate::formats::lnav_decode::{
    get_bits, parse_subframe1, parse_subframe2, parse_subframe3, EphemerisBuilder, EphemerisPart,
};
use crate::orbits::gps::GpsEphemeris;

const GPS_L1CA_PREAMBLE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
const GPS_L1CA_NAV_BIT_LENGTH_MS: usize = 20;
const GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS: usize = 300;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BitSyncResult {
    pub bit_start_ms: usize,
    pub bits: Vec<i8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaNavigationBit {
    pub bit_index: usize,
    pub start_prompt_index: usize,
    pub end_prompt_index_exclusive: usize,
    pub sign: i8,
    pub prompt_sum: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaNavigationBits {
    pub bit_start_ms: usize,
    pub bits: Vec<GpsL1CaNavigationBit>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaLnavSubframeAlignment {
    pub start_bit_index: usize,
    pub end_bit_index_exclusive: usize,
    pub start_prompt_index: usize,
    pub end_prompt_index_exclusive: usize,
    pub inverted: bool,
    pub word_count: usize,
    pub parity_ok_count: usize,
}

pub fn demodulate_gps_l1ca_navigation_bits(prompt_i: &[f32]) -> GpsL1CaNavigationBits {
    let mut best_offset = 0;
    let mut best_metric = f64::MIN;
    for offset in 0..GPS_L1CA_NAV_BIT_LENGTH_MS {
        let mut metric = 0.0_f64;
        let mut idx = offset;
        while idx + GPS_L1CA_NAV_BIT_LENGTH_MS <= prompt_i.len() {
            let sum: f64 = prompt_i[idx..idx + GPS_L1CA_NAV_BIT_LENGTH_MS]
                .iter()
                .map(|v| *v as f64)
                .sum();
            metric += sum.abs();
            idx += GPS_L1CA_NAV_BIT_LENGTH_MS;
        }
        if metric > best_metric {
            best_metric = metric;
            best_offset = offset;
        }
    }

    let mut bits = Vec::new();
    let mut idx = best_offset;
    let mut bit_index = 0;
    while idx + GPS_L1CA_NAV_BIT_LENGTH_MS <= prompt_i.len() {
        let end_prompt_index_exclusive = idx + GPS_L1CA_NAV_BIT_LENGTH_MS;
        let prompt_sum: f32 = prompt_i[idx..end_prompt_index_exclusive].iter().sum();
        bits.push(GpsL1CaNavigationBit {
            bit_index,
            start_prompt_index: idx,
            end_prompt_index_exclusive,
            sign: if prompt_sum >= 0.0 { 1 } else { -1 },
            prompt_sum,
        });
        idx = end_prompt_index_exclusive;
        bit_index += 1;
    }

    GpsL1CaNavigationBits { bit_start_ms: best_offset, bits }
}

pub fn bit_sync_from_prompt(prompt_i: &[f32]) -> BitSyncResult {
    let demodulation = demodulate_gps_l1ca_navigation_bits(prompt_i);
    BitSyncResult {
        bit_start_ms: demodulation.bit_start_ms,
        bits: demodulation.bits.into_iter().map(|bit| bit.sign).collect(),
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
struct AlignedGpsL1CaLnavSubframe {
    alignment: GpsL1CaLnavSubframeAlignment,
    words: Vec<GpsWord>,
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

        words.push(GpsWord { data, parity_ok, d29_star: prev_d29, d30_star: prev_d30 });

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

pub fn align_gps_l1ca_lnav_subframes(
    bits: &[i8],
    bit_start_ms: usize,
) -> Vec<GpsL1CaLnavSubframeAlignment> {
    aligned_gps_l1ca_lnav_subframes(bits, bit_start_ms)
        .into_iter()
        .map(|subframe| subframe.alignment)
        .collect()
}

pub fn decode_subframes(bits: &[i8]) -> (Vec<GpsEphemeris>, LnavDecodeStats) {
    let mut ephemerides = Vec::new();
    let mut parity_ok = 0;
    let mut parity_total = 0;
    let mut builder = EphemerisBuilder::default();

    let aligned_subframes = aligned_gps_l1ca_lnav_subframes(bits, 0);
    for subframe in &aligned_subframes {
        for word in &subframe.words {
            parity_total += 1;
            if word.parity_ok {
                parity_ok += 1;
            }
        }
        if subframe.words.len() >= 10 {
            let info = parse_how(&subframe.words[1]);
            if info.subframe_id == 1 || info.subframe_id == 2 || info.subframe_id == 3 {
                if let Some(part) = parse_ephemeris(&subframe.words, info.subframe_id) {
                    builder.merge(part);
                    if let Some(eph) = builder.try_build() {
                        ephemerides.push(eph);
                        builder = EphemerisBuilder::default();
                    }
                }
            }
        }
    }

    let parity_pass_rate =
        if parity_total > 0 { parity_ok as f64 / parity_total as f64 } else { 0.0 };

    (
        ephemerides,
        LnavDecodeStats {
            preamble_hits: aligned_subframes.len(),
            parity_pass_rate,
        },
    )
}

fn parse_how(word: &GpsWord) -> SubframeInfo {
    let tow = get_bits(word.data, 1, 17) as f64 * 6.0;
    let subframe_id = get_bits(word.data, 19, 3) as u8;
    SubframeInfo { tow_s: tow, subframe_id, parity_ok_count: if word.parity_ok { 1 } else { 0 } }
}

fn parse_ephemeris(words: &[GpsWord], subframe_id: u8) -> Option<EphemerisPart> {
    match subframe_id {
        1 => parse_subframe1(words),
        2 => parse_subframe2(words),
        3 => parse_subframe3(words),
        _ => None,
    }
}

fn aligned_gps_l1ca_lnav_subframes(
    bits: &[i8],
    bit_start_ms: usize,
) -> Vec<AlignedGpsL1CaLnavSubframe> {
    let mut aligned_subframes = Vec::new();
    let mut idx = 0;
    while idx + GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS <= bits.len() {
        let chunk = &bits[idx..idx + GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS];
        if let Some((preamble_idx, inverted)) = find_preamble(chunk) {
            if preamble_idx != 0 {
                idx += preamble_idx;
                continue;
            }
            let mut chunk_bits = chunk.to_vec();
            if inverted {
                for bit in &mut chunk_bits {
                    *bit *= -1;
                }
            }
            let words = decode_words(&chunk_bits);
            let parity_ok_count = words.iter().filter(|word| word.parity_ok).count();
            let alignment = GpsL1CaLnavSubframeAlignment {
                start_bit_index: idx,
                end_bit_index_exclusive: idx + GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS,
                start_prompt_index: bit_start_ms + idx * GPS_L1CA_NAV_BIT_LENGTH_MS,
                end_prompt_index_exclusive: bit_start_ms
                    + (idx + GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS) * GPS_L1CA_NAV_BIT_LENGTH_MS,
                inverted,
                word_count: words.len(),
                parity_ok_count,
            };
            aligned_subframes.push(AlignedGpsL1CaLnavSubframe { alignment, words });
        }
        idx += GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS;
    }
    aligned_subframes
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set_bits(data: &mut u32, start: usize, len: usize, value: u32) {
        let shift = 24 - (start - 1) - len;
        let mask = ((1_u32 << len) - 1) << shift;
        *data &= !mask;
        *data |= (value << shift) & mask;
    }

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
        if prev_d30 == 1 {
            for bit in &mut bits {
                *bit = 1 - *bit;
            }
        }
        bits
    }

    fn encode_subframe(subframe_id: u8, tow_count: u32) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_bits(&mut how, 1, 17, tow_count);
        set_bits(&mut how, 19, 3, subframe_id as u32);

        let mut words = vec![tlm, how];
        for offset in 0..8_u32 {
            words.push(0x012345 + offset * 0x010101);
        }

        let mut prev_d29 = 0_u8;
        let mut prev_d30 = 0_u8;
        let mut bits = Vec::with_capacity(GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS);
        for data in words {
            let encoded = encode_word(data, prev_d29, prev_d30);
            prev_d29 = encoded[28];
            prev_d30 = encoded[29];
            bits.extend(encoded.into_iter().map(|bit| if bit == 1 { 1 } else { -1 }));
        }
        bits
    }

    #[test]
    fn aligned_subframes_report_bit_and_prompt_boundaries() {
        let mut bits = vec![-1; 17];
        bits.extend(encode_subframe(1, 1));

        let aligned = align_gps_l1ca_lnav_subframes(&bits, 3);

        assert_eq!(aligned.len(), 1, "aligned={aligned:?}");
        assert_eq!(aligned[0].start_bit_index, 17);
        assert_eq!(aligned[0].end_bit_index_exclusive, 317);
        assert_eq!(aligned[0].start_prompt_index, 343);
        assert_eq!(aligned[0].end_prompt_index_exclusive, 6343);
        assert!(!aligned[0].inverted);
        assert_eq!(aligned[0].word_count, 10);
        assert!(aligned[0].parity_ok_count > 0, "aligned={aligned:?}");
    }

    #[test]
    fn aligned_subframes_normalize_inverted_preamble_polarity() {
        let bits = encode_subframe(2, 2)
            .into_iter()
            .map(|bit| bit * -1)
            .collect::<Vec<_>>();

        let aligned = align_gps_l1ca_lnav_subframes(&bits, 0);

        assert_eq!(aligned.len(), 1, "aligned={aligned:?}");
        assert_eq!(aligned[0].start_bit_index, 0);
        assert!(aligned[0].inverted);
        assert_eq!(aligned[0].word_count, 10);
        assert!(aligned[0].parity_ok_count > 0, "aligned={aligned:?}");
    }
}
