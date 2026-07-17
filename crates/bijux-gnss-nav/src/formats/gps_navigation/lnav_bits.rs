#![allow(missing_docs)]
#![allow(dead_code)]

use serde::{Deserialize, Serialize};

use crate::formats::lnav_decode::{
    decode_subframe1_clock, decode_subframe2_orbit, decode_subframe3_orbit,
    ephemeris_part_from_subframe1_clock, ephemeris_part_from_subframe2_orbit,
    ephemeris_part_from_subframe3_orbit, get_bits, parse_subframe1, parse_subframe2,
    parse_subframe3, EphemerisBuilder, EphemerisPart, GpsL1CaLnavEphemerisRejection,
    GpsL1CaLnavSubframe1Clock, GpsL1CaLnavSubframe2Orbit, GpsL1CaLnavSubframe3Orbit,
};
use crate::orbits::gps::GpsEphemeris;

const GPS_L1CA_PREAMBLE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
const GPS_L1CA_NAV_BIT_LENGTH_MS: usize = 20;
const GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS: usize = 300;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BitSyncResult {
    pub bit_start_ms: usize,
    pub sync_confidence: f64,
    pub bits: Vec<i8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaNavigationBit {
    pub bit_index: usize,
    pub start_prompt_index: usize,
    pub end_prompt_index_exclusive: usize,
    pub sign: i8,
    pub prompt_sum: f32,
    pub confidence: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaNavigationBits {
    pub bit_start_ms: usize,
    pub sync_confidence: f64,
    pub best_offset_metric: f64,
    pub next_best_offset_metric: f64,
    pub complete_window_count: usize,
    pub bits: Vec<GpsL1CaNavigationBit>,
}

#[derive(Debug, Clone, Copy)]
struct GpsL1CaSymbolOffsetScore {
    offset: usize,
    metric: f64,
    complete_window_count: usize,
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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaLnavAlignment {
    pub bit_start_ms: usize,
    pub bit_count: usize,
    pub subframes: Vec<GpsL1CaLnavSubframeAlignment>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaTlmWord {
    pub preamble: u8,
    pub parity_ok: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaHowWord {
    pub tow_count: u32,
    pub tow_start_s: u32,
    pub alert: bool,
    pub anti_spoof: bool,
    pub subframe_id: u8,
    pub parity_ok: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaWordParitySummary {
    pub word_count: usize,
    pub passed_word_count: usize,
    pub failed_word_indexes: Vec<usize>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaLnavDecodedSubframe {
    pub alignment: GpsL1CaLnavSubframeAlignment,
    pub tlm: GpsL1CaTlmWord,
    pub how: GpsL1CaHowWord,
    pub clock: Option<GpsL1CaLnavSubframe1Clock>,
    pub orbit_subframe_2: Option<GpsL1CaLnavSubframe2Orbit>,
    pub orbit_subframe_3: Option<GpsL1CaLnavSubframe3Orbit>,
    pub parity: GpsL1CaWordParitySummary,
    pub word_parity_ok: Vec<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsL1CaLnavDecodedStream {
    pub bit_start_ms: usize,
    pub bit_count: usize,
    pub subframes: Vec<GpsL1CaLnavDecodedSubframe>,
}

pub fn demodulate_gps_l1ca_navigation_bits(prompt_i: &[f32]) -> GpsL1CaNavigationBits {
    let (best_score, next_best_score) = gps_l1ca_symbol_offset_scores(prompt_i);
    let sync_confidence =
        gps_l1ca_symbol_sync_confidence(best_score.metric, next_best_score.metric);

    let mut bits = Vec::new();
    let mut idx = best_score.offset;
    let mut bit_index = 0;
    while idx + GPS_L1CA_NAV_BIT_LENGTH_MS <= prompt_i.len() {
        let end_prompt_index_exclusive = idx + GPS_L1CA_NAV_BIT_LENGTH_MS;
        let prompt_sum: f32 = prompt_i[idx..end_prompt_index_exclusive].iter().sum();
        let prompt_energy: f64 = prompt_i[idx..end_prompt_index_exclusive]
            .iter()
            .map(|sample| sample.abs() as f64)
            .sum();
        let confidence = if prompt_energy > f64::EPSILON {
            (prompt_sum.abs() as f64 / prompt_energy).clamp(0.0, 1.0)
        } else {
            0.0
        };
        bits.push(GpsL1CaNavigationBit {
            bit_index,
            start_prompt_index: idx,
            end_prompt_index_exclusive,
            sign: if prompt_sum >= 0.0 { 1 } else { -1 },
            prompt_sum,
            confidence,
        });
        idx = end_prompt_index_exclusive;
        bit_index += 1;
    }

    GpsL1CaNavigationBits {
        bit_start_ms: best_score.offset,
        sync_confidence,
        best_offset_metric: best_score.metric,
        next_best_offset_metric: next_best_score.metric,
        complete_window_count: best_score.complete_window_count,
        bits,
    }
}

pub fn bit_sync_from_prompt(prompt_i: &[f32]) -> BitSyncResult {
    let demodulation = demodulate_gps_l1ca_navigation_bits(prompt_i);
    BitSyncResult {
        bit_start_ms: demodulation.bit_start_ms,
        sync_confidence: demodulation.sync_confidence,
        bits: demodulation.bits.into_iter().map(|bit| bit.sign).collect(),
    }
}

fn gps_l1ca_symbol_offset_scores(
    prompt_i: &[f32],
) -> (GpsL1CaSymbolOffsetScore, GpsL1CaSymbolOffsetScore) {
    let mut scores = (0..GPS_L1CA_NAV_BIT_LENGTH_MS)
        .map(|offset| gps_l1ca_symbol_offset_score(prompt_i, offset))
        .collect::<Vec<_>>();
    scores.sort_by(|left, right| {
        right
            .metric
            .partial_cmp(&left.metric)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.offset.cmp(&right.offset))
    });
    let best = scores.first().copied().unwrap_or(GpsL1CaSymbolOffsetScore {
        offset: 0,
        metric: 0.0,
        complete_window_count: 0,
    });
    let next_best = scores.get(1).copied().unwrap_or(GpsL1CaSymbolOffsetScore {
        offset: best.offset,
        metric: 0.0,
        complete_window_count: 0,
    });
    (best, next_best)
}

fn gps_l1ca_symbol_offset_score(prompt_i: &[f32], offset: usize) -> GpsL1CaSymbolOffsetScore {
    let mut summed_coherent_energy = 0.0_f64;
    let mut complete_window_count = 0usize;
    let mut sampled_window_count = 0.0_f64;
    let mut idx = offset;
    while idx + GPS_L1CA_NAV_BIT_LENGTH_MS <= prompt_i.len() {
        let sum: f64 =
            prompt_i[idx..idx + GPS_L1CA_NAV_BIT_LENGTH_MS].iter().map(|v| *v as f64).sum();
        summed_coherent_energy += sum.abs();
        complete_window_count += 1;
        sampled_window_count += 1.0;
        idx += GPS_L1CA_NAV_BIT_LENGTH_MS;
    }
    if idx < prompt_i.len() {
        let partial_sum: f64 = prompt_i[idx..].iter().map(|v| *v as f64).sum();
        summed_coherent_energy += partial_sum.abs();
        sampled_window_count += (prompt_i.len() - idx) as f64 / GPS_L1CA_NAV_BIT_LENGTH_MS as f64;
    }
    let metric = if sampled_window_count > f64::EPSILON {
        summed_coherent_energy / sampled_window_count
    } else {
        0.0
    };
    GpsL1CaSymbolOffsetScore { offset, metric, complete_window_count }
}

fn gps_l1ca_symbol_sync_confidence(best_metric: f64, next_best_metric: f64) -> f64 {
    if best_metric <= f64::EPSILON {
        return 0.0;
    }
    ((best_metric - next_best_metric.max(0.0)) / best_metric).clamp(0.0, 1.0)
}

pub fn align_gps_l1ca_lnav_from_prompt(prompt_i: &[f32]) -> GpsL1CaLnavAlignment {
    let demodulation = demodulate_gps_l1ca_navigation_bits(prompt_i);
    let bit_signs = demodulation.bits.iter().map(|bit| bit.sign).collect::<Vec<_>>();
    let subframes = align_gps_l1ca_lnav_subframes(&bit_signs, demodulation.bit_start_ms);
    GpsL1CaLnavAlignment {
        bit_start_ms: demodulation.bit_start_ms,
        bit_count: bit_signs.len(),
        subframes,
    }
}

pub fn decode_gps_l1ca_lnav_from_prompt(prompt_i: &[f32]) -> GpsL1CaLnavDecodedStream {
    let demodulation = demodulate_gps_l1ca_navigation_bits(prompt_i);
    let bit_signs = demodulation.bits.iter().map(|bit| bit.sign).collect::<Vec<_>>();
    let subframes = decode_gps_l1ca_lnav_subframes(&bit_signs, demodulation.bit_start_ms);
    GpsL1CaLnavDecodedStream {
        bit_start_ms: demodulation.bit_start_ms,
        bit_count: bit_signs.len(),
        subframes,
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

pub fn decode_tlm_word(word: &GpsWord) -> GpsL1CaTlmWord {
    GpsL1CaTlmWord { preamble: get_bits(word.data, 1, 8) as u8, parity_ok: word.parity_ok }
}

pub fn decode_how_word(word: &GpsWord) -> GpsL1CaHowWord {
    let tow_count = get_bits(word.data, 1, 17);
    GpsL1CaHowWord {
        tow_count,
        tow_start_s: tow_count * 6,
        alert: get_bits(word.data, 18, 1) == 1,
        anti_spoof: get_bits(word.data, 19, 1) == 1,
        subframe_id: get_bits(word.data, 20, 3) as u8,
        parity_ok: word.parity_ok,
    }
}

pub fn summarize_word_parity(words: &[GpsWord]) -> GpsL1CaWordParitySummary {
    let failed_word_indexes = words
        .iter()
        .enumerate()
        .filter_map(|(word_index, word)| (!word.parity_ok).then_some(word_index))
        .collect::<Vec<_>>();
    GpsL1CaWordParitySummary {
        word_count: words.len(),
        passed_word_count: words.len() - failed_word_indexes.len(),
        failed_word_indexes,
    }
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
    pub ephemeris_rejections: Vec<GpsL1CaLnavEphemerisRejection>,
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

pub fn decode_gps_l1ca_lnav_subframes(
    bits: &[i8],
    bit_start_ms: usize,
) -> Vec<GpsL1CaLnavDecodedSubframe> {
    aligned_gps_l1ca_lnav_subframes(bits, bit_start_ms)
        .into_iter()
        .filter_map(|subframe| {
            if subframe.words.len() < 2 {
                return None;
            }
            let how = decode_how_word(&subframe.words[1]);
            let clock =
                (how.subframe_id == 1).then(|| decode_subframe1_clock(&subframe.words)).flatten();
            let orbit_subframe_2 =
                (how.subframe_id == 2).then(|| decode_subframe2_orbit(&subframe.words)).flatten();
            let orbit_subframe_3 =
                (how.subframe_id == 3).then(|| decode_subframe3_orbit(&subframe.words)).flatten();
            Some(GpsL1CaLnavDecodedSubframe {
                tlm: decode_tlm_word(&subframe.words[0]),
                clock,
                orbit_subframe_2,
                orbit_subframe_3,
                how,
                parity: summarize_word_parity(&subframe.words),
                word_parity_ok: subframe.words.iter().map(|word| word.parity_ok).collect(),
                alignment: subframe.alignment,
            })
        })
        .collect()
}

pub fn decode_subframes(
    bits: &[i8],
    reference_week: Option<u32>,
) -> (Vec<GpsEphemeris>, LnavDecodeStats) {
    let decoded_subframes = decode_gps_l1ca_lnav_subframes(bits, 0);
    let parity_total =
        decoded_subframes.iter().map(|subframe| subframe.parity.word_count).sum::<usize>();
    let parity_ok =
        decoded_subframes.iter().map(|subframe| subframe.parity.passed_word_count).sum::<usize>();
    let parity_pass_rate =
        if parity_total > 0 { parity_ok as f64 / parity_total as f64 } else { 0.0 };
    let (ephemerides, ephemeris_rejections) =
        ephemerides_from_decoded_gps_l1ca_lnav(0, &decoded_subframes, reference_week);

    (
        ephemerides,
        LnavDecodeStats {
            preamble_hits: decoded_subframes.len(),
            parity_pass_rate,
            ephemeris_rejections,
        },
    )
}

pub fn ephemerides_from_decoded_gps_l1ca_lnav(
    prn: u8,
    subframes: &[GpsL1CaLnavDecodedSubframe],
    reference_week: Option<u32>,
) -> (Vec<GpsEphemeris>, Vec<GpsL1CaLnavEphemerisRejection>) {
    let mut ephemerides = Vec::new();
    let mut ephemeris_rejections = Vec::new();
    let mut builder = reference_week
        .map(|week| EphemerisBuilder::with_reference_week(prn, week))
        .unwrap_or_else(|| EphemerisBuilder::with_prn(prn));

    for subframe in subframes {
        let Some(part) = ephemeris_part_from_decoded_subframe(subframe) else {
            continue;
        };
        if let Err(rejection) = builder.merge(part.clone()) {
            ephemeris_rejections.push(rejection);
            builder.reset();
            let _ = builder.merge(part);
        } else {
            match builder.try_build_checked() {
                Ok(Some(eph)) => {
                    ephemerides.push(eph);
                    builder.reset();
                }
                Ok(None) => {}
                Err(rejection) => {
                    ephemeris_rejections.push(rejection);
                    builder.reset();
                }
            }
        }
    }

    (ephemerides, ephemeris_rejections)
}

fn subframe_info_from_how(word: &GpsWord) -> SubframeInfo {
    let how = decode_how_word(word);
    SubframeInfo {
        tow_s: how.tow_start_s as f64,
        subframe_id: how.subframe_id,
        parity_ok_count: if how.parity_ok { 1 } else { 0 },
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

fn ephemeris_part_from_decoded_subframe(
    subframe: &GpsL1CaLnavDecodedSubframe,
) -> Option<EphemerisPart> {
    if let Some(clock) = &subframe.clock {
        return Some(ephemeris_part_from_subframe1_clock(clock));
    }
    if let Some(orbit) = &subframe.orbit_subframe_2 {
        return Some(ephemeris_part_from_subframe2_orbit(orbit));
    }
    if let Some(orbit) = &subframe.orbit_subframe_3 {
        return Some(ephemeris_part_from_subframe3_orbit(orbit));
    }
    None
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

    fn encode_signed(value: i32, bits: usize) -> u32 {
        let mask = (1_u32 << bits) - 1;
        (value as u32) & mask
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

    fn flip_signed_bit(bits: &mut [i8], bit_index: usize) {
        bits[bit_index] *= -1;
    }

    fn encode_single_word_signed(data: u32) -> Vec<i8> {
        encode_word(data, 0, 0).into_iter().map(|bit| if bit == 1 { 1 } else { -1 }).collect()
    }

    fn encode_subframe(subframe_id: u8, tow_count: u32) -> Vec<i8> {
        encode_subframe_with_how(subframe_id, tow_count, false, false)
    }

    fn encode_subframe_with_how(
        subframe_id: u8,
        tow_count: u32,
        alert: bool,
        anti_spoof: bool,
    ) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_bits(&mut how, 1, 17, tow_count);
        set_bits(&mut how, 18, 1, u32::from(alert));
        set_bits(&mut how, 19, 1, u32::from(anti_spoof));
        set_bits(&mut how, 20, 3, subframe_id as u32);

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

    struct ClockSubframeRequest {
        tow_count: u32,
        week: u16,
        sv_accuracy: u8,
        sv_health: u8,
        iodc: u16,
        toc_raw: u32,
        af2_raw: i32,
        af1_raw: i32,
        af0_raw: i32,
        tgd_raw: i32,
    }

    struct OrbitSubframe2Request {
        tow_count: u32,
        iode: u8,
        crs_raw: i32,
        delta_n_raw: i32,
        m0_raw: i32,
        cuc_raw: i32,
        e_raw: u32,
        cus_raw: i32,
        sqrt_a_raw: u32,
        toe_raw: u32,
    }

    struct OrbitSubframe3Request {
        tow_count: u32,
        iode: u8,
        cic_raw: i32,
        omega0_raw: i32,
        cis_raw: i32,
        i0_raw: i32,
        crc_raw: i32,
        w_raw: i32,
        omegadot_raw: i32,
        idot_raw: i32,
    }

    fn encode_subframe_1_with_clock(request: ClockSubframeRequest) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_bits(&mut how, 1, 17, request.tow_count);
        set_bits(&mut how, 20, 3, 1);

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 10, request.week as u32);
        set_bits(&mut w3, 11, 4, request.sv_accuracy as u32);
        set_bits(&mut w3, 17, 6, request.sv_health as u32);
        set_bits(&mut w3, 23, 2, ((request.iodc >> 8) & 0b11) as u32);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 17, 8, encode_signed(request.tgd_raw, 8));

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 8, (request.iodc & 0xFF) as u32);
        set_bits(&mut w8, 9, 16, request.toc_raw);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 8, encode_signed(request.af2_raw, 8));
        set_bits(&mut w9, 9, 16, encode_signed(request.af1_raw, 16));

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 22, encode_signed(request.af0_raw, 22));

        let words = [tlm, how, w3, 0, 0, 0, w7, w8, w9, w10];

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

    fn encode_subframe_2_with_orbit(request: OrbitSubframe2Request) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_bits(&mut how, 1, 17, request.tow_count);
        set_bits(&mut how, 20, 3, 2);

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 8, request.iode as u32);
        set_bits(&mut w3, 9, 16, encode_signed(request.crs_raw, 16));

        let mut w4 = 0_u32;
        set_bits(&mut w4, 1, 16, encode_signed(request.delta_n_raw, 16));
        set_bits(&mut w4, 17, 8, ((request.m0_raw as u32) >> 24) & 0xFF);

        let mut w5 = 0_u32;
        set_bits(&mut w5, 1, 24, (request.m0_raw as u32) & 0xFF_FFFF);

        let mut w6 = 0_u32;
        set_bits(&mut w6, 1, 16, encode_signed(request.cuc_raw, 16));
        set_bits(&mut w6, 17, 8, (request.e_raw >> 24) & 0xFF);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 1, 24, request.e_raw & 0xFF_FFFF);

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 16, encode_signed(request.cus_raw, 16));
        set_bits(&mut w8, 17, 8, (request.sqrt_a_raw >> 24) & 0xFF);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 24, request.sqrt_a_raw & 0xFF_FFFF);

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 16, request.toe_raw);

        let words = [tlm, how, w3, w4, w5, w6, w7, w8, w9, w10];

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

    fn encode_subframe_3_with_orbit(request: OrbitSubframe3Request) -> Vec<i8> {
        let mut tlm = 0_u32;
        set_bits(&mut tlm, 1, 8, 0x8B);

        let mut how = 0_u32;
        set_bits(&mut how, 1, 17, request.tow_count);
        set_bits(&mut how, 20, 3, 3);

        let mut w3 = 0_u32;
        set_bits(&mut w3, 1, 16, encode_signed(request.cic_raw, 16));
        set_bits(&mut w3, 17, 8, ((request.omega0_raw as u32) >> 24) & 0xFF);

        let mut w4 = 0_u32;
        set_bits(&mut w4, 1, 24, (request.omega0_raw as u32) & 0xFF_FFFF);

        let mut w5 = 0_u32;
        set_bits(&mut w5, 1, 16, encode_signed(request.cis_raw, 16));
        set_bits(&mut w5, 17, 8, ((request.i0_raw as u32) >> 24) & 0xFF);

        let mut w6 = 0_u32;
        set_bits(&mut w6, 1, 24, (request.i0_raw as u32) & 0xFF_FFFF);

        let mut w7 = 0_u32;
        set_bits(&mut w7, 1, 16, encode_signed(request.crc_raw, 16));
        set_bits(&mut w7, 17, 8, ((request.w_raw as u32) >> 24) & 0xFF);

        let mut w8 = 0_u32;
        set_bits(&mut w8, 1, 24, (request.w_raw as u32) & 0xFF_FFFF);

        let mut w9 = 0_u32;
        set_bits(&mut w9, 1, 24, encode_signed(request.omegadot_raw, 24));

        let mut w10 = 0_u32;
        set_bits(&mut w10, 1, 8, request.iode as u32);
        set_bits(&mut w10, 9, 14, encode_signed(request.idot_raw, 14));

        let words = [tlm, how, w3, w4, w5, w6, w7, w8, w9, w10];

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

    fn decoded_subframe_from_hex(hex: &str) -> GpsL1CaLnavDecodedSubframe {
        let words = crate::formats::lnav_decode::decode_subframe_hex(hex)
            .expect("subframe words")
            .into_iter()
            .map(|data| GpsWord { data, parity_ok: true, d29_star: 0, d30_star: 0 })
            .collect::<Vec<_>>();
        let how = decode_how_word(&words[1]);
        let clock = (how.subframe_id == 1).then(|| decode_subframe1_clock(&words)).flatten();
        let orbit_subframe_2 =
            (how.subframe_id == 2).then(|| decode_subframe2_orbit(&words)).flatten();
        let orbit_subframe_3 =
            (how.subframe_id == 3).then(|| decode_subframe3_orbit(&words)).flatten();
        GpsL1CaLnavDecodedSubframe {
            alignment: GpsL1CaLnavSubframeAlignment {
                start_bit_index: 0,
                end_bit_index_exclusive: GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS,
                start_prompt_index: 0,
                end_prompt_index_exclusive: GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS
                    * GPS_L1CA_NAV_BIT_LENGTH_MS,
                inverted: false,
                word_count: words.len(),
                parity_ok_count: words.len(),
            },
            tlm: decode_tlm_word(&words[0]),
            clock,
            orbit_subframe_2,
            orbit_subframe_3,
            how,
            parity: summarize_word_parity(&words),
            word_parity_ok: vec![true; words.len()],
        }
    }

    #[test]
    fn navigation_symbol_sync_confidence_prefers_coherent_boundaries() {
        let mut prompt = vec![0.2_f32; 6];
        prompt.extend(std::iter::repeat_n(1.0_f32, GPS_L1CA_NAV_BIT_LENGTH_MS));
        prompt.extend(std::iter::repeat_n(-1.0_f32, GPS_L1CA_NAV_BIT_LENGTH_MS));
        prompt.extend(std::iter::repeat_n(1.0_f32, GPS_L1CA_NAV_BIT_LENGTH_MS));

        let demodulation = demodulate_gps_l1ca_navigation_bits(&prompt);

        assert_eq!(demodulation.bit_start_ms, 6);
        assert_eq!(demodulation.complete_window_count, 3);
        assert!(
            demodulation.sync_confidence > 0.05,
            "expected a measurable boundary margin: {demodulation:?}"
        );
        assert!(
            demodulation.bits.iter().all(|bit| bit.confidence >= 0.99),
            "clean bits should have high polarity confidence: {demodulation:?}"
        );
    }

    #[test]
    fn navigation_symbol_sync_confidence_rejects_ambiguous_constant_prompt_history() {
        let prompt = vec![1.0_f32; GPS_L1CA_NAV_BIT_LENGTH_MS * 3];

        let demodulation = demodulate_gps_l1ca_navigation_bits(&prompt);

        assert_eq!(demodulation.sync_confidence, 0.0);
        assert_eq!(demodulation.best_offset_metric, demodulation.next_best_offset_metric);
        assert!(
            demodulation.bits.iter().all(|bit| bit.confidence >= 0.99),
            "constant prompt history has confident signs but ambiguous boundaries: {demodulation:?}"
        );
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
        let bits = encode_subframe(2, 2).into_iter().map(|bit| -bit).collect::<Vec<_>>();

        let aligned = align_gps_l1ca_lnav_subframes(&bits, 0);

        assert_eq!(aligned.len(), 1, "aligned={aligned:?}");
        assert_eq!(aligned[0].start_bit_index, 0);
        assert!(aligned[0].inverted);
        assert_eq!(aligned[0].word_count, 10);
        assert!(aligned[0].parity_ok_count > 0, "aligned={aligned:?}");
    }

    #[test]
    fn prompt_history_alignment_uses_demodulated_bit_offset() {
        let mut prompt = vec![0.25_f32; 7];
        for bit in encode_subframe(3, 3) {
            prompt.extend(std::iter::repeat_n(bit as f32, GPS_L1CA_NAV_BIT_LENGTH_MS));
        }

        let alignment = align_gps_l1ca_lnav_from_prompt(&prompt);

        assert_eq!(alignment.bit_start_ms, 7);
        assert_eq!(alignment.bit_count, GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS);
        assert_eq!(alignment.subframes.len(), 1, "alignment={alignment:?}");
        assert_eq!(alignment.subframes[0].start_bit_index, 0);
        assert_eq!(alignment.subframes[0].start_prompt_index, 7);
        assert_eq!(
            alignment.subframes[0].end_prompt_index_exclusive,
            7 + GPS_L1CA_LNAV_SUBFRAME_LENGTH_BITS * GPS_L1CA_NAV_BIT_LENGTH_MS
        );
    }

    #[test]
    fn decoded_tlm_word_reports_preamble_and_parity() {
        let bits = encode_subframe(1, 1);
        let words = decode_words(&bits);
        let tlm = decode_tlm_word(&words[0]);

        assert_eq!(tlm.preamble, 0x8B);
        assert!(tlm.parity_ok, "tlm={tlm:?}");
    }

    #[test]
    fn decoded_how_word_reports_tow_flags_and_subframe_id() {
        let bits = encode_subframe_with_how(4, 12_345, true, true);
        let words = decode_words(&bits);
        let how = decode_how_word(&words[1]);

        assert_eq!(how.tow_count, 12_345);
        assert_eq!(how.tow_start_s, 74_070);
        assert!(how.alert, "how={how:?}");
        assert!(how.anti_spoof, "how={how:?}");
        assert_eq!(how.subframe_id, 4);
        assert!(how.parity_ok, "how={how:?}");
    }

    #[test]
    fn decoded_subframes_emit_control_words_and_word_parity() {
        let decoded =
            decode_gps_l1ca_lnav_subframes(&encode_subframe_with_how(5, 54_321, true, false), 11);

        assert_eq!(decoded.len(), 1, "decoded={decoded:?}");
        assert_eq!(decoded[0].alignment.start_prompt_index, 11);
        assert_eq!(decoded[0].tlm.preamble, 0x8B);
        assert_eq!(decoded[0].how.tow_count, 54_321);
        assert_eq!(decoded[0].how.subframe_id, 5);
        assert!(decoded[0].how.alert);
        assert!(!decoded[0].how.anti_spoof);
        assert_eq!(decoded[0].word_parity_ok.len(), 10);
        assert_eq!(decoded[0].word_parity_ok[0], decoded[0].tlm.parity_ok);
        assert_eq!(decoded[0].word_parity_ok[1], decoded[0].how.parity_ok);
        assert_eq!(decoded[0].parity.word_count, 10);
        assert_eq!(
            decoded[0].parity.passed_word_count + decoded[0].parity.failed_word_indexes.len(),
            10
        );
    }

    #[test]
    fn decoded_subframes_emit_subframe_1_clock_fields() {
        let decoded = decode_gps_l1ca_lnav_subframes(
            &encode_subframe_1_with_clock(ClockSubframeRequest {
                tow_count: 3,
                week: 987,
                sv_accuracy: 2,
                sv_health: 0b10_1101,
                iodc: 0x2AB,
                toc_raw: 21_600,
                af2_raw: -12,
                af1_raw: 3_210,
                af0_raw: -123_456,
                tgd_raw: -20,
            }),
            4,
        );

        assert_eq!(decoded.len(), 1, "decoded={decoded:?}");
        let clock = decoded[0].clock.as_ref().expect("subframe 1 clock");
        assert_eq!(decoded[0].how.subframe_id, 1);
        assert_eq!(clock.week, 987);
        assert_eq!(clock.sv_accuracy, 2);
        assert_eq!(clock.sv_health, 0b10_1101);
        assert_eq!(clock.iodc, 0x2AB);
        assert!((clock.toc_s - 345_600.0).abs() < f64::EPSILON);
        assert!((clock.af2 - -12.0 * 2f64.powi(-55)).abs() < f64::EPSILON);
        assert!((clock.af1 - 3_210.0 * 2f64.powi(-43)).abs() < f64::EPSILON);
        assert!((clock.af0 - -123_456.0 * 2f64.powi(-31)).abs() < f64::EPSILON);
        assert!((clock.tgd - -20.0 * 2f64.powi(-31)).abs() < f64::EPSILON);
    }

    #[test]
    fn decoded_subframes_emit_subframe_2_and_3_orbit_fields() {
        let mut bits = encode_subframe_2_with_orbit(OrbitSubframe2Request {
            tow_count: 3,
            iode: 0xA5,
            crs_raw: -512,
            delta_n_raw: 1234,
            m0_raw: -0x1234_5678,
            cuc_raw: -777,
            e_raw: 0x0123_4567,
            cus_raw: 911,
            sqrt_a_raw: 0x0056_789A,
            toe_raw: 21_600,
        });
        bits.extend(encode_subframe_3_with_orbit(OrbitSubframe3Request {
            tow_count: 4,
            iode: 0x5A,
            cic_raw: -321,
            omega0_raw: 0x2345_6789_u32 as i32,
            cis_raw: 654,
            i0_raw: -0x1234_0000,
            crc_raw: 2047,
            w_raw: 0x1112_1314_u32 as i32,
            omegadot_raw: -0x34567,
            idot_raw: 0x1234,
        }));

        let decoded = decode_gps_l1ca_lnav_subframes(&bits, 2);

        assert_eq!(decoded.len(), 2, "decoded={decoded:?}");
        assert_eq!(decoded[0].how.subframe_id, 2);
        assert!(decoded[0].clock.is_none());
        assert!(decoded[0].orbit_subframe_3.is_none());
        let subframe_2 = decoded[0].orbit_subframe_2.as_ref().expect("subframe 2 orbit");
        assert_eq!(subframe_2.iode, 0xA5);

        assert_eq!(decoded[1].how.subframe_id, 3);
        assert!(decoded[1].clock.is_none());
        assert!(decoded[1].orbit_subframe_2.is_none());
        let subframe_3 = decoded[1].orbit_subframe_3.as_ref().expect("subframe 3 orbit");
        assert_eq!(subframe_3.iode, 0x5A);
    }

    #[test]
    fn decoded_subframes_assemble_broadcast_ephemeris() {
        let decoded = vec![
            decoded_subframe_from_hex(
                "8b0284a1b8a52850000724918b913e21a92dc6ee0b217b0c00ffb72fac04",
            ),
            decoded_subframe_from_hex(
                "8b0284a1b92b21fac82899520ec7b7fb31061550921d09a10d62b87b0c7c",
            ),
            decoded_subframe_from_hex(
                "8b0284a1b9adff7d74db71f3ffaa2840ed6e0fe024cddf1effadc82106c4",
            ),
        ];
        let (ephemerides, rejections) =
            ephemerides_from_decoded_gps_l1ca_lnav(1, &decoded, Some(2209));

        assert!(rejections.is_empty(), "rejections={rejections:?}");
        assert_eq!(ephemerides.len(), 1, "ephemerides={ephemerides:?}");
        assert_eq!(ephemerides[0].sat.prn, 1);
        assert_eq!(ephemerides[0].week, 2209);
        assert_eq!(ephemerides[0].iodc, 33);
        assert_eq!(ephemerides[0].iode, 33);
        assert!((ephemerides[0].toe_s - 504_000.0).abs() < 16.0);
        assert!((ephemerides[0].toc_s - 504_000.0).abs() < 1.0);
    }

    #[test]
    fn decode_subframes_refuse_mixed_issue_numbers() {
        let mut bits = encode_subframe_1_with_clock(ClockSubframeRequest {
            tow_count: 1,
            week: 42,
            sv_accuracy: 0,
            sv_health: 0,
            iodc: 0x1A5,
            toc_raw: 21_600,
            af2_raw: 0,
            af1_raw: 0,
            af0_raw: 0,
            tgd_raw: 0,
        });
        bits.extend(encode_subframe_2_with_orbit(OrbitSubframe2Request {
            tow_count: 2,
            iode: 0xA5,
            crs_raw: -512,
            delta_n_raw: 1234,
            m0_raw: -0x1234_5678,
            cuc_raw: -777,
            e_raw: 0x0123_4567,
            cus_raw: 911,
            sqrt_a_raw: 0x0056_789A,
            toe_raw: 21_600,
        }));
        bits.extend(encode_subframe_3_with_orbit(OrbitSubframe3Request {
            tow_count: 3,
            iode: 0x22,
            cic_raw: -321,
            omega0_raw: 0x2345_6789_u32 as i32,
            cis_raw: 654,
            i0_raw: -0x1234_0000,
            crc_raw: 2047,
            w_raw: 0x1112_1314_u32 as i32,
            omegadot_raw: -0x34567,
            idot_raw: 0x1234,
        }));

        let (ephemerides, stats) = decode_subframes(&bits, None);

        assert!(ephemerides.is_empty(), "ephemerides={ephemerides:?}");
        assert_eq!(stats.ephemeris_rejections.len(), 1, "stats={stats:?}");
        assert_eq!(
            stats.ephemeris_rejections[0].reason,
            crate::formats::lnav_decode::GpsL1CaLnavEphemerisRejectionReason::IodeMismatch
        );
        assert_eq!(stats.ephemeris_rejections[0].existing_iode, Some(0xA5));
        assert_ne!(
            stats.ephemeris_rejections[0].incoming_iode,
            stats.ephemeris_rejections[0].existing_iode
        );
    }

    #[test]
    fn decoded_subframes_refuse_mixed_issue_numbers() {
        let mut bits = encode_subframe_1_with_clock(ClockSubframeRequest {
            tow_count: 1,
            week: 42,
            sv_accuracy: 0,
            sv_health: 0,
            iodc: 0x1A5,
            toc_raw: 21_600,
            af2_raw: 0,
            af1_raw: 0,
            af0_raw: 0,
            tgd_raw: 0,
        });
        bits.extend(encode_subframe_2_with_orbit(OrbitSubframe2Request {
            tow_count: 2,
            iode: 0xA5,
            crs_raw: -512,
            delta_n_raw: 1234,
            m0_raw: -0x1234_5678,
            cuc_raw: -777,
            e_raw: 0x0123_4567,
            cus_raw: 911,
            sqrt_a_raw: 0x0056_789A,
            toe_raw: 21_600,
        }));
        bits.extend(encode_subframe_3_with_orbit(OrbitSubframe3Request {
            tow_count: 3,
            iode: 0x22,
            cic_raw: -321,
            omega0_raw: 0x2345_6789_u32 as i32,
            cis_raw: 654,
            i0_raw: -0x1234_0000,
            crc_raw: 2047,
            w_raw: 0x1112_1314_u32 as i32,
            omegadot_raw: -0x34567,
            idot_raw: 0x1234,
        }));

        let decoded = decode_gps_l1ca_lnav_subframes(&bits, 0);
        let (ephemerides, rejections) =
            ephemerides_from_decoded_gps_l1ca_lnav(12, &decoded, Some(2209));

        assert!(ephemerides.is_empty(), "ephemerides={ephemerides:?}");
        assert_eq!(rejections.len(), 1, "rejections={rejections:?}");
        assert_eq!(
            rejections[0].reason,
            crate::formats::lnav_decode::GpsL1CaLnavEphemerisRejectionReason::IodeMismatch
        );
        assert_eq!(rejections[0].existing_iode, Some(0xA5));
        assert_ne!(rejections[0].incoming_iode, rejections[0].existing_iode);
    }

    #[test]
    fn corrupted_payload_bit_fails_word_parity() {
        let mut bits = encode_single_word_signed(0xABCDE);
        flip_signed_bit(&mut bits, 12);

        let words = decode_words(&bits);

        assert_eq!(words.len(), 1);
        assert!(!words[0].parity_ok, "words={words:?}");
        let parity = summarize_word_parity(&words);
        assert_eq!(parity.failed_word_indexes, vec![0]);
    }

    #[test]
    fn corrupted_parity_bit_fails_word_parity() {
        let mut bits = encode_single_word_signed(0xABCDE);
        flip_signed_bit(&mut bits, 29);

        let words = decode_words(&bits);

        assert_eq!(words.len(), 1);
        assert!(!words[0].parity_ok, "words={words:?}");
        let parity = summarize_word_parity(&words);
        assert_eq!(parity.failed_word_indexes, vec![0]);
    }
}
