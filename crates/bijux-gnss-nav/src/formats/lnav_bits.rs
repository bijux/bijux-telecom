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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaLnavDecodedSubframe {
    pub alignment: GpsL1CaLnavSubframeAlignment,
    pub tlm: GpsL1CaTlmWord,
    pub how: GpsL1CaHowWord,
    pub parity: GpsL1CaWordParitySummary,
    pub word_parity_ok: Vec<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GpsL1CaLnavDecodedStream {
    pub bit_start_ms: usize,
    pub bit_count: usize,
    pub subframes: Vec<GpsL1CaLnavDecodedSubframe>,
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
            Some(GpsL1CaLnavDecodedSubframe {
                tlm: decode_tlm_word(&subframe.words[0]),
                how: decode_how_word(&subframe.words[1]),
                parity: summarize_word_parity(&subframe.words),
                word_parity_ok: subframe.words.iter().map(|word| word.parity_ok).collect(),
                alignment: subframe.alignment,
            })
        })
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
            let info = subframe_info_from_how(&subframe.words[1]);
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

    fn flip_signed_bit(bits: &mut [i8], bit_index: usize) {
        bits[bit_index] *= -1;
    }

    fn encode_single_word_signed(data: u32) -> Vec<i8> {
        encode_word(data, 0, 0)
            .into_iter()
            .map(|bit| if bit == 1 { 1 } else { -1 })
            .collect()
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
        let decoded = decode_gps_l1ca_lnav_subframes(
            &encode_subframe_with_how(5, 54_321, true, false),
            11,
        );

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
        assert_eq!(decoded[0].parity.passed_word_count + decoded[0].parity.failed_word_indexes.len(), 10);
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
