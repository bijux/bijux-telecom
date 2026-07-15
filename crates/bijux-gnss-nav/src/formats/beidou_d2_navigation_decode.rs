#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::beidou::{
    BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
    BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
};
use crate::time::{resolve_beidou_week_rollover, RolloverResolutionError};

const BEIDOU_D2_PAGE_BITS: usize = 300;
const BEIDOU_D2_PREAMBLE: u16 = 0b111_0001_0010;
const BEIDOU_D2_SUBFRAME_ID: u8 = 1;
const BCH_GENERATOR: u16 = 0b1_0011;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2ParitySummary {
    pub corrected_blocks: u8,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2Page {
    pub page_number: u8,
    pub sow_s: u32,
    pub bits: Vec<u8>,
    pub parity: BeidouD2ParitySummary,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouD2ClockStatusPage {
    pub bdt: BeidouSystemTime,
    pub urai: u8,
    pub signal_health: BeidouSignalHealth,
    pub toc_s: f64,
    pub aodc: u8,
    pub tgd1_s: f64,
    pub tgd2_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouD2IonosphericPage {
    pub ionosphere: BeidouIonosphericCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouD2ClockCorrectionPage {
    pub af0: f64,
    pub af1_msb4: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouD2ClockContinuationPage {
    pub af1_middle6: u8,
    pub af1_lsb12: u16,
    pub af2: f64,
    pub aode: u8,
    pub delta_n: f64,
    pub cuc_msb14: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouD2EphemerisPages {
    pub aode: u8,
    pub toe_s: f64,
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BeidouD2EphemerisRejectionReason {
    DuplicatePage,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2EphemerisRejection {
    pub reason: BeidouD2EphemerisRejectionReason,
    pub page_number: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BeidouD2NavigationRejectionReason {
    InvalidSatelliteId,
    DuplicatePage,
    AmbiguousWeekRollover,
    InvalidWeekRollover,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2NavigationRejection {
    pub reason: BeidouD2NavigationRejectionReason,
    pub sat: Option<SatId>,
    pub page_number: Option<u8>,
    pub incoming_sow_s: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BeidouD2PageRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    InvalidPreamble,
    InvalidSubframeId,
    InvalidPageNumber,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD2PageRejection {
    pub reason: BeidouD2PageRejectionReason,
    pub bit_index: Option<usize>,
    pub subframe_id: Option<u8>,
    pub page_number: Option<u8>,
}

impl BeidouD2Page {
    pub fn unsigned_bits(&self, start: usize, len: usize) -> u64 {
        unsigned_bits(&self.bits, start, len)
    }

    pub fn signed_bits(&self, start: usize, len: usize) -> i64 {
        signed_bits(&self.bits, start, len)
    }

    pub fn concat_unsigned_bits(&self, parts: &[(usize, usize)]) -> u64 {
        concat_unsigned_bits(&self.bits, parts)
    }

    pub fn concat_signed_bits(&self, parts: &[(usize, usize)]) -> i64 {
        concat_signed_bits(&self.bits, parts)
    }
}

pub fn decode_beidou_d2_page(bits: &[u8]) -> Result<BeidouD2Page, BeidouD2PageRejection> {
    let normalized = normalize_bits(bits)?;
    let corrected = correct_bch_blocks(normalized);
    validate_header(&corrected.bits)?;
    let page_number = unsigned_bits(&corrected.bits, 43, 4) as u8;
    if !(1..=10).contains(&page_number) {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidPageNumber,
            bit_index: None,
            subframe_id: Some(subframe_id(&corrected.bits)),
            page_number: Some(page_number),
        });
    }
    Ok(BeidouD2Page {
        page_number,
        sow_s: sow_s(&corrected.bits),
        bits: corrected.bits.to_vec(),
        parity: BeidouD2ParitySummary { corrected_blocks: corrected.corrected_blocks },
    })
}

pub fn decode_beidou_d2_clock_status_page(page: &BeidouD2Page) -> Option<BeidouD2ClockStatusPage> {
    (page.page_number == 1).then(|| BeidouD2ClockStatusPage {
        bdt: BeidouSystemTime { week: page.unsigned_bits(65, 13) as u16, sow_s: page.sow_s },
        signal_health: BeidouSignalHealth {
            autonomous_satellite_good: page.unsigned_bits(47, 1) == 0,
        },
        aodc: page.unsigned_bits(48, 5) as u8,
        urai: page.unsigned_bits(61, 4) as u8,
        toc_s: page.concat_unsigned_bits(&[(78, 5), (91, 12)]) as f64 * 8.0,
        tgd1_s: page.signed_bits(103, 10) as f64 * 1.0e-10,
        tgd2_s: page.signed_bits(121, 10) as f64 * 1.0e-10,
    })
}

pub fn decode_beidou_d2_ionospheric_page(page: &BeidouD2Page) -> Option<BeidouD2IonosphericPage> {
    (page.page_number == 2).then(|| BeidouD2IonosphericPage {
        ionosphere: BeidouIonosphericCorrection {
            alpha0: page.concat_signed_bits(&[(47, 6), (61, 2)]) as f64 * 2f64.powi(-30),
            alpha1: page.signed_bits(63, 8) as f64 * 2f64.powi(-27),
            alpha2: page.signed_bits(71, 8) as f64 * 2f64.powi(-24),
            alpha3: page.concat_signed_bits(&[(79, 4), (91, 4)]) as f64 * 2f64.powi(-24),
            beta0: page.signed_bits(95, 8) as f64 * 2f64.powi(11),
            beta1: page.signed_bits(103, 8) as f64 * 2f64.powi(14),
            beta2: page.concat_signed_bits(&[(111, 2), (121, 6)]) as f64 * 2f64.powi(16),
            beta3: page.signed_bits(127, 8) as f64 * 2f64.powi(16),
        },
    })
}

pub fn decode_beidou_d2_clock_correction_page(
    page: &BeidouD2Page,
) -> Option<BeidouD2ClockCorrectionPage> {
    (page.page_number == 3).then(|| BeidouD2ClockCorrectionPage {
        af0: page.concat_signed_bits(&[(91, 12), (121, 12)]) as f64 * 2f64.powi(-33),
        af1_msb4: page.unsigned_bits(133, 4) as u8,
    })
}

pub fn decode_beidou_d2_clock_continuation_page(
    page: &BeidouD2Page,
) -> Option<BeidouD2ClockContinuationPage> {
    (page.page_number == 4).then(|| BeidouD2ClockContinuationPage {
        af1_middle6: page.unsigned_bits(47, 6) as u8,
        af1_lsb12: page.unsigned_bits(61, 12) as u16,
        af2: page.concat_signed_bits(&[(73, 10), (91, 1)]) as f64 * 2f64.powi(-66),
        aode: page.unsigned_bits(92, 5) as u8,
        delta_n: page.signed_bits(97, 16) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        cuc_msb14: page.unsigned_bits(121, 14) as u16,
    })
}

pub fn decode_beidou_d2_ephemeris_pages(
    pages: &[BeidouD2Page],
) -> Result<Option<BeidouD2EphemerisPages>, BeidouD2EphemerisRejection> {
    let mut builder = BeidouD2EphemerisBuilder::default();
    for page in pages {
        builder.merge(page)?;
    }
    Ok(builder.try_build())
}

pub fn decode_beidou_d2_broadcast_navigation_data(
    sat: SatId,
    pages: &[BeidouD2Page],
) -> Result<Option<BeidouBroadcastNavigationData>, BeidouD2NavigationRejection> {
    decode_beidou_d2_broadcast_navigation_data_inner(sat, pages, None)
}

pub fn decode_beidou_d2_broadcast_navigation_data_with_reference_week(
    sat: SatId,
    pages: &[BeidouD2Page],
    reference_week: u32,
) -> Result<Option<BeidouBroadcastNavigationData>, BeidouD2NavigationRejection> {
    decode_beidou_d2_broadcast_navigation_data_inner(sat, pages, Some(reference_week))
}

fn decode_beidou_d2_broadcast_navigation_data_inner(
    sat: SatId,
    pages: &[BeidouD2Page],
    reference_week: Option<u32>,
) -> Result<Option<BeidouBroadcastNavigationData>, BeidouD2NavigationRejection> {
    if sat.constellation != Constellation::Beidou {
        return Err(BeidouD2NavigationRejection {
            reason: BeidouD2NavigationRejectionReason::InvalidSatelliteId,
            sat: Some(sat),
            page_number: None,
            incoming_sow_s: None,
        });
    }

    let mut builder = BeidouD2NavigationBuilder::new(sat).with_reference_week(reference_week);
    for page in pages {
        builder.merge(page)?;
    }
    builder.try_build_checked()
}

fn normalize_bits(bits: &[u8]) -> Result<[u8; BEIDOU_D2_PAGE_BITS], BeidouD2PageRejection> {
    if bits.len() != BEIDOU_D2_PAGE_BITS {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidBitCount,
            bit_index: None,
            subframe_id: None,
            page_number: None,
        });
    }

    let mut normalized = [0_u8; BEIDOU_D2_PAGE_BITS];
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(BeidouD2PageRejection {
                reason: BeidouD2PageRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                subframe_id: None,
                page_number: None,
            });
        }
        normalized[idx] = bit;
    }
    Ok(normalized)
}

#[derive(Debug, Clone, Copy)]
struct CorrectedD2Bits {
    bits: [u8; BEIDOU_D2_PAGE_BITS],
    corrected_blocks: u8,
}

fn correct_bch_blocks(mut bits: [u8; BEIDOU_D2_PAGE_BITS]) -> CorrectedD2Bits {
    let mut corrected_blocks = 0_u8;
    if correct_bch_codeword(&mut bits, 16).is_some() {
        corrected_blocks += 1;
    }
    for word_index in 1..10 {
        let start = word_index * 30 + 1;
        if correct_bch_split_codeword(&mut bits, start, start + 22).is_some() {
            corrected_blocks += 1;
        }
        if correct_bch_split_codeword(&mut bits, start + 11, start + 26).is_some() {
            corrected_blocks += 1;
        }
    }
    CorrectedD2Bits { bits, corrected_blocks }
}

fn correct_bch_codeword(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize) -> Option<usize> {
    let mut codeword = unsigned_bits(bits, start, 15) as u16;
    let syndrome = bch_syndrome(codeword);
    if syndrome == 0 {
        return None;
    }
    for offset in 0..15 {
        let mask = 1_u16 << (14 - offset);
        if bch_syndrome(codeword ^ mask) == 0 {
            codeword ^= mask;
            set_unsigned_bits(bits, start, 15, u64::from(codeword));
            return Some(start + offset);
        }
    }
    None
}

fn correct_bch_split_codeword(
    bits: &mut [u8; BEIDOU_D2_PAGE_BITS],
    info_start: usize,
    parity_start: usize,
) -> Option<usize> {
    let mut codeword = ((unsigned_bits(bits, info_start, 11) as u16) << 4)
        | unsigned_bits(bits, parity_start, 4) as u16;
    let syndrome = bch_syndrome(codeword);
    if syndrome == 0 {
        return None;
    }
    for offset in 0..15 {
        let mask = 1_u16 << (14 - offset);
        if bch_syndrome(codeword ^ mask) == 0 {
            codeword ^= mask;
            set_unsigned_bits(bits, info_start, 11, u64::from((codeword >> 4) & 0x07FF));
            set_unsigned_bits(bits, parity_start, 4, u64::from(codeword & 0x000F));
            return Some(if offset < 11 {
                info_start + offset
            } else {
                parity_start + offset - 11
            });
        }
    }
    None
}

fn bch_syndrome(codeword: u16) -> u16 {
    let mut value = codeword;
    for degree in (4..=14).rev() {
        if (value & (1_u16 << degree)) != 0 {
            value ^= BCH_GENERATOR << (degree - 4);
        }
    }
    value & 0x0F
}

fn validate_header(bits: &[u8; BEIDOU_D2_PAGE_BITS]) -> Result<(), BeidouD2PageRejection> {
    if unsigned_bits(bits, 1, 11) as u16 != BEIDOU_D2_PREAMBLE {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidPreamble,
            bit_index: Some(0),
            subframe_id: Some(subframe_id(bits)),
            page_number: Some(unsigned_bits(bits, 43, 4) as u8),
        });
    }
    let actual_subframe_id = subframe_id(bits);
    if actual_subframe_id != BEIDOU_D2_SUBFRAME_ID {
        return Err(BeidouD2PageRejection {
            reason: BeidouD2PageRejectionReason::InvalidSubframeId,
            bit_index: None,
            subframe_id: Some(actual_subframe_id),
            page_number: Some(unsigned_bits(bits, 43, 4) as u8),
        });
    }
    Ok(())
}

fn subframe_id(bits: &[u8]) -> u8 {
    unsigned_bits(bits, 16, 3) as u8
}

fn sow_s(bits: &[u8]) -> u32 {
    concat_unsigned_bits(bits, &[(19, 8), (31, 12)]) as u32
}

fn bit(bits: &[u8], position: usize) -> u8 {
    bits[position - 1]
}

fn unsigned_bits(bits: &[u8], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0_u64, |acc, position| (acc << 1) | u64::from(bit(bits, position)))
}

fn signed_bits(bits: &[u8], start: usize, len: usize) -> i64 {
    signed_from_unsigned(unsigned_bits(bits, start, len), len)
}

fn concat_unsigned_bits(bits: &[u8], parts: &[(usize, usize)]) -> u64 {
    parts.iter().fold(0_u64, |acc, (start, len)| (acc << len) | unsigned_bits(bits, *start, *len))
}

fn concat_signed_bits(bits: &[u8], parts: &[(usize, usize)]) -> i64 {
    let total_bits = parts.iter().map(|(_, len)| *len).sum();
    signed_from_unsigned(concat_unsigned_bits(bits, parts), total_bits)
}

fn signed_from_unsigned(value: u64, bits: usize) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

#[derive(Debug, Default, Clone)]
struct BeidouD2EphemerisBuilder {
    page_4: Option<BeidouD2ClockContinuationPage>,
    page_5: Option<BeidouD2Page>,
    page_6: Option<BeidouD2Page>,
    page_7: Option<BeidouD2Page>,
    page_8: Option<BeidouD2Page>,
    page_9: Option<BeidouD2Page>,
    page_10: Option<BeidouD2Page>,
}

impl BeidouD2EphemerisBuilder {
    fn merge(&mut self, page: &BeidouD2Page) -> Result<(), BeidouD2EphemerisRejection> {
        match page.page_number {
            4 => {
                if self.page_4.is_some() {
                    return Err(duplicate_ephemeris_page(4));
                }
                self.page_4 = decode_beidou_d2_clock_continuation_page(page);
            }
            5 => merge_page_slot(&mut self.page_5, page)?,
            6 => merge_page_slot(&mut self.page_6, page)?,
            7 => merge_page_slot(&mut self.page_7, page)?,
            8 => merge_page_slot(&mut self.page_8, page)?,
            9 => merge_page_slot(&mut self.page_9, page)?,
            10 => merge_page_slot(&mut self.page_10, page)?,
            _ => {}
        }
        Ok(())
    }

    fn try_build(&self) -> Option<BeidouD2EphemerisPages> {
        let (
            Some(page_4),
            Some(page_5),
            Some(page_6),
            Some(page_7),
            Some(page_8),
            Some(page_9),
            Some(page_10),
        ) = (
            self.page_4,
            self.page_5.as_ref(),
            self.page_6.as_ref(),
            self.page_7.as_ref(),
            self.page_8.as_ref(),
            self.page_9.as_ref(),
            self.page_10.as_ref(),
        )
        else {
            return None;
        };

        let cuc_raw = ((u64::from(page_4.cuc_msb14)) << 4) | page_5.unsigned_bits(47, 4);
        let m0_raw = page_5.concat_unsigned_bits(&[(51, 2), (61, 22), (91, 8)]);
        let cus_raw = page_5.concat_unsigned_bits(&[(99, 14), (121, 4)]);
        let e_raw = (page_5.unsigned_bits(125, 10) << 22) | page_6.unsigned_bits(61, 22);
        let sqrt_a_raw = page_6.concat_unsigned_bits(&[(91, 22), (121, 10)]);
        let crs_raw = (page_6.unsigned_bits(131, 12) << 6) | page_7.unsigned_bits(47, 6);
        let toe_raw = page_7.unsigned_bits(61, 17);
        let i0_raw = page_7.concat_unsigned_bits(&[(78, 5), (91, 22), (121, 5)]);
        let cic_raw = (page_7.unsigned_bits(126, 17) << 1) | page_8.unsigned_bits(47, 1);
        let omegadot_raw = page_8.concat_unsigned_bits(&[(48, 5), (61, 19)]);
        let cis_raw = page_8.concat_unsigned_bits(&[(80, 3), (91, 15)]);
        let idot_raw = page_8.concat_unsigned_bits(&[(106, 7), (121, 7)]);
        let crc_raw = (page_8.unsigned_bits(128, 15) << 3) | page_9.unsigned_bits(47, 3);
        let omega0_raw = page_9.concat_unsigned_bits(&[(50, 3), (61, 22), (91, 7)]);
        let w_raw = page_9.concat_unsigned_bits(&[(98, 15), (121, 17)]);
        let _page_10_time_tag = page_10.sow_s;

        Some(BeidouD2EphemerisPages {
            aode: page_4.aode,
            toe_s: toe_raw as f64 * 8.0,
            sqrt_a: sqrt_a_raw as f64 * 2f64.powi(-19),
            e: e_raw as f64 * 2f64.powi(-33),
            i0: signed_from_unsigned(i0_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
            idot: signed_from_unsigned(idot_raw, 14) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
            omega0: signed_from_unsigned(omega0_raw, 32) as f64
                * 2f64.powi(-31)
                * std::f64::consts::PI,
            omegadot: signed_from_unsigned(omegadot_raw, 24) as f64
                * 2f64.powi(-43)
                * std::f64::consts::PI,
            w: signed_from_unsigned(w_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
            m0: signed_from_unsigned(m0_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
            delta_n: page_4.delta_n,
            cuc: signed_from_unsigned(cuc_raw, 18) as f64 * 2f64.powi(-31),
            cus: signed_from_unsigned(cus_raw, 18) as f64 * 2f64.powi(-31),
            crc: signed_from_unsigned(crc_raw, 18) as f64 * 2f64.powi(-6),
            crs: signed_from_unsigned(crs_raw, 18) as f64 * 2f64.powi(-6),
            cic: signed_from_unsigned(cic_raw, 18) as f64 * 2f64.powi(-31),
            cis: signed_from_unsigned(cis_raw, 18) as f64 * 2f64.powi(-31),
        })
    }
}

fn merge_page_slot(
    slot: &mut Option<BeidouD2Page>,
    page: &BeidouD2Page,
) -> Result<(), BeidouD2EphemerisRejection> {
    if slot.is_some() {
        return Err(duplicate_ephemeris_page(page.page_number));
    }
    *slot = Some(page.clone());
    Ok(())
}

fn duplicate_ephemeris_page(page_number: u8) -> BeidouD2EphemerisRejection {
    BeidouD2EphemerisRejection {
        reason: BeidouD2EphemerisRejectionReason::DuplicatePage,
        page_number,
    }
}

#[derive(Debug, Default, Clone)]
struct BeidouD2NavigationBuilder {
    sat: Option<SatId>,
    reference_week: Option<u32>,
    seen_pages: [bool; 11],
    clock_status: Option<BeidouD2ClockStatusPage>,
    ionosphere: Option<BeidouD2IonosphericPage>,
    clock_correction: Option<BeidouD2ClockCorrectionPage>,
    clock_continuation: Option<BeidouD2ClockContinuationPage>,
    pages: Vec<BeidouD2Page>,
}

impl BeidouD2NavigationBuilder {
    fn new(sat: SatId) -> Self {
        Self { sat: Some(sat), ..Self::default() }
    }

    fn with_reference_week(mut self, reference_week: Option<u32>) -> Self {
        self.reference_week = reference_week;
        self
    }

    fn merge(&mut self, page: &BeidouD2Page) -> Result<(), BeidouD2NavigationRejection> {
        if self.seen_pages[usize::from(page.page_number)] {
            return Err(BeidouD2NavigationRejection {
                reason: BeidouD2NavigationRejectionReason::DuplicatePage,
                sat: self.sat,
                page_number: Some(page.page_number),
                incoming_sow_s: Some(page.sow_s),
            });
        }
        self.seen_pages[usize::from(page.page_number)] = true;

        match page.page_number {
            1 => self.clock_status = decode_beidou_d2_clock_status_page(page),
            2 => self.ionosphere = decode_beidou_d2_ionospheric_page(page),
            3 => self.clock_correction = decode_beidou_d2_clock_correction_page(page),
            4 => self.clock_continuation = decode_beidou_d2_clock_continuation_page(page),
            _ => {}
        }
        self.pages.push(page.clone());
        Ok(())
    }

    fn try_build_checked(
        &self,
    ) -> Result<Option<BeidouBroadcastNavigationData>, BeidouD2NavigationRejection> {
        let Some(sat) = self.sat else {
            return Ok(None);
        };
        let Some(clock_status) = self.clock_status else {
            return Ok(None);
        };
        let Some(ionosphere) = self.ionosphere else {
            return Ok(None);
        };
        let Some(clock_correction) = self.clock_correction else {
            return Ok(None);
        };
        let Some(clock_continuation) = self.clock_continuation else {
            return Ok(None);
        };
        let ephemeris = decode_beidou_d2_ephemeris_pages(&self.pages).map_err(|error| {
            BeidouD2NavigationRejection {
                reason: BeidouD2NavigationRejectionReason::DuplicatePage,
                sat: self.sat,
                page_number: Some(error.page_number),
                incoming_sow_s: None,
            }
        })?;
        let Some(ephemeris) = ephemeris else {
            return Ok(None);
        };

        let bdt = self.resolved_bdt(clock_status)?;
        let af1_raw = (u64::from(clock_correction.af1_msb4) << 18)
            | (u64::from(clock_continuation.af1_middle6) << 12)
            | u64::from(clock_continuation.af1_lsb12);
        let clock = BeidouClockCorrection {
            toc_s: clock_status.toc_s,
            aodc: clock_status.aodc,
            af0: clock_correction.af0,
            af1: signed_from_unsigned(af1_raw, 22) as f64 * 2f64.powi(-50),
            af2: clock_continuation.af2,
            tgd1_s: clock_status.tgd1_s,
            tgd2_s: clock_status.tgd2_s,
        };

        Ok(Some(BeidouBroadcastNavigationData {
            sat,
            bdt,
            urai: clock_status.urai,
            signal_health: clock_status.signal_health,
            clock,
            ephemeris: BeidouEphemeris {
                sat,
                aode: ephemeris.aode,
                toe_s: ephemeris.toe_s,
                sqrt_a: ephemeris.sqrt_a,
                e: ephemeris.e,
                i0: ephemeris.i0,
                idot: ephemeris.idot,
                omega0: ephemeris.omega0,
                omegadot: ephemeris.omegadot,
                w: ephemeris.w,
                m0: ephemeris.m0,
                delta_n: ephemeris.delta_n,
                cuc: ephemeris.cuc,
                cus: ephemeris.cus,
                crc: ephemeris.crc,
                crs: ephemeris.crs,
                cic: ephemeris.cic,
                cis: ephemeris.cis,
            },
            ionosphere: ionosphere.ionosphere,
        }))
    }

    fn resolved_bdt(
        &self,
        clock_status: BeidouD2ClockStatusPage,
    ) -> Result<BeidouSystemTime, BeidouD2NavigationRejection> {
        let Some(reference_week) = self.reference_week else {
            return Ok(clock_status.bdt);
        };
        let resolution = resolve_beidou_week_rollover(clock_status.bdt.week, reference_week)
            .map_err(|error| self.week_rollover_rejection(error, clock_status.bdt.sow_s))?;
        let Ok(week) = u16::try_from(resolution.week) else {
            return Err(self.week_rollover_rejection(
                RolloverResolutionError::TruncatedValueOutOfRange,
                clock_status.bdt.sow_s,
            ));
        };
        Ok(BeidouSystemTime { week, sow_s: clock_status.bdt.sow_s })
    }

    fn week_rollover_rejection(
        &self,
        error: RolloverResolutionError,
        incoming_sow_s: u32,
    ) -> BeidouD2NavigationRejection {
        BeidouD2NavigationRejection {
            reason: match error {
                RolloverResolutionError::AmbiguousReference => {
                    BeidouD2NavigationRejectionReason::AmbiguousWeekRollover
                }
                RolloverResolutionError::InvalidCycle
                | RolloverResolutionError::TruncatedValueOutOfRange => {
                    BeidouD2NavigationRejectionReason::InvalidWeekRollover
                }
            },
            sat: self.sat,
            page_number: Some(1),
            incoming_sow_s: Some(incoming_sow_s),
        }
    }
}

fn set_unsigned_bits(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize, len: usize, value: u64) {
    for offset in 0..len {
        let shift = len - offset - 1;
        bits[start + offset - 1] = ((value >> shift) & 1) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        bch_syndrome, decode_beidou_d2_page, set_unsigned_bits, BeidouD2PageRejectionReason,
        BEIDOU_D2_PAGE_BITS, BEIDOU_D2_PREAMBLE,
    };
    use bijux_gnss_core::api::{Constellation, SatId};

    fn set_common_header(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], page_number: u8, sow_s: u32) {
        set_unsigned_bits(bits, 1, 11, u64::from(BEIDOU_D2_PREAMBLE));
        set_unsigned_bits(bits, 16, 3, 1);
        set_unsigned_bits(bits, 19, 8, u64::from(sow_s >> 12));
        set_unsigned_bits(bits, 31, 12, u64::from(sow_s & 0x0FFF));
        set_unsigned_bits(bits, 43, 4, u64::from(page_number));
    }

    fn encode_signed(value: i64, bits: usize) -> u64 {
        let mask = if bits == 64 { u64::MAX } else { (1_u64 << bits) - 1 };
        (value as u64) & mask
    }

    fn set_signed_bits(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize, len: usize, value: i64) {
        set_unsigned_bits(bits, start, len, encode_signed(value, len));
    }

    fn set_split_signed_bits(
        bits: &mut [u8; BEIDOU_D2_PAGE_BITS],
        parts: &[(usize, usize)],
        value: i64,
    ) {
        let total_bits: usize = parts.iter().map(|(_, len)| *len).sum();
        let encoded = encode_signed(value, total_bits);
        let mut remaining = total_bits;
        for (start, len) in parts {
            remaining -= *len;
            set_unsigned_bits(bits, *start, *len, (encoded >> remaining) & ((1_u64 << len) - 1));
        }
    }

    fn apply_bch_parity(bits: &mut [u8; BEIDOU_D2_PAGE_BITS]) {
        set_codeword_parity(bits, 16);
        for word_index in 1..10 {
            let start = word_index * 30 + 1;
            set_split_codeword_parity(bits, start, start + 22);
            set_split_codeword_parity(bits, start + 11, start + 26);
        }
    }

    fn set_codeword_parity(bits: &mut [u8; BEIDOU_D2_PAGE_BITS], start: usize) {
        set_unsigned_bits(bits, start + 11, 4, 0);
        let codeword_without_parity = super::unsigned_bits(bits, start, 15) as u16;
        let parity = bch_syndrome(codeword_without_parity);
        set_unsigned_bits(bits, start + 11, 4, u64::from(parity));
    }

    fn set_split_codeword_parity(
        bits: &mut [u8; BEIDOU_D2_PAGE_BITS],
        info_start: usize,
        parity_start: usize,
    ) {
        set_unsigned_bits(bits, parity_start, 4, 0);
        let codeword_without_parity = ((super::unsigned_bits(bits, info_start, 11) as u16) << 4)
            | super::unsigned_bits(bits, parity_start, 4) as u16;
        let parity = bch_syndrome(codeword_without_parity);
        set_unsigned_bits(bits, parity_start, 4, u64::from(parity));
    }

    fn valid_page_bits(page_number: u8) -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, page_number, 345_678);
        set_unsigned_bits(&mut bits, 47, 6, 0x15);
        set_unsigned_bits(&mut bits, 61, 11, 0x321);
        apply_bch_parity(&mut bits);
        bits
    }

    fn clock_status_page_bits() -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, 1, 345_678);
        set_unsigned_bits(&mut bits, 47, 1, 0);
        set_unsigned_bits(&mut bits, 48, 5, 17);
        set_unsigned_bits(&mut bits, 61, 4, 5);
        set_unsigned_bits(&mut bits, 65, 13, 1_234);
        set_unsigned_bits(&mut bits, 78, 5, 0b10_101);
        set_unsigned_bits(&mut bits, 91, 12, 0x5A5);
        set_signed_bits(&mut bits, 103, 10, -77);
        set_signed_bits(&mut bits, 121, 10, 55);
        apply_bch_parity(&mut bits);
        bits
    }

    fn ionospheric_page_bits() -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, 2, 345_681);
        set_split_signed_bits(&mut bits, &[(47, 6), (61, 2)], -12);
        set_signed_bits(&mut bits, 63, 8, 23);
        set_signed_bits(&mut bits, 71, 8, -34);
        set_split_signed_bits(&mut bits, &[(79, 4), (91, 4)], 45);
        set_signed_bits(&mut bits, 95, 8, -56);
        set_signed_bits(&mut bits, 103, 8, 67);
        set_split_signed_bits(&mut bits, &[(111, 2), (121, 6)], -78);
        set_signed_bits(&mut bits, 127, 8, 89);
        apply_bch_parity(&mut bits);
        bits
    }

    fn clock_correction_page_bits() -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, 3, 345_684);
        set_split_signed_bits(&mut bits, &[(91, 12), (121, 12)], -0x12_345);
        set_unsigned_bits(&mut bits, 133, 4, 0b1010);
        apply_bch_parity(&mut bits);
        bits
    }

    fn decoded_page(bits: &[u8; BEIDOU_D2_PAGE_BITS]) -> super::BeidouD2Page {
        decode_beidou_d2_page(bits).expect("valid D2 page")
    }

    fn decoded_navigation_pages() -> Vec<super::BeidouD2Page> {
        let mut pages = vec![
            decoded_page(&clock_status_page_bits()),
            decoded_page(&ionospheric_page_bits()),
            decoded_page(&clock_correction_page_bits()),
            decoded_page(&clock_continuation_page_bits()),
        ];
        for page_number in 5..=10 {
            pages.push(decoded_page(&ephemeris_page_bits(page_number)));
        }
        pages
    }

    fn clock_continuation_page_bits() -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, 4, 345_687);
        set_unsigned_bits(&mut bits, 47, 6, 0b10_1010);
        set_unsigned_bits(&mut bits, 61, 12, 0x5A5);
        set_split_signed_bits(&mut bits, &[(73, 10), (91, 1)], -321);
        set_unsigned_bits(&mut bits, 92, 5, 19);
        set_signed_bits(&mut bits, 97, 16, -2_345);
        set_unsigned_bits(&mut bits, 121, 14, 0x1234);
        apply_bch_parity(&mut bits);
        bits
    }

    fn ephemeris_page_bits(page_number: u8) -> [u8; BEIDOU_D2_PAGE_BITS] {
        let mut bits = [0_u8; BEIDOU_D2_PAGE_BITS];
        set_common_header(&mut bits, page_number, 345_690 + u32::from(page_number));
        match page_number {
            5 => {
                set_unsigned_bits(&mut bits, 47, 4, 0x0A);
                set_unsigned_bits(&mut bits, 51, 2, 0b10);
                set_unsigned_bits(&mut bits, 61, 22, 0x2A_AAAA);
                set_unsigned_bits(&mut bits, 91, 8, 0x5A);
                set_unsigned_bits(&mut bits, 99, 14, 0x2AAA);
                set_unsigned_bits(&mut bits, 121, 4, 0x0B);
                set_unsigned_bits(&mut bits, 125, 10, 0x155);
            }
            6 => {
                set_unsigned_bits(&mut bits, 61, 22, 0x12_3456);
                set_unsigned_bits(&mut bits, 91, 22, 0x23_4567);
                set_unsigned_bits(&mut bits, 121, 10, 0x2AA);
                set_unsigned_bits(&mut bits, 131, 12, 0x555);
            }
            7 => {
                set_unsigned_bits(&mut bits, 47, 6, 0x25);
                set_unsigned_bits(&mut bits, 61, 17, 0x1_2345);
                set_unsigned_bits(&mut bits, 78, 5, 0x0A);
                set_unsigned_bits(&mut bits, 91, 22, 0x12_3456);
                set_unsigned_bits(&mut bits, 121, 5, 0x0B);
                set_unsigned_bits(&mut bits, 126, 17, 0x1_1111);
            }
            8 => {
                set_unsigned_bits(&mut bits, 47, 1, 1);
                set_unsigned_bits(&mut bits, 48, 5, 0x12);
                set_unsigned_bits(&mut bits, 61, 19, 0x5_4321);
                set_unsigned_bits(&mut bits, 80, 3, 0x05);
                set_unsigned_bits(&mut bits, 91, 15, 0x4567);
                set_unsigned_bits(&mut bits, 106, 7, 0x2A);
                set_unsigned_bits(&mut bits, 121, 7, 0x35);
                set_unsigned_bits(&mut bits, 128, 15, 0x1234);
            }
            9 => {
                set_unsigned_bits(&mut bits, 47, 3, 0x05);
                set_unsigned_bits(&mut bits, 50, 3, 0x03);
                set_unsigned_bits(&mut bits, 61, 22, 0x34_5678);
                set_unsigned_bits(&mut bits, 91, 7, 0x55);
                set_unsigned_bits(&mut bits, 98, 15, 0x2345);
                set_unsigned_bits(&mut bits, 121, 17, 0x1_2345);
            }
            10 => {}
            _ => unreachable!("test page number"),
        }
        apply_bch_parity(&mut bits);
        bits
    }

    #[test]
    fn d2_page_decodes_header_page_number_and_sow() {
        let page = decode_beidou_d2_page(&valid_page_bits(4)).expect("valid D2 page");

        assert_eq!(page.page_number, 4);
        assert_eq!(page.sow_s, 345_678);
        assert_eq!(page.parity.corrected_blocks, 0);
        assert_eq!(page.unsigned_bits(47, 6), 0x15);
        assert_eq!(page.unsigned_bits(61, 11), 0x321);
    }

    #[test]
    fn d2_page_corrects_single_bch_bit_error() {
        let mut bits = valid_page_bits(2);
        bits[60] ^= 1;

        let page = decode_beidou_d2_page(&bits).expect("correctable D2 page");

        assert_eq!(page.page_number, 2);
        assert_eq!(page.parity.corrected_blocks, 1);
        assert_eq!(page.unsigned_bits(61, 11), 0x321);
    }

    #[test]
    fn d2_page_rejects_invalid_shape_and_header() {
        let short = vec![0_u8; BEIDOU_D2_PAGE_BITS - 1];
        let rejection = decode_beidou_d2_page(&short).expect_err("bit count rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidBitCount);

        let mut non_binary = valid_page_bits(1);
        non_binary[42] = 2;
        let rejection = decode_beidou_d2_page(&non_binary).expect_err("binary rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::NonBinaryBit);
        assert_eq!(rejection.bit_index, Some(42));

        let mut bad_subframe = valid_page_bits(1);
        set_unsigned_bits(&mut bad_subframe, 16, 3, 2);
        apply_bch_parity(&mut bad_subframe);
        let rejection = decode_beidou_d2_page(&bad_subframe).expect_err("subframe rejection");
        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidSubframeId);
    }

    #[test]
    fn d2_page_rejects_page_numbers_outside_subcommutated_frame() {
        let mut bits = valid_page_bits(1);
        set_unsigned_bits(&mut bits, 43, 4, 0);
        apply_bch_parity(&mut bits);

        let rejection = decode_beidou_d2_page(&bits).expect_err("page rejection");

        assert_eq!(rejection.reason, BeidouD2PageRejectionReason::InvalidPageNumber);
        assert_eq!(rejection.page_number, Some(0));
    }

    #[test]
    fn clock_status_page_decodes_bdt_clock_and_delays() {
        let bits = clock_status_page_bits();
        let page = decode_beidou_d2_page(&bits).expect("valid clock page");

        let clock = super::decode_beidou_d2_clock_status_page(&page).expect("page 1");

        assert_eq!(clock.bdt.week, 1_234);
        assert_eq!(clock.bdt.sow_s, 345_678);
        assert!(clock.signal_health.autonomous_satellite_good);
        assert_eq!(clock.aodc, 17);
        assert_eq!(clock.urai, 5);
        assert_eq!(clock.toc_s, (((0b10_101_u64 << 12) | 0x5A5_u64) as f64) * 8.0);
        assert!((clock.tgd1_s - -77.0e-10).abs() < 1.0e-18);
        assert!((clock.tgd2_s - 55.0e-10).abs() < 1.0e-18);
    }

    #[test]
    fn ionospheric_page_decodes_broadcast_coefficients() {
        let bits = ionospheric_page_bits();
        let page = decode_beidou_d2_page(&bits).expect("valid ionosphere page");

        let ionosphere = super::decode_beidou_d2_ionospheric_page(&page).expect("page 2");

        assert_eq!(ionosphere.ionosphere.alpha0, -12.0 * 2f64.powi(-30));
        assert_eq!(ionosphere.ionosphere.alpha1, 23.0 * 2f64.powi(-27));
        assert_eq!(ionosphere.ionosphere.alpha2, -34.0 * 2f64.powi(-24));
        assert_eq!(ionosphere.ionosphere.alpha3, 45.0 * 2f64.powi(-24));
        assert_eq!(ionosphere.ionosphere.beta0, -56.0 * 2f64.powi(11));
        assert_eq!(ionosphere.ionosphere.beta1, 67.0 * 2f64.powi(14));
        assert_eq!(ionosphere.ionosphere.beta2, -78.0 * 2f64.powi(16));
        assert_eq!(ionosphere.ionosphere.beta3, 89.0 * 2f64.powi(16));
    }

    #[test]
    fn clock_correction_page_decodes_af0_and_af1_high_bits() {
        let bits = clock_correction_page_bits();
        let page = decode_beidou_d2_page(&bits).expect("valid clock correction page");

        let correction = super::decode_beidou_d2_clock_correction_page(&page).expect("page 3");

        assert_eq!(correction.af0, -0x12_345_i64 as f64 * 2f64.powi(-33));
        assert_eq!(correction.af1_msb4, 0b1010);
    }

    #[test]
    fn clock_continuation_page_decodes_drift_and_first_orbit_terms() {
        let bits = clock_continuation_page_bits();
        let page = decode_beidou_d2_page(&bits).expect("valid continuation page");

        let continuation = super::decode_beidou_d2_clock_continuation_page(&page).expect("page 4");

        assert_eq!(continuation.af1_middle6, 0b10_1010);
        assert_eq!(continuation.af1_lsb12, 0x5A5);
        assert_eq!(continuation.af2, -321.0 * 2f64.powi(-66));
        assert_eq!(continuation.aode, 19);
        assert_eq!(continuation.delta_n, -2_345.0 * 2f64.powi(-43) * std::f64::consts::PI);
        assert_eq!(continuation.cuc_msb14, 0x1234);
    }

    #[test]
    fn ephemeris_pages_assemble_scaled_orbit_terms() {
        let pages = [
            decoded_page(&clock_continuation_page_bits()),
            decoded_page(&ephemeris_page_bits(5)),
            decoded_page(&ephemeris_page_bits(6)),
            decoded_page(&ephemeris_page_bits(7)),
            decoded_page(&ephemeris_page_bits(8)),
            decoded_page(&ephemeris_page_bits(9)),
            decoded_page(&ephemeris_page_bits(10)),
        ];

        let ephemeris = super::decode_beidou_d2_ephemeris_pages(&pages)
            .expect("unique pages")
            .expect("complete ephemeris");

        let cuc_raw = (0x1234_u64 << 4) | 0x0A;
        let m0_raw = (0b10_u64 << 30) | (0x2A_AAAA_u64 << 8) | 0x5A;
        let cus_raw = (0x2AAA_u64 << 4) | 0x0B;
        let e_raw = (0x155_u64 << 22) | 0x12_3456;
        let sqrt_a_raw = (0x23_4567_u64 << 10) | 0x2AA;
        let crs_raw = (0x555_u64 << 6) | 0x25;
        let i0_raw = (0x0A_u64 << 27) | (0x12_3456_u64 << 5) | 0x0B;
        let cic_raw = (0x1_1111_u64 << 1) | 1;
        let omegadot_raw = (0x12_u64 << 19) | 0x5_4321;
        let cis_raw = (0x05_u64 << 15) | 0x4567;
        let idot_raw = (0x2A_u64 << 7) | 0x35;
        let crc_raw = (0x1234_u64 << 3) | 0x05;
        let omega0_raw = (0x03_u64 << 29) | (0x34_5678_u64 << 7) | 0x55;
        let w_raw = (0x2345_u64 << 17) | 0x1_2345;

        assert_eq!(ephemeris.aode, 19);
        assert_eq!(ephemeris.toe_s, 0x1_2345_u64 as f64 * 8.0);
        assert_eq!(ephemeris.sqrt_a, sqrt_a_raw as f64 * 2f64.powi(-19));
        assert_eq!(ephemeris.e, e_raw as f64 * 2f64.powi(-33));
        assert_eq!(
            ephemeris.i0,
            super::signed_from_unsigned(i0_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI
        );
        assert_eq!(
            ephemeris.idot,
            super::signed_from_unsigned(idot_raw, 14) as f64
                * 2f64.powi(-43)
                * std::f64::consts::PI
        );
        assert_eq!(
            ephemeris.omega0,
            super::signed_from_unsigned(omega0_raw, 32) as f64
                * 2f64.powi(-31)
                * std::f64::consts::PI
        );
        assert_eq!(
            ephemeris.omegadot,
            super::signed_from_unsigned(omegadot_raw, 24) as f64
                * 2f64.powi(-43)
                * std::f64::consts::PI
        );
        assert_eq!(
            ephemeris.w,
            super::signed_from_unsigned(w_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI
        );
        assert_eq!(
            ephemeris.m0,
            super::signed_from_unsigned(m0_raw, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI
        );
        assert_eq!(ephemeris.delta_n, -2_345.0 * 2f64.powi(-43) * std::f64::consts::PI);
        assert_eq!(ephemeris.cuc, super::signed_from_unsigned(cuc_raw, 18) as f64 * 2f64.powi(-31));
        assert_eq!(ephemeris.cus, super::signed_from_unsigned(cus_raw, 18) as f64 * 2f64.powi(-31));
        assert_eq!(ephemeris.crc, super::signed_from_unsigned(crc_raw, 18) as f64 * 2f64.powi(-6));
        assert_eq!(ephemeris.crs, super::signed_from_unsigned(crs_raw, 18) as f64 * 2f64.powi(-6));
        assert_eq!(ephemeris.cic, super::signed_from_unsigned(cic_raw, 18) as f64 * 2f64.powi(-31));
        assert_eq!(ephemeris.cis, super::signed_from_unsigned(cis_raw, 18) as f64 * 2f64.powi(-31));
    }

    #[test]
    fn ephemeris_pages_reject_duplicate_subcommutated_page() {
        let pages = [
            decoded_page(&clock_continuation_page_bits()),
            decoded_page(&ephemeris_page_bits(5)),
            decoded_page(&ephemeris_page_bits(5)),
        ];

        let rejection =
            super::decode_beidou_d2_ephemeris_pages(&pages).expect_err("duplicate page rejection");

        assert_eq!(rejection.reason, super::BeidouD2EphemerisRejectionReason::DuplicatePage);
        assert_eq!(rejection.page_number, 5);
    }

    #[test]
    fn broadcast_navigation_assembles_complete_d2_record() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 3 };
        let pages = decoded_navigation_pages();

        let navigation = super::decode_beidou_d2_broadcast_navigation_data(sat, &pages)
            .expect("navigation assembly")
            .expect("complete navigation");

        let af1_raw = (0b1010_u64 << 18) | (0b10_1010_u64 << 12) | 0x5A5;

        assert_eq!(navigation.sat, sat);
        assert_eq!(navigation.bdt.week, 1_234);
        assert_eq!(navigation.bdt.sow_s, 345_678);
        assert_eq!(navigation.urai, 5);
        assert!(navigation.signal_health.autonomous_satellite_good);
        assert_eq!(navigation.clock.aodc, 17);
        assert_eq!(navigation.clock.af0, -0x12_345_i64 as f64 * 2f64.powi(-33));
        assert_eq!(
            navigation.clock.af1,
            super::signed_from_unsigned(af1_raw, 22) as f64 * 2f64.powi(-50)
        );
        assert_eq!(navigation.clock.af2, -321.0 * 2f64.powi(-66));
        assert_eq!(navigation.clock.tgd1_s, -77.0e-10);
        assert!((navigation.clock.tgd2_s - 55.0e-10).abs() < 1.0e-18);
        assert_eq!(navigation.ephemeris.sat, sat);
        assert_eq!(navigation.ephemeris.aode, 19);
        assert_eq!(navigation.ephemeris.toe_s, 0x1_2345_u64 as f64 * 8.0);
        assert_eq!(navigation.ionosphere.alpha0, -12.0 * 2f64.powi(-30));
    }

    #[test]
    fn broadcast_navigation_waits_for_complete_d2_page_set() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 3 };
        let mut pages = decoded_navigation_pages();
        pages.pop();

        let navigation = super::decode_beidou_d2_broadcast_navigation_data(sat, &pages)
            .expect("incomplete navigation");

        assert!(navigation.is_none());
    }

    #[test]
    fn broadcast_navigation_rejects_duplicate_d2_page() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 3 };
        let mut pages = decoded_navigation_pages();
        pages.push(decoded_page(&ephemeris_page_bits(7)));

        let rejection = super::decode_beidou_d2_broadcast_navigation_data(sat, &pages)
            .expect_err("duplicate page rejection");

        assert_eq!(rejection.reason, super::BeidouD2NavigationRejectionReason::DuplicatePage);
        assert_eq!(rejection.page_number, Some(7));
    }

    #[test]
    fn broadcast_navigation_rejects_non_beidou_satellite_id() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let pages = decoded_navigation_pages();

        let rejection = super::decode_beidou_d2_broadcast_navigation_data(sat, &pages)
            .expect_err("satellite rejection");

        assert_eq!(rejection.reason, super::BeidouD2NavigationRejectionReason::InvalidSatelliteId);
        assert_eq!(rejection.sat, Some(sat));
    }

    #[test]
    fn broadcast_navigation_resolves_truncated_beidou_week() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 3 };
        let mut pages = decoded_navigation_pages();
        let mut page_bits = clock_status_page_bits();
        set_unsigned_bits(&mut page_bits, 65, 13, 4);
        apply_bch_parity(&mut page_bits);
        pages[0] = decoded_page(&page_bits);

        let navigation = super::decode_beidou_d2_broadcast_navigation_data_with_reference_week(
            sat, &pages, 8_190,
        )
        .expect("navigation assembly")
        .expect("complete navigation");

        assert_eq!(navigation.bdt.week, 8_196);
    }
}
