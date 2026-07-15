#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::galileo::{
    GalileoEphemeris, GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags,
    GalileoSystemTime,
};
use crate::time::{resolve_galileo_week_rollover, RolloverResolutionError};

const GALILEO_FNAV_PAGE_BITS: usize = 244;
const GALILEO_FNAV_PAYLOAD_BITS: usize = 214;
const GALILEO_FNAV_CRC_BITS: usize = 24;
const GALILEO_FNAV_TAIL_BITS: usize = 6;
const CRC24Q_POLY: u32 = 0x18_64CFB;
const CRC24Q_MASK: u32 = 0xFF_FFFF;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavPage {
    pub page_type: u8,
    pub bits: Vec<u8>,
    pub transmitted_crc: u32,
    pub computed_crc: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavSignalStatus {
    pub e5a_signal_health: u8,
    pub e5a_data_valid: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavClockCorrection {
    pub t0c_s: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub bgd_e1_e5a_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavClockStatusPage {
    pub svid: u8,
    pub iodnav: u16,
    pub sisa_e1_e5a: u8,
    pub clock: GalileoFnavClockCorrection,
    pub ionosphere: GalileoIonosphericCorrection,
    pub signal_status: GalileoFnavSignalStatus,
    pub gst: GalileoSystemTime,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavKeplerianPage {
    pub iodnav: u16,
    pub m0: f64,
    pub omegadot: f64,
    pub e: f64,
    pub sqrt_a: f64,
    pub omega0: f64,
    pub idot: f64,
    pub gst: GalileoSystemTime,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavHarmonicPage {
    pub iodnav: u16,
    pub i0: f64,
    pub w: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub toe_s: f64,
    pub gst: GalileoSystemTime,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavSupplementaryEphemerisPage {
    pub iodnav: u16,
    pub cic: f64,
    pub cis: f64,
    pub tow_s: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoFnavBroadcastNavigationData {
    pub sat: SatId,
    pub iodnav: u16,
    pub gst: GalileoSystemTime,
    pub sisa_e1_e5a: u8,
    pub signal_status: GalileoFnavSignalStatus,
    pub clock: GalileoFnavClockCorrection,
    pub ephemeris: GalileoEphemeris,
    pub ionosphere: GalileoIonosphericCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GalileoFnavPageRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    NonZeroTail,
    CrcMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavPageRejection {
    pub reason: GalileoFnavPageRejectionReason,
    pub bit_index: Option<usize>,
    pub page_type: Option<u8>,
    pub transmitted_crc: Option<u32>,
    pub computed_crc: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GalileoFnavNavigationRejectionReason {
    IodnavMismatch,
    SatelliteIdMismatch,
    AmbiguousWeekRollover,
    InvalidWeekRollover,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoFnavNavigationRejection {
    pub page_type: u8,
    pub reason: GalileoFnavNavigationRejectionReason,
    pub existing_iodnav: Option<u16>,
    pub incoming_iodnav: Option<u16>,
    pub existing_svid: Option<u8>,
    pub incoming_svid: Option<u8>,
}

impl GalileoFnavPage {
    pub fn unsigned_bits(&self, start: usize, len: usize) -> u64 {
        unsigned_bits(&self.bits, start, len)
    }

    pub fn signed_bits(&self, start: usize, len: usize) -> i64 {
        signed_from_unsigned(self.unsigned_bits(start, len), len)
    }
}

pub fn decode_galileo_fnav_page(bits: &[u8]) -> Result<GalileoFnavPage, GalileoFnavPageRejection> {
    let normalized = normalize_fnav_bits(bits)?;
    let page_type = unsigned_bits(&normalized, 1, 6) as u8;

    let tail_start = GALILEO_FNAV_PAGE_BITS - GALILEO_FNAV_TAIL_BITS + 1;
    for position in tail_start..=GALILEO_FNAV_PAGE_BITS {
        if normalized[position - 1] != 0 {
            return Err(GalileoFnavPageRejection {
                reason: GalileoFnavPageRejectionReason::NonZeroTail,
                bit_index: Some(position - 1),
                page_type: Some(page_type),
                transmitted_crc: None,
                computed_crc: None,
            });
        }
    }

    let transmitted_crc =
        unsigned_bits(&normalized, GALILEO_FNAV_PAYLOAD_BITS + 1, GALILEO_FNAV_CRC_BITS) as u32;
    let computed_crc = fnav_crc24q(&normalized[..GALILEO_FNAV_PAYLOAD_BITS]);
    if transmitted_crc != computed_crc {
        return Err(GalileoFnavPageRejection {
            reason: GalileoFnavPageRejectionReason::CrcMismatch,
            bit_index: None,
            page_type: Some(page_type),
            transmitted_crc: Some(transmitted_crc),
            computed_crc: Some(computed_crc),
        });
    }

    Ok(GalileoFnavPage { page_type, bits: normalized, transmitted_crc, computed_crc })
}

pub fn decode_galileo_fnav_clock_status_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavClockStatusPage> {
    (page.page_type == 1).then(|| GalileoFnavClockStatusPage {
        svid: page.unsigned_bits(7, 6) as u8,
        iodnav: page.unsigned_bits(13, 10) as u16,
        clock: GalileoFnavClockCorrection {
            t0c_s: page.unsigned_bits(23, 14) as f64 * 60.0,
            af0: page.signed_bits(37, 31) as f64 * 2f64.powi(-34),
            af1: page.signed_bits(68, 21) as f64 * 2f64.powi(-46),
            af2: page.signed_bits(89, 6) as f64 * 2f64.powi(-59),
            bgd_e1_e5a_s: page.signed_bits(144, 10) as f64 * 2f64.powi(-32),
        },
        sisa_e1_e5a: page.unsigned_bits(95, 8) as u8,
        ionosphere: GalileoIonosphericCorrection {
            ai0: page.unsigned_bits(103, 11) as f64 * 2f64.powi(-2),
            ai1: page.signed_bits(114, 11) as f64 * 2f64.powi(-8),
            ai2: page.signed_bits(125, 14) as f64 * 2f64.powi(-15),
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: page.unsigned_bits(139, 1) != 0,
                region_2: page.unsigned_bits(140, 1) != 0,
                region_3: page.unsigned_bits(141, 1) != 0,
                region_4: page.unsigned_bits(142, 1) != 0,
                region_5: page.unsigned_bits(143, 1) != 0,
            },
        },
        signal_status: GalileoFnavSignalStatus {
            e5a_signal_health: page.unsigned_bits(154, 2) as u8,
            e5a_data_valid: page.unsigned_bits(188, 1) == 0,
        },
        gst: GalileoSystemTime {
            week: page.unsigned_bits(156, 12) as u16,
            tow_s: page.unsigned_bits(168, 20) as u32,
        },
    })
}

pub fn decode_galileo_fnav_keplerian_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavKeplerianPage> {
    (page.page_type == 2).then(|| GalileoFnavKeplerianPage {
        iodnav: page.unsigned_bits(7, 10) as u16,
        m0: page.signed_bits(17, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        omegadot: page.signed_bits(49, 24) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        e: page.unsigned_bits(73, 32) as f64 * 2f64.powi(-33),
        sqrt_a: page.unsigned_bits(105, 32) as f64 * 2f64.powi(-19),
        omega0: page.signed_bits(137, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        idot: page.signed_bits(169, 14) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        gst: GalileoSystemTime {
            week: page.unsigned_bits(183, 12) as u16,
            tow_s: page.unsigned_bits(195, 20) as u32,
        },
    })
}

pub fn decode_galileo_fnav_harmonic_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavHarmonicPage> {
    (page.page_type == 3).then(|| GalileoFnavHarmonicPage {
        iodnav: page.unsigned_bits(7, 10) as u16,
        i0: page.signed_bits(17, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        w: page.signed_bits(49, 32) as f64 * 2f64.powi(-31) * std::f64::consts::PI,
        delta_n: page.signed_bits(81, 16) as f64 * 2f64.powi(-43) * std::f64::consts::PI,
        cuc: page.signed_bits(97, 16) as f64 * 2f64.powi(-29),
        cus: page.signed_bits(113, 16) as f64 * 2f64.powi(-29),
        crc: page.signed_bits(129, 16) as f64 * 2f64.powi(-5),
        crs: page.signed_bits(145, 16) as f64 * 2f64.powi(-5),
        toe_s: page.unsigned_bits(161, 14) as f64 * 60.0,
        gst: GalileoSystemTime {
            week: page.unsigned_bits(175, 12) as u16,
            tow_s: page.unsigned_bits(187, 20) as u32,
        },
    })
}

pub fn decode_galileo_fnav_supplementary_ephemeris_page(
    page: &GalileoFnavPage,
) -> Option<GalileoFnavSupplementaryEphemerisPage> {
    (page.page_type == 4).then(|| GalileoFnavSupplementaryEphemerisPage {
        iodnav: page.unsigned_bits(7, 10) as u16,
        cic: page.signed_bits(17, 16) as f64 * 2f64.powi(-29),
        cis: page.signed_bits(33, 16) as f64 * 2f64.powi(-29),
        tow_s: page.unsigned_bits(190, 20) as u32,
    })
}

pub fn decode_galileo_fnav_broadcast_navigation(
    pages: &[GalileoFnavPage],
) -> Result<Option<GalileoFnavBroadcastNavigationData>, GalileoFnavNavigationRejection> {
    let mut builder = GalileoFnavNavigationBuilder::default();
    for page in pages {
        builder.merge(page)?;
    }
    builder.try_build_checked()
}

pub fn decode_galileo_fnav_broadcast_navigation_with_reference_week(
    pages: &[GalileoFnavPage],
    reference_week: u32,
) -> Result<Option<GalileoFnavBroadcastNavigationData>, GalileoFnavNavigationRejection> {
    let mut builder = GalileoFnavNavigationBuilder::with_reference_week(reference_week);
    for page in pages {
        builder.merge(page)?;
    }
    builder.try_build_checked()
}

fn normalize_fnav_bits(bits: &[u8]) -> Result<Vec<u8>, GalileoFnavPageRejection> {
    if bits.len() != GALILEO_FNAV_PAGE_BITS {
        return Err(GalileoFnavPageRejection {
            reason: GalileoFnavPageRejectionReason::InvalidBitCount,
            bit_index: None,
            page_type: None,
            transmitted_crc: None,
            computed_crc: None,
        });
    }

    let mut normalized = Vec::with_capacity(GALILEO_FNAV_PAGE_BITS);
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(GalileoFnavPageRejection {
                reason: GalileoFnavPageRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                page_type: None,
                transmitted_crc: None,
                computed_crc: None,
            });
        }
        normalized.push(bit);
    }
    Ok(normalized)
}

fn fnav_crc24q(bits: &[u8]) -> u32 {
    let mut crc = 0_u32;
    for bit in bits {
        let top = ((crc >> 23) & 1) as u8;
        crc = ((crc << 1) & CRC24Q_MASK) | u32::from(*bit);
        if (top ^ *bit) != 0 {
            crc ^= CRC24Q_POLY & CRC24Q_MASK;
        }
    }
    crc & CRC24Q_MASK
}

fn unsigned_bits(bits: &[u8], start: usize, len: usize) -> u64 {
    let mut value = 0_u64;
    for bit in &bits[start - 1..start - 1 + len] {
        value = (value << 1) | u64::from(*bit);
    }
    value
}

fn signed_from_unsigned(value: u64, bits: usize) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

#[derive(Debug, Default, Clone)]
struct GalileoFnavNavigationBuilder {
    reference_week: Option<u32>,
    svid: Option<u8>,
    iodnav: Option<u16>,
    clock_status: Option<GalileoFnavClockStatusPage>,
    keplerian: Option<GalileoFnavKeplerianPage>,
    harmonic: Option<GalileoFnavHarmonicPage>,
    supplementary: Option<GalileoFnavSupplementaryEphemerisPage>,
}

impl GalileoFnavNavigationBuilder {
    fn with_reference_week(reference_week: u32) -> Self {
        Self { reference_week: Some(reference_week), ..Self::default() }
    }

    fn merge(&mut self, page: &GalileoFnavPage) -> Result<(), GalileoFnavNavigationRejection> {
        match page.page_type {
            1 => {
                if let Some(clock_status) = decode_galileo_fnav_clock_status_page(page) {
                    self.check_iodnav(page.page_type, clock_status.iodnav)?;
                    self.check_svid(page.page_type, clock_status.svid)?;
                    self.iodnav = Some(clock_status.iodnav);
                    self.svid = Some(clock_status.svid);
                    self.clock_status = Some(clock_status);
                }
            }
            2 => {
                if let Some(keplerian) = decode_galileo_fnav_keplerian_page(page) {
                    self.check_iodnav(page.page_type, keplerian.iodnav)?;
                    self.iodnav = Some(keplerian.iodnav);
                    self.keplerian = Some(keplerian);
                }
            }
            3 => {
                if let Some(harmonic) = decode_galileo_fnav_harmonic_page(page) {
                    self.check_iodnav(page.page_type, harmonic.iodnav)?;
                    self.iodnav = Some(harmonic.iodnav);
                    self.harmonic = Some(harmonic);
                }
            }
            4 => {
                if let Some(supplementary) = decode_galileo_fnav_supplementary_ephemeris_page(page)
                {
                    self.check_iodnav(page.page_type, supplementary.iodnav)?;
                    self.iodnav = Some(supplementary.iodnav);
                    self.supplementary = Some(supplementary);
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn try_build_checked(
        &self,
    ) -> Result<Option<GalileoFnavBroadcastNavigationData>, GalileoFnavNavigationRejection> {
        let (Some(clock_status), Some(keplerian), Some(harmonic), Some(supplementary)) =
            (&self.clock_status, &self.keplerian, &self.harmonic, &self.supplementary)
        else {
            return Ok(None);
        };

        let sat = SatId { constellation: Constellation::Galileo, prn: clock_status.svid };
        let mut gst = clock_status.gst;
        if let Some(reference_week) = self.reference_week {
            gst.week = resolve_galileo_week_rollover(gst.week, reference_week)
                .map_err(|error| galileo_fnav_week_rollover_rejection(error, clock_status.iodnav))?
                .week as u16;
        }

        Ok(Some(GalileoFnavBroadcastNavigationData {
            sat,
            iodnav: clock_status.iodnav,
            gst,
            sisa_e1_e5a: clock_status.sisa_e1_e5a,
            signal_status: clock_status.signal_status,
            clock: clock_status.clock,
            ephemeris: GalileoEphemeris {
                sat,
                iodnav: clock_status.iodnav,
                toe_s: harmonic.toe_s,
                sqrt_a: keplerian.sqrt_a,
                e: keplerian.e,
                i0: harmonic.i0,
                idot: keplerian.idot,
                omega0: keplerian.omega0,
                omegadot: keplerian.omegadot,
                w: harmonic.w,
                m0: keplerian.m0,
                delta_n: harmonic.delta_n,
                cuc: harmonic.cuc,
                cus: harmonic.cus,
                crc: harmonic.crc,
                crs: harmonic.crs,
                cic: supplementary.cic,
                cis: supplementary.cis,
            },
            ionosphere: clock_status.ionosphere.clone(),
        }))
    }

    fn check_iodnav(
        &self,
        page_type: u8,
        incoming_iodnav: u16,
    ) -> Result<(), GalileoFnavNavigationRejection> {
        if let Some(existing_iodnav) = self.iodnav {
            if existing_iodnav != incoming_iodnav {
                return Err(GalileoFnavNavigationRejection {
                    page_type,
                    reason: GalileoFnavNavigationRejectionReason::IodnavMismatch,
                    existing_iodnav: Some(existing_iodnav),
                    incoming_iodnav: Some(incoming_iodnav),
                    existing_svid: self.svid,
                    incoming_svid: None,
                });
            }
        }
        Ok(())
    }

    fn check_svid(
        &self,
        page_type: u8,
        incoming_svid: u8,
    ) -> Result<(), GalileoFnavNavigationRejection> {
        if let Some(existing_svid) = self.svid {
            if existing_svid != incoming_svid {
                return Err(GalileoFnavNavigationRejection {
                    page_type,
                    reason: GalileoFnavNavigationRejectionReason::SatelliteIdMismatch,
                    existing_iodnav: self.iodnav,
                    incoming_iodnav: self.iodnav,
                    existing_svid: Some(existing_svid),
                    incoming_svid: Some(incoming_svid),
                });
            }
        }
        Ok(())
    }
}

fn galileo_fnav_week_rollover_rejection(
    error: RolloverResolutionError,
    iodnav: u16,
) -> GalileoFnavNavigationRejection {
    GalileoFnavNavigationRejection {
        page_type: 1,
        reason: match error {
            RolloverResolutionError::AmbiguousReference => {
                GalileoFnavNavigationRejectionReason::AmbiguousWeekRollover
            }
            RolloverResolutionError::InvalidCycle
            | RolloverResolutionError::TruncatedValueOutOfRange => {
                GalileoFnavNavigationRejectionReason::InvalidWeekRollover
            }
        },
        existing_iodnav: Some(iodnav),
        incoming_iodnav: Some(iodnav),
        existing_svid: None,
        incoming_svid: None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        decode_galileo_fnav_page, fnav_crc24q, GalileoFnavPageRejectionReason,
        GALILEO_FNAV_CRC_BITS, GALILEO_FNAV_PAGE_BITS, GALILEO_FNAV_PAYLOAD_BITS,
        GALILEO_FNAV_TAIL_BITS,
    };
    use bijux_gnss_core::api::Constellation;

    fn set_bits(bits: &mut [u8], start: usize, len: usize, value: u64) {
        for offset in 0..len {
            let shift = len - offset - 1;
            bits[start + offset - 1] = ((value >> shift) & 1) as u8;
        }
    }

    fn encode_signed(value: i64, bits: usize) -> u64 {
        let mask = (1_u64 << bits) - 1;
        value as u64 & mask
    }

    fn apply_crc(bits: &mut [u8]) {
        let crc = fnav_crc24q(&bits[..GALILEO_FNAV_PAYLOAD_BITS]);
        set_bits(bits, GALILEO_FNAV_PAYLOAD_BITS + 1, GALILEO_FNAV_CRC_BITS, u64::from(crc));
    }

    fn valid_page_bits(page_type: u8) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, u64::from(page_type));
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 14, 2_345);
        apply_crc(&mut bits);
        bits
    }

    fn clock_status_page_bits(iodnav: u16, svid: u8) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 1);
        set_bits(&mut bits, 7, 6, u64::from(svid));
        set_bits(&mut bits, 13, 10, u64::from(iodnav));
        set_bits(&mut bits, 23, 14, 1_111);
        set_bits(&mut bits, 37, 31, encode_signed(-0x1ABCD_i64, 31));
        set_bits(&mut bits, 68, 21, encode_signed(0x01234_i64, 21));
        set_bits(&mut bits, 89, 6, encode_signed(-0x09_i64, 6));
        set_bits(&mut bits, 95, 8, 77);
        set_bits(&mut bits, 103, 11, 211);
        set_bits(&mut bits, 114, 11, encode_signed(-87_i64, 11));
        set_bits(&mut bits, 125, 14, encode_signed(0x00A5_i64, 14));
        set_bits(&mut bits, 139, 1, 1);
        set_bits(&mut bits, 140, 1, 1);
        set_bits(&mut bits, 142, 1, 1);
        set_bits(&mut bits, 144, 10, encode_signed(-12_i64, 10));
        set_bits(&mut bits, 154, 2, 2);
        set_bits(&mut bits, 156, 12, 2_222);
        set_bits(&mut bits, 168, 20, 456_789);
        set_bits(&mut bits, 188, 1, 0);
        apply_crc(&mut bits);
        bits
    }

    fn keplerian_page_bits(iodnav: u16) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 2);
        set_bits(&mut bits, 7, 10, u64::from(iodnav));
        set_bits(&mut bits, 17, 32, encode_signed(-0x1020_304_i64, 32));
        set_bits(&mut bits, 49, 24, encode_signed(-0x04_321_i64, 24));
        set_bits(&mut bits, 73, 32, 0x0123_4567);
        set_bits(&mut bits, 105, 32, 0x0987_6543);
        set_bits(&mut bits, 137, 32, encode_signed(0x1020_3040_i64, 32));
        set_bits(&mut bits, 169, 14, encode_signed(-0x03A5_i64, 14));
        set_bits(&mut bits, 183, 12, 2_223);
        set_bits(&mut bits, 195, 20, 456_799);
        apply_crc(&mut bits);
        bits
    }

    fn harmonic_page_bits(iodnav: u16) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 3);
        set_bits(&mut bits, 7, 10, u64::from(iodnav));
        set_bits(&mut bits, 17, 32, encode_signed(-0x1234_567_i64, 32));
        set_bits(&mut bits, 49, 32, encode_signed(0x2345_6789_i64, 32));
        set_bits(&mut bits, 81, 16, encode_signed(0x0F0F_i64, 16));
        set_bits(&mut bits, 97, 16, encode_signed(-321_i64, 16));
        set_bits(&mut bits, 113, 16, encode_signed(654_i64, 16));
        set_bits(&mut bits, 129, 16, encode_signed(1_111_i64, 16));
        set_bits(&mut bits, 145, 16, encode_signed(-2_222_i64, 16));
        set_bits(&mut bits, 161, 14, 2_345);
        set_bits(&mut bits, 175, 12, 2_224);
        set_bits(&mut bits, 187, 20, 456_809);
        apply_crc(&mut bits);
        bits
    }

    fn supplementary_page_bits(iodnav: u16) -> Vec<u8> {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 4);
        set_bits(&mut bits, 7, 10, u64::from(iodnav));
        set_bits(&mut bits, 17, 16, encode_signed(-123_i64, 16));
        set_bits(&mut bits, 33, 16, encode_signed(456_i64, 16));
        set_bits(&mut bits, 190, 20, 456_819);
        apply_crc(&mut bits);
        bits
    }

    fn decoded_navigation_pages(iodnav: u16, svid: u8) -> Vec<super::GalileoFnavPage> {
        [
            clock_status_page_bits(iodnav, svid),
            keplerian_page_bits(iodnav),
            harmonic_page_bits(iodnav),
            supplementary_page_bits(iodnav),
        ]
        .into_iter()
        .map(|bits| decode_galileo_fnav_page(&bits).expect("valid F/NAV page"))
        .collect()
    }

    #[test]
    fn fnav_page_decodes_type_and_crc() {
        let bits = valid_page_bits(1);

        let page = decode_galileo_fnav_page(&bits).expect("valid F/NAV page");

        assert_eq!(page.page_type, 1);
        assert_eq!(page.transmitted_crc, page.computed_crc);
        assert_eq!(page.unsigned_bits(7, 10), 0x155);
    }

    #[test]
    fn fnav_page_rejects_corrupted_payload() {
        let mut bits = valid_page_bits(2);
        bits[30] ^= 1;

        let rejection = decode_galileo_fnav_page(&bits).expect_err("CRC rejection");

        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::CrcMismatch);
        assert_eq!(rejection.page_type, Some(2));
        assert_ne!(rejection.transmitted_crc, rejection.computed_crc);
    }

    #[test]
    fn fnav_page_rejects_invalid_shape_before_crc() {
        let short_bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS - 1];
        let rejection = decode_galileo_fnav_page(&short_bits).expect_err("bit count rejection");
        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::InvalidBitCount);

        let mut non_binary = valid_page_bits(1);
        non_binary[10] = 2;
        let rejection = decode_galileo_fnav_page(&non_binary).expect_err("binary rejection");
        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::NonBinaryBit);
        assert_eq!(rejection.bit_index, Some(10));
    }

    #[test]
    fn fnav_page_rejects_nonzero_tail_bits() {
        let mut bits = valid_page_bits(1);
        bits[GALILEO_FNAV_PAGE_BITS - 1] = 1;

        let rejection = decode_galileo_fnav_page(&bits).expect_err("tail rejection");

        assert_eq!(rejection.reason, GalileoFnavPageRejectionReason::NonZeroTail);
        assert_eq!(rejection.bit_index, Some(GALILEO_FNAV_PAGE_BITS - 1));
    }

    #[test]
    fn fnav_page_tail_boundary_matches_layout() {
        assert_eq!(
            GALILEO_FNAV_PAYLOAD_BITS + GALILEO_FNAV_CRC_BITS + GALILEO_FNAV_TAIL_BITS,
            GALILEO_FNAV_PAGE_BITS
        );
    }

    #[test]
    fn clock_status_page_decodes_e5a_clock_and_status_fields() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 1);
        set_bits(&mut bits, 7, 6, 19);
        set_bits(&mut bits, 13, 10, 0x1A5);
        set_bits(&mut bits, 23, 14, 1_111);
        set_bits(&mut bits, 37, 31, encode_signed(-0x1ABCD_i64, 31));
        set_bits(&mut bits, 68, 21, encode_signed(0x01234_i64, 21));
        set_bits(&mut bits, 89, 6, encode_signed(-0x09_i64, 6));
        set_bits(&mut bits, 95, 8, 77);
        set_bits(&mut bits, 103, 11, 211);
        set_bits(&mut bits, 114, 11, encode_signed(-87_i64, 11));
        set_bits(&mut bits, 125, 14, encode_signed(0x00A5_i64, 14));
        set_bits(&mut bits, 139, 1, 1);
        set_bits(&mut bits, 140, 1, 1);
        set_bits(&mut bits, 142, 1, 1);
        set_bits(&mut bits, 144, 10, encode_signed(-12_i64, 10));
        set_bits(&mut bits, 154, 2, 2);
        set_bits(&mut bits, 156, 12, 2_222);
        set_bits(&mut bits, 168, 20, 456_789);
        set_bits(&mut bits, 188, 1, 0);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 1 page");

        let status = super::decode_galileo_fnav_clock_status_page(&page).expect("type 1 payload");

        assert_eq!(status.svid, 19);
        assert_eq!(status.iodnav, 0x1A5);
        assert_eq!(status.sisa_e1_e5a, 77);
        assert_eq!(status.clock.t0c_s, 1_111.0 * 60.0);
        assert_eq!(status.clock.af0, -0x1ABCD_i64 as f64 * 2f64.powi(-34));
        assert_eq!(status.clock.af1, 0x01234_i64 as f64 * 2f64.powi(-46));
        assert_eq!(status.clock.af2, -0x09_i64 as f64 * 2f64.powi(-59));
        assert_eq!(status.clock.bgd_e1_e5a_s, -12.0 * 2f64.powi(-32));
        assert_eq!(status.ionosphere.ai0, 211.0 * 2f64.powi(-2));
        assert_eq!(status.ionosphere.ai1, -87.0 * 2f64.powi(-8));
        assert_eq!(status.ionosphere.ai2, 0x00A5_i64 as f64 * 2f64.powi(-15));
        assert!(status.ionosphere.disturbance_flags.region_1);
        assert!(status.ionosphere.disturbance_flags.region_2);
        assert!(!status.ionosphere.disturbance_flags.region_3);
        assert!(status.ionosphere.disturbance_flags.region_4);
        assert!(!status.ionosphere.disturbance_flags.region_5);
        assert_eq!(status.signal_status.e5a_signal_health, 2);
        assert!(status.signal_status.e5a_data_valid);
        assert_eq!(status.gst.week, 2_222);
        assert_eq!(status.gst.tow_s, 456_789);
    }

    #[test]
    fn keplerian_page_decodes_orbit_shape_and_gst_fields() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 2);
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 32, encode_signed(-0x1020_304_i64, 32));
        set_bits(&mut bits, 49, 24, encode_signed(-0x04_321_i64, 24));
        set_bits(&mut bits, 73, 32, 0x0123_4567);
        set_bits(&mut bits, 105, 32, 0x0987_6543);
        set_bits(&mut bits, 137, 32, encode_signed(0x1020_3040_i64, 32));
        set_bits(&mut bits, 169, 14, encode_signed(-0x03A5_i64, 14));
        set_bits(&mut bits, 183, 12, 2_223);
        set_bits(&mut bits, 195, 20, 456_799);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 2 page");

        let keplerian = super::decode_galileo_fnav_keplerian_page(&page).expect("type 2 payload");

        assert_eq!(keplerian.iodnav, 0x155);
        assert_eq!(keplerian.m0, -0x1020_304_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI);
        assert_eq!(
            keplerian.omegadot,
            -0x04_321_i64 as f64 * 2f64.powi(-43) * std::f64::consts::PI
        );
        assert_eq!(keplerian.e, 0x0123_4567_u64 as f64 * 2f64.powi(-33));
        assert_eq!(keplerian.sqrt_a, 0x0987_6543_u64 as f64 * 2f64.powi(-19));
        assert_eq!(
            keplerian.omega0,
            0x1020_3040_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI
        );
        assert_eq!(keplerian.idot, -0x03A5_i64 as f64 * 2f64.powi(-43) * std::f64::consts::PI);
        assert_eq!(keplerian.gst.week, 2_223);
        assert_eq!(keplerian.gst.tow_s, 456_799);
    }

    #[test]
    fn harmonic_page_decodes_ephemeris_corrections_and_toe() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 3);
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 32, encode_signed(-0x1234_567_i64, 32));
        set_bits(&mut bits, 49, 32, encode_signed(0x2345_6789_i64, 32));
        set_bits(&mut bits, 81, 16, encode_signed(0x0F0F_i64, 16));
        set_bits(&mut bits, 97, 16, encode_signed(-321_i64, 16));
        set_bits(&mut bits, 113, 16, encode_signed(654_i64, 16));
        set_bits(&mut bits, 129, 16, encode_signed(1_111_i64, 16));
        set_bits(&mut bits, 145, 16, encode_signed(-2_222_i64, 16));
        set_bits(&mut bits, 161, 14, 2_345);
        set_bits(&mut bits, 175, 12, 2_224);
        set_bits(&mut bits, 187, 20, 456_809);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 3 page");

        let harmonic = super::decode_galileo_fnav_harmonic_page(&page).expect("type 3 payload");

        assert_eq!(harmonic.iodnav, 0x155);
        assert_eq!(harmonic.i0, -0x1234_567_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI);
        assert_eq!(harmonic.w, 0x2345_6789_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI);
        assert_eq!(harmonic.delta_n, 0x0F0F_i64 as f64 * 2f64.powi(-43) * std::f64::consts::PI);
        assert_eq!(harmonic.cuc, -321.0 * 2f64.powi(-29));
        assert_eq!(harmonic.cus, 654.0 * 2f64.powi(-29));
        assert_eq!(harmonic.crc, 1_111.0 * 2f64.powi(-5));
        assert_eq!(harmonic.crs, -2_222.0 * 2f64.powi(-5));
        assert_eq!(harmonic.toe_s, 2_345.0 * 60.0);
        assert_eq!(harmonic.gst.week, 2_224);
        assert_eq!(harmonic.gst.tow_s, 456_809);
    }

    #[test]
    fn supplementary_ephemeris_page_decodes_inclination_corrections() {
        let mut bits = vec![0_u8; GALILEO_FNAV_PAGE_BITS];
        set_bits(&mut bits, 1, 6, 4);
        set_bits(&mut bits, 7, 10, 0x155);
        set_bits(&mut bits, 17, 16, encode_signed(-123_i64, 16));
        set_bits(&mut bits, 33, 16, encode_signed(456_i64, 16));
        set_bits(&mut bits, 190, 20, 456_819);
        apply_crc(&mut bits);
        let page = decode_galileo_fnav_page(&bits).expect("valid type 4 page");

        let supplementary =
            super::decode_galileo_fnav_supplementary_ephemeris_page(&page).expect("type 4 payload");

        assert_eq!(supplementary.iodnav, 0x155);
        assert_eq!(supplementary.cic, -123.0 * 2f64.powi(-29));
        assert_eq!(supplementary.cis, 456.0 * 2f64.powi(-29));
        assert_eq!(supplementary.tow_s, 456_819);
    }

    #[test]
    fn broadcast_navigation_assembles_complete_fnav_record() {
        let pages = decoded_navigation_pages(0x155, 19);

        let navigation = super::decode_galileo_fnav_broadcast_navigation(&pages)
            .expect("consistent pages")
            .expect("complete F/NAV navigation record");

        assert_eq!(navigation.sat.constellation, Constellation::Galileo);
        assert_eq!(navigation.sat.prn, 19);
        assert_eq!(navigation.iodnav, 0x155);
        assert_eq!(navigation.gst.week, 2_222);
        assert_eq!(navigation.gst.tow_s, 456_789);
        assert_eq!(navigation.sisa_e1_e5a, 77);
        assert_eq!(navigation.signal_status.e5a_signal_health, 2);
        assert!(navigation.signal_status.e5a_data_valid);
        assert_eq!(navigation.clock.t0c_s, 1_111.0 * 60.0);
        assert_eq!(navigation.clock.bgd_e1_e5a_s, -12.0 * 2f64.powi(-32));
        assert_eq!(navigation.ephemeris.sat.prn, 19);
        assert_eq!(navigation.ephemeris.iodnav, 0x155);
        assert_eq!(navigation.ephemeris.toe_s, 2_345.0 * 60.0);
        assert_eq!(navigation.ephemeris.cic, -123.0 * 2f64.powi(-29));
        assert_eq!(navigation.ephemeris.cis, 456.0 * 2f64.powi(-29));
        assert_eq!(navigation.ionosphere.ai1, -87.0 * 2f64.powi(-8));
    }

    #[test]
    fn broadcast_navigation_waits_for_complete_ephemeris_set() {
        let pages = decoded_navigation_pages(0x155, 19);

        let navigation = super::decode_galileo_fnav_broadcast_navigation(&pages[..3])
            .expect("consistent incomplete pages");

        assert!(navigation.is_none());
    }

    #[test]
    fn broadcast_navigation_rejects_mismatched_iodnav() {
        let pages = vec![
            decode_galileo_fnav_page(&clock_status_page_bits(0x155, 19)).expect("type 1"),
            decode_galileo_fnav_page(&keplerian_page_bits(0x155)).expect("type 2"),
            decode_galileo_fnav_page(&harmonic_page_bits(0x156)).expect("mismatched type 3"),
            decode_galileo_fnav_page(&supplementary_page_bits(0x155)).expect("type 4"),
        ];

        let rejection =
            super::decode_galileo_fnav_broadcast_navigation(&pages).expect_err("IODnav rejection");

        assert_eq!(rejection.reason, super::GalileoFnavNavigationRejectionReason::IodnavMismatch);
        assert_eq!(rejection.page_type, 3);
        assert_eq!(rejection.existing_iodnav, Some(0x155));
        assert_eq!(rejection.incoming_iodnav, Some(0x156));
    }

    #[test]
    fn broadcast_navigation_resolves_truncated_galileo_week() {
        let pages = decoded_navigation_pages(0x155, 19);

        let navigation =
            super::decode_galileo_fnav_broadcast_navigation_with_reference_week(&pages, 6_318)
                .expect("consistent pages")
                .expect("complete F/NAV navigation record");

        assert_eq!(navigation.gst.week, 6_318);
    }
}
