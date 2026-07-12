#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use crate::orbits::beidou::{
    BeidouClockCorrection, BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
};

const BEIDOU_D1_SUBFRAME_BITS: usize = 300;
const BEIDOU_D1_PREAMBLE: u16 = 0b111_0001_0010;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BeidouD1Subframe {
    Clock(BeidouD1ClockSubframe),
    Ephemeris1(BeidouD1Ephemeris1Subframe),
    Ephemeris2(BeidouD1Ephemeris2Subframe),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeidouD1ClockSubframe {
    pub bdt: BeidouSystemTime,
    pub urai: u8,
    pub signal_health: BeidouSignalHealth,
    pub clock: BeidouClockCorrection,
    pub ionosphere: BeidouIonosphericCorrection,
    pub aode: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeidouD1Ephemeris1Subframe {
    pub sow_s: u32,
    pub delta_n: f64,
    pub cuc: f64,
    pub m0: f64,
    pub e: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub sqrt_a: f64,
    pub toe_msb2: u8,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BeidouD1Ephemeris2Subframe {
    pub sow_s: u32,
    pub toe_lsb15: u16,
    pub i0: f64,
    pub cic: f64,
    pub omegadot: f64,
    pub cis: f64,
    pub idot: f64,
    pub omega0: f64,
    pub w: f64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BeidouD1SubframeRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    InvalidPreamble,
    InvalidSubframeId,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouD1SubframeRejection {
    pub reason: BeidouD1SubframeRejectionReason,
    pub bit_index: Option<usize>,
    pub subframe_id: Option<u8>,
}

pub fn decode_beidou_b1i_subframe(
    bits: &[u8],
) -> Result<BeidouD1Subframe, BeidouD1SubframeRejection> {
    let normalized = normalize_bits(bits)?;
    match subframe_id(&normalized) {
        1 => Ok(BeidouD1Subframe::Clock(decode_beidou_b1i_clock_subframe_bits(&normalized)?)),
        2 => Ok(BeidouD1Subframe::Ephemeris1(decode_beidou_b1i_ephemeris_1_subframe_bits(
            &normalized,
        )?)),
        3 => Ok(BeidouD1Subframe::Ephemeris2(decode_beidou_b1i_ephemeris_2_subframe_bits(
            &normalized,
        )?)),
        value => Err(BeidouD1SubframeRejection {
            reason: BeidouD1SubframeRejectionReason::InvalidSubframeId,
            bit_index: None,
            subframe_id: Some(value),
        }),
    }
}

pub fn decode_beidou_b1i_clock_subframe(
    bits: &[u8],
) -> Result<BeidouD1ClockSubframe, BeidouD1SubframeRejection> {
    let normalized = normalize_bits(bits)?;
    decode_beidou_b1i_clock_subframe_bits(&normalized)
}

pub fn decode_beidou_b1i_ephemeris_1_subframe(
    bits: &[u8],
) -> Result<BeidouD1Ephemeris1Subframe, BeidouD1SubframeRejection> {
    let normalized = normalize_bits(bits)?;
    decode_beidou_b1i_ephemeris_1_subframe_bits(&normalized)
}

pub fn decode_beidou_b1i_ephemeris_2_subframe(
    bits: &[u8],
) -> Result<BeidouD1Ephemeris2Subframe, BeidouD1SubframeRejection> {
    let normalized = normalize_bits(bits)?;
    decode_beidou_b1i_ephemeris_2_subframe_bits(&normalized)
}

fn decode_beidou_b1i_clock_subframe_bits(
    bits: &[u8; BEIDOU_D1_SUBFRAME_BITS],
) -> Result<BeidouD1ClockSubframe, BeidouD1SubframeRejection> {
    validate_header(bits, 1)?;
    Ok(BeidouD1ClockSubframe {
        bdt: BeidouSystemTime {
            week: unsigned_bits(bits, 61, 13) as u16,
            sow_s: concat_unsigned_bits(bits, &[(19, 8), (31, 12)]) as u32,
        },
        urai: unsigned_bits(bits, 49, 4) as u8,
        signal_health: BeidouSignalHealth { autonomous_satellite_good: bit(bits, 43) == 0 },
        clock: BeidouClockCorrection {
            toc_s: concat_unsigned_bits(bits, &[(74, 9), (91, 8)]) as f64 * 8.0,
            aodc: unsigned_bits(bits, 44, 5) as u8,
            af0: concat_signed_bits(bits, &[(226, 7), (241, 17)]) as f64 * 2f64.powi(-33),
            af1: concat_signed_bits(bits, &[(258, 5), (271, 17)]) as f64 * 2f64.powi(-50),
            af2: signed_bits(bits, 215, 11) as f64 * 2f64.powi(-66),
            tgd1_s: signed_bits(bits, 99, 10) as f64 * 1.0e-10,
            tgd2_s: concat_signed_bits(bits, &[(109, 4), (121, 6)]) as f64 * 1.0e-10,
        },
        ionosphere: BeidouIonosphericCorrection {
            alpha0: signed_bits(bits, 127, 8) as f64 * 2f64.powi(-30),
            alpha1: signed_bits(bits, 135, 8) as f64 * 2f64.powi(-27),
            alpha2: signed_bits(bits, 151, 8) as f64 * 2f64.powi(-24),
            alpha3: signed_bits(bits, 159, 8) as f64 * 2f64.powi(-24),
            beta0: concat_signed_bits(bits, &[(167, 6), (181, 2)]) as f64 * 2f64.powi(11),
            beta1: signed_bits(bits, 183, 8) as f64 * 2f64.powi(14),
            beta2: signed_bits(bits, 191, 8) as f64 * 2f64.powi(16),
            beta3: concat_signed_bits(bits, &[(199, 4), (211, 4)]) as f64 * 2f64.powi(16),
        },
        aode: unsigned_bits(bits, 288, 5) as u8,
    })
}

fn decode_beidou_b1i_ephemeris_1_subframe_bits(
    bits: &[u8; BEIDOU_D1_SUBFRAME_BITS],
) -> Result<BeidouD1Ephemeris1Subframe, BeidouD1SubframeRejection> {
    validate_header(bits, 2)?;
    Ok(BeidouD1Ephemeris1Subframe {
        sow_s: concat_unsigned_bits(bits, &[(19, 8), (31, 12)]) as u32,
        delta_n: concat_signed_bits(bits, &[(43, 10), (61, 6)]) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
        cuc: concat_signed_bits(bits, &[(67, 16), (91, 2)]) as f64 * 2f64.powi(-31),
        m0: concat_signed_bits(bits, &[(93, 20), (121, 12)]) as f64
            * 2f64.powi(-31)
            * std::f64::consts::PI,
        e: concat_unsigned_bits(bits, &[(133, 10), (151, 22)]) as f64 * 2f64.powi(-33),
        cus: signed_bits(bits, 181, 18) as f64 * 2f64.powi(-31),
        crc: concat_signed_bits(bits, &[(199, 4), (211, 14)]) as f64 * 2f64.powi(-6),
        crs: concat_signed_bits(bits, &[(225, 8), (241, 10)]) as f64 * 2f64.powi(-6),
        sqrt_a: concat_unsigned_bits(bits, &[(251, 12), (271, 20)]) as f64 * 2f64.powi(-19),
        toe_msb2: unsigned_bits(bits, 291, 2) as u8,
    })
}

fn decode_beidou_b1i_ephemeris_2_subframe_bits(
    bits: &[u8; BEIDOU_D1_SUBFRAME_BITS],
) -> Result<BeidouD1Ephemeris2Subframe, BeidouD1SubframeRejection> {
    validate_header(bits, 3)?;
    Ok(BeidouD1Ephemeris2Subframe {
        sow_s: concat_unsigned_bits(bits, &[(19, 8), (31, 12)]) as u32,
        toe_lsb15: concat_unsigned_bits(bits, &[(43, 10), (61, 5)]) as u16,
        i0: concat_signed_bits(bits, &[(66, 17), (91, 15)]) as f64
            * 2f64.powi(-31)
            * std::f64::consts::PI,
        cic: concat_signed_bits(bits, &[(106, 7), (121, 11)]) as f64 * 2f64.powi(-31),
        omegadot: concat_signed_bits(bits, &[(132, 11), (151, 13)]) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
        cis: concat_signed_bits(bits, &[(164, 9), (181, 9)]) as f64 * 2f64.powi(-31),
        idot: concat_signed_bits(bits, &[(190, 13), (211, 1)]) as f64
            * 2f64.powi(-43)
            * std::f64::consts::PI,
        omega0: concat_signed_bits(bits, &[(212, 21), (241, 11)]) as f64
            * 2f64.powi(-31)
            * std::f64::consts::PI,
        w: concat_signed_bits(bits, &[(252, 11), (271, 21)]) as f64
            * 2f64.powi(-31)
            * std::f64::consts::PI,
    })
}

fn normalize_bits(bits: &[u8]) -> Result<[u8; BEIDOU_D1_SUBFRAME_BITS], BeidouD1SubframeRejection> {
    if bits.len() != BEIDOU_D1_SUBFRAME_BITS {
        return Err(BeidouD1SubframeRejection {
            reason: BeidouD1SubframeRejectionReason::InvalidBitCount,
            bit_index: None,
            subframe_id: None,
        });
    }

    let mut normalized = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(BeidouD1SubframeRejection {
                reason: BeidouD1SubframeRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                subframe_id: None,
            });
        }
        normalized[idx] = bit;
    }
    Ok(normalized)
}

fn validate_header(
    bits: &[u8; BEIDOU_D1_SUBFRAME_BITS],
    expected_subframe_id: u8,
) -> Result<(), BeidouD1SubframeRejection> {
    if unsigned_bits(bits, 1, 11) as u16 != BEIDOU_D1_PREAMBLE {
        return Err(BeidouD1SubframeRejection {
            reason: BeidouD1SubframeRejectionReason::InvalidPreamble,
            bit_index: Some(0),
            subframe_id: Some(subframe_id(bits)),
        });
    }
    let actual_subframe_id = subframe_id(bits);
    if actual_subframe_id != expected_subframe_id {
        return Err(BeidouD1SubframeRejection {
            reason: BeidouD1SubframeRejectionReason::InvalidSubframeId,
            bit_index: None,
            subframe_id: Some(actual_subframe_id),
        });
    }
    Ok(())
}

fn subframe_id(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS]) -> u8 {
    unsigned_bits(bits, 16, 3) as u8
}

fn bit(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS], position: usize) -> u8 {
    bits[position - 1]
}

fn unsigned_bits(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS], start: usize, len: usize) -> u64 {
    (start..start + len).fold(0_u64, |acc, position| (acc << 1) | u64::from(bit(bits, position)))
}

fn signed_bits(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS], start: usize, len: usize) -> i64 {
    signed_from_unsigned(unsigned_bits(bits, start, len), len)
}

fn concat_unsigned_bits(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS], parts: &[(usize, usize)]) -> u64 {
    parts.iter().fold(0_u64, |acc, (start, len)| (acc << len) | unsigned_bits(bits, *start, *len))
}

fn concat_signed_bits(bits: &[u8; BEIDOU_D1_SUBFRAME_BITS], parts: &[(usize, usize)]) -> i64 {
    let total_bits = parts.iter().map(|(_, len)| *len).sum();
    signed_from_unsigned(concat_unsigned_bits(bits, parts), total_bits)
}

fn signed_from_unsigned(value: u64, bits: usize) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

#[cfg(test)]
mod tests {
    use super::{
        decode_beidou_b1i_clock_subframe, decode_beidou_b1i_ephemeris_1_subframe,
        decode_beidou_b1i_ephemeris_2_subframe, decode_beidou_b1i_subframe, BeidouD1Subframe,
        BEIDOU_D1_PREAMBLE, BEIDOU_D1_SUBFRAME_BITS,
    };

    fn set_unsigned_bits(
        bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS],
        start: usize,
        len: usize,
        value: u64,
    ) {
        for offset in 0..len {
            let shift = len - offset - 1;
            bits[start + offset - 1] = ((value >> shift) & 1) as u8;
        }
    }

    fn set_split_unsigned_bits(
        bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS],
        parts: &[(usize, usize)],
        value: u64,
    ) {
        let total_bits: usize = parts.iter().map(|(_, len)| *len).sum();
        let mut remaining = total_bits;
        for (start, len) in parts {
            remaining -= *len;
            let part_value = (value >> remaining) & ((1_u64 << len) - 1);
            set_unsigned_bits(bits, *start, *len, part_value);
        }
    }

    fn encode_signed(value: i64, bits: usize) -> u64 {
        let mask = if bits == 64 { u64::MAX } else { (1_u64 << bits) - 1 };
        (value as u64) & mask
    }

    fn set_split_signed_bits(
        bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS],
        parts: &[(usize, usize)],
        value: i64,
    ) {
        let total_bits: usize = parts.iter().map(|(_, len)| *len).sum();
        set_split_unsigned_bits(bits, parts, encode_signed(value, total_bits));
    }

    fn set_common_header(bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS], subframe_id: u8, sow_s: u32) {
        set_unsigned_bits(bits, 1, 11, u64::from(BEIDOU_D1_PREAMBLE));
        set_unsigned_bits(bits, 16, 3, u64::from(subframe_id));
        set_split_unsigned_bits(bits, &[(19, 8), (31, 12)], u64::from(sow_s));
    }

    fn sample_clock_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
        let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
        set_common_header(&mut bits, 1, 345_678);
        set_unsigned_bits(&mut bits, 43, 1, 0);
        set_unsigned_bits(&mut bits, 44, 5, 17);
        set_unsigned_bits(&mut bits, 49, 4, 5);
        set_unsigned_bits(&mut bits, 61, 13, 1_234);
        set_split_unsigned_bits(&mut bits, &[(74, 9), (91, 8)], 12_345);
        set_signed_bits(&mut bits, 99, 10, -77);
        set_split_signed_bits(&mut bits, &[(109, 4), (121, 6)], 55);
        set_signed_bits(&mut bits, 127, 8, -12);
        set_signed_bits(&mut bits, 135, 8, 23);
        set_signed_bits(&mut bits, 151, 8, -34);
        set_signed_bits(&mut bits, 159, 8, 45);
        set_split_signed_bits(&mut bits, &[(167, 6), (181, 2)], -56);
        set_signed_bits(&mut bits, 183, 8, 67);
        set_signed_bits(&mut bits, 191, 8, -78);
        set_split_signed_bits(&mut bits, &[(199, 4), (211, 4)], 89);
        set_signed_bits(&mut bits, 215, 11, -321);
        set_split_signed_bits(&mut bits, &[(226, 7), (241, 17)], 0x12_3456);
        set_split_signed_bits(&mut bits, &[(258, 5), (271, 17)], -0x1_2345);
        set_unsigned_bits(&mut bits, 288, 5, 19);
        bits
    }

    fn sample_ephemeris_1_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
        let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
        set_common_header(&mut bits, 2, 345_684);
        set_split_signed_bits(&mut bits, &[(43, 10), (61, 6)], -2_345);
        set_split_signed_bits(&mut bits, &[(67, 16), (91, 2)], 0x1_2345);
        set_split_signed_bits(&mut bits, &[(93, 20), (121, 12)], -0x1ABC_DEF);
        set_split_unsigned_bits(&mut bits, &[(133, 10), (151, 22)], 0x1234_5678);
        set_signed_bits(&mut bits, 181, 18, -54_321);
        set_split_signed_bits(&mut bits, &[(199, 4), (211, 14)], 12_345);
        set_split_signed_bits(&mut bits, &[(225, 8), (241, 10)], -9_876);
        set_split_unsigned_bits(&mut bits, &[(251, 12), (271, 20)], 0x2345_6789);
        set_unsigned_bits(&mut bits, 291, 2, 0b10);
        bits
    }

    fn sample_ephemeris_2_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
        let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
        set_common_header(&mut bits, 3, 345_690);
        set_split_unsigned_bits(&mut bits, &[(43, 10), (61, 5)], 0x4321);
        set_split_signed_bits(&mut bits, &[(66, 17), (91, 15)], 0x1234_5678);
        set_split_signed_bits(&mut bits, &[(106, 7), (121, 11)], -45_678);
        set_split_signed_bits(&mut bits, &[(132, 11), (151, 13)], -0x12_3456);
        set_split_signed_bits(&mut bits, &[(164, 9), (181, 9)], 34_567);
        set_split_signed_bits(&mut bits, &[(190, 13), (211, 1)], -0x1AAA);
        set_split_signed_bits(&mut bits, &[(212, 21), (241, 11)], -0x1234_5678);
        set_split_signed_bits(&mut bits, &[(252, 11), (271, 21)], 0x2345_6789);
        bits
    }

    fn set_signed_bits(
        bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS],
        start: usize,
        len: usize,
        value: i64,
    ) {
        set_unsigned_bits(bits, start, len, encode_signed(value, len));
    }

    #[test]
    fn clock_subframe_decodes_clock_and_iono_terms() {
        let bits = sample_clock_subframe();
        let subframe = decode_beidou_b1i_clock_subframe(&bits).expect("clock subframe");

        assert_eq!(subframe.bdt.week, 1_234);
        assert_eq!(subframe.bdt.sow_s, 345_678);
        assert_eq!(subframe.urai, 5);
        assert!(subframe.signal_health.autonomous_satellite_good);
        assert_eq!(subframe.clock.aodc, 17);
        assert_eq!(subframe.aode, 19);
        assert!((subframe.clock.toc_s - 12_345.0 * 8.0).abs() < f64::EPSILON);
        assert!((subframe.clock.tgd1_s - -77.0e-10).abs() < 1.0e-18);
        assert!((subframe.clock.tgd2_s - 55.0e-10).abs() < 1.0e-18);
        assert!((subframe.ionosphere.alpha0 - (-12.0 * 2f64.powi(-30))).abs() < 1.0e-18);
        assert!((subframe.ionosphere.beta3 - (89.0 * 2f64.powi(16))).abs() < 1.0e-6);
    }

    #[test]
    fn ephemeris_subframes_decode_orbit_terms() {
        let ephemeris_1 = decode_beidou_b1i_ephemeris_1_subframe(&sample_ephemeris_1_subframe())
            .expect("subframe 2");
        let ephemeris_2 = decode_beidou_b1i_ephemeris_2_subframe(&sample_ephemeris_2_subframe())
            .expect("subframe 3");

        assert_eq!(ephemeris_1.sow_s, 345_684);
        assert_eq!(ephemeris_1.toe_msb2, 0b10);
        assert!((ephemeris_1.e - 0x1234_5678_u64 as f64 * 2f64.powi(-33)).abs() < 1.0e-12);
        assert!((ephemeris_1.sqrt_a - 0x2345_6789_u64 as f64 * 2f64.powi(-19)).abs() < 1.0e-9);

        assert_eq!(ephemeris_2.sow_s, 345_690);
        assert_eq!(ephemeris_2.toe_lsb15, 0x4321);
        assert!(
            (ephemeris_2.idot - (-0x1AAA_i64) as f64 * 2f64.powi(-43) * std::f64::consts::PI).abs()
                < 1.0e-18
        );
        assert!(
            (ephemeris_2.w - 0x2345_6789_i64 as f64 * 2f64.powi(-31) * std::f64::consts::PI).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn subframe_dispatch_matches_fraid() {
        let clock_bits = sample_clock_subframe();
        let ephemeris_1_bits = sample_ephemeris_1_subframe();
        let ephemeris_2_bits = sample_ephemeris_2_subframe();

        assert!(matches!(
            decode_beidou_b1i_subframe(&clock_bits).expect("subframe"),
            BeidouD1Subframe::Clock(_)
        ));
        assert!(matches!(
            decode_beidou_b1i_subframe(&ephemeris_1_bits).expect("subframe"),
            BeidouD1Subframe::Ephemeris1(_)
        ));
        assert!(matches!(
            decode_beidou_b1i_subframe(&ephemeris_2_bits).expect("subframe"),
            BeidouD1Subframe::Ephemeris2(_)
        ));
    }
}
