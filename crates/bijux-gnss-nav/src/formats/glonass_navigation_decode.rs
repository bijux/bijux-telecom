#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{glonass_slot_sat, GlonassFrequencyChannel, GlonassSlot};

use crate::orbits::glonass::{
    glonass_satellite_type_from_word, semicircles_to_radians, GlonassAlmanacEntry,
    GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame, GlonassFrameTime,
    GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassStateVector,
    GlonassSystemTime,
};

const GLONASS_STRING_BITS: usize = 85;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassStringParitySummary {
    pub corrected_bit_index: Option<u8>,
    pub syndrome: u8,
    pub overall_parity_error: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassNavigationString {
    pub string_number: u8,
    pub bits: Vec<u8>,
    pub parity: GlonassStringParitySummary,
}

impl GlonassNavigationString {
    pub fn bit(&self, position: usize) -> u8 {
        self.bits[GLONASS_STRING_BITS - position]
    }

    pub fn unsigned_bits(&self, low: usize, high: usize) -> u64 {
        let mut value = 0_u64;
        for position in (low..=high).rev() {
            value = (value << 1) | u64::from(self.bit(position));
        }
        value
    }

    pub fn sign_magnitude(&self, low: usize, high: usize, lsb: f64) -> f64 {
        let raw = self.unsigned_bits(low, high);
        let width = high - low + 1;
        let sign_mask = 1_u64 << (width - 1);
        let magnitude_mask = sign_mask - 1;
        let magnitude = (raw & magnitude_mask) as f64 * lsb;
        if raw & sign_mask == 0 { magnitude } else { -magnitude }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GlonassNavigationStringRejectionReason {
    InvalidBitCount,
    NonBinaryBit,
    UnrecoverableParity,
    InvalidStringNumber,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassNavigationStringRejection {
    pub reason: GlonassNavigationStringRejectionReason,
    pub bit_index: Option<usize>,
    pub string_number: Option<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GlonassNavigationFrameRejectionReason {
    DuplicateString,
    InvalidFrequencyChannel,
    SlotMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassNavigationFrameRejection {
    pub reason: GlonassNavigationFrameRejectionReason,
    pub string_number: Option<u8>,
    pub expected_slot: Option<GlonassSlot>,
    pub reported_slot: Option<GlonassSlot>,
}

pub fn decode_glonass_navigation_string(
    bits: &[u8],
) -> Result<GlonassNavigationString, GlonassNavigationStringRejection> {
    let mut normalized = normalize_bits(bits)?;
    let (syndrome, overall_parity_error) = parity_state(&normalized);
    let mut corrected_bit_index = None;

    match (syndrome, overall_parity_error) {
        (0, false) => {}
        (0, true) => {
            flip_bit(&mut normalized, 8);
            corrected_bit_index = Some(8);
        }
        (syndrome, true) => {
            let corrected = correction_index(syndrome);
            if corrected == 0 || corrected > GLONASS_STRING_BITS {
                return Err(GlonassNavigationStringRejection {
                    reason: GlonassNavigationStringRejectionReason::UnrecoverableParity,
                    bit_index: None,
                    string_number: None,
                });
            }
            flip_bit(&mut normalized, corrected);
            corrected_bit_index = Some(corrected as u8);
        }
        (_, false) => {
            return Err(GlonassNavigationStringRejection {
                reason: GlonassNavigationStringRejectionReason::UnrecoverableParity,
                bit_index: None,
                string_number: None,
            });
        }
    }

    let decoded = GlonassNavigationString {
        string_number: decode_string_number(&normalized),
        bits: normalized.to_vec(),
        parity: GlonassStringParitySummary { corrected_bit_index, syndrome, overall_parity_error },
    };
    if !(1..=15).contains(&decoded.string_number) {
        return Err(GlonassNavigationStringRejection {
            reason: GlonassNavigationStringRejectionReason::InvalidStringNumber,
            bit_index: None,
            string_number: Some(decoded.string_number),
        });
    }
    Ok(decoded)
}

pub fn decode_glonass_broadcast_navigation_frame(
    slot: GlonassSlot,
    strings: &[GlonassNavigationString],
) -> Result<Option<GlonassBroadcastNavigationFrame>, GlonassNavigationFrameRejection> {
    let mut by_number: [Option<&GlonassNavigationString>; 16] = [None; 16];
    for string in strings {
        let index = usize::from(string.string_number);
        if by_number[index].is_some() {
            return Err(GlonassNavigationFrameRejection {
                reason: GlonassNavigationFrameRejectionReason::DuplicateString,
                string_number: Some(string.string_number),
                expected_slot: None,
                reported_slot: None,
            });
        }
        by_number[index] = Some(string);
    }

    let (Some(string_1), Some(string_2), Some(string_3), Some(string_4)) =
        (by_number[1], by_number[2], by_number[3], by_number[4])
    else {
        return Ok(None);
    };

    let immediate = decode_immediate_navigation(slot, string_1, string_2, string_3, string_4)?;
    let mut frame = GlonassBroadcastNavigationFrame::new(slot, immediate);
    frame.system_time = by_number[5].map(decode_almanac_time);
    for even_number in [6_usize, 8, 10, 12, 14] {
        if let (Some(even), Some(odd)) = (by_number[even_number], by_number[even_number + 1]) {
            frame.almanac_entries.push(decode_almanac_entry(even, odd)?);
        }
    }
    Ok(Some(frame))
}

fn normalize_bits(
    bits: &[u8],
) -> Result<[u8; GLONASS_STRING_BITS], GlonassNavigationStringRejection> {
    if bits.len() != GLONASS_STRING_BITS {
        return Err(GlonassNavigationStringRejection {
            reason: GlonassNavigationStringRejectionReason::InvalidBitCount,
            bit_index: None,
            string_number: None,
        });
    }
    let mut normalized = [0_u8; GLONASS_STRING_BITS];
    for (idx, bit) in bits.iter().copied().enumerate() {
        if bit > 1 {
            return Err(GlonassNavigationStringRejection {
                reason: GlonassNavigationStringRejectionReason::NonBinaryBit,
                bit_index: Some(idx),
                string_number: None,
            });
        }
        normalized[idx] = bit;
    }
    Ok(normalized)
}

fn parity_state(bits: &[u8; GLONASS_STRING_BITS]) -> (u8, bool) {
    let checksums = [
        checksum_c1(bits),
        checksum_c2(bits),
        checksum_c3(bits),
        checksum_c4(bits),
        checksum_c5(bits),
        checksum_c6(bits),
        checksum_c7(bits),
    ];
    let syndrome = checksums
        .iter()
        .enumerate()
        .fold(0_u8, |acc, (idx, value)| acc | (*value << idx));
    (syndrome, checksum_overall(bits) == 1)
}

fn checksum_c1(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 1) ^ xor_positions(
        bits,
        &[
            9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45,
            47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
        ],
    )
}

fn checksum_c2(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 2) ^ xor_positions(
        bits,
        &[
            9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45,
            48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84,
        ],
    )
}

fn checksum_c3(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 3) ^ xor_ranges(bits, &[(10, 12), (16, 19), (23, 26), (31, 34), (38, 41), (46, 49), (54, 57), (62, 65), (69, 72), (77, 80), (85, 85)])
}

fn checksum_c4(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 4) ^ xor_ranges(bits, &[(13, 19), (27, 34), (42, 49), (58, 65), (73, 80)])
}

fn checksum_c5(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 5) ^ xor_ranges(bits, &[(20, 34), (50, 65), (81, 85)])
}

fn checksum_c6(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 6) ^ xor_range(bits, 35, 85)
}

fn checksum_c7(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 7) ^ xor_range(bits, 66, 85)
}

fn checksum_overall(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    xor_range(bits, 1, 85)
}

fn correction_index(syndrome: u8) -> usize {
    let highest_set = 8 - syndrome.leading_zeros() as usize;
    usize::from(syndrome) + 8 - highest_set
}

fn decode_immediate_navigation(
    slot: GlonassSlot,
    string_1: &GlonassNavigationString,
    string_2: &GlonassNavigationString,
    string_3: &GlonassNavigationString,
    string_4: &GlonassNavigationString,
) -> Result<GlonassImmediateNavigationData, GlonassNavigationFrameRejection> {
    let reported_slot = string_4
        .unsigned_bits(11, 15)
        .try_into()
        .ok()
        .and_then(GlonassSlot::new);
    if let Some(reported_slot) = reported_slot {
        if reported_slot != slot {
            return Err(GlonassNavigationFrameRejection {
                reason: GlonassNavigationFrameRejectionReason::SlotMismatch,
                string_number: Some(4),
                expected_slot: Some(slot),
                reported_slot: Some(reported_slot),
            });
        }
    }

    let p1_code = string_1.unsigned_bits(77, 78) as u8;
    let tb_update_interval_min = p1_update_interval_minutes(p1_code);
    let tb_is_odd = matches!(tb_update_interval_min, 30 | 60)
        .then(|| string_2.unsigned_bits(77, 77) != 0);
    let frame_time = GlonassFrameTime {
        hour: string_1.unsigned_bits(72, 76) as u8,
        minute: string_1.unsigned_bits(66, 71) as u8,
        half_minute: string_1.unsigned_bits(65, 65) != 0,
    };

    Ok(GlonassImmediateNavigationData {
        sat: glonass_slot_sat(slot),
        frame_time,
        ephemeris_reference_time_s: string_2.unsigned_bits(70, 76) as u32 * 15 * 60,
        tb_update_interval_min,
        tb_is_odd,
        state_vector: GlonassStateVector {
            x_m: string_1.sign_magnitude(9, 35, 2f64.powi(-11) * 1_000.0),
            y_m: string_2.sign_magnitude(9, 35, 2f64.powi(-11) * 1_000.0),
            z_m: string_3.sign_magnitude(9, 35, 2f64.powi(-11) * 1_000.0),
            vx_mps: string_1.sign_magnitude(41, 64, 2f64.powi(-20) * 1_000.0),
            vy_mps: string_2.sign_magnitude(41, 64, 2f64.powi(-20) * 1_000.0),
            vz_mps: string_3.sign_magnitude(41, 64, 2f64.powi(-20) * 1_000.0),
            ax_mps2: string_1.sign_magnitude(36, 40, 2f64.powi(-30) * 1_000.0),
            ay_mps2: string_2.sign_magnitude(36, 40, 2f64.powi(-30) * 1_000.0),
            az_mps2: string_3.sign_magnitude(36, 40, 2f64.powi(-30) * 1_000.0),
        },
        relative_frequency_bias: string_3.sign_magnitude(69, 79, 2f64.powi(-40)),
        clock_bias_s: string_4.sign_magnitude(59, 80, 2f64.powi(-30)),
        l2_l1_delay_s: Some(string_4.sign_magnitude(54, 58, 2f64.powi(-30))),
        health: GlonassImmediateHealth {
            line_unhealthy: string_3.unsigned_bits(65, 65) != 0,
            status_code: string_2.unsigned_bits(78, 80) as u8,
        },
        immediate_data_age_days: string_4.unsigned_bits(49, 53) as u8,
        satellite_type: glonass_satellite_type_from_word(string_4.unsigned_bits(9, 10) as u8),
        reported_slot,
        system_time: Some(GlonassSystemTime {
            day_number: string_4.unsigned_bits(16, 26) as u16,
            four_year_interval: None,
        }),
        accuracy_code: Some(string_4.unsigned_bits(30, 33) as u8),
    })
}

fn p1_update_interval_minutes(code: u8) -> u8 {
    match code {
        0 => 0,
        1 => 30,
        2 => 45,
        3 => 60,
        _ => unreachable!("two-bit P1 code must stay within range"),
    }
}

fn decode_almanac_time(string_5: &GlonassNavigationString) -> GlonassAlmanacTimeData {
    GlonassAlmanacTimeData {
        system_time: GlonassSystemTime {
            day_number: string_5.unsigned_bits(70, 80) as u16,
            four_year_interval: nonzero_u8(string_5.unsigned_bits(32, 36) as u8),
        },
        utc_offset_s: string_5.sign_magnitude(38, 69, 2f64.powi(-31)),
        gps_minus_glonass_s: string_5.sign_magnitude(10, 31, 2f64.powi(-30) * 86_400.0),
    }
}

fn decode_almanac_entry(
    even: &GlonassNavigationString,
    odd: &GlonassNavigationString,
) -> Result<GlonassAlmanacEntry, GlonassNavigationFrameRejection> {
    let slot = GlonassSlot::new(even.unsigned_bits(73, 77) as u8);
    let frequency_word = odd.unsigned_bits(10, 14) as u8;
    let Some(frequency_channel) = decode_frequency_channel(frequency_word) else {
        return Err(GlonassNavigationFrameRejection {
            reason: GlonassNavigationFrameRejectionReason::InvalidFrequencyChannel,
            string_number: Some(odd.string_number),
            expected_slot: None,
            reported_slot: slot,
        });
    };
    let Some(slot) = slot else {
        return Err(GlonassNavigationFrameRejection {
            reason: GlonassNavigationFrameRejectionReason::SlotMismatch,
            string_number: Some(even.string_number),
            expected_slot: None,
            reported_slot: None,
        });
    };

    Ok(GlonassAlmanacEntry {
        sat: glonass_slot_sat(slot),
        frequency_channel,
        health_operational: even.unsigned_bits(80, 80) == 0,
        longitude_of_ascending_node_rad: semicircles_to_radians(
            even.sign_magnitude(42, 62, 2f64.powi(-20)),
        ),
        ascending_node_time_s: odd.unsigned_bits(44, 64) as f64 * 2f64.powi(-5),
        inclination_delta_rad: semicircles_to_radians(
            even.sign_magnitude(24, 41, 2f64.powi(-20)),
        ),
        draconian_period_correction_s: odd.sign_magnitude(22, 43, 2f64.powi(-9)),
        draconian_period_rate_s_per_orbit: odd.sign_magnitude(15, 21, 2f64.powi(-14)),
        eccentricity: even.unsigned_bits(9, 23) as f64 * 2f64.powi(-20),
        argument_of_perigee_rad: semicircles_to_radians(
            odd.sign_magnitude(65, 80, 2f64.powi(-15)),
        ),
        clock_bias_s: even.sign_magnitude(63, 72, 2f64.powi(-18)),
        satellite_type: glonass_satellite_type_from_word(even.unsigned_bits(78, 79) as u8),
    })
}

fn decode_frequency_channel(word: u8) -> Option<GlonassFrequencyChannel> {
    let channel = match word {
        0..=13 => word as i8,
        25..=31 => word as i8 - 32,
        _ => return None,
    };
    GlonassFrequencyChannel::new(channel)
}

fn nonzero_u8(value: u8) -> Option<u8> {
    (value != 0).then_some(value)
}

fn decode_string_number(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    unsigned_bits(bits, 81, 84) as u8
}

fn bit(bits: &[u8; GLONASS_STRING_BITS], position: usize) -> u8 {
    bits[GLONASS_STRING_BITS - position]
}

fn flip_bit(bits: &mut [u8; GLONASS_STRING_BITS], position: usize) {
    let idx = GLONASS_STRING_BITS - position;
    bits[idx] ^= 1;
}

fn xor_positions(bits: &[u8; GLONASS_STRING_BITS], positions: &[usize]) -> u8 {
    positions.iter().fold(0_u8, |acc, position| acc ^ bit(bits, *position))
}

fn xor_range(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u8 {
    (low..=high).fold(0_u8, |acc, position| acc ^ bit(bits, position))
}

fn xor_ranges(bits: &[u8; GLONASS_STRING_BITS], ranges: &[(usize, usize)]) -> u8 {
    ranges.iter().fold(0_u8, |acc, (low, high)| acc ^ xor_range(bits, *low, *high))
}

fn unsigned_bits(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u64 {
    let mut value = 0_u64;
    for position in (low..=high).rev() {
        value = (value << 1) | u64::from(bit(bits, position));
    }
    value
}

#[cfg(test)]
mod tests {
    use super::{
        checksum_c1, checksum_c2, checksum_c3, checksum_c4, checksum_c5, checksum_c6,
        checksum_c7, decode_glonass_broadcast_navigation_frame, decode_glonass_navigation_string,
        flip_bit, GlonassNavigationFrameRejectionReason, GlonassNavigationStringRejectionReason,
        GLONASS_STRING_BITS,
    };
    use bijux_gnss_core::api::GlonassSlot;
    use crate::orbits::glonass::GlonassSatelliteType;

    #[test]
    fn navigation_string_decodes_string_number_and_payload_bits() {
        let bits = encoded_string(3, &[(65, 1), (64, 1), (20, 1)]);
        let decoded = decode_glonass_navigation_string(&bits).expect("decoded string");

        assert_eq!(decoded.string_number, 3);
        assert_eq!(decoded.bit(65), 1);
        assert_eq!(decoded.bit(64), 1);
        assert_eq!(decoded.bit(20), 1);
        assert_eq!(decoded.parity.corrected_bit_index, None);
    }

    #[test]
    fn navigation_string_corrects_single_data_bit_error() {
        let mut bits = encoded_string(7, &[(65, 1), (30, 1), (22, 1)]);
        flip_bit(&mut bits, 30);

        let decoded = decode_glonass_navigation_string(&bits).expect("single-bit correction");

        assert_eq!(decoded.string_number, 7);
        assert_eq!(decoded.parity.corrected_bit_index, Some(30));
        assert_eq!(decoded.bit(30), 1);
    }

    #[test]
    fn navigation_string_extracts_unsigned_and_signed_fields() {
        let mut bits = encoded_string(4, &[]);
        set_unsigned_bits(&mut bits, 9, 13, 0b1_0110);
        set_sign_magnitude_bits(&mut bits, 20, 24, -6);
        apply_check_bits(&mut bits);

        let decoded = decode_glonass_navigation_string(&bits).expect("decoded field string");

        assert_eq!(decoded.unsigned_bits(9, 13), 0b1_0110);
        assert_eq!(decoded.sign_magnitude(20, 24, 0.5), -3.0);
    }

    #[test]
    fn navigation_string_rejects_multiple_bit_errors() {
        let mut bits = encoded_string(12, &[(65, 1), (55, 1)]);
        flip_bit(&mut bits, 30);
        flip_bit(&mut bits, 31);

        let rejection = decode_glonass_navigation_string(&bits).expect_err("multiple-bit rejection");

        assert_eq!(rejection.reason, GlonassNavigationStringRejectionReason::UnrecoverableParity);
    }

    #[test]
    fn broadcast_frame_decodes_immediate_state_and_clock_terms() {
        let slot = GlonassSlot::new(8).expect("slot");
        let mut string_1 = encoded_string(1, &[]);
        set_unsigned_bits(&mut string_1, 77, 78, 3);
        set_unsigned_bits(&mut string_1, 72, 76, 12);
        set_unsigned_bits(&mut string_1, 66, 71, 34);
        set_unsigned_bits(&mut string_1, 65, 65, 1);
        set_sign_magnitude_bits(&mut string_1, 9, 35, 12_345);
        set_sign_magnitude_bits(&mut string_1, 41, 64, -543_210);
        set_sign_magnitude_bits(&mut string_1, 36, 40, 9);
        apply_check_bits(&mut string_1);

        let mut string_2 = encoded_string(2, &[]);
        set_unsigned_bits(&mut string_2, 70, 76, 66);
        set_unsigned_bits(&mut string_2, 77, 77, 1);
        set_unsigned_bits(&mut string_2, 78, 80, 5);
        set_sign_magnitude_bits(&mut string_2, 9, 35, -14_000);
        set_sign_magnitude_bits(&mut string_2, 41, 64, 321_000);
        set_sign_magnitude_bits(&mut string_2, 36, 40, -6);
        apply_check_bits(&mut string_2);

        let mut string_3 = encoded_string(3, &[]);
        set_unsigned_bits(&mut string_3, 66, 67, 2);
        set_unsigned_bits(&mut string_3, 65, 65, 1);
        set_unsigned_bits(&mut string_3, 80, 80, 1);
        set_sign_magnitude_bits(&mut string_3, 69, 79, -50);
        set_sign_magnitude_bits(&mut string_3, 9, 35, 8_765);
        set_sign_magnitude_bits(&mut string_3, 41, 64, -123_456);
        set_sign_magnitude_bits(&mut string_3, 36, 40, 4);
        apply_check_bits(&mut string_3);

        let mut string_4 = encoded_string(4, &[]);
        set_unsigned_bits(&mut string_4, 9, 10, 1);
        set_unsigned_bits(&mut string_4, 11, 15, u64::from(slot.value()));
        set_unsigned_bits(&mut string_4, 16, 26, 1_200);
        set_unsigned_bits(&mut string_4, 30, 33, 6);
        set_unsigned_bits(&mut string_4, 34, 34, 1);
        set_unsigned_bits(&mut string_4, 49, 53, 9);
        set_sign_magnitude_bits(&mut string_4, 54, 58, 7);
        set_sign_magnitude_bits(&mut string_4, 59, 80, -2_000);
        apply_check_bits(&mut string_4);

        let decoded_strings = [string_1, string_2, string_3, string_4]
            .into_iter()
            .map(|bits| decode_glonass_navigation_string(&bits).expect("decoded string"))
            .collect::<Vec<_>>();

        let frame =
            decode_glonass_broadcast_navigation_frame(slot, &decoded_strings).expect("frame decode");
        let frame = frame.expect("complete immediate frame");

        assert_eq!(frame.sat.prn, slot.value());
        assert_eq!(frame.immediate.frame_time.hour, 12);
        assert_eq!(frame.immediate.frame_time.minute, 34);
        assert!(frame.immediate.frame_time.half_minute);
        assert_eq!(frame.immediate.frame_time.seconds_of_day(), 45_270);
        assert_eq!(frame.immediate.ephemeris_reference_time_s, 59_400);
        assert_eq!(frame.immediate.tb_update_interval_min, 60);
        assert_eq!(frame.immediate.tb_is_odd, Some(true));
        assert_eq!(frame.immediate.health.status_code, 5);
        assert!(frame.immediate.health.line_unhealthy);
        assert_eq!(frame.immediate.immediate_data_age_days, 9);
        assert_eq!(frame.immediate.satellite_type, GlonassSatelliteType::GlonassM);
        assert_eq!(frame.immediate.reported_slot, Some(slot));
        assert_eq!(frame.immediate.system_time.expect("system time").day_number, 1_200);
        assert_eq!(frame.immediate.accuracy_code, Some(6));
        assert!((frame.immediate.relative_frequency_bias + 50.0 * 2f64.powi(-40)).abs() < 1.0e-18);
        assert!((frame.immediate.clock_bias_s + 2_000.0 * 2f64.powi(-30)).abs() < 1.0e-15);
        assert!((frame.immediate.l2_l1_delay_s.expect("delay") - 7.0 * 2f64.powi(-30)).abs() < 1.0e-15);
        assert!((frame.immediate.state_vector.x_m - 12_345.0 * 2f64.powi(-11) * 1_000.0).abs() < 1.0e-9);
        assert!((frame.immediate.state_vector.y_m + 14_000.0 * 2f64.powi(-11) * 1_000.0).abs() < 1.0e-9);
        assert!((frame.immediate.state_vector.z_m - 8_765.0 * 2f64.powi(-11) * 1_000.0).abs() < 1.0e-9);
        assert!((frame.immediate.state_vector.vx_mps + 543_210.0 * 2f64.powi(-20) * 1_000.0).abs() < 1.0e-9);
        assert!((frame.immediate.state_vector.vy_mps - 321_000.0 * 2f64.powi(-20) * 1_000.0).abs() < 1.0e-9);
        assert!((frame.immediate.state_vector.vz_mps + 123_456.0 * 2f64.powi(-20) * 1_000.0).abs() < 1.0e-9);
    }

    #[test]
    fn broadcast_frame_rejects_reported_slot_mismatch() {
        let expected_slot = GlonassSlot::new(8).expect("slot");
        let wrong_slot = GlonassSlot::new(9).expect("slot");
        let string_1 = decode_glonass_navigation_string(&encoded_string(1, &[])).expect("string 1");
        let string_2 = decode_glonass_navigation_string(&encoded_string(2, &[])).expect("string 2");
        let string_3 = decode_glonass_navigation_string(&encoded_string(3, &[])).expect("string 3");
        let mut string_4_bits = encoded_string(4, &[]);
        set_unsigned_bits(&mut string_4_bits, 11, 15, u64::from(wrong_slot.value()));
        apply_check_bits(&mut string_4_bits);
        let string_4 = decode_glonass_navigation_string(&string_4_bits).expect("string 4");

        let rejection = decode_glonass_broadcast_navigation_frame(
            expected_slot,
            &[string_1, string_2, string_3, string_4],
        )
        .expect_err("slot mismatch rejection");

        assert_eq!(rejection.reason, GlonassNavigationFrameRejectionReason::SlotMismatch);
        assert_eq!(rejection.expected_slot, Some(expected_slot));
        assert_eq!(rejection.reported_slot, Some(wrong_slot));
    }

    #[test]
    fn broadcast_frame_decodes_almanac_time_and_frequency_channel() {
        let slot = GlonassSlot::new(8).expect("slot");
        let mut string_5 = encoded_string(5, &[]);
        set_sign_magnitude_bits(&mut string_5, 10, 31, 512);
        set_unsigned_bits(&mut string_5, 32, 36, 9);
        set_sign_magnitude_bits(&mut string_5, 38, 69, -1_024);
        set_unsigned_bits(&mut string_5, 70, 80, 777);
        apply_check_bits(&mut string_5);

        let mut string_6 = encoded_string(6, &[]);
        set_unsigned_bits(&mut string_6, 9, 23, 1_200);
        set_sign_magnitude_bits(&mut string_6, 24, 41, -50);
        set_sign_magnitude_bits(&mut string_6, 42, 62, 1_000);
        set_sign_magnitude_bits(&mut string_6, 63, 72, -20);
        set_unsigned_bits(&mut string_6, 73, 77, 11);
        set_unsigned_bits(&mut string_6, 78, 79, 1);
        set_unsigned_bits(&mut string_6, 80, 80, 0);
        apply_check_bits(&mut string_6);

        let mut string_7 = encoded_string(7, &[]);
        set_unsigned_bits(&mut string_7, 10, 14, 28);
        set_sign_magnitude_bits(&mut string_7, 15, 21, -3);
        set_sign_magnitude_bits(&mut string_7, 22, 43, 200);
        set_unsigned_bits(&mut string_7, 44, 64, 10_000);
        set_sign_magnitude_bits(&mut string_7, 65, 80, -200);
        apply_check_bits(&mut string_7);

        let decoded_strings = [
            encoded_string(1, &[]),
            encoded_string(2, &[]),
            encoded_string(3, &[]),
            encoded_string(4, &[]),
            string_5,
            string_6,
            string_7,
        ]
        .into_iter()
        .map(|bits| decode_glonass_navigation_string(&bits).expect("decoded string"))
        .collect::<Vec<_>>();

        let frame =
            decode_glonass_broadcast_navigation_frame(slot, &decoded_strings).expect("frame decode");
        let frame = frame.expect("frame");
        let system_time = frame.system_time.expect("system time");
        let entry = frame.almanac_entries.first().expect("almanac entry");

        assert_eq!(system_time.system_time.day_number, 777);
        assert_eq!(system_time.system_time.four_year_interval, Some(9));
        assert!((system_time.utc_offset_s + 1_024.0 * 2f64.powi(-31)).abs() < 1.0e-15);
        assert!((system_time.gps_minus_glonass_s - 512.0 * 2f64.powi(-30) * 86_400.0).abs() < 1.0e-12);
        assert_eq!(entry.sat.prn, 11);
        assert_eq!(entry.frequency_channel.value(), -4);
        assert!(entry.health_operational);
        assert_eq!(entry.satellite_type, GlonassSatelliteType::GlonassM);
        assert!((entry.longitude_of_ascending_node_rad - std::f64::consts::PI * 1_000.0 * 2f64.powi(-20)).abs() < 1.0e-12);
        assert!((entry.inclination_delta_rad + std::f64::consts::PI * 50.0 * 2f64.powi(-20)).abs() < 1.0e-12);
        assert!((entry.ascending_node_time_s - 10_000.0 * 2f64.powi(-5)).abs() < 1.0e-12);
        assert!((entry.draconian_period_correction_s - 200.0 * 2f64.powi(-9)).abs() < 1.0e-12);
        assert!((entry.draconian_period_rate_s_per_orbit + 3.0 * 2f64.powi(-14)).abs() < 1.0e-12);
        assert!((entry.eccentricity - 1_200.0 * 2f64.powi(-20)).abs() < 1.0e-12);
        assert!((entry.argument_of_perigee_rad + std::f64::consts::PI * 200.0 * 2f64.powi(-15)).abs() < 1.0e-12);
        assert!((entry.clock_bias_s + 20.0 * 2f64.powi(-18)).abs() < 1.0e-15);
    }

    #[test]
    fn broadcast_frame_rejects_invalid_almanac_frequency_word() {
        let slot = GlonassSlot::new(8).expect("slot");
        let mut string_6 = encoded_string(6, &[]);
        set_unsigned_bits(&mut string_6, 73, 77, 11);
        apply_check_bits(&mut string_6);

        let mut string_7 = encoded_string(7, &[]);
        set_unsigned_bits(&mut string_7, 10, 14, 20);
        apply_check_bits(&mut string_7);

        let decoded_strings = [
            encoded_string(1, &[]),
            encoded_string(2, &[]),
            encoded_string(3, &[]),
            encoded_string(4, &[]),
            string_6,
            string_7,
        ]
        .into_iter()
        .map(|bits| decode_glonass_navigation_string(&bits).expect("decoded string"))
        .collect::<Vec<_>>();

        let rejection = decode_glonass_broadcast_navigation_frame(slot, &decoded_strings)
            .expect_err("invalid frequency word rejection");

        assert_eq!(rejection.reason, GlonassNavigationFrameRejectionReason::InvalidFrequencyChannel);
        assert_eq!(rejection.string_number, Some(7));
    }

    fn encoded_string(string_number: u8, ones: &[(usize, u8)]) -> [u8; GLONASS_STRING_BITS] {
        let mut bits = [0_u8; GLONASS_STRING_BITS];
        set_unsigned_bits(&mut bits, 81, 84, u64::from(string_number));
        for (position, value) in ones {
            set_bit(&mut bits, *position, *value);
        }
        apply_check_bits(&mut bits);
        bits
    }

    fn apply_check_bits(bits: &mut [u8; GLONASS_STRING_BITS]) {
        for position in 1..=8 {
            set_bit(bits, position, 0);
        }
        set_bit(bits, 1, checksum_c1(bits));
        set_bit(bits, 2, checksum_c2(bits));
        set_bit(bits, 3, checksum_c3(bits));
        set_bit(bits, 4, checksum_c4(bits));
        set_bit(bits, 5, checksum_c5(bits));
        set_bit(bits, 6, checksum_c6(bits));
        set_bit(bits, 7, checksum_c7(bits));
        set_bit(bits, 8, xor_range(bits, 1, 85));
    }

    fn set_unsigned_bits(bits: &mut [u8; GLONASS_STRING_BITS], low: usize, high: usize, value: u64) {
        for (offset, position) in (low..=high).enumerate() {
            let bit_value = ((value >> offset) & 1) as u8;
            set_bit(bits, position, bit_value);
        }
    }

    fn set_sign_magnitude_bits(
        bits: &mut [u8; GLONASS_STRING_BITS],
        low: usize,
        high: usize,
        signed_value: i64,
    ) {
        let width = high - low + 1;
        let magnitude_mask = (1_u64 << (width - 1)) - 1;
        let sign_bit = u64::from(signed_value.is_negative());
        let magnitude = signed_value.unsigned_abs() & magnitude_mask;
        set_unsigned_bits(bits, low, high, magnitude | (sign_bit << (width - 1)));
    }

    fn set_bit(bits: &mut [u8; GLONASS_STRING_BITS], position: usize, value: u8) {
        bits[GLONASS_STRING_BITS - position] = value;
    }

    fn xor_range(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u8 {
        (low..=high).fold(0_u8, |acc, position| acc ^ bits[GLONASS_STRING_BITS - position])
    }
}
