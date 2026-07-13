use bijux_gnss_core::api::GlonassSlot;
use bijux_gnss_nav::api::{
    decode_glonass_broadcast_navigation_frame, decode_glonass_navigation_string,
};

const GLONASS_STRING_BITS: usize = 85;

#[test]
fn decoded_glonass_frame_exposes_immediate_time_and_channel_data() {
    let slot = GlonassSlot::new(8).expect("slot");
    let strings = sample_navigation_strings(slot);
    let decoded = strings
        .iter()
        .map(|bits| decode_glonass_navigation_string(bits).expect("decoded string"))
        .collect::<Vec<_>>();

    let frame = decode_glonass_broadcast_navigation_frame(slot, &decoded)
        .expect("frame decode")
        .expect("complete frame");

    assert_eq!(frame.sat.prn, slot.value());
    assert_eq!(frame.immediate.frame_time.seconds_of_day(), 45_270);
    assert_eq!(frame.immediate.ephemeris_reference_time_s, 59_400);
    assert_eq!(frame.immediate.system_time.expect("system time").day_number, 1_200);
    assert_eq!(frame.system_time.expect("almanac time").system_time.day_number, 777);
    assert_eq!(frame.almanac_entries.len(), 1);
    assert_eq!(frame.almanac_entries[0].sat.prn, 11);
    assert_eq!(frame.almanac_entries[0].frequency_channel.value(), -4);
}

#[test]
fn corrected_glonass_string_still_assembles_frame() {
    let slot = GlonassSlot::new(8).expect("slot");
    let mut strings = sample_navigation_strings(slot);
    flip_bit(&mut strings[6], 43);

    let decoded = strings
        .iter()
        .map(|bits| decode_glonass_navigation_string(bits).expect("decoded string"))
        .collect::<Vec<_>>();

    assert_eq!(decoded[6].parity.corrected_bit_index, Some(43));

    let frame = decode_glonass_broadcast_navigation_frame(slot, &decoded)
        .expect("frame decode")
        .expect("complete frame");

    assert_eq!(frame.almanac_entries[0].frequency_channel.value(), -4);
    assert!(
        (frame.almanac_entries[0].draconian_period_correction_s - 200.0 * 2f64.powi(-9)).abs()
            < 1.0e-12
    );
}

fn sample_navigation_strings(slot: GlonassSlot) -> Vec<[u8; GLONASS_STRING_BITS]> {
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

    vec![string_1, string_2, string_3, string_4, string_5, string_6, string_7]
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
    set_bit(bits, 8, checksum_overall(bits));
}

fn set_unsigned_bits(bits: &mut [u8; GLONASS_STRING_BITS], low: usize, high: usize, value: u64) {
    for (offset, position) in (low..=high).enumerate() {
        set_bit(bits, position, ((value >> offset) & 1) as u8);
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

fn flip_bit(bits: &mut [u8; GLONASS_STRING_BITS], position: usize) {
    bits[GLONASS_STRING_BITS - position] ^= 1;
}

fn bit(bits: &[u8; GLONASS_STRING_BITS], position: usize) -> u8 {
    bits[GLONASS_STRING_BITS - position]
}

fn checksum_c1(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 1)
        ^ xor_positions(
            bits,
            &[
                9, 10, 12, 13, 15, 17, 19, 20, 22, 24, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45,
                47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84,
            ],
        )
}

fn checksum_c2(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 2)
        ^ xor_positions(
            bits,
            &[
                9, 11, 12, 14, 15, 18, 19, 21, 22, 25, 26, 29, 30, 33, 34, 36, 37, 40, 41, 44, 45,
                48, 49, 52, 53, 56, 57, 60, 61, 64, 65, 67, 68, 71, 72, 75, 76, 79, 80, 83, 84,
            ],
        )
}

fn checksum_c3(bits: &[u8; GLONASS_STRING_BITS]) -> u8 {
    bit(bits, 3)
        ^ xor_ranges(
            bits,
            &[
                (10, 12),
                (16, 19),
                (23, 26),
                (31, 34),
                (38, 41),
                (46, 49),
                (54, 57),
                (62, 65),
                (69, 72),
                (77, 80),
                (85, 85),
            ],
        )
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

fn xor_positions(bits: &[u8; GLONASS_STRING_BITS], positions: &[usize]) -> u8 {
    positions.iter().fold(0_u8, |acc, position| acc ^ bit(bits, *position))
}

fn xor_range(bits: &[u8; GLONASS_STRING_BITS], low: usize, high: usize) -> u8 {
    (low..=high).fold(0_u8, |acc, position| acc ^ bit(bits, position))
}

fn xor_ranges(bits: &[u8; GLONASS_STRING_BITS], ranges: &[(usize, usize)]) -> u8 {
    ranges.iter().fold(0_u8, |acc, (low, high)| acc ^ xor_range(bits, *low, *high))
}
