use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    decode_beidou_b1i_subframe, decode_beidou_broadcast_navigation_data,
    BeidouBroadcastNavigationData,
};

pub const BEIDOU_D1_SUBFRAME_BITS: usize = 300;
const BEIDOU_D1_PREAMBLE: u16 = 0b111_0001_0010;

pub fn sample_beidou_sat() -> SatId {
    SatId { constellation: Constellation::Beidou, prn: 11 }
}

#[allow(dead_code)]
pub fn sample_beidou_navigation() -> BeidouBroadcastNavigationData {
    let sat = sample_beidou_sat();
    let subframes = [
        decode_beidou_b1i_subframe(&sample_clock_subframe()).expect("clock"),
        decode_beidou_b1i_subframe(&sample_ephemeris_1_subframe()).expect("ephemeris 1"),
        decode_beidou_b1i_subframe(&sample_ephemeris_2_subframe()).expect("ephemeris 2"),
    ];

    decode_beidou_broadcast_navigation_data(sat, &subframes)
        .expect("navigation assembly")
        .expect("complete navigation")
}

pub fn sample_clock_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
    let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
    set_common_header(&mut bits, 1, 345_678);
    set_unsigned_bits(&mut bits, 43, 1, 0);
    set_unsigned_bits(&mut bits, 44, 5, 17);
    set_unsigned_bits(&mut bits, 49, 4, 5);
    set_unsigned_bits(&mut bits, 61, 13, 1_234);
    set_split_unsigned_bits(&mut bits, &[(74, 9), (91, 8)], 8_100);
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

pub fn sample_ephemeris_1_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
    sample_ephemeris_1_subframe_with_sow(345_684)
}

pub fn sample_ephemeris_1_subframe_with_sow(sow_s: u32) -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
    let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
    set_common_header(&mut bits, 2, sow_s);
    set_split_signed_bits(&mut bits, &[(43, 10), (61, 6)], -2_345);
    set_split_signed_bits(&mut bits, &[(67, 16), (91, 2)], 0x1_2345);
    set_split_signed_bits(&mut bits, &[(93, 20), (121, 12)], -0x1ABC_DEF);
    set_split_unsigned_bits(&mut bits, &[(133, 10), (151, 22)], 0x1234_5678);
    set_signed_bits(&mut bits, 181, 18, -54_321);
    set_split_signed_bits(&mut bits, &[(199, 4), (211, 14)], 12_345);
    set_split_signed_bits(&mut bits, &[(225, 8), (241, 10)], -9_876);
    set_split_unsigned_bits(&mut bits, &[(251, 12), (271, 20)], 0x2345_6789);
    set_unsigned_bits(&mut bits, 291, 2, 0b00);
    bits
}

pub fn sample_ephemeris_2_subframe() -> [u8; BEIDOU_D1_SUBFRAME_BITS] {
    let mut bits = [0_u8; BEIDOU_D1_SUBFRAME_BITS];
    set_common_header(&mut bits, 3, 345_690);
    set_split_unsigned_bits(&mut bits, &[(43, 10), (61, 5)], 8_100);
    set_split_signed_bits(&mut bits, &[(66, 17), (91, 15)], 0x1234_5678);
    set_split_signed_bits(&mut bits, &[(106, 7), (121, 11)], -45_678);
    set_split_signed_bits(&mut bits, &[(132, 11), (151, 13)], -0x12_3456);
    set_split_signed_bits(&mut bits, &[(164, 9), (181, 9)], 34_567);
    set_split_signed_bits(&mut bits, &[(190, 13), (211, 1)], -0x1AAA);
    set_split_signed_bits(&mut bits, &[(212, 21), (241, 11)], -0x1234_5678);
    set_split_signed_bits(&mut bits, &[(252, 11), (271, 21)], 0x2345_6789);
    bits
}

fn set_common_header(bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS], subframe_id: u8, sow_s: u32) {
    set_unsigned_bits(bits, 1, 11, u64::from(BEIDOU_D1_PREAMBLE));
    set_unsigned_bits(bits, 16, 3, u64::from(subframe_id));
    set_split_unsigned_bits(bits, &[(19, 8), (31, 12)], u64::from(sow_s));
}

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

fn set_signed_bits(bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS], start: usize, len: usize, value: i64) {
    set_unsigned_bits(bits, start, len, encode_signed(value, len));
}

fn set_split_signed_bits(
    bits: &mut [u8; BEIDOU_D1_SUBFRAME_BITS],
    parts: &[(usize, usize)],
    value: i64,
) {
    let total_bits: usize = parts.iter().map(|(_, len)| *len).sum();
    set_split_unsigned_bits(bits, parts, encode_signed(value, total_bits));
}

fn encode_signed(value: i64, bits: usize) -> u64 {
    let mask = if bits == 64 { u64::MAX } else { (1_u64 << bits) - 1 };
    (value as u64) & mask
}
