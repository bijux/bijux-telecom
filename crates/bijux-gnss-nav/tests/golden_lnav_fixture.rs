#![allow(missing_docs)]
use bijux_gnss_nav::decode_rawephem_hex;

#[test]
fn rawephem_fixture_decodes_week_and_toe() {
    // From NovAtel RAWEPHEMA ASCII example (GPS PRN 1, week 2209, ref secs 504000).
    let sub1 = "8b0284a1b8a52850000724918b913e21a92dc6ee0b217b0c00ffb72fac04";
    let sub2 = "8b0284a1b92b21fac82899520ec7b7fb31061550921d09a10d62b87b0c7c";
    let sub3 = "8b0284a1b9adff7d74db71f3ffaa2840ed6e0fe024cddf1effadc82106c4";

    let eph = decode_rawephem_hex(1, sub1, sub2, sub3).expect("decoded ephemeris");
    assert_eq!(eph.sat.prn, 1);
    assert_eq!(eph.week, 2209 % 1024);
    assert!((eph.toc_s - 504000.0).abs() < 1.0);
    assert!((eph.toe_s - 504000.0).abs() < 16.0);
    assert!(eph.sqrt_a > 5000.0);
}
