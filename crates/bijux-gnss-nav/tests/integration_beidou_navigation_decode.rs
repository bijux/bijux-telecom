mod support;

use bijux_gnss_nav::api::{
    decode_beidou_b1i_subframe, decode_beidou_broadcast_navigation_data,
    BeidouD1BatchRejectionReason,
};
use support::beidou_d1_fixture::{
    sample_beidou_sat, sample_clock_subframe, sample_ephemeris_1_subframe,
    sample_ephemeris_1_subframe_with_sow, sample_ephemeris_2_subframe,
};

#[test]
fn public_beidou_decoder_assembles_broadcast_navigation() {
    let sat = sample_beidou_sat();
    let subframes = [
        decode_beidou_b1i_subframe(&sample_clock_subframe()).expect("clock"),
        decode_beidou_b1i_subframe(&sample_ephemeris_1_subframe()).expect("ephemeris 1"),
        decode_beidou_b1i_subframe(&sample_ephemeris_2_subframe()).expect("ephemeris 2"),
    ];

    let navigation = decode_beidou_broadcast_navigation_data(sat, &subframes)
        .expect("navigation assembly")
        .expect("complete navigation");

    assert_eq!(navigation.sat, sat);
    assert_eq!(navigation.bdt.week, 1_234);
    assert_eq!(navigation.bdt.sow_s, 345_678);
    assert_eq!(navigation.ephemeris.aode, 19);
    assert_eq!(navigation.clock.aodc, 17);
    assert_eq!(navigation.ephemeris.toe_s, 64_800.0);
    assert!(navigation.signal_health.autonomous_satellite_good);
    assert!((navigation.clock.tgd1_s - -77.0e-10).abs() < 1.0e-18);
}

#[test]
fn public_beidou_decoder_rejects_nonconsecutive_subframes() {
    let sat = sample_beidou_sat();
    let clock = decode_beidou_b1i_subframe(&sample_clock_subframe()).expect("clock");
    let ephemeris_1 = decode_beidou_b1i_subframe(&sample_ephemeris_1_subframe_with_sow(345_700))
        .expect("ephemeris 1");

    let rejection = decode_beidou_broadcast_navigation_data(sat, &[clock, ephemeris_1])
        .expect_err("sow mismatch rejection");

    assert_eq!(rejection.reason, BeidouD1BatchRejectionReason::NonConsecutiveSow);
    assert_eq!(rejection.subframe_id, Some(2));
    assert_eq!(rejection.expected_sow_s, Some(345_684));
    assert_eq!(rejection.incoming_sow_s, Some(345_700));
}
