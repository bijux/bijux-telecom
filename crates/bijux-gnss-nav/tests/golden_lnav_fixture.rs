#![allow(missing_docs)]
mod support;

use support::broadcast_reference::gps_prn1_20220513_fixture;

#[test]
fn rawephem_fixture_decodes_broadcast_ephemeris_fields() {
    let eph = gps_prn1_20220513_fixture().ephemeris;
    let angle_lsb = 2f64.powi(-31) * std::f64::consts::PI;
    let rate_lsb = 2f64.powi(-43) * std::f64::consts::PI;

    assert_eq!(eph.sat.prn, 1);
    assert_eq!(eph.week, 2209);
    assert_eq!(eph.sv_health, 0);
    assert_eq!(eph.iodc, 33);
    assert_eq!(eph.iode, 33);
    assert!((eph.toc_s - 504000.0).abs() < 1.0);
    assert!((eph.af0 - 0.0003637080080807209).abs() < f64::EPSILON);
    assert!((eph.af1 - -8.29913915367797e-12).abs() < f64::EPSILON);
    assert_eq!(eph.af2, 0.0);
    assert!((eph.crs - -41.75).abs() <= 2f64.powi(-5));
    assert!((eph.delta_n - 3.7119403314999e-09).abs() <= rate_lsb);
    assert!((eph.m0 - 2.013999821508).abs() <= angle_lsb);
    assert!((eph.cuc - -2.292916178703308e-06).abs() <= 2f64.powi(-29));
    assert!((eph.e - 0.011881368467584252).abs() <= 2f64.powi(-33));
    assert!((eph.cus - 1.3845041394233704e-05).abs() <= 2f64.powi(-29));
    assert!((eph.sqrt_a - 5153.673202514648).abs() <= 2f64.powi(-19));
    assert!((eph.toe_s - 504000.0).abs() < 16.0);
    assert!((eph.cic - -2.4400651454925537e-07).abs() <= 2f64.powi(-29));
    assert!((eph.omega0 - 2.868107379133225).abs() <= angle_lsb);
    assert!((eph.cis - -1.601874828338623e-07).abs() <= 2f64.powi(-29));
    assert!((eph.i0 - 0.987972546485047).abs() <= angle_lsb);
    assert!((eph.crc - 127.0).abs() <= 2f64.powi(-5));
    assert!((eph.w - 0.9033106216217657).abs() <= angle_lsb);
    assert!((eph.omegadot - -7.517455989359174e-09).abs() <= rate_lsb);
    assert!((eph.idot - 1.5464929890690433e-10).abs() <= rate_lsb);
}
