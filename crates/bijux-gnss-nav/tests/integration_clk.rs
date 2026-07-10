#![allow(missing_docs)]
use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::ClkProvider;

#[test]
fn clk_interpolates_bias() {
    let data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000010
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
";
    let provider: ClkProvider = data.parse().expect("parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let bias = provider.bias_s(sat, 900.0).expect("bias");
    assert!(bias > 0.0);
    assert!(bias <= 0.000000003 + 1e-12);
}

#[test]
fn clk_reads_bias_when_sigma_is_present() {
    let data = "\
AS G01 2022 05 13 20 00  0.000000  2    3.637033030964e-04  3.617285566500e-12
";
    let provider: ClkProvider = data.parse().expect("parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let bias = provider.bias_s(sat, 0.0).expect("bias");
    assert!((bias - 3.637033030964e-04).abs() < 1e-18);
    let sigma = provider.sigma_s(sat, 0.0).expect("sigma");
    assert!((sigma - 3.617285566500e-12).abs() < 1e-24);
}
