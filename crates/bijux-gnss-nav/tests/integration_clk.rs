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
    assert_eq!(bias, 3.0e-9);
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

#[test]
fn clk_interpolates_cubic_bias_and_sigma_between_epochs() {
    let data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000001
AS G01 2020 01 01 00 15 00.000000  2  0.000000010  0.000000002
AS G01 2020 01 01 00 30 00.000000  2  0.000000049  0.000000009
AS G01 2020 01 01 00 45 00.000000  2  0.000000142  0.000000028
";
    let provider: ClkProvider = data.parse().expect("parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };

    let bias = provider.bias_s(sat, 1_350.0).expect("interpolated bias");
    let sigma = provider.sigma_s(sat, 1_350.0).expect("interpolated sigma");

    assert!((bias - 2.425e-8).abs() < 1e-18);
    assert!((sigma - 4.375e-9).abs() < 1e-18);
}
