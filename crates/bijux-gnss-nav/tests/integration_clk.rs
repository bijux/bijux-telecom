#![allow(missing_docs)]
use bijux_gnss_core::{Constellation, SatId};
use bijux_gnss_nav::ClkProvider;

#[test]
fn clk_interpolates_bias() {
    let data = "\
AS G01 2020 01 01 00 00 00.000000  0  0.000000  0.000000001
AS G01 2020 01 01 00 15 00.000000  0  0.000000  0.000000003
";
    let provider: ClkProvider = data.parse().expect("parse");
    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 1,
    };
    let bias = provider.bias_s(sat, 900.0).expect("bias");
    assert!(bias > 0.0);
    assert!(bias <= 0.000000003 + 1e-12);
}
