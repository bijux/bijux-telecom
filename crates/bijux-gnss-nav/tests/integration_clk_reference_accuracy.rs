#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::ClkProvider;

const REDUCED_IGS_MAX_BIAS_ERROR_S: f64 = 1.0e-10;
const REDUCED_IGS_RMS_BIAS_ERROR_S: f64 = 8.0e-11;
const REDUCED_IGS_MAX_SIGMA_ERROR_S: f64 = 1.0e-11;
const REDUCED_IGS_RMS_SIGMA_ERROR_S: f64 = 6.0e-12;

fn load_clk(name: &str) -> ClkProvider {
    std::fs::read_to_string(fixture_path(name))
        .unwrap_or_else(|_| panic!("read fixture {}", fixture_path(name).display()))
        .parse()
        .unwrap_or_else(|_| panic!("parse CLK fixture {name}"))
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}

#[test]
fn reduced_igs_clk_fixtures_measure_subnanosecond_interpolation_error() {
    let fixtures = [
        (
            "gps_prn1_20220513_igs_reference.clk",
            SatId { constellation: Constellation::Gps, prn: 1 },
        ),
        (
            "gps_prn2_20220514_igs_reference.clk",
            SatId { constellation: Constellation::Gps, prn: 2 },
        ),
    ];

    for (name, sat) in fixtures {
        let provider = load_clk(name);
        let summary = provider
            .interpolation_summary(sat)
            .unwrap_or_else(|| panic!("clock interpolation summary for {name}"));

        assert_eq!(summary.sample_count, 3);
        assert!(
            summary.max_bias_error_s <= REDUCED_IGS_MAX_BIAS_ERROR_S,
            "{name} exceeded reduced-IGS max bias interpolation budget: {:.6} ns",
            summary.max_bias_error_s * 1e9
        );
        assert!(
            summary.rms_bias_error_s <= REDUCED_IGS_RMS_BIAS_ERROR_S,
            "{name} exceeded reduced-IGS RMS bias interpolation budget: {:.6} ns",
            summary.rms_bias_error_s * 1e9
        );
        assert!(
            summary.max_sigma_error_s.expect("sigma interpolation max error")
                <= REDUCED_IGS_MAX_SIGMA_ERROR_S,
            "{name} exceeded reduced-IGS max sigma interpolation budget: {:.6} ns",
            summary.max_sigma_error_s.expect("sigma interpolation max error") * 1e9
        );
        assert!(
            summary.rms_sigma_error_s.expect("sigma interpolation RMS error")
                <= REDUCED_IGS_RMS_SIGMA_ERROR_S,
            "{name} exceeded reduced-IGS RMS sigma interpolation budget: {:.6} ns",
            summary.rms_sigma_error_s.expect("sigma interpolation RMS error") * 1e9
        );
    }
}
