#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{Sp3InterpolationEdgePolicy, Sp3InterpolationWindowPolicy, Sp3Provider};

const REDUCED_IGS_MAX_INTERPOLATION_ERROR_M: f64 = 2_400.0;
const REDUCED_IGS_RMS_INTERPOLATION_ERROR_M: f64 = 2_100.0;

fn load_sp3(name: &str) -> Sp3Provider {
    std::fs::read_to_string(fixture_path(name))
        .unwrap_or_else(|_| panic!("read fixture {}", fixture_path(name).display()))
        .parse()
        .unwrap_or_else(|_| panic!("parse SP3 fixture {name}"))
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}

#[test]
fn reduced_igs_sp3_fixtures_measure_interpolation_error_within_documented_budget() {
    let fixtures = [
        (
            "gps_prn1_20220513_igs_reference.sp3",
            SatId { constellation: Constellation::Gps, prn: 1 },
        ),
        (
            "gps_prn2_20220514_igs_reference.sp3",
            SatId { constellation: Constellation::Gps, prn: 2 },
        ),
    ];

    for (name, sat) in fixtures {
        let provider = load_sp3(name);
        let summary = provider
            .interpolation_summary(sat)
            .unwrap_or_else(|| panic!("interpolation summary for {name}"));

        assert_eq!(summary.policy.support_point_count, 4);
        assert_eq!(summary.policy.polynomial_order, 3);
        assert_eq!(summary.policy.window_policy, Sp3InterpolationWindowPolicy::NearestCentered);
        assert_eq!(summary.policy.edge_policy, Sp3InterpolationEdgePolicy::InsideCoverageOnly);
        assert_eq!(summary.policy.max_gap_multiplier, 1.5);
        assert_eq!(summary.policy.max_gap_margin_s, 1.0);
        assert_eq!(summary.sample_count, 3);
        assert_eq!(summary.withheld_sample_count, 3);
        assert_eq!(summary.withheld_edge_sample_count, 2);
        assert!(
            summary.max_position_error_m <= REDUCED_IGS_MAX_INTERPOLATION_ERROR_M,
            "{name} exceeded documented reduced-SP3 max interpolation error budget: {:.3} m",
            summary.max_position_error_m
        );
        assert!(
            summary.rms_position_error_m <= REDUCED_IGS_RMS_INTERPOLATION_ERROR_M,
            "{name} exceeded documented reduced-SP3 RMS interpolation error budget: {:.3} m",
            summary.rms_position_error_m
        );
    }
}
