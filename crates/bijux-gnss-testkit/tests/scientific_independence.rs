#![allow(missing_docs)]

use std::fs;
use std::path::Path;

fn fixture_source(relative_path: &str) -> String {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let source_path = crate_root.join(relative_path);
    fs::read_to_string(&source_path)
        .unwrap_or_else(|error| panic!("read {}: {error}", source_path.display()))
}

#[test]
fn truth_fixtures_do_not_call_nav_geometry_or_differencing_helpers() {
    let forbidden_fragments = [
        "sat_state_gps_l1ca_at_receive_time",
        "sat_state_gps_l1ca_from_observation",
        "choose_rtk_single_difference_reference_signal",
        "rtk_single_differences_from_obs_epochs",
        "rtk_double_differences_from_single_differences",
        ".range_correction_m(",
    ];

    for relative_path in [
        "src/public_station.rs",
        "src/public_troposphere.rs",
        "src/antenna_validation.rs",
        "src/rtk_baseline.rs",
    ] {
        let source = fixture_source(relative_path);
        for forbidden in forbidden_fragments {
            assert!(
                !source.contains(forbidden),
                "{relative_path} must not depend on nav helper {forbidden}",
            );
        }
    }
}
