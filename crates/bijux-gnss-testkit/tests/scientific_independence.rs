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
        "sim::expected_acquisition_code_phase_samples",
        "sim::expected_acquisition_code_phase_samples_f64",
    ];

    for relative_path in [
        "src/signal/acquisition.rs",
        "src/position_truth/mod.rs",
        "src/position_truth/observation_synthesis.rs",
        "src/position_truth/residual_model.rs",
        "src/position_truth/scenario_catalog.rs",
        "src/reference_data/station_truth.rs",
        "src/reference_data/troposphere_elevation.rs",
        "src/antenna/effects.rs",
        "src/antenna/synthesis.rs",
        "src/reference_data/rtk_baseline/scenario.rs",
        "src/signal/synthesis.rs",
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
