#![allow(missing_docs)]

use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_receiver::api::{
    sim::{validate_synthetic_navigation_run, SyntheticNavigationValidationScenario},
    ReceiverConfig,
};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

#[test]
fn synthetic_navigation_validation_run_builds_run_level_accuracy_artifact() {
    let repo = repo_root();
    let config: ReceiverConfig = toml::from_str(
        &fs::read_to_string(repo.join("configs/receiver_low_rate.toml"))
            .expect("read receiver config"),
    )
    .expect("parse receiver config");
    let scenario: SyntheticNavigationValidationScenario = toml::from_str(
        &fs::read_to_string(repo.join("configs/scenarios/synthetic_navigation_accuracy.toml"))
            .expect("read synthetic navigation scenario"),
    )
    .expect("parse synthetic navigation scenario");

    let run = validate_synthetic_navigation_run(
        &config.to_pipeline_config(),
        &scenario,
        config.navigation.hatch_window,
    )
    .expect("build synthetic navigation validation run");

    assert_eq!(run.artifact.scenario_id, "synthetic_navigation_accuracy");
    assert_eq!(run.artifact.data_source.satellite_count, scenario.satellites.len());
    assert_eq!(run.artifact.reference_truth.satellite_count, scenario.ephemerides.len());
    assert_eq!(run.acquisition_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.tracking_accuracy.satellite_count, scenario.satellites.len());
    assert_eq!(run.observation_accuracy.satellite_count, scenario.satellites.len());
    assert!(run.pvt_accuracy.epoch_count > 0);
}
