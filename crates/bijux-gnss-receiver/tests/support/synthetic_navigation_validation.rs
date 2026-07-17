#![allow(dead_code, missing_docs)]

use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_receiver::api::{sim::SyntheticNavigationValidationScenario, ReceiverConfig};

pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

pub fn bounded_navigation_validation_fixture() -> (ReceiverConfig, SyntheticNavigationValidationScenario) {
    let repo = repo_root();
    let mut config: ReceiverConfig = toml::from_str(
        &fs::read_to_string(repo.join("configs/receiver_low_rate.toml"))
            .expect("read receiver config"),
    )
    .expect("parse receiver config");
    let mut scenario: SyntheticNavigationValidationScenario = toml::from_str(
        &fs::read_to_string(repo.join("configs/scenarios/synthetic_navigation_accuracy.toml"))
            .expect("read synthetic navigation scenario"),
    )
    .expect("parse synthetic navigation scenario");

    scenario.duration_s = 0.08;

    config.sample_rate_hz = scenario.sample_rate_hz;
    config.intermediate_freq_hz = scenario.intermediate_freq_hz;
    config.code_freq_basis_hz = 1_023_000.0;
    config.code_length = 1023;
    config.acquisition.doppler_search_hz = 1_500;
    config.acquisition.doppler_step_hz = 250;
    config.tracking.max_channels = scenario.satellites.len();
    config.tracking.per_epoch_budget_ms = 100.0;
    config.navigation.hatch_window = 20;

    (config, scenario)
}
