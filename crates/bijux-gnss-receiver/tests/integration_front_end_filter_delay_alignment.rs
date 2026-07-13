#![allow(missing_docs)]

use std::fs;
use std::path::{Path, PathBuf};

use bijux_gnss_receiver::api::{
    signal::FrontEndFilterSpec,
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
fn synthetic_navigation_validation_run_carries_band_pass_source_and_receiver_delay() {
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
    let source_front_end_filter = FrontEndFilterSpec::BandPass {
        center_hz: 400_000.0,
        bandwidth_hz: 1_000_000.0,
        taps: 19,
    };
    let receiver_front_end_filter = FrontEndFilterSpec::BandPass {
        center_hz: 300_000.0,
        bandwidth_hz: 800_000.0,
        taps: 23,
    };
    let combined_sample_delay_samples = source_front_end_filter.group_delay_samples() as u64
        + receiver_front_end_filter.group_delay_samples() as u64;

    config.front_end.filter = Some(receiver_front_end_filter.clone());
    scenario.source_front_end_filter = Some(source_front_end_filter.clone());

    let run = validate_synthetic_navigation_run(
        &config.to_pipeline_config(),
        &scenario,
        config.navigation.hatch_window,
    )
    .expect("build synthetic navigation validation run with band-pass front-end filters");

    assert_eq!(run.truth_bundle.source_front_end_filter, Some(source_front_end_filter.clone()));
    assert_eq!(
        run.truth_bundle.source_front_end_sample_delay_samples,
        source_front_end_filter.group_delay_samples() as u64
    );
    assert!(!run.pipeline_artifacts.acquisitions.is_empty());
    for acquisition in &run.pipeline_artifacts.acquisitions {
        let alignment = acquisition
            .signal_delay_alignment
            .as_ref()
            .expect("synthetic alignment on acquisition");
        assert_eq!(alignment.sample_delay_samples, combined_sample_delay_samples);
    }
}
