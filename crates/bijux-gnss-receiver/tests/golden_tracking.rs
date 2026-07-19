#![allow(missing_docs)]
mod support;

use std::fs;

use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
use support::navigation_truth::truth_seeded_acquisition_results;
use support::tracking_truth::epoch_indices_with_lock_state;

#[test]
fn golden_tracking_from_scenario() {
    let scenario = load_scenario();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };

    let frame = generate_l1_ca_multi(&config, &scenario);
    let source_time = bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0);
    let acq_results = truth_seeded_acquisition_results(&config, source_time, &scenario);
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &acq_results);

    for track in &tracks {
        assert!(!track.epochs.is_empty());
        assert!(
            track.epochs.iter().all(|epoch| epoch.lock),
            "tracking dropped lock for {:?}-{}: epochs={:?}",
            track.sat.constellation,
            track.sat.prn,
            track.epochs
        );
        assert!(
            epoch_indices_with_lock_state(&track.epochs, "lost").is_empty(),
            "tracking entered lost state for {:?}-{}: epochs={:?}",
            track.sat.constellation,
            track.sat.prn,
            track.epochs
        );
        let mean_cn0 = track.epochs.iter().map(|epoch| epoch.cn0_dbhz).sum::<f64>()
            / track.epochs.len() as f64;
        assert!(
            mean_cn0.is_finite() && mean_cn0 >= 20.0,
            "tracking CN0 too low for {:?}-{}: {mean_cn0}",
            track.sat.constellation,
            track.sat.prn
        );
    }
}

fn load_scenario() -> SyntheticScenario {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
