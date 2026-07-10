#![allow(missing_docs)]

mod support;

use bijux_gnss_receiver::api::{
    sim::SyntheticSignalSource, Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

use support::navigation_truth::four_satellite_pvt_scenario;

#[test]
fn receiver_tracks_four_satellites_with_persisted_channel_history() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let profile = four_satellite_pvt_scenario(&config);
    let receiver = Receiver::new(config.clone(), ReceiverRuntime::default());
    let mut source = SyntheticSignalSource::new(&config, &profile.scenario);

    let artifacts = receiver.run(&mut source).expect("receiver run");

    assert_eq!(artifacts.tracking.len(), 4, "receiver must track four channels");
    assert!(
        artifacts.tracking.iter().all(|track| track.epochs.len() > profile.target_epoch_idx as usize),
        "each tracked channel must persist through the shared PVT-ready epoch",
    );
    assert!(
        artifacts
            .tracking
            .iter()
            .all(|track| track.epochs.iter().any(|epoch| epoch.lock && epoch.dll_lock)),
        "each tracked channel must emit a locked tracking epoch",
    );
}
