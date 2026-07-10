#![allow(missing_docs)]

mod support;

use std::sync::OnceLock;

use bijux_gnss_core::api::ReceiverSampleTrace;
use bijux_gnss_receiver::api::{
    signal::samples_per_code, sim::SyntheticSignalSource, ReceiverPipelineConfig, ReceiverRuntime,
    SignalSource, TrackingArtifacts, TrackingEngine,
};

use support::navigation_truth::{
    multisatellite_pvt_scenario, truth_seeded_acquisition_results, SyntheticPvtScenario,
};

const CONTINUOUS_TRACKING_DURATION_S: f64 = 30.0;
const STREAMING_TRACKING_CODE_PERIODS: usize = 100;
const EXPECTED_CONTINUOUS_TRACKING_EPOCHS: u64 = 30_000;

#[derive(Clone)]
struct MultisatTrackingRun {
    config: ReceiverPipelineConfig,
    profile: SyntheticPvtScenario,
    artifacts: TrackingArtifacts,
}

fn continuity_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn run_multisat_tracking_session() -> MultisatTrackingRun {
    let config = continuity_tracking_config();
    let profile = multisatellite_pvt_scenario(
        &config,
        CONTINUOUS_TRACKING_DURATION_S,
        "multisat-tracking-continuity",
    );
    let mut source = SyntheticSignalSource::new(&config, &profile.scenario);
    let acquisition_frame_len =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let acquisition_frame = source
        .next_frame(acquisition_frame_len)
        .expect("synthetic acquisition frame")
        .expect("non-empty multisatellite source");
    let source_time = ReceiverSampleTrace::from_sample_time(acquisition_frame.t0);
    let acquisitions = truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&acquisitions);
    tracking.track_session_frame(&mut session, &acquisition_frame);
    let streaming_frame_len = acquisition_frame_len * STREAMING_TRACKING_CODE_PERIODS;

    while let Some(frame) =
        source.next_frame(streaming_frame_len).expect("synthetic tracking frame")
    {
        tracking.track_session_frame(&mut session, &frame);
    }

    assert!(source.is_done(), "tracking source did not reach end of stream");
    MultisatTrackingRun { config, profile, artifacts: tracking.finish_tracking_session(session) }
}

fn multisat_tracking_run() -> &'static MultisatTrackingRun {
    static RUN: OnceLock<MultisatTrackingRun> = OnceLock::new();
    RUN.get_or_init(run_multisat_tracking_session)
}

#[test]
fn tracking_session_consumes_full_thirty_second_multisat_span() {
    let run = multisat_tracking_run();
    let expected_samples =
        (CONTINUOUS_TRACKING_DURATION_S * run.config.sampling_freq_hz).round() as u64;

    assert_eq!(run.artifacts.processed_input_samples, expected_samples);
    assert_eq!(run.artifacts.processed_input_epochs, EXPECTED_CONTINUOUS_TRACKING_EPOCHS);
    assert!(
        run.artifacts.tracking.iter().filter(|track| !track.epochs.is_empty()).count() >= 4,
        "expected at least four tracked multisatellite channels",
    );
}
