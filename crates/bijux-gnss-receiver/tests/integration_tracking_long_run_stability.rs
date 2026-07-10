#![allow(missing_docs)]

mod support;

use std::sync::OnceLock;

use bijux_gnss_core::api::{Constellation, ReceiverSampleTrace, SatId};
use bijux_gnss_receiver::api::{
    signal::samples_per_code,
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    ReceiverPipelineConfig, ReceiverRuntime, SignalSource, TrackingArtifacts, TrackingEngine,
};

use support::navigation_truth::truth_seeded_acquisition_results;

const LONG_RUN_DURATION_S: f64 = 601.0;
const LONG_RUN_TRACKING_INTEGRATION_MS: u32 = 20;
const STREAMING_TRACKING_EPOCHS_PER_FRAME: usize = 250;

#[derive(Clone)]
struct LongRunTrackingSession {
    config: ReceiverPipelineConfig,
    artifacts: TrackingArtifacts,
}

fn long_run_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 102_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        tracking_integration_ms: LONG_RUN_TRACKING_INTEGRATION_MS,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn long_run_tracking_scenario(config: &ReceiverPipelineConfig) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: LONG_RUN_DURATION_S,
        seed: 0x10A6_5E55,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 12 },
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "tracking-long-run-stability".to_string(),
    }
}

fn tracking_epoch_samples(config: &ReceiverPipelineConfig) -> usize {
    samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
        * config.tracking_integration_ms.max(1) as usize
}

fn run_long_run_tracking_session() -> LongRunTrackingSession {
    let config = long_run_tracking_config();
    let scenario = long_run_tracking_scenario(&config);
    let tracking_epoch_samples = tracking_epoch_samples(&config);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let acquisition_frame = source
        .next_frame(tracking_epoch_samples)
        .expect("synthetic acquisition frame")
        .expect("non-empty long-run source");
    let source_time = ReceiverSampleTrace::from_sample_time(acquisition_frame.t0);
    let acquisitions = truth_seeded_acquisition_results(&config, source_time, &scenario);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let mut session = tracking.begin_tracking_session(&acquisitions);
    tracking.track_session_frame(&mut session, &acquisition_frame);

    let streaming_frame_len = tracking_epoch_samples * STREAMING_TRACKING_EPOCHS_PER_FRAME;
    while let Some(frame) = source.next_frame(streaming_frame_len).expect("synthetic tracking frame")
    {
        tracking.track_session_frame(&mut session, &frame);
    }

    assert!(source.is_done(), "tracking source did not reach end of stream");
    LongRunTrackingSession { config, artifacts: tracking.finish_tracking_session(session) }
}

fn long_run_tracking_session() -> &'static LongRunTrackingSession {
    static RUN: OnceLock<LongRunTrackingSession> = OnceLock::new();
    RUN.get_or_init(run_long_run_tracking_session)
}

#[test]
fn tracking_session_consumes_full_ten_minute_signal_span() {
    let run = long_run_tracking_session();
    let expected_samples = (LONG_RUN_DURATION_S * run.config.sampling_freq_hz).round() as u64;
    let expected_input_epochs = (LONG_RUN_DURATION_S * 1000.0).round() as u64;
    let track = run.artifacts.tracking.first().expect("tracked channel");
    let expected_tracking_epochs =
        ((LONG_RUN_DURATION_S * 1000.0) / LONG_RUN_TRACKING_INTEGRATION_MS as f64).round()
            as usize;

    assert_eq!(run.artifacts.processed_input_samples, expected_samples);
    assert_eq!(run.artifacts.processed_input_epochs, expected_input_epochs);
    assert_eq!(run.artifacts.tracking.len(), 1, "{:?}", run.artifacts.tracking);
    assert_eq!(track.epochs.len(), expected_tracking_epochs, "epochs={:?}", track.epochs);
}
