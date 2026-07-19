#![allow(missing_docs)]

mod support;

use std::sync::OnceLock;

use bijux_gnss_core::api::{Constellation, ReceiverSampleTrace, SatId, SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    signal::samples_per_code,
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    ReceiverPipelineConfig, ReceiverRuntime, SignalSource, TrackingArtifacts, TrackingEngine,
};

use support::navigation_truth::truth_seeded_acquisition_results;

const MULTISAT_STREAMING_DURATION_S: f64 = 0.25;
const STREAMING_TRACKING_CODE_PERIODS: usize = 100;

#[derive(Clone)]
struct MultisatTrackingRun {
    config: ReceiverPipelineConfig,
    artifacts: TrackingArtifacts,
}

fn continuity_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 204_600.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn continuity_tracking_scenario(config: &ReceiverPipelineConfig) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: MULTISAT_STREAMING_DURATION_S,
        seed: 0x24A0_7711,
        satellites: vec![
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: -900.0,
                code_phase_chips: 48.25,
                carrier_phase_rad: 0.10,
                cn0_db_hz: 54.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: -250.0,
                code_phase_chips: 212.5,
                carrier_phase_rad: 0.35,
                cn0_db_hz: 53.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 11 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: 420.0,
                code_phase_chips: 496.75,
                carrier_phase_rad: 0.60,
                cn0_db_hz: 52.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 19 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: 1_050.0,
                code_phase_chips: 768.125,
                carrier_phase_rad: 0.85,
                cn0_db_hz: 55.0,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "multisat-streaming-continuity".to_string(),
    }
}

fn run_multisat_tracking_session() -> MultisatTrackingRun {
    let config = continuity_tracking_config();
    let scenario = continuity_tracking_scenario(&config);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let acquisition_frame_len =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let acquisition_frame = source
        .next_frame(acquisition_frame_len)
        .expect("synthetic acquisition frame")
        .expect("non-empty multisatellite source");
    let source_time = ReceiverSampleTrace::from_sample_time(acquisition_frame.t0);
    let acquisitions = truth_seeded_acquisition_results(&config, source_time, &scenario);
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
    MultisatTrackingRun { config, artifacts: tracking.finish_tracking_session(session) }
}

fn multisat_tracking_run() -> &'static MultisatTrackingRun {
    static RUN: OnceLock<MultisatTrackingRun> = OnceLock::new();
    RUN.get_or_init(run_multisat_tracking_session)
}

#[test]
fn tracking_session_consumes_multisatellite_streamed_signal_span() {
    let run = multisat_tracking_run();
    let samples_per_epoch = samples_per_code(
        run.config.sampling_freq_hz,
        run.config.code_freq_basis_hz,
        run.config.code_length,
    ) as u64;
    let expected_samples =
        (MULTISAT_STREAMING_DURATION_S * run.config.sampling_freq_hz).round() as u64;
    let expected_input_epochs = expected_samples / samples_per_epoch;

    assert_eq!(run.artifacts.processed_input_samples, expected_samples);
    assert_eq!(run.artifacts.processed_input_epochs, expected_input_epochs);
    assert!(
        run.artifacts.tracking.iter().filter(|track| !track.epochs.is_empty()).count() >= 4,
        "expected at least four tracked multisatellite channels",
    );
}
