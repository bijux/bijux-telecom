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
use support::tracking_truth::{
    carrier_frequency_error_hz, carrier_phase_steps_cycles, code_phase_error_samples,
    stable_tracking_window,
};

const LONG_RUN_DURATION_S: f64 = 601.0;
const LONG_RUN_TRACKING_INTEGRATION_MS: u32 = 20;
const REQUIRED_STABLE_TRACKING_EPOCHS: usize = 30_000;
const STREAMING_TRACKING_EPOCHS_PER_FRAME: usize = 250;
const LONG_RUN_LOCKED_CARRIER_ERROR_MAX_HZ: f64 = 5.0;
const LONG_RUN_LOCKED_CODE_ERROR_MAX_SAMPLES: f64 = 1.0;
const LONG_RUN_PHASE_STEP_ERROR_MAX_CYCLES: f64 = 0.2;
const LONG_RUN_CN0_SETTLING_EPOCHS: usize = 8;
const LONG_RUN_MIN_MEAN_CN0_DBHZ: f64 = 45.0;
const LONG_RUN_CN0_SPAN_MAX_DBHZ: f64 = 4.0;
const LONG_RUN_CN0_DRIFT_MAX_DBHZ: f64 = 1.5;

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
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
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
    while let Some(frame) =
        source.next_frame(streaming_frame_len).expect("synthetic tracking frame")
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

fn mean(values: &[f64]) -> f64 {
    values.iter().sum::<f64>() / values.len() as f64
}

#[test]
fn tracking_session_consumes_full_ten_minute_signal_span() {
    let run = long_run_tracking_session();
    let expected_samples = (LONG_RUN_DURATION_S * run.config.sampling_freq_hz).round() as u64;
    let expected_input_epochs = (LONG_RUN_DURATION_S * 1000.0).round() as u64;
    let track = run.artifacts.tracking.first().expect("tracked channel");
    let expected_tracking_epochs =
        ((LONG_RUN_DURATION_S * 1000.0) / LONG_RUN_TRACKING_INTEGRATION_MS as f64).round() as usize;

    assert_eq!(run.artifacts.processed_input_samples, expected_samples);
    assert_eq!(run.artifacts.processed_input_epochs, expected_input_epochs);
    assert_eq!(run.artifacts.tracking.len(), 1, "{:?}", run.artifacts.tracking);
    assert_eq!(track.epochs.len(), expected_tracking_epochs, "epochs={:?}", track.epochs);
}

#[test]
fn tracking_session_maintains_stable_long_run_lock_metrics() {
    let run = long_run_tracking_session();
    let scenario = long_run_tracking_scenario(&run.config);
    let signal = scenario.satellites.first().expect("long-run signal");
    let track = run.artifacts.tracking.first().expect("tracked channel");
    let stable_window = stable_tracking_window(&track.epochs, REQUIRED_STABLE_TRACKING_EPOCHS);
    let expected_code_phase_samples =
        signal.code_phase_chips * run.config.sampling_freq_hz / run.config.code_freq_basis_hz;
    let expected_phase_step_cycles =
        signal.doppler_hz * LONG_RUN_TRACKING_INTEGRATION_MS as f64 / 1000.0;
    let carrier_errors_hz = stable_window
        .iter()
        .map(|epoch| carrier_frequency_error_hz(epoch, signal.doppler_hz))
        .collect::<Vec<_>>();
    let code_errors_samples = stable_window
        .iter()
        .map(|epoch| code_phase_error_samples(&run.config, epoch, expected_code_phase_samples))
        .collect::<Vec<_>>();
    let phase_step_errors_cycles = carrier_phase_steps_cycles(stable_window)
        .into_iter()
        .map(|step_cycles| (step_cycles - expected_phase_step_cycles).abs())
        .collect::<Vec<_>>();
    let cn0_values_dbhz = stable_window
        .iter()
        .skip(LONG_RUN_CN0_SETTLING_EPOCHS)
        .map(|epoch| epoch.cn0_dbhz)
        .collect::<Vec<_>>();
    let cn0_head = &cn0_values_dbhz[..500];
    let cn0_tail = &cn0_values_dbhz[cn0_values_dbhz.len() - 500..];
    let mean_cn0_dbhz = mean(&cn0_values_dbhz);
    let cn0_span_dbhz = cn0_values_dbhz.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - cn0_values_dbhz.iter().copied().fold(f64::INFINITY, f64::min);
    let cn0_drift_dbhz = (mean(cn0_head) - mean(cn0_tail)).abs();

    assert!(
        stable_window.len() >= REQUIRED_STABLE_TRACKING_EPOCHS,
        "stable_window_len={} total_epochs={}",
        stable_window.len(),
        track.epochs.len(),
    );
    assert!(
        carrier_errors_hz.iter().all(|error_hz| *error_hz <= LONG_RUN_LOCKED_CARRIER_ERROR_MAX_HZ),
        "carrier_errors_hz={carrier_errors_hz:?}"
    );
    assert!(
        code_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= LONG_RUN_LOCKED_CODE_ERROR_MAX_SAMPLES),
        "code_errors_samples={code_errors_samples:?}"
    );
    assert!(
        phase_step_errors_cycles.iter().all(|error_cycles| error_cycles.is_finite()
            && *error_cycles <= LONG_RUN_PHASE_STEP_ERROR_MAX_CYCLES),
        "phase_step_errors_cycles={phase_step_errors_cycles:?}"
    );
    assert!(
        cn0_values_dbhz.iter().all(|cn0_dbhz| cn0_dbhz.is_finite()),
        "cn0_values_dbhz contained non-finite values"
    );
    assert!(mean_cn0_dbhz >= LONG_RUN_MIN_MEAN_CN0_DBHZ, "mean_cn0_dbhz={mean_cn0_dbhz}");
    assert!(cn0_span_dbhz <= LONG_RUN_CN0_SPAN_MAX_DBHZ, "cn0_span_dbhz={cn0_span_dbhz}");
    assert!(
        cn0_drift_dbhz <= LONG_RUN_CN0_DRIFT_MAX_DBHZ,
        "cn0_drift_dbhz={cn0_drift_dbhz} mean_head={} mean_tail={}",
        mean(cn0_head),
        mean(cn0_tail)
    );
}
