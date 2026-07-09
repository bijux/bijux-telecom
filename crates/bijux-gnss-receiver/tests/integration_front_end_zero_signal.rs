#![allow(missing_docs)]

use bijux_gnss_core::api::{AcqHypothesis, SampleTime, SamplesFrame, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario},
    Metric, MetricsSink, Receiver, ReceiverConfig, ReceiverRuntime, ReceiverRuntimeConfig,
    SignalSource, TraceRecord, TraceSink,
};
use bijux_gnss_signal::api::samples_per_code;
use num_complex::Complex;
use std::sync::{Arc, Mutex};

#[derive(Clone)]
struct SingleFrameSource {
    frame: Option<SamplesFrame>,
}

#[derive(Default)]
struct CapturedTrace {
    events: Mutex<Vec<TraceRecord>>,
}

#[derive(Default)]
struct NoopMetrics;

impl MetricsSink for NoopMetrics {
    fn metric(&self, _m: Metric) {}
}

impl TraceSink for CapturedTrace {
    fn record(&self, t: TraceRecord) {
        self.events.lock().expect("trace lock").push(t);
    }
}

impl SignalSource for SingleFrameSource {
    type Error = bijux_gnss_receiver::api::SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.frame
            .as_ref()
            .map(|frame| frame.t0.sample_rate_hz)
            .unwrap_or(4_092_000.0)
    }

    fn next_frame(&mut self, _frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        Ok(self.frame.take())
    }

    fn is_done(&self) -> bool {
        self.frame.is_none()
    }
}

fn zero_signal_frame(profile: &ReceiverConfig) -> SamplesFrame {
    let config = profile.to_pipeline_config();
    let sample_count = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        vec![Complex::new(0.0, 0.0); sample_count],
    )
}

fn noise_only_frame(profile: &ReceiverConfig, seed: u64) -> SamplesFrame {
    let config = profile.to_pipeline_config();
    generate_l1_ca_multi(
        &config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s: config.code_length as f64 / config.code_freq_basis_hz,
            seed,
            satellites: Vec::new(),
            ephemerides: Vec::new(),
            id: "noise_only".to_string(),
        },
    )
}

#[test]
fn receiver_rejects_zero_signal_window_before_search() {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    profile.acquisition.doppler_search_hz = 1_500;
    profile.acquisition.doppler_step_hz = 250;

    let trace = Arc::new(CapturedTrace::default());
    let runtime = ReceiverRuntime::with_sinks(
        ReceiverRuntimeConfig::default(),
        Arc::new(bijux_gnss_receiver::api::NullLogger),
        trace.clone(),
        Arc::new(NoopMetrics),
    );

    let mut source = SingleFrameSource { frame: Some(zero_signal_frame(&profile)) };
    let artifacts = Receiver::new(profile.to_pipeline_config(), runtime)
        .run(&mut source)
        .expect("run zero-signal receiver pipeline");

    assert_eq!(artifacts.acquisitions.len(), 32);
    assert!(artifacts
        .acquisitions
        .iter()
        .all(|result| result.hypothesis.to_string() == AcqHypothesis::Rejected.to_string()));
    assert!(artifacts
        .acquisitions
        .iter()
        .all(|result| {
            result
                .explain_selection_reason
                .as_deref()
                .expect("zero-signal explain reason")
                .contains("zero_signal_input")
        }));
    assert_eq!(artifacts.acquisition_explain.len(), 32);
    assert!(artifacts.tracking.iter().all(|result| result.epochs.is_empty()));
    assert!(artifacts.observations.is_empty());

    let events = trace.events.lock().expect("trace lock");
    let event = events
        .iter()
        .find(|event| event.name == "acquisition_front_end_rejection")
        .expect("front-end rejection trace");
    assert!(event.fields.iter().any(|(name, value)| *name == "reason" && value == "zero_signal_input"));
    assert!(event.fields.iter().any(|(name, value)| *name == "centered_rms" && value == "0.000000000"));
}

#[test]
fn deterministic_noise_only_input_never_produces_accepted_satellites() {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    profile.acquisition.doppler_search_hz = 1_500;
    profile.acquisition.doppler_step_hz = 250;

    let mut source = SingleFrameSource { frame: Some(noise_only_frame(&profile, 0x5EED_CAFE)) };
    let artifacts = Receiver::new(profile.to_pipeline_config(), ReceiverRuntime::default())
        .run(&mut source)
        .expect("run noise-only receiver pipeline");

    assert_eq!(artifacts.acquisitions.len(), 32);
    assert!(artifacts
        .acquisitions
        .iter()
        .all(|result| result.hypothesis.to_string() != AcqHypothesis::Accepted.to_string()));
}
