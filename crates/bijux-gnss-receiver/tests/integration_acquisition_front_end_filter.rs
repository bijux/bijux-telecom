#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    Metric, MetricsSink, Receiver, ReceiverConfig, ReceiverRuntime, ReceiverRuntimeConfig,
    SampleSourceError, SignalSource, TraceRecord, TraceSink,
};
use bijux_gnss_signal::api::FrontEndFilterSpec;
use num_complex::Complex;
use std::sync::{Arc, Mutex};

#[derive(Clone)]
struct SingleFrameSource {
    frame: Option<SamplesFrame>,
}

impl SignalSource for SingleFrameSource {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.frame.as_ref().map(|frame| frame.t0.sample_rate_hz).unwrap_or(4_092_000.0)
    }

    fn next_frame(&mut self, _frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        Ok(self.frame.take())
    }

    fn is_done(&self) -> bool {
        self.frame.is_none()
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[derive(Default)]
struct CapturedMetrics {
    values: Mutex<Vec<(String, f64)>>,
}

impl MetricsSink for CapturedMetrics {
    fn metric(&self, metric: Metric) {
        self.values.lock().expect("metric lock").push((metric.name.to_string(), metric.value));
    }
}

#[derive(Default)]
struct CapturedTrace {
    events: Mutex<Vec<TraceRecord>>,
}

impl TraceSink for CapturedTrace {
    fn record(&self, trace: TraceRecord) {
        self.events.lock().expect("trace lock").push(trace);
    }
}

#[test]
fn receiver_front_end_filter_improves_acquisition_under_out_of_band_interference() {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    profile.acquisition.doppler_search_hz = 1_000;
    profile.acquisition.doppler_step_hz = 250;
    profile.acquisition.integration_ms = 1;
    profile.acquisition.noncoherent_integration = 1;
    profile.acquisition.peak_mean_threshold = 1.5;
    profile.acquisition.peak_second_threshold = 1.1;

    let pipeline = profile.to_pipeline_config();
    let duration_s = 1_023.0 / profile.code_freq_basis_hz;
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let clean = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 35.0,
            navigation_data: false.into(),
        },
        0xF17E_4EAD,
        duration_s,
    );

    let mut interfered = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: clean.t0.sample_rate_hz },
        Seconds(clean.dt_s.0),
        clean.iq.clone(),
    );
    for (index, sample) in interfered.iq.iter_mut().enumerate() {
        let phase =
            2.0 * std::f64::consts::PI * 1_250_000.0 * index as f64 / clean.t0.sample_rate_hz;
        let interferer = Complex::new(phase.cos() as f32, phase.sin() as f32) * 6.0;
        *sample += interferer;
    }

    let mut disabled_source = SingleFrameSource { frame: Some(interfered.clone()) };
    let disabled_artifacts =
        Receiver::new(profile.to_pipeline_config(), ReceiverRuntime::default())
            .run(&mut disabled_source)
            .expect("run without front-end filter");

    let metrics = Arc::new(CapturedMetrics::default());
    let trace = Arc::new(CapturedTrace::default());
    let runtime = ReceiverRuntime::with_sinks(
        ReceiverRuntimeConfig::default(),
        Arc::new(bijux_gnss_receiver::api::NullLogger),
        trace.clone(),
        metrics.clone(),
    );
    let mut enabled_profile = profile;
    enabled_profile.front_end.filter =
        Some(FrontEndFilterSpec::LowPass { cutoff_hz: 850_000.0, taps: 81 });
    let mut enabled_source = SingleFrameSource { frame: Some(interfered) };
    let enabled_artifacts = Receiver::new(enabled_profile.to_pipeline_config(), runtime)
        .run(&mut enabled_source)
        .expect("run with front-end filter");

    let disabled = disabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("unfiltered acquisition for target sat");
    let enabled = enabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("filtered acquisition for target sat");

    assert!(
        enabled.peak_mean_ratio > disabled.peak_mean_ratio,
        "front-end filter did not improve acquisition quality: before={} after={}",
        disabled.peak_mean_ratio,
        enabled.peak_mean_ratio
    );

    let metric_values = metrics.values.lock().expect("metric lock");
    assert!(metric_values.iter().any(|(name, value)| {
        name == "front_end_filter_group_delay_samples" && (*value - 40.0).abs() <= f64::EPSILON
    }));

    let trace_events = trace.events.lock().expect("trace lock");
    assert!(trace_events.iter().any(|event| event.name == "front_end_filter"));
    assert!(trace_events.iter().any(|event| {
        event.name == "front_end_filter_applied"
            && event.fields.iter().any(|(name, value)| *name == "stage" && value == "acquisition")
    }));
}
