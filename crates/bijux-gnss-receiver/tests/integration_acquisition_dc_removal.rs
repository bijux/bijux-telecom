#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    Metric, MetricsSink, Receiver, ReceiverConfig, ReceiverRuntime, ReceiverRuntimeConfig,
    SignalSource, TraceRecord, TraceSink,
};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
struct SingleFrameSource {
    frame: Option<SamplesFrame>,
}

#[derive(Default)]
struct CapturedMetrics {
    values: Mutex<Vec<(String, f64)>>,
}

impl MetricsSink for CapturedMetrics {
    fn metric(&self, m: Metric) {
        self.values
            .lock()
            .expect("metric lock")
            .push((m.name.to_string(), m.value));
    }
}

#[derive(Default)]
struct CapturedTrace {
    events: Mutex<Vec<TraceRecord>>,
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

    fn next_frame(
        &mut self,
        _frame_len: usize,
    ) -> Result<Option<SamplesFrame>, Self::Error> {
        Ok(self.frame.take())
    }

    fn is_done(&self) -> bool {
        self.frame.is_none()
    }
}

#[test]
fn receiver_dc_removal_preserves_or_improves_biased_acquisition_margin() {
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
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 35.0,
            data_bit_flip: false,
        },
        0xC0FF_EE11,
        duration_s,
    );

    let mut biased = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: clean.t0.sample_rate_hz },
        Seconds(clean.dt_s.0),
        clean.iq.clone(),
    );
    for sample in &mut biased.iq {
        sample.re += 4.0;
        sample.im -= 2.0;
    }

    let mut disabled_profile = profile.clone();
    disabled_profile.front_end.remove_dc_offset = false;
    let mut disabled_source = SingleFrameSource { frame: Some(biased.clone()) };
    let disabled_artifacts = Receiver::new(disabled_profile.to_pipeline_config(), ReceiverRuntime::default())
        .run(&mut disabled_source)
        .expect("run without dc removal");

    let mut enabled_profile = profile;
    enabled_profile.front_end.remove_dc_offset = true;
    let mut enabled_source = SingleFrameSource { frame: Some(biased) };
    let enabled_artifacts = Receiver::new(enabled_profile.to_pipeline_config(), ReceiverRuntime::default())
        .run(&mut enabled_source)
        .expect("run with dc removal");

    let disabled = disabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("uncorrected acquisition for target sat");
    let enabled = enabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("corrected acquisition for target sat");

    assert!(
        enabled.peak_mean_ratio + f32::EPSILON >= disabled.peak_mean_ratio,
        "dc removal reduced peak quality: before={} after={}",
        disabled.peak_mean_ratio,
        enabled.peak_mean_ratio
    );
}

#[test]
fn receiver_dc_removal_reports_power_imbalance_diagnostics() {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    profile.acquisition.doppler_search_hz = 1_000;
    profile.acquisition.doppler_step_hz = 250;
    profile.acquisition.integration_ms = 1;
    profile.acquisition.noncoherent_integration = 1;
    profile.acquisition.peak_mean_threshold = 1.5;
    profile.acquisition.peak_second_threshold = 1.1;
    profile.front_end.remove_dc_offset = true;

    let pipeline = profile.to_pipeline_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let clean = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat,
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 35.0,
            data_bit_flip: false,
        },
        0xC0FF_EE11,
        1_023.0 / profile.code_freq_basis_hz,
    );

    let mut biased = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: clean.t0.sample_rate_hz },
        Seconds(clean.dt_s.0),
        clean.iq.clone(),
    );
    for sample in &mut biased.iq {
        sample.re += 4.0;
        sample.im -= 2.0;
    }

    let metrics = Arc::new(CapturedMetrics::default());
    let trace = Arc::new(CapturedTrace::default());
    let runtime = ReceiverRuntime::with_sinks(
        ReceiverRuntimeConfig::default(),
        Arc::new(bijux_gnss_receiver::api::NullLogger),
        trace.clone(),
        metrics.clone(),
    );

    let mut source = SingleFrameSource { frame: Some(biased) };
    Receiver::new(profile.to_pipeline_config(), runtime)
        .run(&mut source)
        .expect("run with diagnostics");

    let values = metrics.values.lock().expect("metric lock");
    assert!(
        values.iter().any(|(name, value)| {
            name == "front_end_i_power_before_removal" && *value > 0.0
        }),
        "missing I power metric"
    );
    assert!(
        values.iter().any(|(name, value)| {
            name == "front_end_q_power_before_removal" && *value > 0.0
        }),
        "missing Q power metric"
    );
    assert!(
        values.iter().any(|(name, value)| {
            name == "front_end_iq_power_ratio_before_removal" && *value > 1.0
        }),
        "missing I/Q power ratio metric"
    );
    assert!(
        values.iter().any(|(name, value)| {
            name == "front_end_power_imbalance_warning_before_removal" && *value == 1.0
        }),
        "missing power imbalance warning metric"
    );

    let events = trace.events.lock().expect("trace lock");
    let event = events
        .iter()
        .find(|event| event.name == "front_end_dc_removal")
        .expect("dc removal trace");
    assert!(
        event.fields.iter().any(|(name, value)| *name == "iq_power_ratio" && !value.is_empty())
    );
    assert!(
        event.fields.iter().any(|(name, value)| {
            *name == "power_imbalance_warning" && value == "true"
        })
    );
}
