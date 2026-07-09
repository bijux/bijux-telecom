#![allow(missing_docs)]

use bijux_gnss_core::api::{SampleTime, SamplesFrame, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime, SampleSourceError, SignalSource,
};

struct SegmentedSignalSource {
    frames: Vec<SamplesFrame>,
    next_index: usize,
    sample_rate_hz: f64,
}

impl SegmentedSignalSource {
    fn from_frame(frame: SamplesFrame, chunk_len: usize) -> Self {
        let mut frames = Vec::new();
        let mut start = 0usize;
        while start < frame.len() {
            let end = (start + chunk_len.max(1)).min(frame.len());
            frames.push(SamplesFrame::new(
                SampleTime {
                    sample_index: frame.t0.sample_index + start as u64,
                    sample_rate_hz: frame.t0.sample_rate_hz,
                },
                Seconds(frame.dt_s.0),
                frame.iq[start..end].to_vec(),
            ));
            start = end;
        }
        Self { sample_rate_hz: frame.t0.sample_rate_hz, frames, next_index: 0 }
    }
}

impl SignalSource for SegmentedSignalSource {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    fn next_frame(&mut self, _frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        if self.next_index >= self.frames.len() {
            return Ok(None);
        }
        let frame = self.frames[self.next_index].clone();
        self.next_index += 1;
        Ok(Some(frame))
    }

    fn is_done(&self) -> bool {
        self.next_index >= self.frames.len()
    }
}

struct CountingSignalSource<S> {
    inner: S,
    max_requested_frame_len: usize,
    request_count: usize,
}

impl<S> CountingSignalSource<S> {
    fn new(inner: S) -> Self {
        Self { inner, max_requested_frame_len: 0, request_count: 0 }
    }
}

impl<S> SignalSource for CountingSignalSource<S>
where
    S: SignalSource<Error = SampleSourceError>,
{
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.inner.sample_rate_hz()
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        self.max_requested_frame_len = self.max_requested_frame_len.max(frame_len);
        self.request_count += 1;
        self.inner.next_frame(frame_len)
    }

    fn is_done(&self) -> bool {
        self.inner.is_done()
    }
}

#[test]
fn receiver_run_consumes_multiple_stream_frames_after_acquisition() {
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
    let runtime = ReceiverRuntime::default();
    let receiver = Receiver::new(config.clone(), runtime);
    let duration_s = 0.012;
    let full_frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat: bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: 1,
            },
            doppler_hz: 500.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            data_bit_flip: false,
        },
        17,
        duration_s,
    );
    let mut source = SegmentedSignalSource::from_frame(full_frame, 3 * 1_023);

    let artifacts = receiver.run(&mut source).expect("receiver run");

    assert!(source.is_done(), "receiver did not consume the streamed source");
    assert_eq!(artifacts.processed_input_epochs, 12);
    assert_eq!(artifacts.processed_input_samples, 12 * 1_023);
    assert!(
        artifacts.tracking.iter().any(|track| track.epochs.len() > 1),
        "tracking did not advance past the acquisition frame",
    );
    assert!(
        artifacts
            .tracking
            .iter()
            .flat_map(|track| track.epochs.iter())
            .any(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock),
        "streamed tracking never reached a stable locked epoch",
    );
}

#[test]
fn receiver_tracks_sixty_seconds_with_bounded_stream_reads() {
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
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        duration_s: 60.0,
        seed: 41,
        satellites: vec![SyntheticSignalParams {
            sat: bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: 1,
            },
            doppler_hz: 350.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 50.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-long-stream".to_string(),
    };
    let runtime = ReceiverRuntime::default();
    let receiver = Receiver::new(config.clone(), runtime);
    let expected_samples = (scenario.duration_s * config.sampling_freq_hz).round() as u64;
    let mut source = CountingSignalSource::new(SyntheticSignalSource::new(&config, &scenario));

    let artifacts = receiver.run(&mut source).expect("receiver run");

    assert!(source.is_done(), "receiver did not consume the 60-second source");
    assert!(source.request_count > 10, "receiver did not stream in bounded chunks");
    assert!(
        source.max_requested_frame_len <= 100 * 1_023,
        "receiver requested an oversized frame: {}",
        source.max_requested_frame_len
    );
    assert_eq!(artifacts.processed_input_samples, expected_samples);
    assert_eq!(artifacts.processed_input_epochs, 60_000);
    let tracked_epochs =
        artifacts.tracking.iter().map(|track| track.epochs.len() as u64).sum::<u64>();
    assert!(tracked_epochs >= 60_000, "tracked epochs={tracked_epochs}");
    let last_sample_index = artifacts
        .tracking
        .iter()
        .flat_map(|track| track.epochs.iter().map(|epoch| epoch.sample_index))
        .max()
        .expect("tracking epochs");
    assert_eq!(last_sample_index, expected_samples - 1_023);
}
