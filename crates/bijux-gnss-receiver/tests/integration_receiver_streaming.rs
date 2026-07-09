#![allow(missing_docs)]

use bijux_gnss_core::api::{SampleTime, SamplesFrame, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
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
