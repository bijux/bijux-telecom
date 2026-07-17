#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqRequest, Constellation, SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalCode,
};
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

    fn as_any(&self) -> &dyn std::any::Any {
        self
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

    fn as_any(&self) -> &dyn std::any::Any {
        self.inner.as_any()
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
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 500.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        17,
        duration_s,
    );
    let mut source =
        CountingSignalSource::new(SegmentedSignalSource::from_frame(full_frame, 3 * 1_023));

    let artifacts = receiver.run(&mut source).expect("receiver run");

    assert!(source.is_done(), "receiver did not consume the streamed source");
    assert!(source.request_count > 1, "receiver did not stream beyond acquisition");
    assert!(
        source.max_requested_frame_len <= 100 * 1_023,
        "receiver requested an oversized frame: {}",
        source.max_requested_frame_len
    );
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
            .any(|epoch| epoch.lock && epoch.dll_lock),
        "streamed tracking never reached a DLL-backed locked epoch",
    );
    let assumptions = artifacts
        .tracking
        .iter()
        .flat_map(|track| track.epochs.iter())
        .find_map(|epoch| epoch.tracking_assumptions.as_ref())
        .expect("tracking assumptions");
    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
}

#[test]
fn receiver_tracks_galileo_e1_across_stream_frames() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.120,
        seed: 0x6A11_E1F0,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::E1,
            signal_code: bijux_gnss_core::api::SignalCode::E1B,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-galileo-stream".to_string(),
    };
    let runtime = ReceiverRuntime::default();
    let receiver = Receiver::new(config.clone(), runtime);
    let expected_samples = (scenario.duration_s * config.sampling_freq_hz).round() as u64;
    let galileo_samples_per_code = (config.sampling_freq_hz
        * (config.code_length as f64 / config.code_freq_basis_hz))
        .round() as usize;
    let mut source =
        CountingSignalSource::new(SyntheticSignalSource::new_signal_only(&config, &scenario));

    let request = AcqRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1B,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: None,
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    };
    let artifacts =
        receiver.run_with_acquisition_requests(&mut source, &[request]).expect("receiver run");
    let track = artifacts
        .tracking
        .iter()
        .find(|result| result.sat.constellation == Constellation::Galileo && result.sat.prn == 11)
        .expect("Galileo tracking result");

    assert!(source.is_done(), "receiver did not consume the Galileo E1 source");
    assert!(
        source.request_count >= 2,
        "receiver did not request separate acquisition and tracking frames"
    );
    assert!(
        source.max_requested_frame_len <= 100 * galileo_samples_per_code,
        "receiver requested an oversized Galileo tracking frame: {}",
        source.max_requested_frame_len
    );
    assert_eq!(artifacts.processed_input_samples, expected_samples);
    assert!(
        track.epochs.len() >= 20,
        "Galileo tracking did not advance across stream frames: {} epochs",
        track.epochs.len()
    );
    assert!(track.epochs.iter().any(|epoch| epoch.lock), "Galileo prompt never locked");
    let assumptions = track
        .epochs
        .iter()
        .find_map(|epoch| epoch.tracking_assumptions.as_ref())
        .expect("tracking assumptions");
    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
}
