#![allow(missing_docs)]

use num_complex::Complex;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqTrackingSeed, AcqUncertainty, Chips, Constellation, Cycles, Hertz,
    ReceiverSampleTrace, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
    TrackEpoch, TrackTransition, TrackingAssumptions,
};

use crate::engine::receiver_config::{ReceiverPipelineConfig, TrackingParams};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_core::api::Sample;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::{
    adaptive_bandwidth, carrier_frequency_error_hz_from_phase_delta, code_value_at_phase,
    discriminators, estimate_cn0_dbhz, first_order_angular_loop_coefficients,
    first_order_loop_coefficients, phase_lock_loop_coefficients, wipeoff_carrier,
};
use bijux_gnss_signal::api::{generate_ca_code, Prn};

const DLL_CODE_PHASE_CORRECTION_GAIN: f64 = 0.5;
const SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ: f64 = 18.0;
const TRACKING_LOCK_MIN_CN0_DBHZ: f64 = 28.0;
const TRACKING_LOCK_REFUSAL_EPOCHS: u8 = 3;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_FRACTION: f64 = 0.0025;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_SAMPLES: f64 = 12.0;
const SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS: usize = 4;
const SAMPLE_RATE_MISMATCH_MIN_UNSTABLE_EPOCHS_IN_WINDOW: usize = 3;
const CYCLE_SLIP_PHASE_DELTA_CYCLES: f64 = 0.35;
const NAV_BIT_PHASE_STEP_CYCLES: f64 = 0.5;
const NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES: f64 = 0.1;
const PLL_LOCK_MAX_PHASE_ERROR_RAD: f32 = 0.2;
const PULL_IN_REQUIRED_STABLE_EPOCHS: u8 = 1;
const FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER: f64 = 4.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    Idle,
    Acquired,
    PullIn,
    Tracking,
    Lost,
}

impl std::fmt::Display for ChannelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Idle => "idle",
            Self::Acquired => "acquired",
            Self::PullIn => "pull_in",
            Self::Tracking => "tracking",
            Self::Lost => "lost",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelEvent {
    Acquire,
    PullIn,
    Track,
    Lose,
    Reset,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub id: u8,
    pub state: ChannelState,
}

impl Channel {
    pub fn new(id: u8) -> Self {
        Self { id, state: ChannelState::Idle }
    }

    pub fn apply(&mut self, event: ChannelEvent) {
        let next = match (self.state, event) {
            (ChannelState::Idle, ChannelEvent::Acquire) => ChannelState::Acquired,
            (ChannelState::Acquired, ChannelEvent::PullIn) => ChannelState::PullIn,
            (ChannelState::PullIn, ChannelEvent::Track) => ChannelState::Tracking,
            (_, ChannelEvent::Lose) => ChannelState::Lost,
            (_, ChannelEvent::Reset) => ChannelState::Idle,
            (state, _) => state,
        };
        self.state = Tracking::transition_state(self.id, self.state, next);
    }
}

#[derive(Debug, Clone)]
pub struct CorrelatorOutput {
    pub early: Complex<f32>,
    pub prompt: Complex<f32>,
    pub late: Complex<f32>,
    pub early_late_noise_weight_energy: f64,
}

#[derive(Debug, Clone)]
pub struct TrackingResult {
    pub sat: SatId,
    pub carrier_hz: f64,
    pub code_phase_samples: f64,
    pub acquisition_hypothesis: String,
    pub acquisition_score: f32,
    pub acquisition_code_phase_samples: usize,
    pub acquisition_carrier_hz: f64,
    pub acq_to_track_state: String,
    pub epochs: Vec<TrackEpoch>,
    pub transitions: Vec<TrackTransition>,
}

#[derive(Debug, Clone)]
struct LoopState {
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    acquisition_cn0_proxy_dbhz: f64,
    prev_prompt: Option<Complex<f32>>,
    prev_prompt_phase_cycles: Option<f64>,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition_count: u32,
    pull_in_stable_epochs: u8,
    weak_cn0_epochs: u8,
    state: ChannelState,
    unlocked_count: u8,
}

#[derive(Debug, Clone)]
pub(crate) struct IncrementalTrackingState {
    channels: Vec<IncrementalTrackingChannel>,
}

#[derive(Debug, Clone)]
struct TrackingStartContext {
    seed: AcqTrackingSeed,
    acquisition_hypothesis: String,
    acquisition_score: f32,
    acquisition_code_phase_samples: usize,
    acquisition_carrier_hz: f64,
    acquisition_cn0_proxy_dbhz: f64,
    acq_to_track_state: String,
}

#[derive(Debug, Clone)]
struct IncrementalTrackingChannel {
    sat: SatId,
    channel_id: u8,
    start_source_time: ReceiverSampleTrace,
    signal_band: SignalBand,
    acquisition_uncertainty: Option<AcqUncertainty>,
    acquisition_hypothesis: String,
    acquisition_score: f32,
    acquisition_code_phase_samples: usize,
    acquisition_doppler_hz: f64,
    acquisition_resolved_code_phase_samples: f64,
    acquisition_carrier_hz: f64,
    acq_to_track_state: String,
    tracking_params: TrackingParams,
    state: LoopState,
    epochs: Vec<TrackEpoch>,
    transitions: Vec<TrackTransition>,
}

#[derive(Debug, Clone, Copy)]
struct CodePhaseStabilityDiagnostic {
    first_unstable_epoch_index: usize,
    max_abs_phase_step_samples: f64,
    phase_step_limit_samples: f64,
}

#[derive(Debug, Clone, Copy)]
struct CodeLoopUpdate {
    code_rate_hz: f64,
    code_phase_samples: f64,
}

#[derive(Debug, Clone, Copy)]
struct CarrierLoopUpdate {
    carrier_hz: f64,
    carrier_phase_cycles: f64,
}

#[derive(Debug, Clone, Copy)]
struct CodeLoopInput {
    current_code_rate_hz: f64,
    current_code_phase_samples: f64,
    epoch_len_samples: usize,
    coherent_integration_s: f64,
    nominal_code_rate_hz: f64,
    dll_bw_hz: f64,
    dll_err: f32,
    samples_per_chip: f64,
    samples_per_code: usize,
}

#[derive(Debug, Clone, Copy)]
struct CarrierLoopInput {
    current_carrier_hz: f64,
    current_carrier_phase_cycles: f64,
    epoch_len_samples: usize,
    sample_rate_hz: f64,
    coherent_integration_s: f64,
    pll_bw_hz: f64,
    pll_err_rad: f64,
    fll_bw_hz: f64,
    fll_err_hz: f64,
    apply_fll: bool,
}

#[derive(Debug, Clone, Copy)]
struct PromptPhaseDecision {
    aligned_phase_cycles: f64,
    aligned_phase_delta_cycles: f64,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition: bool,
    cycle_slip: bool,
}

fn should_apply_fll(state: ChannelState, raw_fll_lock: bool) -> bool {
    state == ChannelState::PullIn || !raw_fll_lock
}

/// Tracking engine with basic E/P/L correlation per epoch.
pub struct Tracking {
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
}

impl Tracking {
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        Self { config, runtime }
    }

    pub fn transition_state(channel: u8, from: ChannelState, to: ChannelState) -> ChannelState {
        let _ = (channel, from, to);
        to
    }

    fn tracking_start_context(
        &self,
        acquisition: &bijux_gnss_core::api::AcqResult,
    ) -> Option<TrackingStartContext> {
        if !matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous) {
            return None;
        }
        Some(TrackingStartContext {
            seed: acquisition.tracking_seed(),
            acquisition_hypothesis: acquisition.hypothesis.to_string(),
            acquisition_score: acquisition.score,
            acquisition_code_phase_samples: acquisition.code_phase_samples,
            acquisition_carrier_hz: carrier_hz_from_doppler_hz(
                self.config.intermediate_freq_hz,
                acquisition.doppler_hz.0,
            ),
            acquisition_cn0_proxy_dbhz: acquisition.cn0_proxy as f64,
            acq_to_track_state: acq_to_track_state(&acquisition.hypothesis).to_string(),
        })
    }

    pub fn correlate_epoch(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> CorrelatorOutput {
        let sample_rate_hz = self.config.sampling_freq_hz;
        let n = frame.len();
        let code = ca_code_or_default(sat.prn);
        let nominal_chips_per_sample = self.config.code_freq_basis_hz / sample_rate_hz;
        let tracked_chips_per_sample = code_rate_hz / sample_rate_hz;
        let base_chip_phase = code_phase_samples * nominal_chips_per_sample;
        let carrier_phase_offset_rad = carrier_phase_offset_rad(
            carrier_freq_hz,
            sample_rate_hz,
            frame.t0.sample_index,
            carrier_phase_cycles,
        );

        let mixed = wipeoff_carrier(
            &frame.iq[..n],
            carrier_freq_hz,
            sample_rate_hz,
            frame.t0.sample_index,
            carrier_phase_offset_rad,
        )
        .expect("tracking carrier wipeoff requires finite carrier inputs");
        let mut early = Complex::new(0.0f32, 0.0f32);
        let mut prompt = Complex::new(0.0f32, 0.0f32);
        let mut late = Complex::new(0.0f32, 0.0f32);
        let mut early_late_noise_weight_energy = 0.0f64;

        for (i, mixed_sample) in mixed.iter().take(n).enumerate() {
            let chip_phase = base_chip_phase + i as f64 * tracked_chips_per_sample;
            let early_code = Complex::new(
                code_value_at_phase(&code, chip_phase - early_late_spacing_chips).unwrap_or(0.0),
                0.0,
            );
            let prompt_code =
                Complex::new(code_value_at_phase(&code, chip_phase).unwrap_or(0.0), 0.0);
            let late_code = Complex::new(
                code_value_at_phase(&code, chip_phase + early_late_spacing_chips).unwrap_or(0.0),
                0.0,
            );
            let noise_weight = early_code - late_code;
            early_late_noise_weight_energy += noise_weight.norm_sqr() as f64;

            early += *mixed_sample * early_code;
            prompt += *mixed_sample * prompt_code;
            late += *mixed_sample * late_code;
        }

        CorrelatorOutput { early, prompt, late, early_late_noise_weight_energy }
    }

    pub fn track_epoch(
        &self,
        frame: &SamplesFrame,
        channel_id: u8,
        sat: SatId,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (TrackEpoch, CorrelatorOutput) {
        let clock = SampleClock::new(self.config.sampling_freq_hz);
        let epoch = clock.epoch_from_samples(frame.t0.sample_index);
        let correlator = self.correlate_epoch(
            frame,
            sat,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        );
        let coherent_samples = frame.len();
        let cn0_dbhz = estimate_cn0_dbhz(
            correlator.prompt,
            correlator.early - correlator.late,
            self.config.sampling_freq_hz,
            coherent_samples as f64,
            correlator.early_late_noise_weight_energy,
        );
        if !correlator.prompt.re.is_finite() || !correlator.prompt.im.is_finite() {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "TRACK_NUMERIC_INVALID",
                "tracking correlator produced NaN/Inf",
            ));
        }
        let track_epoch = TrackEpoch {
            epoch,
            sample_index: frame.t0.sample_index,
            source_time: ReceiverSampleTrace::from_sample_time(frame.t0),
            sat,
            prompt_i: correlator.prompt.re,
            prompt_q: correlator.prompt.im,
            early_i: correlator.early.re,
            early_q: correlator.early.im,
            late_i: correlator.late.re,
            late_q: correlator.late.im,
            carrier_hz: Hertz(carrier_freq_hz),
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            code_rate_hz: Hertz(code_rate_hz),
            code_phase_samples: Chips(code_phase_samples),
            lock: correlator.prompt.norm() > 0.0,
            cn0_dbhz,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: ChannelState::Idle.to_string(),
            lock_state_reason: Some("initializing".to_string()),
            channel_id: Some(channel_id),
            channel_uid: format!("{:?}-{:02}-ch{:02}", sat.constellation, sat.prn, channel_id),
            tracking_provenance: format!(
                "channel={} sat={:?}-{}",
                channel_id, sat.constellation, sat.prn
            ),
            tracking_assumptions: Some(default_tracking_assumptions(&self.config)),
            processing_ms: None,
        };
        (track_epoch, correlator)
    }

    pub fn run(&self, samples: &[Sample]) -> Vec<TrackingResult> {
        if samples.is_empty() {
            return Vec::new();
        }
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: self.config.sampling_freq_hz },
            Seconds(1.0 / self.config.sampling_freq_hz),
            samples.to_vec(),
        );
        let epochs = self.track_epochs(
            &frame,
            0,
            SatId { constellation: Constellation::Gps, prn: 1 },
            0.0,
            0.0,
            f64::INFINITY,
            self.config.tracking_params(SignalBand::L1),
            5,
        );
        vec![TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: AcqHypothesis::Deferred.to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: epochs.0,
            transitions: epochs.1,
        }]
    }

    pub fn track_from_acquisition(
        &self,
        frame: &SamplesFrame,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> Vec<TrackingResult> {
        let mut incremental_tracking = self.begin_incremental_tracking(acquisitions);
        self.track_incremental_frame(&mut incremental_tracking, frame);
        self.reacquire_incremental_tracking(&mut incremental_tracking, frame);
        self.finish_incremental_tracking(incremental_tracking)
    }

    fn reacquire_incremental_tracking(
        &self,
        tracking: &mut IncrementalTrackingState,
        frame: &SamplesFrame,
    ) {
        for channel in &mut tracking.channels {
            let mut consecutive_lock_loss = 0usize;
            let mut reacquire_seed = None;
            for epoch in &channel.epochs {
                if epoch.lock {
                    consecutive_lock_loss = 0;
                    continue;
                }
                consecutive_lock_loss += 1;
                if consecutive_lock_loss >= 3 {
                    reacquire_seed = Some((epoch.carrier_hz.0, epoch.code_phase_samples.0));
                    break;
                }
            }
            let Some((carrier_hz, code_phase_samples)) = reacquire_seed else {
                continue;
            };
            let Some(channel_frame) = tracking_frame_from_start_sample(
                frame,
                Some(channel.start_source_time.sample_index),
            ) else {
                continue;
            };
            let Some((carrier_hz, code_phase_samples)) = self.quick_reacquire(
                &channel_frame,
                channel.sat,
                carrier_hz,
                code_phase_samples,
                channel.acquisition_uncertainty.as_ref(),
            ) else {
                continue;
            };
            let (epochs, transitions) = self.track_epochs(
                &channel_frame,
                channel.channel_id,
                channel.sat,
                carrier_hz,
                code_phase_samples,
                channel.state.acquisition_cn0_proxy_dbhz,
                channel.tracking_params,
                self.epochs_in_frame(&channel_frame, channel.tracking_params),
            );
            channel.epochs = epochs;
            channel.transitions.extend(transitions);
        }
    }

    fn apply_sample_rate_mismatch_diagnostic(
        &self,
        sat: SatId,
        acquisition_uncertainty: Option<&AcqUncertainty>,
        epochs: &mut [TrackEpoch],
    ) {
        let Some(acquisition_uncertainty) = acquisition_uncertainty else {
            return;
        };
        if acquisition_uncertainty.code_phase_samples > 0.5 + f64::EPSILON {
            return;
        }
        if acquisition_uncertainty.doppler_hz
            > self.config.acquisition_doppler_step_hz.max(1) as f64 + f64::EPSILON
        {
            return;
        }
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let Some(diagnostic) = detect_sample_rate_mismatch(epochs, samples_per_code) else {
            return;
        };

        let first_epoch = &epochs[diagnostic.first_unstable_epoch_index];
        self.runtime.trace.record(TraceRecord {
            name: "tracking_sample_rate_mismatch",
            fields: vec![
                ("constellation", format!("{:?}", sat.constellation)),
                ("prn", sat.prn.to_string()),
                ("first_epoch_idx", first_epoch.epoch.index.to_string()),
                (
                    "max_abs_phase_step_samples",
                    format!("{:.6}", diagnostic.max_abs_phase_step_samples),
                ),
                ("phase_step_limit_samples", format!("{:.6}", diagnostic.phase_step_limit_samples)),
            ],
        });

        for epoch in epochs.iter_mut().skip(diagnostic.first_unstable_epoch_index) {
            epoch.lock_state_reason = Some("sample_rate_mismatch".to_string());
        }
    }

    fn epochs_in_frame(&self, frame: &SamplesFrame, tracking_params: TrackingParams) -> usize {
        tracking_epoch_count(
            frame.len(),
            tracking_epoch_samples(
                self.config.sampling_freq_hz,
                self.config.code_freq_basis_hz,
                self.config.code_length,
                tracking_params,
            ),
        )
    }

    fn track_epochs(
        &self,
        frame: &SamplesFrame,
        channel_id: u8,
        sat: SatId,
        carrier_hz: f64,
        code_phase_samples: f64,
        acquisition_cn0_proxy_dbhz: f64,
        tracking_params: TrackingParams,
        epochs: usize,
    ) -> (Vec<TrackEpoch>, Vec<TrackTransition>) {
        let mut state = LoopState {
            carrier_hz,
            carrier_phase_cycles: 0.0,
            code_rate_hz: self.config.code_freq_basis_hz,
            code_phase_samples,
            acquisition_cn0_proxy_dbhz,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            state: ChannelState::Acquired,
            unlocked_count: 0,
        };

        let mut out = Vec::new();
        let mut transitions = Vec::new();
        self.append_tracked_epochs(
            frame,
            channel_id,
            sat,
            tracking_params,
            epochs,
            &mut state,
            &mut out,
            &mut transitions,
        );
        (out, transitions)
    }

    pub(crate) fn begin_incremental_tracking(
        &self,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> IncrementalTrackingState {
        let channels = acquisitions
            .iter()
            .filter_map(|acq| self.tracking_start_context(acq))
            .take(self.config.channels.max(1))
            .enumerate()
            .map(|(channel_idx, context)| {
                let channel_id = channel_idx as u8;
                let tracking_params = self.config.tracking_params(context.seed.signal_band);
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_sat_start",
                    fields: vec![
                        ("constellation", format!("{:?}", context.seed.sat.constellation)),
                        ("prn", context.seed.sat.prn.to_string()),
                        ("hypothesis", context.acquisition_hypothesis.clone()),
                        ("to_track", context.acq_to_track_state.clone()),
                        ("cn0_proxy_dbhz", format!("{:.3}", context.acquisition_cn0_proxy_dbhz)),
                    ],
                });
                IncrementalTrackingChannel {
                    sat: context.seed.sat,
                    channel_id,
                    start_source_time: context.seed.source_time,
                    signal_band: context.seed.signal_band,
                    acquisition_uncertainty: context.seed.uncertainty.clone(),
                    acquisition_hypothesis: context.acquisition_hypothesis,
                    acquisition_score: context.acquisition_score,
                    acquisition_code_phase_samples: context.acquisition_code_phase_samples,
                    acquisition_doppler_hz: context.seed.doppler_hz.0,
                    acquisition_resolved_code_phase_samples: context.seed.code_phase_samples.0,
                    acquisition_carrier_hz: context.acquisition_carrier_hz,
                    acq_to_track_state: context.acq_to_track_state,
                    tracking_params,
                    state: LoopState {
                        carrier_hz: context.acquisition_carrier_hz,
                        carrier_phase_cycles: 0.0,
                        code_rate_hz: self.config.code_freq_basis_hz,
                        code_phase_samples: context.seed.code_phase_samples.0,
                        acquisition_cn0_proxy_dbhz: context.acquisition_cn0_proxy_dbhz,
                        prev_prompt: None,
                        prev_prompt_phase_cycles: None,
                        nav_bit_phase_offset_cycles: 0.0,
                        nav_bit_transition_count: 0,
                        pull_in_stable_epochs: 0,
                        weak_cn0_epochs: 0,
                        state: ChannelState::Acquired,
                        unlocked_count: 0,
                    },
                    epochs: Vec::new(),
                    transitions: Vec::new(),
                }
            })
            .collect();
        IncrementalTrackingState { channels }
    }

    pub(crate) fn track_incremental_frame(
        &self,
        tracking: &mut IncrementalTrackingState,
        frame: &SamplesFrame,
    ) {
        for channel in &mut tracking.channels {
            let Some(channel_frame) = tracking_frame_from_start_sample(
                frame,
                if channel.epochs.is_empty() {
                    Some(channel.start_source_time.sample_index)
                } else {
                    None
                },
            ) else {
                continue;
            };
            self.append_tracked_epochs(
                &channel_frame,
                channel.channel_id,
                channel.sat,
                channel.tracking_params,
                self.epochs_in_frame(&channel_frame, channel.tracking_params),
                &mut channel.state,
                &mut channel.epochs,
                &mut channel.transitions,
            );
        }
    }

    pub(crate) fn finish_incremental_tracking(
        &self,
        tracking: IncrementalTrackingState,
    ) -> Vec<TrackingResult> {
        tracking
            .channels
            .into_iter()
            .map(|mut channel| {
                for epoch in &mut channel.epochs {
                    epoch.tracking_provenance = format!(
                        "acq_hypothesis={} acq_score={:.6} acq_signal_band={:?} acq_doppler_hz={:.3} acq_carrier_hz={:.3} acq_code_phase_samples={} acq_resolved_code_phase_samples={:.6} acq_start_sample_index={}",
                        channel.acquisition_hypothesis,
                        channel.acquisition_score,
                        channel.signal_band,
                        channel.acquisition_doppler_hz,
                        channel.acquisition_carrier_hz,
                        channel.acquisition_code_phase_samples,
                        channel.acquisition_resolved_code_phase_samples,
                        channel.start_source_time.sample_index,
                    );
                }
                self.apply_sample_rate_mismatch_diagnostic(
                    channel.sat,
                    channel.acquisition_uncertainty.as_ref(),
                    &mut channel.epochs,
                );
                let stability_signature = tracking_stability_signature(&channel.epochs);
                let outcome = if channel.epochs.is_empty() { "not_tracked" } else { "tracked" };
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_sat_done",
                    fields: vec![
                        ("constellation", format!("{:?}", channel.sat.constellation)),
                        ("prn", channel.sat.prn.to_string()),
                        ("outcome", outcome.to_string()),
                        ("hypothesis", channel.acquisition_hypothesis.clone()),
                        ("epochs", channel.epochs.len().to_string()),
                        ("stability_signature", stability_signature),
                    ],
                });
                TrackingResult {
                    sat: channel.sat,
                    carrier_hz: channel.acquisition_carrier_hz,
                    code_phase_samples: channel.acquisition_resolved_code_phase_samples,
                    acquisition_hypothesis: channel.acquisition_hypothesis,
                    acquisition_score: channel.acquisition_score,
                    acquisition_code_phase_samples: channel.acquisition_code_phase_samples,
                    acquisition_carrier_hz: channel.acquisition_carrier_hz,
                    acq_to_track_state: channel.acq_to_track_state,
                    epochs: channel.epochs,
                    transitions: channel.transitions,
                }
            })
            .collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn append_tracked_epochs(
        &self,
        frame: &SamplesFrame,
        channel_id: u8,
        sat: SatId,
        tracking_params: TrackingParams,
        epochs: usize,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let samples_per_epoch = tracking_epoch_samples(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
            tracking_params,
        );
        let samples_per_chip = samples_per_code as f64 / self.config.code_length as f64;
        for epoch_idx in 0..epochs {
            let alloc_before = crate::engine::alloc::allocation_count();
            let start = epoch_idx * samples_per_epoch;
            let end = (start + samples_per_epoch).min(frame.len());
            if start >= end {
                break;
            }
            let epoch_frame = SamplesFrame::new(
                SampleTime {
                    sample_index: frame.t0.sample_index + start as u64,
                    sample_rate_hz: frame.t0.sample_rate_hz,
                },
                frame.dt_s,
                frame.iq[start..end].to_vec(),
            );

            let (mut track_epoch, corr) = self.track_epoch(
                &epoch_frame,
                channel_id,
                sat,
                state.carrier_hz,
                state.carrier_phase_cycles,
                state.code_rate_hz,
                state.code_phase_samples,
                tracking_params.early_late_spacing_chips,
            );
            track_epoch.processing_ms = None;
            let alloc_after = crate::engine::alloc::allocation_count();
            if alloc_after > alloc_before {
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "TRACK_ALLOCATIONS",
                    format!("tracking epoch allocated {} times", alloc_after - alloc_before),
                ));
            }

            let (dll_err, _raw_pll_err, raw_fll_err, lock) =
                discriminators(corr.early, corr.prompt, corr.late, state.prev_prompt);
            state.prev_prompt = Some(corr.prompt);
            let phase_cycles = corr.prompt.arg() as f64 / (2.0 * std::f64::consts::PI);

            let anti_false_lock =
                (corr.early.norm() - corr.late.norm()).abs() < 0.2 * corr.prompt.norm();

            let phase_decision = classify_prompt_phase(
                phase_cycles,
                state.prev_prompt_phase_cycles,
                state.nav_bit_phase_offset_cycles,
            );
            state.prev_prompt_phase_cycles = Some(phase_decision.aligned_phase_cycles);
            state.nav_bit_phase_offset_cycles = phase_decision.nav_bit_phase_offset_cycles;
            let pll_err = (phase_decision.aligned_phase_cycles * std::f64::consts::TAU) as f32;
            if phase_decision.nav_bit_transition {
                state.nav_bit_transition_count = state.nav_bit_transition_count.saturating_add(1);
            }

            let cycle_slip = phase_decision.cycle_slip;
            let mut cycle_slip_reason = None;
            if cycle_slip {
                cycle_slip_reason = Some("phase_discontinuity".to_string());
            }

            let cn0_dbhz = track_epoch.cn0_dbhz;
            let (dll_bw, pll_bw, fll_bw) = adaptive_bandwidth(
                tracking_params.dll_bw_hz,
                tracking_params.pll_bw_hz,
                tracking_params.fll_bw_hz,
                cn0_dbhz,
            );
            let coherent_integration_s =
                coherent_integration_seconds(epoch_frame.len(), self.config.sampling_freq_hz);
            let raw_fll_err_hz = carrier_frequency_error_hz_from_phase_delta(
                raw_fll_err as f64,
                coherent_integration_s,
            );
            let nav_bit_aware_fll_err_hz = carrier_frequency_error_hz_from_phase_delta(
                phase_decision.aligned_phase_delta_cycles * std::f64::consts::TAU,
                coherent_integration_s,
            );
            let use_nav_bit_aware_fll = phase_decision.nav_bit_transition
                || state.nav_bit_phase_offset_cycles.abs() > f64::EPSILON
                || phase_decision.nav_bit_phase_offset_cycles.abs() > f64::EPSILON;
            let fll_err_hz =
                if use_nav_bit_aware_fll { nav_bit_aware_fll_err_hz } else { raw_fll_err_hz }
                    as f32;
            let raw_pll_lock = pll_err.abs() < PLL_LOCK_MAX_PHASE_ERROR_RAD;
            let raw_fll_lock = (fll_err_hz as f64).abs() <= fll_lock_threshold_hz(fll_bw);
            state.pull_in_stable_epochs = update_pull_in_stable_epochs(
                state.pull_in_stable_epochs,
                lock,
                raw_pll_lock,
                raw_fll_lock,
                cycle_slip,
            );
            let (weak_cn0_epochs, cn0_supports_lock, refuse_lock) = update_prelock_cn0_refusal(
                state.state,
                state.weak_cn0_epochs,
                state.acquisition_cn0_proxy_dbhz,
            );
            state.weak_cn0_epochs = weak_cn0_epochs;
            let ready_for_tracking =
                cn0_supports_lock && state.pull_in_stable_epochs >= PULL_IN_REQUIRED_STABLE_EPOCHS;

            let from_state = state.state;
            let transition = if refuse_lock {
                TransitionDecision {
                    to_state: ChannelState::PullIn,
                    reason: "cn0_below_tracking_lock_floor".to_string(),
                    next_unlocked_count: state.unlocked_count.saturating_add(1),
                }
            } else {
                deterministic_transition_rule(
                    from_state,
                    lock,
                    ready_for_tracking,
                    anti_false_lock,
                    cycle_slip,
                    state.unlocked_count,
                )
            };
            state.unlocked_count = transition.next_unlocked_count;
            state.state = transition.to_state;
            if cycle_slip {
                cycle_slip_reason = match cycle_slip_reason {
                    Some(reason) => Some(reason),
                    None => Some("cycle_slip_detected".to_string()),
                };
            }

            let lock_state = match state.state {
                ChannelState::Idle => "idle".to_string(),
                ChannelState::Acquired => "acquired".to_string(),
                ChannelState::PullIn => "pull_in".to_string(),
                ChannelState::Tracking => "tracking".to_string(),
                ChannelState::Lost => "lost".to_string(),
            };
            let lock_state_reason = Some(transition.reason.clone());

            if cycle_slip_reason.is_none() && state.state == ChannelState::Lost && !lock {
                cycle_slip_reason = Some("lock_reacquire".to_string());
            }

            let dll_lock = dll_err.abs() < 0.2;
            let pll_lock = raw_pll_lock && ready_for_tracking;
            let fll_lock = raw_fll_lock;
            if track_epoch.early_i.is_infinite() || track_epoch.late_i.is_infinite() {
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "TRACK_NUMERIC_INVALID",
                    "infinite tracking sample in early/late output",
                ));
            }

            let code_loop = apply_dll_code_loop(CodeLoopInput {
                current_code_rate_hz: state.code_rate_hz,
                current_code_phase_samples: state.code_phase_samples,
                epoch_len_samples: epoch_frame.len(),
                coherent_integration_s,
                nominal_code_rate_hz: self.config.code_freq_basis_hz,
                dll_bw_hz: dll_bw,
                dll_err,
                samples_per_chip,
                samples_per_code,
            });
            state.code_rate_hz = code_loop.code_rate_hz;
            let carrier_loop = apply_carrier_loop(CarrierLoopInput {
                current_carrier_hz: state.carrier_hz,
                current_carrier_phase_cycles: state.carrier_phase_cycles,
                epoch_len_samples: epoch_frame.len(),
                sample_rate_hz: self.config.sampling_freq_hz,
                coherent_integration_s,
                pll_bw_hz: pll_bw,
                pll_err_rad: pll_err as f64,
                fll_bw_hz: fll_bw,
                fll_err_hz: fll_err_hz as f64,
                apply_fll: should_apply_fll(state.state, raw_fll_lock),
            });
            state.carrier_hz = carrier_loop.carrier_hz;
            state.carrier_phase_cycles = carrier_loop.carrier_phase_cycles;
            state.code_phase_samples = code_loop.code_phase_samples;

            if state.state != from_state {
                transitions.push(TrackTransition {
                    sat,
                    channel_id,
                    epoch_idx: track_epoch.epoch.index,
                    sample_index: track_epoch.sample_index,
                    from_state: from_state.to_string(),
                    to_state: state.state.to_string(),
                    reason: lock_state_reason
                        .clone()
                        .unwrap_or_else(|| "state_transition".to_string()),
                    lock_quality: epoch_lock_quality(lock, pll_lock, dll_lock, fll_lock, cn0_dbhz),
                });
            }

            out.push(TrackEpoch {
                lock,
                carrier_hz: Hertz(state.carrier_hz),
                carrier_phase_cycles: Cycles(state.carrier_phase_cycles),
                code_rate_hz: Hertz(state.code_rate_hz),
                code_phase_samples: Chips(state.code_phase_samples),
                pll_lock,
                dll_lock,
                fll_lock,
                cycle_slip,
                nav_bit_lock: state.nav_bit_transition_count > 0,
                dll_err,
                pll_err,
                fll_err: fll_err_hz,
                anti_false_lock,
                cycle_slip_reason,
                lock_state,
                lock_state_reason,
                tracking_assumptions: Some(tracking_assumptions(tracking_params)),
                ..track_epoch
            });
        }
    }
}

fn acq_to_track_state(hypothesis: &AcqHypothesis) -> &'static str {
    match hypothesis {
        AcqHypothesis::Accepted => "accepted",
        AcqHypothesis::Ambiguous => "degraded",
        AcqHypothesis::Rejected => "rejected",
        AcqHypothesis::Deferred => "deferred",
    }
}

fn ca_code_or_default(prn: u8) -> Vec<i8> {
    match generate_ca_code(Prn(prn)) {
        Ok(code) => code,
        Err(_) => vec![1; 1023],
    }
}

fn epoch_lock_quality(
    lock: bool,
    pll_lock: bool,
    dll_lock: bool,
    fll_lock: bool,
    cn0_dbhz: f64,
) -> f64 {
    let mut quality = (cn0_dbhz / 60.0).clamp(0.0, 1.0);
    if !lock {
        quality *= 0.2;
    }
    if !pll_lock {
        quality *= 0.7;
    }
    if !dll_lock {
        quality *= 0.8;
    }
    if !fll_lock {
        quality *= 0.9;
    }
    quality
}

#[derive(Debug, Clone)]
struct TransitionDecision {
    to_state: ChannelState,
    reason: String,
    next_unlocked_count: u8,
}

fn deterministic_transition_rule(
    from_state: ChannelState,
    lock: bool,
    ready_for_tracking: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
    unlocked_count: u8,
) -> TransitionDecision {
    if cycle_slip {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "cycle_slip".to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
        };
    }
    if ready_for_tracking {
        return TransitionDecision {
            to_state: ChannelState::Tracking,
            reason: "carrier_converged".to_string(),
            next_unlocked_count: 0,
        };
    }
    if lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "carrier_pull_in".to_string(),
            next_unlocked_count: 0,
        };
    }
    if anti_false_lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "anti_false_lock".to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
        };
    }
    let next_unlocked = unlocked_count.saturating_add(1);
    if from_state == ChannelState::Tracking && next_unlocked >= 2 {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "lock_lost".to_string(),
            next_unlocked_count: next_unlocked,
        };
    }
    TransitionDecision {
        to_state: ChannelState::PullIn,
        reason: "carrier_pull_in".to_string(),
        next_unlocked_count: next_unlocked,
    }
}

fn default_tracking_assumptions(config: &ReceiverPipelineConfig) -> TrackingAssumptions {
    tracking_assumptions(config.tracking_params(SignalBand::L1))
}

fn tracking_assumptions(params: TrackingParams) -> TrackingAssumptions {
    TrackingAssumptions {
        integration_ms: params.integration_ms,
        early_late_spacing_chips: params.early_late_spacing_chips,
        dll_bw_hz: params.dll_bw_hz,
        pll_bw_hz: params.pll_bw_hz,
        fll_bw_hz: params.fll_bw_hz,
        discriminator_family: "early_prompt_late".to_string(),
        aiding_mode: "none".to_string(),
    }
}

fn tracking_frame_from_start_sample(
    frame: &SamplesFrame,
    start_sample_index: Option<u64>,
) -> Option<SamplesFrame> {
    let Some(start_sample_index) = start_sample_index else {
        return Some(frame.clone());
    };
    let frame_start = frame.t0.sample_index;
    let frame_end = frame_start + frame.len() as u64;
    if start_sample_index >= frame_end {
        return None;
    }
    if start_sample_index <= frame_start {
        return Some(frame.clone());
    }
    let offset = (start_sample_index - frame_start) as usize;
    Some(SamplesFrame::new(
        SampleTime { sample_index: start_sample_index, sample_rate_hz: frame.t0.sample_rate_hz },
        frame.dt_s,
        frame.iq[offset..].to_vec(),
    ))
}

fn classify_prompt_phase(
    raw_phase_cycles: f64,
    previous_aligned_phase_cycles: Option<f64>,
    nav_bit_phase_offset_cycles: f64,
) -> PromptPhaseDecision {
    let same_offset_phase = wrap_phase_cycles(raw_phase_cycles - nav_bit_phase_offset_cycles);
    let Some(previous_aligned_phase_cycles) = previous_aligned_phase_cycles else {
        return PromptPhaseDecision {
            aligned_phase_cycles: same_offset_phase,
            aligned_phase_delta_cycles: 0.0,
            nav_bit_phase_offset_cycles,
            nav_bit_transition: false,
            cycle_slip: false,
        };
    };

    let same_delta = wrapped_phase_delta_cycles(same_offset_phase, previous_aligned_phase_cycles);
    let flipped_offset = wrap_phase_cycles(nav_bit_phase_offset_cycles + NAV_BIT_PHASE_STEP_CYCLES);
    let flipped_phase = wrap_phase_cycles(raw_phase_cycles - flipped_offset);
    let flipped_delta = wrapped_phase_delta_cycles(flipped_phase, previous_aligned_phase_cycles);

    let nav_bit_transition = same_delta.abs() > CYCLE_SLIP_PHASE_DELTA_CYCLES
        && (same_delta.abs() - NAV_BIT_PHASE_STEP_CYCLES).abs()
            <= NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES
        && flipped_delta.abs() <= NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES
        && flipped_delta.abs() < same_delta.abs();

    let (aligned_phase_cycles, nav_bit_phase_offset_cycles, chosen_delta) = if nav_bit_transition {
        (flipped_phase, flipped_offset, flipped_delta)
    } else {
        (same_offset_phase, nav_bit_phase_offset_cycles, same_delta)
    };

    PromptPhaseDecision {
        aligned_phase_cycles,
        aligned_phase_delta_cycles: chosen_delta,
        nav_bit_phase_offset_cycles,
        nav_bit_transition,
        cycle_slip: chosen_delta.abs() > CYCLE_SLIP_PHASE_DELTA_CYCLES,
    }
}

fn advance_code_phase_samples(
    current_code_phase_samples: f64,
    epoch_len_samples: usize,
    tracked_code_rate_hz: f64,
    nominal_code_rate_hz: f64,
    dll_err: f32,
    samples_per_chip: f64,
    samples_per_code: usize,
) -> f64 {
    let nominal_code_rate_hz = nominal_code_rate_hz.max(1.0);
    let code_period_advance_samples =
        epoch_len_samples as f64 * (tracked_code_rate_hz / nominal_code_rate_hz);
    let dll_correction_samples =
        -(dll_err as f64) * samples_per_chip * DLL_CODE_PHASE_CORRECTION_GAIN;
    wrap_code_phase_samples(
        current_code_phase_samples + code_period_advance_samples + dll_correction_samples,
        samples_per_code,
    )
}

fn apply_dll_code_loop(input: CodeLoopInput) -> CodeLoopUpdate {
    let coefficients = first_order_loop_coefficients(input.dll_bw_hz, input.coherent_integration_s);
    let code_rate_hz =
        input.current_code_rate_hz - coefficients.rate_gain_hz * input.dll_err as f64;
    let code_phase_samples = advance_code_phase_samples(
        input.current_code_phase_samples,
        input.epoch_len_samples,
        code_rate_hz,
        input.nominal_code_rate_hz,
        input.dll_err,
        input.samples_per_chip,
        input.samples_per_code,
    );
    CodeLoopUpdate { code_rate_hz, code_phase_samples }
}

fn wrap_code_phase_samples(code_phase_samples: f64, samples_per_code: usize) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    code_phase_samples.rem_euclid(code_period_samples)
}

fn tracking_stability_signature(epochs: &[TrackEpoch]) -> String {
    let mut rows = epochs
        .iter()
        .map(|epoch| {
            format!(
                "{}|{}|{:.3}|{:.3}|{}|{}|{}",
                epoch.epoch.index,
                epoch.sample_index,
                epoch.carrier_hz.0,
                epoch.code_phase_samples.0,
                epoch.lock_state,
                epoch.lock,
                epoch.cycle_slip
            )
        })
        .collect::<Vec<_>>();
    rows.sort();
    rows.join(";")
}

fn detect_sample_rate_mismatch(
    epochs: &[TrackEpoch],
    samples_per_code: usize,
) -> Option<CodePhaseStabilityDiagnostic> {
    if epochs.len() <= SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS {
        return None;
    }

    let phase_step_limit_samples = SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_SAMPLES
        .max(samples_per_code as f64 * SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_FRACTION);
    let pull_in_phase_step_limit_samples = phase_step_limit_samples * 0.125;
    let phase_bias_limit_samples = (samples_per_code as f64 * 0.00025).max(1.0);
    let baseline_code_phase_samples = epochs[0].code_phase_samples.0;
    let instability_markers = epochs
        .windows(2)
        .enumerate()
        .map(|(index, pair)| {
            let previous = &pair[0];
            let current = &pair[1];
            let phase_step_samples = wrapped_code_phase_delta(
                current.code_phase_samples.0,
                previous.code_phase_samples.0,
                samples_per_code,
            );
            let abs_phase_step_samples = phase_step_samples.abs();
            let abs_phase_bias_samples = wrapped_code_phase_delta(
                current.code_phase_samples.0,
                baseline_code_phase_samples,
                samples_per_code,
            )
            .abs();
            let unstable = abs_phase_step_samples > phase_step_limit_samples
                || (current.lock_state != ChannelState::Tracking.to_string()
                    && abs_phase_step_samples > pull_in_phase_step_limit_samples)
                || (current.lock_state != ChannelState::Tracking.to_string()
                    && abs_phase_bias_samples > phase_bias_limit_samples)
                || current.cycle_slip
                || !current.lock
                || current.lock_state == ChannelState::Lost.to_string();
            let supported = current.cn0_dbhz.is_finite()
                && current.cn0_dbhz >= SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ;
            (index + 1, abs_phase_step_samples, unstable, supported)
        })
        .collect::<Vec<_>>();

    for window in instability_markers.windows(SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS) {
        let unstable_count =
            window.iter().filter(|(_, _, unstable, supported)| *unstable && *supported).count();
        if unstable_count >= SAMPLE_RATE_MISMATCH_MIN_UNSTABLE_EPOCHS_IN_WINDOW {
            let first_unstable_epoch_index = window
                .iter()
                .find(|(_, _, unstable, supported)| *unstable && *supported)
                .map(|(epoch_index, _, _, _)| *epoch_index)
                .expect("window contains unstable epoch");
            let max_abs_phase_step_samples = window
                .iter()
                .filter(|(_, _, unstable, supported)| *unstable && *supported)
                .map(|(_, abs_phase_step_samples, _, _)| *abs_phase_step_samples)
                .fold(0.0_f64, f64::max);
            return Some(CodePhaseStabilityDiagnostic {
                first_unstable_epoch_index,
                max_abs_phase_step_samples,
                phase_step_limit_samples,
            });
        }
    }

    None
}

fn wrapped_code_phase_delta(
    next_code_phase_samples: f64,
    previous_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    let mut delta =
        (next_code_phase_samples - previous_code_phase_samples).rem_euclid(code_period_samples);
    if delta > code_period_samples / 2.0 {
        delta -= code_period_samples;
    }
    delta
}

fn wrap_phase_cycles(phase_cycles: f64) -> f64 {
    let mut wrapped = phase_cycles.rem_euclid(1.0);
    if wrapped > 0.5 {
        wrapped -= 1.0;
    }
    wrapped
}

fn wrap_phase_rad(phase_rad: f64) -> f64 {
    phase_rad.rem_euclid(std::f64::consts::TAU)
}

fn carrier_phase_offset_rad(
    carrier_hz: f64,
    sample_rate_hz: f64,
    sample_index: u64,
    carrier_phase_cycles: f64,
) -> f64 {
    wrap_phase_rad(
        carrier_phase_cycles * std::f64::consts::TAU
            - std::f64::consts::TAU * carrier_hz * (sample_index as f64 / sample_rate_hz),
    )
}

fn wrapped_phase_delta_cycles(next_phase_cycles: f64, previous_phase_cycles: f64) -> f64 {
    wrap_phase_cycles(next_phase_cycles - previous_phase_cycles)
}

fn tracking_epoch_samples(
    sample_rate_hz: f64,
    code_freq_basis_hz: f64,
    code_length: usize,
    tracking_params: TrackingParams,
) -> usize {
    let code_period_samples = samples_per_code(sample_rate_hz, code_freq_basis_hz, code_length);
    code_period_samples.saturating_mul(tracking_params.integration_ms.max(1) as usize)
}

fn tracking_epoch_count(frame_len_samples: usize, epoch_len_samples: usize) -> usize {
    let epoch_len_samples = epoch_len_samples.max(1);
    if frame_len_samples == 0 {
        return 0;
    }
    frame_len_samples.div_ceil(epoch_len_samples)
}

fn coherent_integration_seconds(epoch_len_samples: usize, sample_rate_hz: f64) -> f64 {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return 0.0;
    }
    epoch_len_samples as f64 / sample_rate_hz
}

fn fll_lock_threshold_hz(fll_bw_hz: f64) -> f64 {
    fll_bw_hz.max(5.0)
}

fn update_pull_in_stable_epochs(
    current_stable_epochs: u8,
    prompt_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
) -> u8 {
    if cycle_slip || !prompt_lock || !pll_lock || !fll_lock {
        return 0;
    }
    current_stable_epochs.saturating_add(1)
}

fn update_prelock_cn0_refusal(
    current_state: ChannelState,
    weak_cn0_epochs: u8,
    cn0_dbhz: f64,
) -> (u8, bool, bool) {
    if current_state == ChannelState::Tracking {
        return (0, true, false);
    }
    if !cn0_dbhz.is_finite() || cn0_dbhz >= TRACKING_LOCK_MIN_CN0_DBHZ {
        return (0, cn0_dbhz.is_finite(), false);
    }

    let next_weak_cn0_epochs = weak_cn0_epochs.saturating_add(1);
    (next_weak_cn0_epochs, false, next_weak_cn0_epochs >= TRACKING_LOCK_REFUSAL_EPOCHS)
}

fn bounded_fll_pull_in_correction_hz(fll_err_hz: f64, fll_bw_hz: f64) -> f64 {
    let max_correction_hz = fll_bw_hz.abs().max(1.0) * FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER;
    fll_err_hz.clamp(-max_correction_hz, max_correction_hz)
}

fn apply_carrier_loop(input: CarrierLoopInput) -> CarrierLoopUpdate {
    let pll_coefficients =
        phase_lock_loop_coefficients(input.pll_bw_hz, input.coherent_integration_s);
    let fll_coefficients =
        first_order_angular_loop_coefficients(input.fll_bw_hz, input.coherent_integration_s);
    let mut carrier_hz = input.current_carrier_hz;
    if input.apply_fll {
        carrier_hz += bounded_fll_pull_in_correction_hz(
            input.fll_err_hz * fll_coefficients.error_blend,
            input.fll_bw_hz,
        );
    }
    carrier_hz += pll_coefficients.frequency_gain_hz_per_rad * input.pll_err_rad;

    let carrier_phase_cycles = input.current_carrier_phase_cycles
        + carrier_hz * input.epoch_len_samples as f64 / input.sample_rate_hz
        + pll_coefficients.phase_blend * input.pll_err_rad / std::f64::consts::TAU;

    CarrierLoopUpdate { carrier_hz, carrier_phase_cycles }
}

impl Tracking {
    fn quick_reacquire(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_hz: f64,
        code_phase_samples: f64,
        acquisition_uncertainty: Option<&AcqUncertainty>,
    ) -> Option<(f64, f64)> {
        let doppler_step_hz = self.config.acquisition_doppler_step_hz.max(1) as f64;
        let doppler_step = acquisition_uncertainty
            .map(|uncertainty| uncertainty.doppler_hz.clamp(doppler_step_hz / 2.0, doppler_step_hz))
            .unwrap_or(doppler_step_hz);
        let code_step = acquisition_uncertainty
            .map(|uncertainty| uncertainty.code_phase_samples.clamp(0.5, 2.0))
            .unwrap_or(1.0);
        let doppler_bins = [-2.0, -1.0, 0.0, 1.0, 2.0].map(|bin| bin * doppler_step);
        let code_bins = [-2.0, -1.0, 0.0, 1.0, 2.0].map(|bin| bin * code_step);
        let mut best = None;
        let mut best_metric = 0.0_f32;
        for d in doppler_bins {
            for c in code_bins {
                let corr = self.correlate_epoch(
                    frame,
                    sat,
                    carrier_hz + d,
                    0.0,
                    self.config.code_freq_basis_hz,
                    code_phase_samples + c,
                    0.5,
                );
                let metric = corr.prompt.norm();
                if metric > best_metric {
                    best_metric = metric;
                    best = Some((carrier_hz + d, code_phase_samples + c));
                }
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::{ChannelState, Tracking};
    use crate::engine::receiver_config::ReceiverPipelineConfig;
    use crate::engine::runtime::ReceiverRuntime;
    use crate::sim::synthetic::{generate_l1_ca, SyntheticSignalParams};
    use bijux_gnss_core::api::{
        AcqHypothesis, Chips, Epoch, SampleTime, SamplesFrame, SatId, Seconds, TrackEpoch,
    };
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, discriminators, first_order_angular_loop_coefficients,
        first_order_loop_coefficients, phase_lock_loop_coefficients, sample_ca_code,
        samples_per_code, Prn,
    };
    use num_complex::Complex;
    use serde::Deserialize;

    #[test]
    fn tracking_recovery_from_loss_of_lock() {
        let lost = Tracking::transition_state(1, ChannelState::Tracking, ChannelState::Lost);
        assert_eq!(lost, ChannelState::Lost);
        let pull_in = Tracking::transition_state(1, lost, ChannelState::PullIn);
        assert_eq!(pull_in, ChannelState::PullIn);
        let tracking = Tracking::transition_state(1, pull_in, ChannelState::Tracking);
        assert_eq!(tracking, ChannelState::Tracking);
    }

    #[test]
    fn tracking_reset_after_gap() {
        let lost = Tracking::transition_state(2, ChannelState::Tracking, ChannelState::Lost);
        let reset = Tracking::transition_state(2, lost, ChannelState::Idle);
        assert_eq!(reset, ChannelState::Idle);
    }

    #[test]
    fn ambiguous_hypothesis_is_degraded_for_tracking() {
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Accepted), "accepted");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Ambiguous), "degraded");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Rejected), "rejected");
        assert_eq!(super::acq_to_track_state(&AcqHypothesis::Deferred), "deferred");
    }

    #[test]
    fn deterministic_transition_rule_handles_cycle_slip_first() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            false,
            false,
            false,
            true,
            1,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "cycle_slip");
        assert_eq!(decision.next_unlocked_count, 2);
    }

    #[test]
    fn deterministic_transition_rule_promotes_lock() {
        let decision =
            super::deterministic_transition_rule(ChannelState::PullIn, true, true, false, false, 2);
        assert_eq!(decision.to_state, ChannelState::Tracking);
        assert_eq!(decision.reason, "carrier_converged");
        assert_eq!(decision.next_unlocked_count, 0);
    }

    #[test]
    fn deterministic_transition_rule_marks_loss_after_tracking_failures() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            false,
            false,
            false,
            false,
            1,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "lock_lost");
        assert_eq!(decision.next_unlocked_count, 2);
    }

    #[test]
    fn deterministic_transition_rule_holds_pull_in_until_carrier_converges() {
        let decision = super::deterministic_transition_rule(
            ChannelState::PullIn,
            true,
            false,
            false,
            false,
            1,
        );

        assert_eq!(decision.to_state, ChannelState::PullIn);
        assert_eq!(decision.reason, "carrier_pull_in");
        assert_eq!(decision.next_unlocked_count, 0);
    }

    #[test]
    fn prelock_cn0_refusal_trips_after_repeated_weak_epochs() {
        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 20.0);

        assert_eq!(weak_epochs, 3);
        assert!(!supports_lock);
        assert!(refuse_lock);
    }

    #[test]
    fn prelock_cn0_refusal_resets_on_supported_signal_or_established_tracking() {
        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::PullIn, 2, 30.0);
        assert_eq!(weak_epochs, 0);
        assert!(supports_lock);
        assert!(!refuse_lock);

        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::Tracking, 2, 20.0);
        assert_eq!(weak_epochs, 0);
        assert!(supports_lock);
        assert!(!refuse_lock);
    }

    #[test]
    fn classify_prompt_phase_recovers_continuity_across_nav_bit_flip() {
        let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0);

        assert!(decision.nav_bit_transition);
        assert!(!decision.cycle_slip);
        assert!((decision.aligned_phase_cycles - 0.11).abs() <= 0.02);
        assert!(decision.aligned_phase_delta_cycles.abs() <= 0.02);
        assert!((decision.nav_bit_phase_offset_cycles - 0.5).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_preserves_cycle_slip_for_non_nav_jump() {
        let decision = super::classify_prompt_phase(0.36, Some(0.0), 0.0);

        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
        assert!((decision.aligned_phase_delta_cycles - 0.36).abs() <= 1.0e-9);
        assert!((decision.nav_bit_phase_offset_cycles - 0.0).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_handles_real_synthetic_nav_bit_transitions() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 7 };
        let code_phase_chips = 321.0;
        let samples_per_epoch = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                doppler_hz: 0.0,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 70.0,
                data_bit_flip: true,
            },
            0x7A91B17,
            0.05,
        );
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let mut previous_aligned_phase_cycles = None;
        let mut nav_bit_phase_offset_cycles = 0.0;
        let mut transition_epochs = Vec::new();

        for (epoch_index, epoch_samples) in frame.iq.chunks_exact(samples_per_epoch).enumerate() {
            let sample_index = epoch_index as u64 * samples_per_epoch as u64;
            let epoch_frame = SamplesFrame::new(
                SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                epoch_samples.to_vec(),
            );
            let epoch_code_phase_chips = advance_code_phase_seconds(
                code_phase_chips,
                config.code_freq_basis_hz,
                sample_index as f64 / config.sampling_freq_hz,
                config.code_length,
            )
            .expect("valid epoch code phase");
            let epoch_code_phase_samples =
                epoch_code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
            let (epoch, _) = tracking.track_epoch(
                &epoch_frame,
                0,
                sat,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                epoch_code_phase_samples,
                0.5,
            );
            let raw_phase_cycles =
                (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / (2.0 * std::f64::consts::PI);
            let decision = super::classify_prompt_phase(
                raw_phase_cycles,
                previous_aligned_phase_cycles,
                nav_bit_phase_offset_cycles,
            );
            if decision.nav_bit_transition {
                transition_epochs.push(epoch_index);
            }
            assert!(
                !decision.cycle_slip,
                "unexpected cycle slip at epoch {epoch_index}: raw_phase_cycles={raw_phase_cycles}"
            );
            previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
            nav_bit_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
        }

        assert_eq!(transition_epochs, vec![20, 40]);
    }

    #[test]
    fn advance_code_phase_samples_wraps_nominal_epoch_step() {
        let next = super::advance_code_phase_samples(
            137.5,
            5_000,
            1_023_000.0,
            1_023_000.0,
            0.0,
            4.887585532746823,
            5_000,
        );
        assert!((next - 137.5).abs() < 1.0e-9, "next={next}");
    }

    #[test]
    fn advance_code_phase_samples_applies_dll_correction() {
        let next = super::advance_code_phase_samples(
            250.0,
            5_000,
            1_023_000.0,
            1_023_000.0,
            0.4,
            4.887585532746823,
            5_000,
        );
        assert!(next < 250.0, "next={next}");
    }

    #[test]
    fn tracking_epoch_samples_scale_with_configured_integration_ms() {
        let tracking_params = crate::engine::receiver_config::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 7,
        };

        let epoch_samples =
            super::tracking_epoch_samples(5_000_000.0, 1_023_000.0, 1023, tracking_params);

        assert_eq!(epoch_samples, 35_000);
    }

    #[test]
    fn tracking_epoch_count_includes_trailing_partial_epoch() {
        assert_eq!(super::tracking_epoch_count(60, 20), 3);
        assert_eq!(super::tracking_epoch_count(61, 20), 4);
        assert_eq!(super::tracking_epoch_count(0, 20), 0);
    }

    #[test]
    fn apply_dll_code_loop_updates_code_rate_from_discriminator() {
        let coherent_integration_s = 5_000.0 / 1_023_000.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 5_000,
            coherent_integration_s,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.25,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        let expected = 1_023_000.0
            - first_order_loop_coefficients(2.0, coherent_integration_s).rate_gain_hz * 0.25;
        assert!((update.code_rate_hz - expected).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_dll_code_loop_uses_coherent_interval_to_scale_gain() {
        let short = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 4_092,
            coherent_integration_s: 0.001,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.25,
            samples_per_chip: 4.0,
            samples_per_code: 4_092,
        });
        let long = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 40_920,
            coherent_integration_s: 0.010,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.25,
            samples_per_chip: 4.0,
            samples_per_code: 40_920,
        });

        assert!(
            (short.code_rate_hz - 1_023_000.0).abs() > (long.code_rate_hz - 1_023_000.0).abs(),
            "short={short:?} long={long:?}"
        );
    }

    #[test]
    fn apply_dll_code_loop_pulls_positive_code_error_toward_prompt() {
        let current_code_phase_samples = 250.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!(update.code_phase_samples < current_code_phase_samples, "update={update:?}");
    }

    #[test]
    fn apply_dll_code_loop_pulls_negative_code_error_toward_prompt() {
        let current_code_phase_samples = 250.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            current_code_phase_samples,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: -0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!(update.code_phase_samples > current_code_phase_samples, "update={update:?}");
    }

    #[test]
    fn correlate_epoch_aligns_prompt_with_non_integer_rate_sampled_code() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let code_phase_chips = 245.25;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
        let code_phase_samples =
            code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            code_phase_chips,
            sample_count,
        )
        .expect("valid sampled code");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );

        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );

        assert!(correlator.prompt.norm() > correlator.early.norm());
        assert!(correlator.prompt.norm() > correlator.late.norm());
    }

    #[test]
    fn correlate_epoch_uses_tracked_code_rate_for_code_rate_offset_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let signal_code_rate_hz = config.code_freq_basis_hz + 1_200.0;
        let code_phase_chips = 87.375;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 9 };
        let code_phase_samples =
            code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            signal_code_rate_hz,
            code_phase_chips,
            sample_count,
        )
        .expect("valid sampled code with signal code-rate offset");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );

        let nominal = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );
        let matched = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            signal_code_rate_hz,
            code_phase_samples,
            0.5,
        );

        assert!(
            matched.prompt.norm() > nominal.prompt.norm(),
            "matched_prompt={} nominal_prompt={}",
            matched.prompt.norm(),
            nominal.prompt.norm(),
        );
        assert!(matched.prompt.norm() > matched.early.norm());
        assert!(matched.prompt.norm() > matched.late.norm());
    }

    #[test]
    fn correlate_epoch_uses_tracked_carrier_phase_for_phase_offset_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
        let carrier_phase_rad = 0.35;
        let carrier_phase_cycles = carrier_phase_rad / std::f64::consts::TAU;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad,
                cn0_db_hz: 70.0,
                data_bit_flip: false,
            },
            0xC0A5_1E,
            0.001,
        );

        let unmatched =
            tracking.correlate_epoch(&frame, sat, 0.0, 0.0, config.code_freq_basis_hz, 0.0, 0.5);
        let matched = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            carrier_phase_cycles,
            config.code_freq_basis_hz,
            0.0,
            0.5,
        );

        assert!(
            matched.prompt.re > unmatched.prompt.re,
            "matched={:?} unmatched={:?}",
            matched.prompt,
            unmatched.prompt,
        );
        assert!(
            matched.prompt.im.abs() < unmatched.prompt.im.abs(),
            "matched={:?} unmatched={:?}",
            matched.prompt,
            unmatched.prompt,
        );
    }

    #[test]
    fn apply_carrier_loop_advances_phase_and_frequency_from_pll_error() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
        });

        let pll_coefficients = phase_lock_loop_coefficients(8.0, 0.001);
        assert!(
            (update.carrier_hz - (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25))
                .abs()
                < 1.0e-9,
            "{update:?}"
        );
        let expected_phase_cycles = 12.0
            + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25) * 0.001
            + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
        assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}",);
    }

    #[test]
    fn apply_carrier_loop_uses_fll_correction_during_pull_in() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 80.0,
            current_carrier_phase_cycles: 12.0,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 10.0,
            fll_err_hz: 30.0,
            apply_fll: true,
        });

        let pll_coefficients = phase_lock_loop_coefficients(8.0, 0.001);
        let fll_coefficients = first_order_angular_loop_coefficients(10.0, 0.001);
        let expected_carrier_hz = 80.0
            + super::bounded_fll_pull_in_correction_hz(30.0 * fll_coefficients.error_blend, 10.0)
            + pll_coefficients.frequency_gain_hz_per_rad * 0.25;
        assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn should_apply_fll_keeps_frequency_assistance_until_fll_relocks() {
        assert!(super::should_apply_fll(super::ChannelState::PullIn, true));
        assert!(super::should_apply_fll(super::ChannelState::Tracking, false));
        assert!(!super::should_apply_fll(super::ChannelState::Tracking, true));
    }

    #[test]
    fn apply_carrier_loop_uses_coherent_interval_to_scale_frequency_gain() {
        let short = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
        });
        let long = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_000.0,
            current_carrier_phase_cycles: 12.0,
            epoch_len_samples: 40_920,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.010,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.25,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
        });

        assert!(
            (long.carrier_hz - 1_000.0).abs() > (short.carrier_hz - 1_000.0).abs(),
            "short={short:?} long={long:?}"
        );
    }

    #[test]
    fn dll_pull_in_increases_code_rate_for_faster_signal() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let samples_per_epoch = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
        let code_phase_chips = 211.625;
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 14 };
        let code_phase_samples =
            code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
        let samples = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            signal_code_rate_hz,
            code_phase_chips,
            samples_per_epoch,
        )
        .expect("valid sampled code with faster signal code rate");
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
        );

        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            0.5,
        );
        let (dll_err, _, _, _) =
            discriminators(correlator.early, correlator.prompt, correlator.late, None);
        let code_loop = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: config.code_freq_basis_hz,
            current_code_phase_samples: code_phase_samples,
            epoch_len_samples: samples_per_epoch,
            coherent_integration_s: samples_per_epoch as f64 / config.sampling_freq_hz,
            nominal_code_rate_hz: config.code_freq_basis_hz,
            dll_bw_hz: 900.0,
            dll_err,
            samples_per_chip: samples_per_epoch as f64 / config.code_length as f64,
            samples_per_code: samples_per_epoch,
        });

        assert!(
            code_loop.code_rate_hz > config.code_freq_basis_hz,
            "dll_err={dll_err} code_loop={code_loop:?}",
        );
    }

    #[test]
    fn detect_sample_rate_mismatch_flags_persistent_phase_drift() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 24.0, 48.0, 72.0, 96.0]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_000,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                dll_err: 0.35,
                cn0_dbhz: 46.0,
                lock: true,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: ChannelState::Tracking.to_string(),
                lock_state_reason: Some("locked".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        let diagnostic =
            super::detect_sample_rate_mismatch(&epochs, 5_000).expect("diagnostic must exist");
        assert_eq!(diagnostic.first_unstable_epoch_index, 1);
        assert!(diagnostic.max_abs_phase_step_samples >= 24.0);
    }

    #[test]
    fn detect_sample_rate_mismatch_ignores_small_stable_phase_steps() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 1.5, 2.0, 1.0, 0.5]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_000,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                dll_err: 0.05,
                cn0_dbhz: 46.0,
                lock: true,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                lock_state: ChannelState::Tracking.to_string(),
                lock_state_reason: Some("locked".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        assert!(super::detect_sample_rate_mismatch(&epochs, 5_000).is_none());
    }

    #[test]
    fn incremental_tracking_matches_single_frame_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 4,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let runtime = crate::engine::runtime::ReceiverRuntime::default();
        let tracking = Tracking::new(config.clone(), runtime.clone());
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
        let duration_s = 0.012;
        let frame = crate::sim::synthetic::generate_l1_ca(
            &config,
            crate::sim::synthetic::SyntheticSignalParams {
                sat,
                doppler_hz: 1_000.0,
                code_phase_chips: 10.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                data_bit_flip: false,
            },
            7,
            duration_s,
        );
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(1_000.0),
            carrier_hz: bijux_gnss_core::api::Hertz(1_000.0),
            code_phase_samples: 10,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 48.0,
            score: 0.98,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            uncertainty: None,
        };

        let single = tracking.track_from_acquisition(&frame, &[acquisition.clone()]);
        let mut incremental = tracking.begin_incremental_tracking(&[acquisition]);
        for chunk in split_frame(&frame, 4 * 1_023) {
            tracking.track_incremental_frame(&mut incremental, &chunk);
        }
        let streamed = tracking.finish_incremental_tracking(incremental);

        assert_eq!(single.len(), streamed.len());
        assert_eq!(single[0].epochs.len(), streamed[0].epochs.len());
        let single_keys = single[0]
            .epochs
            .iter()
            .map(|epoch| {
                (
                    epoch.epoch.index,
                    epoch.sample_index,
                    epoch.lock,
                    epoch.lock_state.clone(),
                    epoch.lock_state_reason.clone(),
                )
            })
            .collect::<Vec<_>>();
        let streamed_keys = streamed[0]
            .epochs
            .iter()
            .map(|epoch| {
                (
                    epoch.epoch.index,
                    epoch.sample_index,
                    epoch.lock,
                    epoch.lock_state.clone(),
                    epoch.lock_state_reason.clone(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(single_keys, streamed_keys);
    }

    #[derive(Debug, Deserialize)]
    struct TrackingEventFixture {
        lock: bool,
        anti_false_lock: bool,
        cycle_slip: bool,
    }

    #[derive(Debug, Deserialize)]
    struct TrackingScenarioFixture {
        id: String,
        initial_state: String,
        events: Vec<TrackingEventFixture>,
        expected_final_state: String,
    }

    #[test]
    fn tracking_scenario_fixtures_are_deterministic() {
        for (fixture_name, fixture_raw) in tracking_fixture_specs() {
            let fixture = load_tracking_fixture(fixture_raw, fixture_name);
            let mut state = parse_state(&fixture.initial_state);
            let mut unlocked = 0u8;
            for event in &fixture.events {
                let decision = super::deterministic_transition_rule(
                    state,
                    event.lock,
                    event.lock,
                    event.anti_false_lock,
                    event.cycle_slip,
                    unlocked,
                );
                state = decision.to_state;
                unlocked = decision.next_unlocked_count;
            }
            assert_eq!(
                state,
                parse_state(&fixture.expected_final_state),
                "fixture {} final state mismatch",
                fixture.id
            );
        }
    }

    #[test]
    #[cfg(feature = "alloc-audit")]
    fn tracking_allocations_under_threshold() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let runtime = crate::engine::runtime::ReceiverRuntime::default();
        let samples_per_code = bijux_gnss_signal::api::samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let samples = vec![num_complex::Complex::new(0.0, 0.0); samples_per_code];
        let tracking = Tracking::new(config, runtime);

        let before = crate::engine::alloc::allocation_count();
        let _ = tracking.run(&samples);
        let after = crate::engine::alloc::allocation_count();

        let allocated = after.saturating_sub(before);
        let threshold = 200;
        assert!(
            allocated <= threshold,
            "tracking allocations exceeded threshold: {allocated} > {threshold}"
        );
    }

    fn tracking_fixture_specs() -> Vec<(&'static str, &'static str)> {
        vec![
            (
                "interference_like.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/interference_like.json"
                )),
            ),
            (
                "lock.json",
                include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tracking/lock.json")),
            ),
            (
                "relock.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/relock.json"
                )),
            ),
            (
                "weak_signal.json",
                include_str!(concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/tests/data/tracking/weak_signal.json"
                )),
            ),
        ]
    }

    fn split_frame(frame: &SamplesFrame, chunk_len: usize) -> Vec<SamplesFrame> {
        let mut chunks = Vec::new();
        let mut start = 0usize;
        while start < frame.len() {
            let end = (start + chunk_len.max(1)).min(frame.len());
            chunks.push(SamplesFrame::new(
                SampleTime {
                    sample_index: frame.t0.sample_index + start as u64,
                    sample_rate_hz: frame.t0.sample_rate_hz,
                },
                Seconds(frame.dt_s.0),
                frame.iq[start..end].to_vec(),
            ));
            start = end;
        }
        chunks
    }

    fn load_tracking_fixture(raw: &str, fixture_name: &str) -> TrackingScenarioFixture {
        serde_json::from_str(raw)
            .unwrap_or_else(|err| panic!("parse tracking fixture {fixture_name}: {err}"))
    }

    fn parse_state(value: &str) -> ChannelState {
        match value {
            "Idle" => ChannelState::Idle,
            "Acquired" => ChannelState::Acquired,
            "PullIn" => ChannelState::PullIn,
            "Tracking" => ChannelState::Tracking,
            "Lost" => ChannelState::Lost,
            _ => panic!("unsupported state {value}"),
        }
    }
}
