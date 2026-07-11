#![allow(missing_docs)]

use std::collections::VecDeque;

use num_complex::Complex;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqHypothesis, AcqTrackingSeed, AcqUncertainty, Chips, Constellation, Cycles, Hertz,
    ReceiverSampleTrace, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
    SignalDelayAlignment, TrackEpoch, TrackTransition, TrackingAssumptions, TrackingUncertainty,
};

use crate::engine::receiver_config::{ReceiverPipelineConfig, TrackingParams};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::pipeline::doppler::carrier_hz_from_doppler_hz;
use bijux_gnss_core::api::Sample;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::{
    adaptive_bandwidth, carrier_frequency_error_hz_from_phase_delta, discriminators,
    estimate_cn0_dbhz, first_order_angular_loop_coefficients, first_order_loop_coefficients,
    phase_lock_loop_coefficients,
};
use bijux_gnss_signal::api::{generate_ca_code, Prn};

const DLL_CODE_PHASE_CORRECTION_GAIN: f64 = 0.5;
const SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ: f64 = 18.0;
const TRACKING_LOCK_MIN_CN0_DBHZ: f64 = 28.0;
const TRACKING_LOCK_REFUSAL_EPOCHS: u8 = 3;
const SHORT_FADE_MAX_DURATION_S: f64 = 0.100;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_FRACTION: f64 = 0.0025;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_SAMPLES: f64 = 12.0;
const SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS: usize = 4;
const SAMPLE_RATE_MISMATCH_MIN_UNSTABLE_EPOCHS_IN_WINDOW: usize = 3;
const CYCLE_SLIP_PHASE_DELTA_CYCLES: f64 = 0.35;
const NAV_BIT_PHASE_STEP_CYCLES: f64 = 0.5;
// Pull-in can still carry a few extra tenths of cycle of residual phase error
// when the first 20 ms nav-bit edge arrives on a strong high-Doppler signal.
// Keep the detector wide enough to classify that transition correctly instead
// of escalating it into a false cycle slip.
const NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES: f64 = 0.2;
// Require a meaningful continuity improvement before rewriting a phase jump as
// a nav-bit edge. This keeps smaller non-nav slips from being mistaken for
// half-cycle data-bit transitions.
const NAV_BIT_PHASE_MIN_IMPROVEMENT_CYCLES: f64 = 0.25;
const DLL_LOCK_MAX_CODE_ERROR: f32 = 0.2;
const DLL_HOLD_MAX_CODE_ERROR: f32 = 0.4;
// One-sample-per-chip tracking quantizes the early/late discriminator enough
// that clean pull-in can sit slightly above the nominal fine-resolution lock
// bound without indicating a real code-tracking failure.
const DLL_LOW_RESOLUTION_LOCK_MAX_CODE_ERROR: f32 = 0.6;
const PLL_LOCK_MAX_PHASE_ERROR_RAD: f32 = 0.35;
const PLL_HOLD_MAX_PHASE_ERROR_RAD: f32 = 0.45;
const ANTI_FALSE_LOCK_MAX_EARLY_LATE_TO_PROMPT_RATIO: f32 = 0.9;
const PROMPT_POWER_DROP_RATIO_THRESHOLD: f32 = 0.2;
const DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO: f32 = 0.5;
const DISCRIMINATOR_INSTABILITY_REQUIRED_EPOCHS: u8 = 2;
const DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS: u16 = 2;
const SHORT_FADE_RECOVERY_GRACE_EPOCHS: u16 = 5;
// If prompt energy returns near the short-fade boundary, the loops may need a
// few extra epochs to re-enter tracking without opening a long interruption gap.
const SHORT_FADE_RELOCK_EVIDENCE_GRACE_EPOCHS: u16 = 3;
// Enter steady tracking only after the carrier/code loops stay jointly locked
// across a short sustained window instead of a single optimistic epoch.
const PULL_IN_REQUIRED_STABLE_EPOCHS: u8 = 3;
const FLL_PULL_IN_MAX_CORRECTION_BW_MULTIPLIER: f64 = 4.0;
const REACQUISITION_REQUIRED_LOST_EPOCHS: usize = 3;
const REACQUISITION_CONFIRMATION_EPOCHS: u8 = 2;
const REACQUISITION_PULL_IN_EPOCH_BUDGET: u8 = 20;
const REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ: f64 = 12.0;
const REACQUISITION_STABLE_TRACKING_EPOCHS: u8 = 5;
const TRACKING_CN0_WINDOW_EPOCHS: usize = 8;
const TRACKING_CN0_MIN_WINDOW_EPOCHS: usize = 4;
const TRACKING_UNCERTAINTY_WINDOW_EPOCHS: usize = 8;
const TRACKING_UNCERTAINTY_MIN_CODE_PHASE_SAMPLES: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CARRIER_PHASE_CYCLES: f64 = 0.001;
const TRACKING_UNCERTAINTY_MIN_DOPPLER_HZ: f64 = 0.01;
const TRACKING_UNCERTAINTY_MIN_CN0_DBHZ: f64 = 0.05;
const SAMPLE_RATE_MISMATCH_CATASTROPHIC_PHASE_STEP_MULTIPLIER: f64 = 8.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    Idle,
    Acquired,
    PullIn,
    Tracking,
    Degraded,
    Lost,
}

impl std::fmt::Display for ChannelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Idle => "idle",
            Self::Acquired => "acquired",
            Self::PullIn => "pull_in",
            Self::Tracking => "tracking",
            Self::Degraded => "degraded",
            Self::Lost => "lost",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingChannelState {
    Acquired,
    PullIn,
    Locked,
    Degraded,
    Lost,
    Reacquired,
    Refused,
}

impl std::fmt::Display for TrackingChannelState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Acquired => "acquired",
            Self::PullIn => "pull_in",
            Self::Locked => "locked",
            Self::Degraded => "degraded",
            Self::Lost => "lost",
            Self::Reacquired => "reacquired",
            Self::Refused => "refused",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackingChannelStateEvent {
    pub state: TrackingChannelState,
    pub epoch_idx: u64,
    pub sample_index: u64,
    pub reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrackingChannelStateReport {
    pub sat: SatId,
    pub channel_id: u8,
    pub channel_uid: String,
    pub final_state: TrackingChannelState,
    pub final_reason: Option<String>,
    pub emitted_states: Vec<TrackingChannelStateEvent>,
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

#[derive(Debug, Default, Clone)]
pub struct TrackingArtifacts {
    pub processed_input_samples: u64,
    pub processed_input_epochs: u64,
    pub track_transitions: Vec<TrackTransition>,
    pub channel_state_reports: Vec<TrackingChannelStateReport>,
    pub tracking: Vec<TrackingResult>,
}

#[derive(Debug, Clone)]
pub struct TrackingSession {
    tracking: IncrementalTrackingState,
    processed_input_samples: u64,
    processed_input_epochs: u64,
}

#[derive(Debug, Clone)]
struct LoopState {
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    signal_delay_alignment: Option<SignalDelayAlignment>,
    acquisition_cn0_proxy_dbhz: f64,
    lock_reference_cn0_dbhz: f64,
    prev_prompt: Option<Complex<f32>>,
    prev_prompt_phase_cycles: Option<f64>,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition_count: u32,
    pull_in_stable_epochs: u8,
    weak_cn0_epochs: u8,
    degraded_epochs: u16,
    prompt_power_reference: f32,
    prompt_cn0_window: VecDeque<f64>,
    code_error_window_samples: VecDeque<f64>,
    carrier_phase_error_window_cycles: VecDeque<f64>,
    doppler_error_window_hz: VecDeque<f64>,
    cn0_estimate_window_dbhz: VecDeque<f64>,
    unstable_discriminator_epochs: u8,
    state: ChannelState,
    unlocked_count: u8,
    lost_reason: Option<String>,
    reacquisition_candidate: Option<ReacquisitionSeed>,
    reacquisition_candidate_streak: u8,
    reacquisition_pending: bool,
    reacquisition_attempt_epochs: u8,
    reacquisition_stable_tracking_epochs: u8,
}

#[derive(Debug, Clone)]
pub(crate) struct IncrementalTrackingState {
    channels: Vec<IncrementalTrackingChannel>,
}

#[derive(Debug, Clone)]
struct TrackingStartContext {
    seed: AcqTrackingSeed,
    acquisition_hypothesis: String,
    acquisition_hypothesis_rank: u8,
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
struct TrackingUncertaintyInputs {
    samples_per_chip: f64,
    dll_err: f32,
    pll_err_rad: f64,
    fll_err_hz: f64,
    cn0_dbhz: f64,
    cn0_reference_dbhz: f64,
    integration_ms: u32,
    channel_locked: bool,
    dll_locked: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
    channel_state: ChannelState,
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
    apply_pll_frequency: bool,
}

#[derive(Debug, Clone, Copy)]
struct PromptPhaseDecision {
    aligned_phase_cycles: f64,
    aligned_phase_delta_cycles: f64,
    nav_bit_phase_offset_cycles: f64,
    nav_bit_transition: bool,
    cycle_slip: bool,
}

#[derive(Debug, Clone, Copy)]
struct ReacquisitionSeed {
    carrier_hz: f64,
    code_phase_samples: f64,
    cn0_dbhz: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SustainedLockLossSeed {
    carrier_hz: f64,
    code_phase_samples: f64,
    code_rate_hz: f64,
    sample_index: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReacquisitionOutcome {
    NotNeeded,
    SeedUnavailable,
    Failed,
    Started,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LossOfLockCause {
    PromptPowerDrop,
    DiscriminatorInstability,
    PhaseJump,
}

impl LossOfLockCause {
    fn reason(self) -> &'static str {
        match self {
            Self::PromptPowerDrop => "prompt_power_drop",
            Self::DiscriminatorInstability => "discriminator_instability",
            Self::PhaseJump => "phase_jump",
        }
    }
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
            acquisition_hypothesis_rank: acquisition_hypothesis_rank(acquisition.hypothesis),
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
        let code = ca_code_or_default(sat.prn);
        self.correlate_epoch_range_with_code(
            frame,
            0,
            frame.len(),
            &code,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        )
    }

    fn correlate_epoch_range_with_code(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        code: &[i8],
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> CorrelatorOutput {
        let sample_rate_hz = self.config.sampling_freq_hz;
        let samples = &frame.iq[start..end];
        let n = samples.len();
        let code_period_samples = samples_per_code(
            sample_rate_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let nominal_chips_per_sample = self.config.code_freq_basis_hz / sample_rate_hz;
        let tracked_chips_per_sample = code_rate_hz / sample_rate_hz;
        let epoch_start_code_phase_samples =
            epoch_start_code_phase_samples_from_receiver_phase(code_phase_samples, code_period_samples);
        let base_chip_phase = epoch_start_code_phase_samples * nominal_chips_per_sample;
        let carrier_phase_offset_rad = carrier_phase_offset_rad(carrier_phase_cycles);
        let carrier_radians_per_sample = std::f64::consts::TAU * carrier_freq_hz / sample_rate_hz;
        let carrier_rotation_step = Complex::new(
            carrier_radians_per_sample.cos() as f32,
            -carrier_radians_per_sample.sin() as f32,
        );
        let mut carrier_rotation = Complex::new(
            carrier_phase_offset_rad.cos() as f32,
            -carrier_phase_offset_rad.sin() as f32,
        );
        let mut early = Complex::new(0.0f32, 0.0f32);
        let mut prompt = Complex::new(0.0f32, 0.0f32);
        let mut late = Complex::new(0.0f32, 0.0f32);
        let mut early_late_noise_weight_energy = 0.0f64;

        for (i, sample) in samples.iter().take(n).enumerate() {
            let mixed_sample = *sample * carrier_rotation;
            let chip_phase = base_chip_phase + i as f64 * tracked_chips_per_sample;
            let early_code = Complex::new(
                code_value_at_tracking_phase(code, chip_phase - early_late_spacing_chips),
                0.0,
            );
            let prompt_code = Complex::new(code_value_at_tracking_phase(code, chip_phase), 0.0);
            let late_code = Complex::new(
                code_value_at_tracking_phase(code, chip_phase + early_late_spacing_chips),
                0.0,
            );
            let noise_weight = early_code - late_code;
            early_late_noise_weight_energy += noise_weight.norm_sqr() as f64;

            early += mixed_sample * early_code;
            prompt += mixed_sample * prompt_code;
            late += mixed_sample * late_code;
            carrier_rotation *= carrier_rotation_step;
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
        let code = ca_code_or_default(sat.prn);
        self.track_epoch_range_with_code(
            frame,
            0,
            frame.len(),
            channel_id,
            sat,
            &code,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        )
    }

    fn track_epoch_range_with_code(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        channel_id: u8,
        sat: SatId,
        code: &[i8],
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (TrackEpoch, CorrelatorOutput) {
        let sample_index = frame.t0.sample_index + start as u64;
        let source_time = SampleTime { sample_index, sample_rate_hz: frame.t0.sample_rate_hz };
        let clock = SampleClock::new(self.config.sampling_freq_hz);
        let epoch = clock.epoch_from_samples(sample_index);
        let correlator = self.correlate_epoch_range_with_code(
            frame,
            start,
            end,
            code,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        );
        let coherent_samples = end.saturating_sub(start);
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
            sample_index,
            source_time: ReceiverSampleTrace::from_sample_time(source_time),
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
            navigation_bit_sign: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: ChannelState::Idle.to_string(),
            lock_state_reason: Some("initializing".to_string()),
            channel_id: Some(channel_id),
            channel_uid: tracking_channel_uid(sat, channel_id),
            tracking_provenance: format!(
                "channel={} sat={:?}-{}",
                channel_id, sat.constellation, sat.prn
            ),
            tracking_assumptions: Some(default_tracking_assumptions(&self.config)),
            tracking_uncertainty: None,
            signal_delay_alignment: None,
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
        let mut session = self.begin_tracking_session(acquisitions);
        self.track_session_frame(&mut session, frame);
        self.finish_tracking_session(session).tracking
    }

    pub fn begin_tracking_session(
        &self,
        acquisitions: &[bijux_gnss_core::api::AcqResult],
    ) -> TrackingSession {
        TrackingSession {
            tracking: self.begin_incremental_tracking(acquisitions),
            processed_input_samples: 0,
            processed_input_epochs: 0,
        }
    }

    pub fn track_session_frame(&self, session: &mut TrackingSession, frame: &SamplesFrame) {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        session.processed_input_samples += frame.len() as u64;
        session.processed_input_epochs += code_periods_in_frame(frame.len(), samples_per_code);
        self.track_incremental_frame(&mut session.tracking, frame);
    }

    pub fn finish_tracking_session(&self, session: TrackingSession) -> TrackingArtifacts {
        let tracking = self.finish_incremental_tracking(session.tracking);
        let track_transitions =
            tracking.iter().flat_map(|result| result.transitions.iter().cloned()).collect();
        let channel_state_reports =
            tracking.iter().map(tracking_channel_state_report).collect::<Vec<_>>();
        TrackingArtifacts {
            processed_input_samples: session.processed_input_samples,
            processed_input_epochs: session.processed_input_epochs,
            track_transitions,
            channel_state_reports,
            tracking,
        }
    }

    fn apply_sample_rate_mismatch_diagnostic(
        &self,
        sat: SatId,
        acquisition_uncertainty: Option<&AcqUncertainty>,
        epochs: &mut [TrackEpoch],
    ) {
        if let Some(acquisition_uncertainty) = acquisition_uncertainty {
            if acquisition_uncertainty.code_phase_samples > 0.5 + f64::EPSILON {
                return;
            }
            if acquisition_uncertainty.doppler_hz
                > self.config.acquisition_doppler_step_hz.max(1) as f64 + f64::EPSILON
            {
                return;
            }
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
            let replaceable_reason = epoch.lock_state_reason.as_deref().is_none_or(|reason| {
                matches!(
                    reason,
                    "carrier_pull_in"
                        | "carrier_converged"
                        | "fade_recovered"
                        | "signal_fade"
                )
            });
            if replaceable_reason {
                epoch.lock_state_reason = Some("sample_rate_mismatch".to_string());
            }
        }
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
            signal_delay_alignment: None,
            lock_reference_cn0_dbhz: acquisition_cn0_proxy_dbhz,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            degraded_epochs: 0,
            prompt_power_reference: 0.0,
            prompt_cn0_window: VecDeque::new(),
            code_error_window_samples: VecDeque::new(),
            carrier_phase_error_window_cycles: VecDeque::new(),
            doppler_error_window_hz: VecDeque::new(),
            cn0_estimate_window_dbhz: VecDeque::new(),
            unstable_discriminator_epochs: 0,
            state: ChannelState::Acquired,
            unlocked_count: 0,
            lost_reason: None,
            reacquisition_candidate: None,
            reacquisition_candidate_streak: 0,
            reacquisition_pending: false,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
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
        let mut start_contexts = acquisitions
            .iter()
            .filter_map(|acq| self.tracking_start_context(acq))
            .collect::<Vec<_>>();
        start_contexts.sort_by(compare_tracking_start_contexts);

        let channels = start_contexts
            .into_iter()
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
                        signal_delay_alignment: context.seed.signal_delay_alignment.clone(),
                        acquisition_cn0_proxy_dbhz: context.acquisition_cn0_proxy_dbhz,
                        lock_reference_cn0_dbhz: context.acquisition_cn0_proxy_dbhz,
                        prev_prompt: None,
                        prev_prompt_phase_cycles: None,
                        nav_bit_phase_offset_cycles: 0.0,
                        nav_bit_transition_count: 0,
                        pull_in_stable_epochs: 0,
                        weak_cn0_epochs: 0,
                        degraded_epochs: 0,
                        prompt_power_reference: 0.0,
                        prompt_cn0_window: VecDeque::new(),
                        code_error_window_samples: VecDeque::new(),
                        carrier_phase_error_window_cycles: VecDeque::new(),
                        doppler_error_window_hz: VecDeque::new(),
                        cn0_estimate_window_dbhz: VecDeque::new(),
                        unstable_discriminator_epochs: 0,
                        state: ChannelState::Acquired,
                        unlocked_count: 0,
                        lost_reason: None,
                        reacquisition_candidate: None,
                        reacquisition_candidate_streak: 0,
                        reacquisition_pending: false,
                        reacquisition_attempt_epochs: 0,
                        reacquisition_stable_tracking_epochs: 0,
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
            let code = ca_code_or_default(channel.sat.prn);
            let Some(channel_frame_start) = tracking_frame_start_offset(
                frame,
                if channel.epochs.is_empty() {
                    Some(channel.start_source_time.sample_index)
                } else {
                    None
                },
            ) else {
                continue;
            };
            let samples_per_epoch = tracking_epoch_samples(
                self.config.sampling_freq_hz,
                self.config.code_freq_basis_hz,
                self.config.code_length,
                channel.tracking_params,
            );
            let mut epoch_start = channel_frame_start;
            while epoch_start < frame.len() {
                let epoch_end = (epoch_start + samples_per_epoch).min(frame.len());
                let reacquisition_outcome = if channel.state.state == ChannelState::Lost {
                    let epoch_frame = frame_slice(frame, epoch_start, epoch_end);
                    self.try_reacquire_channel(channel, &epoch_frame)
                } else {
                    ReacquisitionOutcome::NotNeeded
                };
                let epoch_count_before = channel.epochs.len();
                let transition_count_before = channel.transitions.len();
                self.append_tracked_epoch_range(
                    frame,
                    epoch_start,
                    epoch_end,
                    channel.channel_id,
                    channel.sat,
                    &code,
                    channel.tracking_params,
                    &mut channel.state,
                    &mut channel.epochs,
                    &mut channel.transitions,
                );
                apply_reacquisition_annotations(
                    &mut channel.state,
                    &mut channel.epochs[epoch_count_before..],
                    &mut channel.transitions[transition_count_before..],
                    reacquisition_outcome,
                );
                epoch_start = epoch_end;
            }
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
                annotate_navigation_bit_signs(channel.sat, &mut channel.epochs);
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
        let code = ca_code_or_default(sat.prn);
        let samples_per_epoch = tracking_epoch_samples(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
            tracking_params,
        );
        for epoch_idx in 0..epochs {
            let start = epoch_idx * samples_per_epoch;
            let end = (start + samples_per_epoch).min(frame.len());
            if start >= end {
                break;
            }
            self.append_tracked_epoch_range(
                frame,
                start,
                end,
                channel_id,
                sat,
                &code,
                tracking_params,
                state,
                out,
                transitions,
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn append_tracked_epoch_range(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        channel_id: u8,
        sat: SatId,
        code: &[i8],
        tracking_params: TrackingParams,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let samples_per_chip = samples_per_code as f64 / self.config.code_length as f64;
        let alloc_before = crate::engine::alloc::allocation_count();
        let (mut track_epoch, corr) = self.track_epoch_range_with_code(
            frame,
            start,
            end,
            channel_id,
            sat,
            code,
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
        let cycle_slip_reason = cycle_slip.then(|| LossOfLockCause::PhaseJump.reason().to_string());
        let prompt_power = corr.prompt.norm();

        let mut cn0_dbhz = state.acquisition_cn0_proxy_dbhz;
        if !cn0_dbhz.is_finite() || cn0_dbhz <= 0.0 {
            cn0_dbhz = track_epoch.cn0_dbhz;
        }
        let epoch_len_samples = end - start;
        if let Some(windowed_cn0_dbhz) =
            update_windowed_tracking_cn0_estimate(&mut state.prompt_cn0_window, cn0_dbhz)
        {
            cn0_dbhz = windowed_cn0_dbhz;
        }
        track_epoch.cn0_dbhz = cn0_dbhz;
        let (dll_bw, pll_bw, fll_bw) = adaptive_bandwidth(
            tracking_params.dll_bw_hz,
            tracking_params.pll_bw_hz,
            tracking_params.fll_bw_hz,
            cn0_dbhz,
        );
        let coherent_integration_s =
            coherent_integration_seconds(epoch_len_samples, self.config.sampling_freq_hz);
        let raw_fll_err_hz =
            carrier_frequency_error_hz_from_phase_delta(raw_fll_err as f64, coherent_integration_s);
        let nav_bit_aware_fll_err_hz = carrier_frequency_error_hz_from_phase_delta(
            phase_decision.aligned_phase_delta_cycles * std::f64::consts::TAU,
            coherent_integration_s,
        );
        let use_nav_bit_aware_fll = phase_decision.nav_bit_transition
            || state.nav_bit_phase_offset_cycles.abs() > f64::EPSILON
            || phase_decision.nav_bit_phase_offset_cycles.abs() > f64::EPSILON;
        let fll_err_hz =
            if use_nav_bit_aware_fll { nav_bit_aware_fll_err_hz } else { raw_fll_err_hz } as f32;
        let from_state = state.state;
        let raw_dll_lock =
            dll_err.abs() < dll_lock_threshold(samples_per_chip, tracking_params.early_late_spacing_chips);
        let fll_enabled = fll_bw > f64::EPSILON;
        let raw_pll_lock = pll_err.abs() < PLL_LOCK_MAX_PHASE_ERROR_RAD;
        let raw_fll_lock =
            !fll_enabled || (fll_err_hz as f64).abs() <= fll_lock_threshold_hz(fll_bw);
        let sustained_dll_lock = raw_dll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && dll_err.abs()
                < dll_hold_threshold(samples_per_chip, tracking_params.early_late_spacing_chips));
        let sustained_pll_lock = raw_pll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && pll_err.abs() < PLL_HOLD_MAX_PHASE_ERROR_RAD);
        let anti_false_lock =
            anti_false_lock_detected(corr.early, corr.prompt, corr.late) && !sustained_pll_lock;
        state.prompt_power_reference = refresh_prompt_power_reference(
            state.prompt_power_reference,
            prompt_power,
            state.state,
            anti_false_lock,
        );
        let prompt_power_ratio = prompt_power_ratio(prompt_power, state.prompt_power_reference);
        let sustained_prompt_lock = lock
            || (matches!(
                state.state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            )
                && prompt_power_ratio.is_some_and(|ratio| {
                    ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO
                })
                && !cycle_slip
                && !anti_false_lock);
        state.unstable_discriminator_epochs = update_discriminator_instability_epochs(
            state.unstable_discriminator_epochs,
            state.state,
            prompt_power_ratio,
            raw_pll_lock,
            raw_fll_lock,
            cycle_slip,
            anti_false_lock,
        );
        let loss_of_lock_cause = classify_loss_of_lock_cause(
            state.state,
            cycle_slip,
            prompt_power_ratio,
            state.unstable_discriminator_epochs,
        );
        state.pull_in_stable_epochs = update_pull_in_stable_epochs(
            state.pull_in_stable_epochs,
            sustained_prompt_lock,
            sustained_dll_lock,
            sustained_pll_lock,
            raw_fll_lock,
            cycle_slip,
        );
        let (weak_cn0_epochs, cn0_supports_lock, refuse_lock) = update_prelock_cn0_refusal(
            state.state,
            state.weak_cn0_epochs,
            state.acquisition_cn0_proxy_dbhz,
        );
        state.weak_cn0_epochs = weak_cn0_epochs;
        let steady_state_tracking_ready = if from_state == ChannelState::Degraded {
            cn0_supports_lock
                && lock
                && sustained_dll_lock
                && raw_pll_lock
                && raw_fll_lock
                && !cycle_slip
                && !anti_false_lock
        } else {
            cn0_supports_lock
                && sustained_prompt_lock
                && sustained_dll_lock
                && sustained_pll_lock
                && !cycle_slip
                && !anti_false_lock
        };
        let ready_for_tracking =
            if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded) {
                steady_state_tracking_ready
            } else {
                cn0_supports_lock && state.pull_in_stable_epochs >= PULL_IN_REQUIRED_STABLE_EPOCHS
            };
        let short_fade_relock_evidence =
            !anti_false_lock && !cycle_slip && (raw_pll_lock || raw_fll_lock);
        let reliable_reacquisition_reference = cn0_supports_lock
            && sustained_prompt_lock
            && raw_pll_lock
            && raw_fll_lock
            && !cycle_slip
            && !anti_false_lock;
        state.lock_reference_cn0_dbhz = refresh_lock_reference_cn0_dbhz(
            state.lock_reference_cn0_dbhz,
            cn0_dbhz,
            reliable_reacquisition_reference,
        );

        let transition = if refuse_lock {
            TransitionDecision {
                to_state: ChannelState::PullIn,
                reason: "cn0_below_tracking_lock_floor".to_string(),
                next_unlocked_count: state.unlocked_count.saturating_add(1),
                next_degraded_epochs: 0,
            }
        } else {
            deterministic_transition_rule(
                from_state,
                sustained_prompt_lock,
                ready_for_tracking,
                anti_false_lock,
                loss_of_lock_cause,
                state.unlocked_count,
                state.degraded_epochs,
                short_fade_epoch_budget(tracking_params),
                short_fade_relock_evidence,
            )
        };
        state.unlocked_count = transition.next_unlocked_count;
        state.degraded_epochs = transition.next_degraded_epochs;
        state.state = transition.to_state;
        push_tracking_uncertainty_sample(
            &mut state.code_error_window_samples,
            (dll_err.abs() as f64) * samples_per_chip,
        );
        push_tracking_uncertainty_sample(
            &mut state.carrier_phase_error_window_cycles,
            (pll_err.abs() as f64) / std::f64::consts::TAU,
        );
        push_tracking_uncertainty_sample(
            &mut state.doppler_error_window_hz,
            (fll_err_hz as f64).abs(),
        );
        push_tracking_uncertainty_sample(&mut state.cn0_estimate_window_dbhz, cn0_dbhz);
        let tracking_uncertainty = Some(estimate_tracking_uncertainty(
            state,
            TrackingUncertaintyInputs {
                samples_per_chip,
                dll_err,
                pll_err_rad: pll_err as f64,
                fll_err_hz: fll_err_hz as f64,
                cn0_dbhz,
                cn0_reference_dbhz: state.lock_reference_cn0_dbhz,
                integration_ms: tracking_params.integration_ms,
                channel_locked: state.state != ChannelState::Lost && sustained_prompt_lock,
                dll_locked: state.state != ChannelState::Lost && sustained_dll_lock,
                anti_false_lock,
                cycle_slip,
                channel_state: state.state,
            },
        ));
        if state.state == ChannelState::Lost {
            state.reacquisition_pending = false;
            state.reacquisition_attempt_epochs = 0;
            clear_tracking_uncertainty_windows(state);
            if from_state != ChannelState::Lost {
                state.lost_reason = Some(transition.reason.clone());
                state.reacquisition_candidate = None;
                state.reacquisition_candidate_streak = 0;
                state.reacquisition_stable_tracking_epochs = 0;
            }
        } else {
            state.lost_reason = None;
            state.reacquisition_candidate = None;
            state.reacquisition_candidate_streak = 0;
            if !state.reacquisition_pending {
                state.reacquisition_stable_tracking_epochs = 0;
            }
            if !state.reacquisition_pending {
                state.reacquisition_attempt_epochs = 0;
            }
            if !matches!(state.state, ChannelState::Tracking | ChannelState::Degraded) {
                clear_tracking_uncertainty_windows(state);
            }
        }

        let lock_state = match state.state {
            ChannelState::Idle => "idle".to_string(),
            ChannelState::Acquired => "acquired".to_string(),
            ChannelState::PullIn => "pull_in".to_string(),
            ChannelState::Tracking => "tracking".to_string(),
            ChannelState::Degraded => "degraded".to_string(),
            ChannelState::Lost => "lost".to_string(),
        };
        let lock_state_reason = Some(transition.reason.clone());
        let channel_locked = state.state != ChannelState::Lost && sustained_prompt_lock;
        let tracking_state_locked =
            matches!(state.state, ChannelState::Tracking | ChannelState::Degraded);
        let dll_lock = tracking_state_locked && sustained_dll_lock;
        let pll_lock = tracking_state_locked
            && cn0_supports_lock
            && sustained_prompt_lock
            && sustained_pll_lock
            && !cycle_slip;
        let fll_lock = state.state != ChannelState::Lost
            && (raw_fll_lock || (tracking_state_locked && pll_lock));
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
            epoch_len_samples,
            coherent_integration_s,
            nominal_code_rate_hz: self.config.code_freq_basis_hz,
            dll_bw_hz: dll_bw,
            dll_err,
            samples_per_chip,
            samples_per_code,
        });
        state.code_rate_hz = code_loop.code_rate_hz;
        let apply_fll = fll_bw > 0.0 && should_apply_fll(state.state, raw_fll_lock);
        let carrier_loop = apply_carrier_loop(CarrierLoopInput {
            current_carrier_hz: state.carrier_hz,
            current_carrier_phase_cycles: state.carrier_phase_cycles,
            epoch_len_samples,
            sample_rate_hz: self.config.sampling_freq_hz,
            coherent_integration_s,
            pll_bw_hz: pll_bw,
            pll_err_rad: pll_err as f64,
            fll_bw_hz: fll_bw,
            fll_err_hz: fll_err_hz as f64,
            apply_fll,
            apply_pll_frequency: !apply_fll
                || (matches!(state.state, ChannelState::PullIn) && !raw_fll_lock),
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
                reason: lock_state_reason.clone().unwrap_or_else(|| "state_transition".to_string()),
                lock_quality: epoch_lock_quality(lock, pll_lock, dll_lock, fll_lock, cn0_dbhz),
            });
        }

        out.push(TrackEpoch {
            lock: channel_locked,
            carrier_hz: Hertz(state.carrier_hz),
            carrier_phase_cycles: Cycles(state.carrier_phase_cycles),
            code_rate_hz: Hertz(state.code_rate_hz),
            code_phase_samples: Chips(state.code_phase_samples),
            pll_lock,
            dll_lock,
            fll_lock,
            cycle_slip,
            nav_bit_lock: state.nav_bit_transition_count > 0,
            navigation_bit_sign: None,
            dll_err,
            pll_err,
            fll_err: fll_err_hz,
            anti_false_lock,
            cycle_slip_reason,
            lock_state,
            lock_state_reason,
            tracking_assumptions: Some(tracking_assumptions(tracking_params)),
            signal_delay_alignment: state.signal_delay_alignment.clone(),
            tracking_uncertainty,
            ..track_epoch
        });
    }
}

#[cfg(feature = "nav")]
fn annotate_navigation_bit_signs(sat: SatId, epochs: &mut [TrackEpoch]) {
    if sat.constellation != Constellation::Gps || epochs.is_empty() {
        return;
    }

    let prompt_history = epochs.iter().map(|epoch| epoch.prompt_i).collect::<Vec<_>>();
    let demodulation = bijux_gnss_nav::api::demodulate_gps_l1ca_navigation_bits(&prompt_history);
    for bit in demodulation.bits {
        for epoch in epochs[bit.start_prompt_index..bit.end_prompt_index_exclusive].iter_mut() {
            epoch.navigation_bit_sign = Some(bit.sign);
        }
    }
}

#[cfg(not(feature = "nav"))]
fn annotate_navigation_bit_signs(_sat: SatId, _epochs: &mut [TrackEpoch]) {}

fn acq_to_track_state(hypothesis: &AcqHypothesis) -> &'static str {
    match hypothesis {
        AcqHypothesis::Accepted => "accepted",
        AcqHypothesis::Ambiguous => "degraded",
        AcqHypothesis::Rejected => "rejected",
        AcqHypothesis::Deferred => "deferred",
    }
}

fn tracking_channel_uid(sat: SatId, channel_id: u8) -> String {
    format!("{:?}-{:02}-ch{:02}", sat.constellation, sat.prn, channel_id)
}

fn tracking_channel_state_report(result: &TrackingResult) -> TrackingChannelStateReport {
    let sat = result.sat;
    let channel_id = result.epochs.first().and_then(|epoch| epoch.channel_id).unwrap_or_default();
    let channel_uid = result
        .epochs
        .first()
        .map(|epoch| epoch.channel_uid.clone())
        .unwrap_or_else(|| tracking_channel_uid(sat, channel_id));
    let initial_epoch_idx = result.epochs.first().map(|epoch| epoch.epoch.index).unwrap_or(0);
    let initial_sample_index = result.epochs.first().map(|epoch| epoch.sample_index).unwrap_or(0);
    let mut emitted_states = vec![TrackingChannelStateEvent {
        state: TrackingChannelState::Acquired,
        epoch_idx: initial_epoch_idx,
        sample_index: initial_sample_index,
        reason: None,
    }];
    let mut last_steady_state = Some(TrackingChannelState::Acquired);

    for epoch in &result.epochs {
        if let Some(steady_state) = tracking_channel_steady_state(epoch) {
            if last_steady_state != Some(steady_state) {
                emitted_states.push(TrackingChannelStateEvent {
                    state: steady_state,
                    epoch_idx: epoch.epoch.index,
                    sample_index: epoch.sample_index,
                    reason: epoch.lock_state_reason.clone(),
                });
                last_steady_state = Some(steady_state);
            }
        }

        if let Some(marker_state) =
            tracking_channel_marker_state(epoch.lock_state_reason.as_deref())
        {
            let duplicate_marker =
                emitted_states.last().is_some_and(|event| event.state == marker_state);
            if !duplicate_marker {
                emitted_states.push(TrackingChannelStateEvent {
                    state: marker_state,
                    epoch_idx: epoch.epoch.index,
                    sample_index: epoch.sample_index,
                    reason: epoch.lock_state_reason.clone(),
                });
            }
        }
    }

    let (final_state, final_reason) = result
        .epochs
        .last()
        .map(|epoch| {
            (
                tracking_channel_final_state(epoch).unwrap_or(TrackingChannelState::Acquired),
                epoch.lock_state_reason.clone(),
            )
        })
        .unwrap_or((TrackingChannelState::Acquired, None));

    TrackingChannelStateReport {
        sat,
        channel_id,
        channel_uid,
        final_state,
        final_reason,
        emitted_states,
    }
}

fn tracking_channel_steady_state(epoch: &TrackEpoch) -> Option<TrackingChannelState> {
    match epoch.lock_state.as_str() {
        "acquired" => Some(TrackingChannelState::Acquired),
        "pull_in" => Some(TrackingChannelState::PullIn),
        "tracking" => Some(TrackingChannelState::Locked),
        "degraded" => Some(TrackingChannelState::Degraded),
        "lost" => Some(TrackingChannelState::Lost),
        _ => None,
    }
}

fn tracking_channel_marker_state(reason: Option<&str>) -> Option<TrackingChannelState> {
    match reason {
        Some("reacquired") => Some(TrackingChannelState::Reacquired),
        Some("cn0_below_tracking_lock_floor") => Some(TrackingChannelState::Refused),
        _ => None,
    }
}

fn tracking_channel_final_state(epoch: &TrackEpoch) -> Option<TrackingChannelState> {
    if epoch.lock_state_reason.as_deref() == Some("cn0_below_tracking_lock_floor") {
        return Some(TrackingChannelState::Refused);
    }
    tracking_channel_steady_state(epoch)
}

fn acquisition_hypothesis_rank(hypothesis: AcqHypothesis) -> u8 {
    match hypothesis {
        AcqHypothesis::Accepted => 0,
        AcqHypothesis::Ambiguous => 1,
        AcqHypothesis::Rejected => 2,
        AcqHypothesis::Deferred => 3,
    }
}

fn compare_tracking_start_contexts(
    left: &TrackingStartContext,
    right: &TrackingStartContext,
) -> std::cmp::Ordering {
    left.acquisition_hypothesis_rank
        .cmp(&right.acquisition_hypothesis_rank)
        .then_with(|| right.acquisition_score.total_cmp(&left.acquisition_score))
        .then_with(|| {
            right
                .acquisition_cn0_proxy_dbhz
                .total_cmp(&left.acquisition_cn0_proxy_dbhz)
        })
        .then_with(|| left.seed.sat.constellation.cmp(&right.seed.sat.constellation))
        .then_with(|| left.seed.sat.prn.cmp(&right.seed.sat.prn))
}

fn ca_code_or_default(prn: u8) -> Vec<i8> {
    match generate_ca_code(Prn(prn)) {
        Ok(code) => code,
        Err(_) => vec![1; 1023],
    }
}

fn code_value_at_tracking_phase(code: &[i8], chip_phase: f64) -> f32 {
    let wrapped_chip_phase = chip_phase.rem_euclid(code.len() as f64);
    code[wrapped_chip_phase.floor() as usize] as f32
}

fn epoch_start_code_phase_samples_from_receiver_phase(
    receiver_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    (code_period_samples - receiver_code_phase_samples.rem_euclid(code_period_samples))
        .rem_euclid(code_period_samples)
}

fn receiver_code_phase_samples_from_epoch_start_phase(
    epoch_start_code_phase_samples: f64,
    samples_per_code: usize,
) -> f64 {
    let code_period_samples = samples_per_code.max(1) as f64;
    (code_period_samples - epoch_start_code_phase_samples.rem_euclid(code_period_samples))
        .rem_euclid(code_period_samples)
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

fn update_prompt_power_reference(current_reference: f32, prompt_power: f32) -> f32 {
    if !prompt_power.is_finite() || prompt_power <= 0.0 {
        return current_reference;
    }
    if !current_reference.is_finite() || current_reference <= 0.0 {
        return prompt_power;
    }
    (current_reference * 0.98).max(prompt_power)
}

fn refresh_prompt_power_reference(
    current_reference: f32,
    prompt_power: f32,
    state: ChannelState,
    anti_false_lock: bool,
) -> f32 {
    if !current_reference.is_finite() || current_reference <= 0.0 {
        return update_prompt_power_reference(current_reference, prompt_power);
    }
    if matches!(state, ChannelState::Tracking) && !anti_false_lock {
        return update_prompt_power_reference(current_reference, prompt_power);
    }
    current_reference
}

fn refresh_lock_reference_cn0_dbhz(
    current_reference: f64,
    cn0_dbhz: f64,
    reliable_tracking_lock: bool,
) -> f64 {
    if reliable_tracking_lock && cn0_dbhz.is_finite() && cn0_dbhz > 0.0 {
        return cn0_dbhz;
    }
    current_reference
}

fn prompt_power_ratio(prompt_power: f32, prompt_power_reference: f32) -> Option<f32> {
    if !prompt_power.is_finite()
        || !prompt_power_reference.is_finite()
        || prompt_power_reference <= 0.0
    {
        return None;
    }
    Some(prompt_power / prompt_power_reference)
}

fn anti_false_lock_detected(early: Complex<f32>, prompt: Complex<f32>, late: Complex<f32>) -> bool {
    let prompt_norm = prompt.norm();
    if !prompt_norm.is_finite() || prompt_norm <= 0.0 {
        return true;
    }

    let early_late_mean = (early.norm() + late.norm()) * 0.5;
    !early_late_mean.is_finite()
        || early_late_mean >= prompt_norm * ANTI_FALSE_LOCK_MAX_EARLY_LATE_TO_PROMPT_RATIO
}

fn should_apply_fll(state: ChannelState, raw_fll_lock: bool) -> bool {
    matches!(state, ChannelState::PullIn | ChannelState::Degraded) || !raw_fll_lock
}

fn update_windowed_tracking_cn0_estimate(
    prompt_cn0_window: &mut VecDeque<f64>,
    epoch_cn0_dbhz: f64,
) -> Option<f64> {
    if !epoch_cn0_dbhz.is_finite() || epoch_cn0_dbhz <= 0.0 {
        return None;
    }
    prompt_cn0_window.push_back(epoch_cn0_dbhz);
    while prompt_cn0_window.len() > TRACKING_CN0_WINDOW_EPOCHS {
        prompt_cn0_window.pop_front();
    }
    if prompt_cn0_window.len() < TRACKING_CN0_MIN_WINDOW_EPOCHS {
        return None;
    }

    let mean_cn0_linear =
        prompt_cn0_window.iter().map(|value| 10.0_f64.powf(*value / 10.0)).sum::<f64>()
            / prompt_cn0_window.len() as f64;
    if !mean_cn0_linear.is_finite() || mean_cn0_linear <= 0.0 {
        return None;
    }
    Some(10.0 * mean_cn0_linear.log10())
}

fn push_tracking_uncertainty_sample(window: &mut VecDeque<f64>, value: f64) {
    if !value.is_finite() || value < 0.0 {
        return;
    }
    window.push_back(value);
    while window.len() > TRACKING_UNCERTAINTY_WINDOW_EPOCHS {
        window.pop_front();
    }
}

fn rms_window(window: &VecDeque<f64>) -> Option<f64> {
    let count = window.len();
    if count == 0 {
        return None;
    }
    Some((window.iter().map(|value| value * value).sum::<f64>() / count as f64).sqrt())
}

fn stddev_window(window: &VecDeque<f64>) -> Option<f64> {
    let count = window.len();
    if count < 2 {
        return None;
    }
    let mean = window.iter().sum::<f64>() / count as f64;
    Some(
        (window.iter().map(|value| (value - mean).powi(2)).sum::<f64>() / (count - 1) as f64)
            .sqrt(),
    )
}

fn tracking_uncertainty_state_scale(
    channel_state: ChannelState,
    channel_locked: bool,
    dll_locked: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
) -> f64 {
    let mut scale = match channel_state {
        ChannelState::Tracking => 1.0,
        ChannelState::Degraded => 2.0,
        ChannelState::PullIn | ChannelState::Acquired => 4.0,
        ChannelState::Lost | ChannelState::Idle => 8.0,
    };
    if !channel_locked {
        scale *= 2.0;
    }
    if !dll_locked {
        scale *= 1.5;
    }
    if anti_false_lock {
        scale *= 2.0;
    }
    if cycle_slip {
        scale *= 4.0;
    }
    scale
}

fn estimate_tracking_uncertainty(
    state: &LoopState,
    input: TrackingUncertaintyInputs,
) -> TrackingUncertainty {
    let state_scale = tracking_uncertainty_state_scale(
        input.channel_state,
        input.channel_locked,
        input.dll_locked,
        input.anti_false_lock,
        input.cycle_slip,
    );
    let coherent_integration_scale = 1.0 / (input.integration_ms.max(1) as f64).sqrt();
    let cn0_scale = if input.cn0_dbhz.is_finite() {
        10.0_f64.powf(((45.0 - input.cn0_dbhz).clamp(-15.0, 20.0)) / 20.0)
    } else {
        4.0
    };
    let code_fallback = ((input.dll_err.abs() as f64) * input.samples_per_chip)
        .max(TRACKING_UNCERTAINTY_MIN_CODE_PHASE_SAMPLES * cn0_scale * coherent_integration_scale);
    let carrier_fallback = ((input.pll_err_rad.abs()) / std::f64::consts::TAU)
        .max(TRACKING_UNCERTAINTY_MIN_CARRIER_PHASE_CYCLES);
    let doppler_fallback = input.fll_err_hz.abs().max(TRACKING_UNCERTAINTY_MIN_DOPPLER_HZ);
    let cn0_fallback =
        (input.cn0_dbhz - input.cn0_reference_dbhz).abs().max(TRACKING_UNCERTAINTY_MIN_CN0_DBHZ);

    TrackingUncertainty {
        code_phase_samples: rms_window(&state.code_error_window_samples)
            .unwrap_or(code_fallback)
            .max(code_fallback)
            * state_scale,
        carrier_phase_cycles: rms_window(&state.carrier_phase_error_window_cycles)
            .unwrap_or(carrier_fallback)
            .max(carrier_fallback)
            * state_scale,
        doppler_hz: rms_window(&state.doppler_error_window_hz)
            .unwrap_or(doppler_fallback)
            .max(doppler_fallback)
            * state_scale,
        cn0_dbhz: stddev_window(&state.cn0_estimate_window_dbhz)
            .unwrap_or(cn0_fallback)
            .max(TRACKING_UNCERTAINTY_MIN_CN0_DBHZ)
            * state_scale,
    }
}

fn clear_tracking_uncertainty_windows(state: &mut LoopState) {
    state.prompt_cn0_window.clear();
    state.code_error_window_samples.clear();
    state.carrier_phase_error_window_cycles.clear();
    state.doppler_error_window_hz.clear();
    state.cn0_estimate_window_dbhz.clear();
}

fn update_discriminator_instability_epochs(
    current_epochs: u8,
    from_state: ChannelState,
    prompt_power_ratio: Option<f32>,
    raw_pll_lock: bool,
    raw_fll_lock: bool,
    cycle_slip: bool,
    anti_false_lock: bool,
) -> u8 {
    let strong_prompt = prompt_power_ratio
        .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO);
    let unstable = matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && strong_prompt
        && !raw_pll_lock
        && !raw_fll_lock
        && !cycle_slip
        && !anti_false_lock;
    if unstable {
        current_epochs.saturating_add(1)
    } else {
        0
    }
}

fn classify_loss_of_lock_cause(
    from_state: ChannelState,
    cycle_slip: bool,
    prompt_power_ratio: Option<f32>,
    unstable_discriminator_epochs: u8,
) -> Option<LossOfLockCause> {
    let strong_prompt = prompt_power_ratio
        .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO);
    if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && cycle_slip
        && strong_prompt
    {
        return Some(LossOfLockCause::PhaseJump);
    }
    if matches!(from_state, ChannelState::Tracking | ChannelState::Degraded)
        && prompt_power_ratio.is_some_and(|ratio| ratio <= PROMPT_POWER_DROP_RATIO_THRESHOLD)
    {
        return Some(LossOfLockCause::PromptPowerDrop);
    }
    if unstable_discriminator_epochs >= DISCRIMINATOR_INSTABILITY_REQUIRED_EPOCHS {
        return Some(LossOfLockCause::DiscriminatorInstability);
    }
    None
}

fn is_sustained_lock_loss_reason(reason: &str) -> bool {
    matches!(
        reason,
        "lock_lost"
            | "prompt_power_drop"
            | "discriminator_instability"
            | "phase_jump"
            | "reacquisition_failed"
    )
}

#[derive(Debug, Clone)]
struct TransitionDecision {
    to_state: ChannelState,
    reason: String,
    next_unlocked_count: u8,
    next_degraded_epochs: u16,
}

fn deterministic_transition_rule(
    from_state: ChannelState,
    lock: bool,
    ready_for_tracking: bool,
    anti_false_lock: bool,
    loss_of_lock_cause: Option<LossOfLockCause>,
    unlocked_count: u8,
    degraded_epochs: u16,
    short_fade_epoch_budget: u16,
    short_fade_relock_evidence: bool,
) -> TransitionDecision {
    if loss_of_lock_cause == Some(LossOfLockCause::PhaseJump) {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: LossOfLockCause::PhaseJump.reason().to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    if loss_of_lock_cause == Some(LossOfLockCause::DiscriminatorInstability)
        && (from_state != ChannelState::Degraded
            || degraded_epochs < DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS)
    {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: LossOfLockCause::DiscriminatorInstability.reason().to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    if from_state == ChannelState::Lost {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "lost".to_string(),
            next_unlocked_count: unlocked_count,
            next_degraded_epochs: 0,
        };
    }
    if from_state == ChannelState::Degraded {
        if ready_for_tracking {
            return TransitionDecision {
                to_state: ChannelState::Tracking,
                reason: "fade_recovered".to_string(),
                next_unlocked_count: 0,
                next_degraded_epochs: 0,
            };
        }
        let next_degraded_epochs = degraded_epochs.saturating_add(1);
        let effective_short_fade_budget =
            short_fade_epoch_budget.saturating_add(if short_fade_relock_evidence {
                SHORT_FADE_RELOCK_EVIDENCE_GRACE_EPOCHS
            } else {
                0
            });
        if next_degraded_epochs > effective_short_fade_budget {
            let loss_reason =
                loss_of_lock_cause.unwrap_or(LossOfLockCause::PromptPowerDrop).reason().to_string();
            return TransitionDecision {
                to_state: ChannelState::Lost,
                reason: loss_reason,
                next_unlocked_count: 1,
                next_degraded_epochs: 0,
            };
        }
        return TransitionDecision {
            to_state: ChannelState::Degraded,
            reason: "signal_fade".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs,
        };
    }
    if from_state == ChannelState::Tracking && !ready_for_tracking {
        return TransitionDecision {
            to_state: ChannelState::Degraded,
            reason: "signal_fade".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 1,
        };
    }
    if ready_for_tracking {
        return TransitionDecision {
            to_state: ChannelState::Tracking,
            reason: "carrier_converged".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 0,
        };
    }
    if lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "carrier_pull_in".to_string(),
            next_unlocked_count: 0,
            next_degraded_epochs: 0,
        };
    }
    if anti_false_lock {
        return TransitionDecision {
            to_state: ChannelState::PullIn,
            reason: "anti_false_lock".to_string(),
            next_unlocked_count: unlocked_count.saturating_add(1),
            next_degraded_epochs: 0,
        };
    }
    let next_unlocked = unlocked_count.saturating_add(1);
    if from_state == ChannelState::Tracking && next_unlocked >= 2 {
        return TransitionDecision {
            to_state: ChannelState::Lost,
            reason: "lock_lost".to_string(),
            next_unlocked_count: next_unlocked,
            next_degraded_epochs: 0,
        };
    }
    TransitionDecision {
        to_state: ChannelState::PullIn,
        reason: "carrier_pull_in".to_string(),
        next_unlocked_count: next_unlocked,
        next_degraded_epochs: 0,
    }
}

fn sustained_lock_loss_reacquire_seed(epochs: &[TrackEpoch]) -> Option<SustainedLockLossSeed> {
    let mut consecutive_lost_epochs = 0usize;
    for epoch in epochs {
        if !epoch.lock
            && epoch.lock_state == "lost"
            && epoch.lock_state_reason.as_deref().is_some_and(is_sustained_lock_loss_reason)
        {
            consecutive_lost_epochs += 1;
            if consecutive_lost_epochs >= REACQUISITION_REQUIRED_LOST_EPOCHS {
                return Some(SustainedLockLossSeed {
                    carrier_hz: epoch.carrier_hz.0,
                    code_phase_samples: epoch.code_phase_samples.0,
                    code_rate_hz: epoch.code_rate_hz.0,
                    sample_index: epoch.sample_index,
                });
            }
            continue;
        }
        consecutive_lost_epochs = 0;
    }
    None
}

fn project_reacquisition_code_phase_samples(
    seed: SustainedLockLossSeed,
    target_sample_index: u64,
    nominal_code_rate_hz: f64,
    samples_per_code: usize,
) -> f64 {
    let elapsed_samples = target_sample_index.saturating_sub(seed.sample_index) as f64;
    let projected_code_phase_samples =
        seed.code_phase_samples + elapsed_samples * (seed.code_rate_hz / nominal_code_rate_hz);
    wrap_code_phase_samples(projected_code_phase_samples, samples_per_code)
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

fn tracking_frame_start_offset(
    frame: &SamplesFrame,
    start_sample_index: Option<u64>,
) -> Option<usize> {
    let Some(start_sample_index) = start_sample_index else {
        return Some(0);
    };
    let frame_start = frame.t0.sample_index;
    let frame_end = frame_start + frame.len() as u64;
    if start_sample_index >= frame_end {
        return None;
    }
    if start_sample_index <= frame_start {
        return Some(0);
    }
    Some((start_sample_index - frame_start) as usize)
}

fn frame_slice(frame: &SamplesFrame, start: usize, end: usize) -> SamplesFrame {
    SamplesFrame::new(
        SampleTime {
            sample_index: frame.t0.sample_index + start as u64,
            sample_rate_hz: frame.t0.sample_rate_hz,
        },
        frame.dt_s,
        frame.iq[start..end].to_vec(),
    )
}

fn code_periods_in_frame(frame_len: usize, samples_per_code: usize) -> u64 {
    if frame_len == 0 {
        return 0;
    }
    (frame_len / samples_per_code.max(1)).max(1) as u64
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
        && same_delta.abs() - flipped_delta.abs() >= NAV_BIT_PHASE_MIN_IMPROVEMENT_CYCLES
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
    let epoch_start_code_phase_samples = epoch_start_code_phase_samples_from_receiver_phase(
        current_code_phase_samples,
        samples_per_code,
    );
    let code_period_advance_samples =
        epoch_len_samples as f64 * (tracked_code_rate_hz / nominal_code_rate_hz);
    let dll_correction_samples =
        -(dll_err as f64) * samples_per_chip * DLL_CODE_PHASE_CORRECTION_GAIN;
    let next_epoch_start_code_phase_samples = wrap_code_phase_samples(
        epoch_start_code_phase_samples + code_period_advance_samples + dll_correction_samples,
        samples_per_code,
    );
    receiver_code_phase_samples_from_epoch_start_phase(
        next_epoch_start_code_phase_samples,
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
    let catastrophic_phase_step_samples =
        phase_step_limit_samples * SAMPLE_RATE_MISMATCH_CATASTROPHIC_PHASE_STEP_MULTIPLIER;
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
            let unstable = abs_phase_step_samples > phase_step_limit_samples || current.cycle_slip;
            let supported = sample_rate_mismatch_supported_epoch(current);
            (index + 1, abs_phase_step_samples, unstable, supported)
        })
        .collect::<Vec<_>>();

    if let Some((first_unstable_epoch_index, max_abs_phase_step_samples, _, _)) =
        instability_markers
            .iter()
            .copied()
            .find(|(_, abs_phase_step_samples, _, supported)| {
                *supported && *abs_phase_step_samples >= catastrophic_phase_step_samples
            })
    {
        return Some(CodePhaseStabilityDiagnostic {
            first_unstable_epoch_index,
            max_abs_phase_step_samples,
            phase_step_limit_samples,
        });
    }

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

fn sample_rate_mismatch_supported_epoch(epoch: &TrackEpoch) -> bool {
    if !epoch.cn0_dbhz.is_finite() || epoch.cn0_dbhz < SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ {
        return false;
    }
    if !matches!(epoch.lock_state.as_str(), "pull_in" | "tracking" | "degraded") {
        return false;
    }
    if epoch.lock_state_reason.as_deref().is_some_and(|reason| {
        matches!(
            reason,
            "phase_jump"
                | "prompt_power_drop"
                | "discriminator_instability"
                | "lock_lost"
                | "reacquisition_failed"
        )
    }) {
        return false;
    }
    epoch.lock || epoch.fll_lock || epoch.pll_lock
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

fn carrier_phase_offset_rad(carrier_phase_cycles: f64) -> f64 {
    wrap_phase_rad(carrier_phase_cycles * std::f64::consts::TAU)
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

#[cfg(test)]
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

fn dll_lock_threshold(samples_per_chip: f64, early_late_spacing_chips: f64) -> f32 {
    let effective_sample_separation = samples_per_chip * early_late_spacing_chips.abs();
    if effective_sample_separation + f64::EPSILON < 1.0 {
        DLL_LOW_RESOLUTION_LOCK_MAX_CODE_ERROR
    } else {
        DLL_LOCK_MAX_CODE_ERROR
    }
}

fn dll_hold_threshold(samples_per_chip: f64, early_late_spacing_chips: f64) -> f32 {
    dll_lock_threshold(samples_per_chip, early_late_spacing_chips).max(DLL_HOLD_MAX_CODE_ERROR)
}

fn short_fade_epoch_budget(tracking_params: TrackingParams) -> u16 {
    let integration_ms = tracking_params.integration_ms.max(1) as f64;
    ((SHORT_FADE_MAX_DURATION_S * 1000.0) / integration_ms).ceil() as u16
        + SHORT_FADE_RECOVERY_GRACE_EPOCHS
}

fn update_pull_in_stable_epochs(
    current_stable_epochs: u8,
    prompt_lock: bool,
    _dll_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
) -> u8 {
    if cycle_slip || !prompt_lock || (!pll_lock && !fll_lock) {
        return 0;
    }
    current_stable_epochs.saturating_add(1)
}

fn update_prelock_cn0_refusal(
    current_state: ChannelState,
    weak_cn0_epochs: u8,
    cn0_dbhz: f64,
) -> (u8, bool, bool) {
    if matches!(current_state, ChannelState::Tracking | ChannelState::Degraded) {
        let cn0_supports_lock = cn0_dbhz.is_finite() && cn0_dbhz >= TRACKING_LOCK_MIN_CN0_DBHZ;
        return (0, cn0_supports_lock, false);
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

fn reacquisition_min_cn0_dbhz(lock_reference_cn0_dbhz: f64) -> f64 {
    if !lock_reference_cn0_dbhz.is_finite() || lock_reference_cn0_dbhz <= 0.0 {
        return TRACKING_LOCK_MIN_CN0_DBHZ;
    }
    (lock_reference_cn0_dbhz - REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ)
        .max(TRACKING_LOCK_MIN_CN0_DBHZ)
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
    if input.apply_pll_frequency {
        carrier_hz += pll_coefficients.frequency_gain_hz_per_rad * input.pll_err_rad;
    }

    let carrier_phase_cycles = input.current_carrier_phase_cycles
        + carrier_hz * input.epoch_len_samples as f64 / input.sample_rate_hz
        + pll_coefficients.phase_blend * input.pll_err_rad / std::f64::consts::TAU;

    CarrierLoopUpdate { carrier_hz, carrier_phase_cycles }
}

fn apply_reacquisition_annotations(
    state: &mut LoopState,
    epochs: &mut [TrackEpoch],
    transitions: &mut [TrackTransition],
    reacquisition_outcome: ReacquisitionOutcome,
) {
    let Some(last_epoch) = epochs.last_mut() else {
        return;
    };

    if state.reacquisition_pending {
        let stable_tracking_epoch = last_epoch.lock
            && last_epoch.lock_state == ChannelState::Tracking.to_string()
            && last_epoch.lock_state_reason.as_deref() == Some("carrier_converged")
            && last_epoch.pll_lock
            && last_epoch.fll_lock
            && !last_epoch.cycle_slip
            && !last_epoch.anti_false_lock;
        state.reacquisition_stable_tracking_epochs = if stable_tracking_epoch {
            state.reacquisition_stable_tracking_epochs.saturating_add(1)
        } else {
            0
        };
    }

    if state.reacquisition_pending
        && state.reacquisition_stable_tracking_epochs >= REACQUISITION_STABLE_TRACKING_EPOCHS
        && last_epoch.lock_state == ChannelState::Tracking.to_string()
        && last_epoch.lock_state_reason.as_deref() == Some("carrier_converged")
    {
        last_epoch.lock_state_reason = Some("reacquired".to_string());
        if let Some(last_transition) = transitions.last_mut() {
            if last_transition.to_state == ChannelState::Tracking.to_string()
                && last_transition.reason == "carrier_converged"
            {
                last_transition.reason = "reacquired".to_string();
            }
        }
        state.reacquisition_pending = false;
        state.reacquisition_attempt_epochs = 0;
        state.reacquisition_stable_tracking_epochs = 0;
        return;
    }

    if state.reacquisition_pending {
        state.reacquisition_attempt_epochs = state.reacquisition_attempt_epochs.saturating_add(1);
        if state.reacquisition_attempt_epochs >= REACQUISITION_PULL_IN_EPOCH_BUDGET {
            last_epoch.lock_state = ChannelState::Lost.to_string();
            last_epoch.lock_state_reason = Some("reacquisition_failed".to_string());
            if let Some(last_transition) = transitions.last_mut() {
                last_transition.to_state = ChannelState::Lost.to_string();
                last_transition.reason = "reacquisition_failed".to_string();
            }
            state.state = ChannelState::Lost;
            state.lost_reason = Some("reacquisition_failed".to_string());
            state.reacquisition_pending = false;
            state.reacquisition_attempt_epochs = 0;
            state.reacquisition_stable_tracking_epochs = 0;
            state.unlocked_count = 0;
            state.degraded_epochs = 0;
            return;
        }
    }

    if last_epoch.lock_state == ChannelState::Lost.to_string() {
        match reacquisition_outcome {
            ReacquisitionOutcome::Failed => {
                last_epoch.lock_state_reason = Some("reacquisition_failed".to_string());
            }
            ReacquisitionOutcome::SeedUnavailable | ReacquisitionOutcome::NotNeeded => {
                if let Some(lost_reason) = state.lost_reason.as_ref() {
                    last_epoch.lock_state_reason = Some(lost_reason.clone());
                }
            }
            ReacquisitionOutcome::Started => {}
        }
    }
}

impl Tracking {
    fn try_reacquire_channel(
        &self,
        channel: &mut IncrementalTrackingChannel,
        frame: &SamplesFrame,
    ) -> ReacquisitionOutcome {
        if channel.state.state != ChannelState::Lost {
            return ReacquisitionOutcome::NotNeeded;
        }
        let Some(sustained_loss_seed) = sustained_lock_loss_reacquire_seed(&channel.epochs) else {
            channel.state.reacquisition_candidate = None;
            channel.state.reacquisition_candidate_streak = 0;
            return ReacquisitionOutcome::SeedUnavailable;
        };
        let reacquisition_code_phase_samples = project_reacquisition_code_phase_samples(
            sustained_loss_seed,
            frame.t0.sample_index,
            self.config.code_freq_basis_hz,
            samples_per_code(
                self.config.sampling_freq_hz,
                self.config.code_freq_basis_hz,
                self.config.code_length,
            ),
        );
        let Some(seed) = self.quick_reacquire(
            frame,
            channel.sat,
            sustained_loss_seed.carrier_hz,
            reacquisition_code_phase_samples,
            channel.state.lock_reference_cn0_dbhz,
            channel.acquisition_uncertainty.as_ref(),
        ) else {
            channel.state.reacquisition_candidate = None;
            channel.state.reacquisition_candidate_streak = 0;
            return ReacquisitionOutcome::Failed;
        };
        let candidate_streak = if channel.state.reacquisition_candidate.is_some_and(|candidate| {
            self.reacquisition_seed_matches(
                candidate,
                seed,
                channel.acquisition_uncertainty.as_ref(),
            )
        }) {
            channel.state.reacquisition_candidate_streak.saturating_add(1)
        } else {
            1
        };
        channel.state.reacquisition_candidate = Some(seed);
        channel.state.reacquisition_candidate_streak = candidate_streak;
        if candidate_streak < REACQUISITION_CONFIRMATION_EPOCHS {
            return ReacquisitionOutcome::Failed;
        }

        channel.state = LoopState {
            carrier_hz: seed.carrier_hz,
            carrier_phase_cycles: 0.0,
            code_rate_hz: self.config.code_freq_basis_hz,
            code_phase_samples: seed.code_phase_samples,
            signal_delay_alignment: channel.state.signal_delay_alignment.clone(),
            acquisition_cn0_proxy_dbhz: seed.cn0_dbhz,
            lock_reference_cn0_dbhz: seed.cn0_dbhz,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            degraded_epochs: 0,
            prompt_power_reference: 0.0,
            prompt_cn0_window: VecDeque::new(),
            code_error_window_samples: VecDeque::new(),
            carrier_phase_error_window_cycles: VecDeque::new(),
            doppler_error_window_hz: VecDeque::new(),
            cn0_estimate_window_dbhz: VecDeque::new(),
            unstable_discriminator_epochs: 0,
            state: ChannelState::Acquired,
            unlocked_count: 0,
            lost_reason: None,
            reacquisition_candidate: None,
            reacquisition_candidate_streak: 0,
            reacquisition_pending: true,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
        };
        ReacquisitionOutcome::Started
    }

    fn reacquisition_seed_matches(
        &self,
        left: ReacquisitionSeed,
        right: ReacquisitionSeed,
        acquisition_uncertainty: Option<&AcqUncertainty>,
    ) -> bool {
        let doppler_step_hz = self.config.acquisition_doppler_step_hz.max(1) as f64;
        let doppler_tolerance_hz = acquisition_uncertainty
            .map(|uncertainty| uncertainty.doppler_hz.clamp(doppler_step_hz / 2.0, doppler_step_hz))
            .unwrap_or(doppler_step_hz);
        let code_tolerance_samples = acquisition_uncertainty
            .map(|uncertainty| uncertainty.code_phase_samples.clamp(0.5, 2.0))
            .unwrap_or(2.0);
        (left.carrier_hz - right.carrier_hz).abs() <= doppler_tolerance_hz
            && (left.code_phase_samples - right.code_phase_samples).abs() <= code_tolerance_samples
    }

    fn quick_reacquire(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_hz: f64,
        code_phase_samples: f64,
        lock_reference_cn0_dbhz: f64,
        acquisition_uncertainty: Option<&AcqUncertainty>,
    ) -> Option<ReacquisitionSeed> {
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
        let mut best_cn0_dbhz = f64::NEG_INFINITY;
        let min_cn0_dbhz = reacquisition_min_cn0_dbhz(lock_reference_cn0_dbhz);
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
                let cn0_dbhz = estimate_cn0_dbhz(
                    corr.prompt,
                    corr.early - corr.late,
                    self.config.sampling_freq_hz,
                    frame.len() as f64,
                    corr.early_late_noise_weight_energy,
                );
                let anti_false_lock = anti_false_lock_detected(corr.early, corr.prompt, corr.late);
                if !anti_false_lock && cn0_dbhz >= min_cn0_dbhz && cn0_dbhz > best_cn0_dbhz {
                    best_cn0_dbhz = cn0_dbhz;
                    best = Some(ReacquisitionSeed {
                        carrier_hz: carrier_hz + d,
                        code_phase_samples: code_phase_samples + c,
                        cn0_dbhz,
                    });
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
        AcqHypothesis, AcqUncertainty, Chips, Constellation, Epoch, Hertz, ReceiverSampleTrace,
        SampleTime, SamplesFrame, SatId, Seconds, SignalBand, TrackEpoch,
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
            Some(super::LossOfLockCause::PhaseJump),
            1,
            0,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "phase_jump");
        assert_eq!(decision.next_unlocked_count, 2);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_promotes_lock() {
        let decision = super::deterministic_transition_rule(
            ChannelState::PullIn,
            true,
            true,
            false,
            None,
            2,
            0,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Tracking);
        assert_eq!(decision.reason, "carrier_converged");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_degrades_tracking_during_short_fade() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Tracking,
            false,
            false,
            false,
            None,
            1,
            0,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "signal_fade");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 1);
    }

    #[test]
    fn deterministic_transition_rule_recovers_after_short_fade() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            true,
            true,
            false,
            None,
            0,
            4,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Tracking);
        assert_eq!(decision.reason, "fade_recovered");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_keeps_degraded_state_during_fade_cycle_slip() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::PhaseJump),
            0,
            2,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "phase_jump");
        assert_eq!(decision.next_unlocked_count, 1);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_marks_loss_after_fade_budget_exhaustion() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::PromptPowerDrop),
            0,
            100,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Lost);
        assert_eq!(decision.reason, "prompt_power_drop");
        assert_eq!(decision.next_unlocked_count, 1);
        assert_eq!(decision.next_degraded_epochs, 0);
    }

    #[test]
    fn deterministic_transition_rule_grants_short_fade_grace_to_degraded_instability() {
        let decision = super::deterministic_transition_rule(
            ChannelState::Degraded,
            false,
            false,
            false,
            Some(super::LossOfLockCause::DiscriminatorInstability),
            0,
            super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS,
            100,
            false,
        );
        assert_eq!(decision.to_state, ChannelState::Degraded);
        assert_eq!(decision.reason, "signal_fade");
        assert_eq!(
            decision.next_degraded_epochs,
            super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS + 1
        );
    }

    #[test]
    fn classify_loss_of_lock_cause_prioritizes_phase_jump() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.9), 3);

        assert_eq!(cause, Some(super::LossOfLockCause::PhaseJump));
    }

    #[test]
    fn classify_loss_of_lock_cause_treats_weak_prompt_cycle_slips_as_prompt_power_drop() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.1), 0);

        assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
    }

    #[test]
    fn classify_loss_of_lock_cause_detects_prompt_power_drop() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.1), 0);

        assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
    }

    #[test]
    fn classify_loss_of_lock_cause_detects_discriminator_instability() {
        let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.8), 2);

        assert_eq!(cause, Some(super::LossOfLockCause::DiscriminatorInstability));
    }

    #[test]
    fn update_discriminator_instability_epochs_requires_strong_prompt() {
        let epochs = super::update_discriminator_instability_epochs(
            1,
            ChannelState::Tracking,
            Some(0.7),
            false,
            false,
            false,
            false,
        );
        assert_eq!(epochs, 2);

        let reset = super::update_discriminator_instability_epochs(
            epochs,
            ChannelState::Tracking,
            Some(0.1),
            false,
            false,
            false,
            false,
        );
        assert_eq!(reset, 0);
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_ignores_degraded_short_fade_epochs() {
        let epochs = vec![
            track_epoch_with_state(0, false, "degraded", Some("signal_fade")),
            track_epoch_with_state(1, false, "degraded", Some("signal_fade")),
            track_epoch_with_state(2, false, "degraded", Some("signal_fade")),
        ];

        assert_eq!(super::sustained_lock_loss_reacquire_seed(&epochs), None);
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_requires_three_lost_epochs() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("lock_lost")),
            track_epoch_with_state(1, false, "lost", Some("lock_lost")),
            track_epoch_with_state(2, false, "lost", Some("lock_lost")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_accepts_explicit_loss_causes() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("prompt_power_drop")),
            track_epoch_with_state(1, false, "lost", Some("prompt_power_drop")),
            track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn sustained_lock_loss_reacquire_seed_allows_retry_after_failed_attempt() {
        let epochs = vec![
            track_epoch_with_state(0, false, "lost", Some("reacquisition_failed")),
            track_epoch_with_state(1, false, "lost", Some("reacquisition_failed")),
            track_epoch_with_state(2, false, "lost", Some("reacquisition_failed")),
        ];

        assert_eq!(
            super::sustained_lock_loss_reacquire_seed(&epochs),
            Some(super::SustainedLockLossSeed {
                carrier_hz: 2.0,
                code_phase_samples: 2.5,
                code_rate_hz: 0.0,
                sample_index: 2,
            })
        );
    }

    #[test]
    fn project_reacquisition_code_phase_samples_advances_loss_anchor_to_current_epoch() {
        let seed = super::SustainedLockLossSeed {
            carrier_hz: 1200.0,
            code_phase_samples: 5.5,
            code_rate_hz: 1_023_000.0,
            sample_index: 1_000,
        };

        let projected = super::project_reacquisition_code_phase_samples(
            seed,
            1_000 + 2 * 1_023,
            1_023_000.0,
            1_023,
        );

        assert_eq!(projected, 5.5);
    }

    #[test]
    fn reacquisition_seed_matches_respects_acquisition_uncertainty_tolerances() {
        let tracking = Tracking::new(ReceiverPipelineConfig::default(), ReceiverRuntime::default());
        let uncertainty = AcqUncertainty { doppler_hz: 250.0, code_phase_samples: 0.75 };

        assert!(tracking.reacquisition_seed_matches(
            super::ReacquisitionSeed {
                carrier_hz: 1250.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 36.0,
            },
            super::ReacquisitionSeed {
                carrier_hz: 1400.0,
                code_phase_samples: 42.5,
                cn0_dbhz: 34.0,
            },
            Some(&uncertainty),
        ));
        assert!(!tracking.reacquisition_seed_matches(
            super::ReacquisitionSeed {
                carrier_hz: 1250.0,
                code_phase_samples: 42.0,
                cn0_dbhz: 36.0,
            },
            super::ReacquisitionSeed {
                carrier_hz: 1705.0,
                code_phase_samples: 43.0,
                cn0_dbhz: 34.0,
            },
            Some(&uncertainty),
        ));
    }

    #[test]
    fn reacquisition_min_cn0_dbhz_preserves_lock_reference_headroom() {
        assert_eq!(super::reacquisition_min_cn0_dbhz(60.0), 48.0);
        assert_eq!(super::reacquisition_min_cn0_dbhz(35.0), 28.0);
        assert_eq!(super::reacquisition_min_cn0_dbhz(f64::NAN), super::TRACKING_LOCK_MIN_CN0_DBHZ);
    }

    #[test]
    fn code_value_at_tracking_phase_wraps_fractional_chip_phases() {
        let code = vec![1_i8, -1, 1, -1];

        assert_eq!(super::code_value_at_tracking_phase(&code, 0.25), 1.0);
        assert_eq!(super::code_value_at_tracking_phase(&code, 1.75), -1.0);
        assert_eq!(super::code_value_at_tracking_phase(&code, 4.10), 1.0);
        assert_eq!(super::code_value_at_tracking_phase(&code, -0.10), -1.0);
    }

    #[test]
    fn tracking_frame_start_offset_clamps_to_visible_frame_samples() {
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 100, sample_rate_hz: 1_023_000.0 },
            Seconds(1.0 / 1_023_000.0),
            vec![Complex::new(0.0_f32, 0.0); 16],
        );

        assert_eq!(super::tracking_frame_start_offset(&frame, None), Some(0));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(90)), Some(0));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(108)), Some(8));
        assert_eq!(super::tracking_frame_start_offset(&frame, Some(116)), None);
    }

    #[test]
    fn refresh_lock_reference_cn0_dbhz_only_updates_on_reliable_lock() {
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 52.0, true), 52.0);
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, 30.0, false), 48.0);
        assert_eq!(super::refresh_lock_reference_cn0_dbhz(48.0, f64::NAN, true), 48.0);
    }

    #[test]
    fn deterministic_transition_rule_holds_pull_in_until_carrier_converges() {
        let decision = super::deterministic_transition_rule(
            ChannelState::PullIn,
            true,
            false,
            false,
            None,
            1,
            0,
            100,
            false,
        );

        assert_eq!(decision.to_state, ChannelState::PullIn);
        assert_eq!(decision.reason, "carrier_pull_in");
        assert_eq!(decision.next_unlocked_count, 0);
        assert_eq!(decision.next_degraded_epochs, 0);
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
        assert!(!supports_lock);
        assert!(!refuse_lock);

        let (weak_epochs, supports_lock, refuse_lock) =
            super::update_prelock_cn0_refusal(ChannelState::Degraded, 2, 20.0);
        assert_eq!(weak_epochs, 0);
        assert!(!supports_lock);
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
                crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                    &config,
                    &epoch_frame,
                    epoch_code_phase_chips,
                );
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
        assert!(next > 250.0, "next={next}");
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

    fn empty_loop_state() -> super::LoopState {
        super::LoopState {
            carrier_hz: 0.0,
            carrier_phase_cycles: 0.0,
            code_rate_hz: 0.0,
            code_phase_samples: 0.0,
            signal_delay_alignment: None,
            acquisition_cn0_proxy_dbhz: 45.0,
            lock_reference_cn0_dbhz: 45.0,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            degraded_epochs: 0,
            prompt_power_reference: 0.0,
            prompt_cn0_window: std::collections::VecDeque::new(),
            code_error_window_samples: std::collections::VecDeque::new(),
            carrier_phase_error_window_cycles: std::collections::VecDeque::new(),
            doppler_error_window_hz: std::collections::VecDeque::new(),
            cn0_estimate_window_dbhz: std::collections::VecDeque::new(),
            unstable_discriminator_epochs: 0,
            state: ChannelState::Tracking,
            unlocked_count: 0,
            lost_reason: None,
            reacquisition_candidate: None,
            reacquisition_candidate_streak: 0,
            reacquisition_pending: false,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
        }
    }

    #[test]
    fn tracking_uncertainty_rewards_longer_coherent_integration() {
        let state = empty_loop_state();
        let short = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            },
        );
        let long = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                integration_ms: 10,
                ..super::TrackingUncertaintyInputs {
                    samples_per_chip: 4.0,
                    dll_err: 0.0,
                    pll_err_rad: 0.0,
                    fll_err_hz: 0.0,
                    cn0_dbhz: 45.0,
                    cn0_reference_dbhz: 45.0,
                    integration_ms: 1,
                    channel_locked: true,
                    dll_locked: true,
                    anti_false_lock: false,
                    cycle_slip: false,
                    channel_state: ChannelState::Tracking,
                }
            },
        );

        assert!(
            long.code_phase_samples < short.code_phase_samples,
            "longer coherent integration should tighten code-phase uncertainty: short={short:?} long={long:?}"
        );
    }

    #[test]
    fn tracking_uncertainty_penalizes_dll_unlock() {
        let state = empty_loop_state();
        let locked = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            },
        );
        let unlocked = super::estimate_tracking_uncertainty(
            &state,
            super::TrackingUncertaintyInputs {
                dll_locked: false,
                ..super::TrackingUncertaintyInputs {
                    samples_per_chip: 4.0,
                    dll_err: 0.0,
                    pll_err_rad: 0.0,
                    fll_err_hz: 0.0,
                    cn0_dbhz: 45.0,
                    cn0_reference_dbhz: 45.0,
                    integration_ms: 1,
                    channel_locked: true,
                    dll_locked: true,
                    anti_false_lock: false,
                    cycle_slip: false,
                    channel_state: ChannelState::Tracking,
                }
            },
        );

        assert!(
            unlocked.code_phase_samples > locked.code_phase_samples,
            "loss of DLL lock should inflate code-phase uncertainty: locked={locked:?} unlocked={unlocked:?}"
        );
    }

    #[test]
    fn short_fade_epoch_budget_reserves_post_fade_recovery_window() {
        let tracking_params = crate::engine::receiver_config::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };

        assert_eq!(super::short_fade_epoch_budget(tracking_params), 105);
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

        assert!(update.code_phase_samples > current_code_phase_samples, "update={update:?}");
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

        assert!(update.code_phase_samples < current_code_phase_samples, "update={update:?}");
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
        let code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &frame,
                code_phase_chips,
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
        let code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &frame,
                code_phase_chips,
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
    fn correlate_epoch_keeps_carrier_phase_aligned_across_nonzero_epoch_starts() {
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
        let carrier_hz = 120.0;
        let epoch_len_samples = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let second_epoch_start_s = epoch_len_samples as f64 / config.sampling_freq_hz;
        let second_epoch_start_phase_cycles = 0.18;
        let initial_phase_cycles =
            second_epoch_start_phase_cycles - carrier_hz * second_epoch_start_s;
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                doppler_hz: carrier_hz,
                code_phase_chips: 0.0,
                carrier_phase_rad: initial_phase_cycles * std::f64::consts::TAU,
                cn0_db_hz: 90.0,
                data_bit_flip: false,
            },
            0xC0A5_1E,
            0.002,
        );
        let second_epoch = super::frame_slice(&frame, epoch_len_samples, epoch_len_samples * 2);

        let correlator = tracking.correlate_epoch(
            &second_epoch,
            sat,
            carrier_hz,
            second_epoch_start_phase_cycles,
            config.code_freq_basis_hz,
            0.0,
            0.5,
        );

        assert!(
            correlator.prompt.re > correlator.prompt.im.abs() * 10.0,
            "prompt={:?}",
            correlator.prompt,
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
            apply_pll_frequency: true,
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
            apply_pll_frequency: false,
        });

        let fll_coefficients = first_order_angular_loop_coefficients(10.0, 0.001);
        let expected_carrier_hz = 80.0
            + super::bounded_fll_pull_in_correction_hz(30.0 * fll_coefficients.error_blend, 10.0);
        assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_carrier_loop_accumulates_unwrapped_phase_across_epochs() {
        let first = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_500.0,
            current_carrier_phase_cycles: 128.25,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });
        let second = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: first.carrier_hz,
            current_carrier_phase_cycles: first.carrier_phase_cycles,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.001,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });

        assert!((first.carrier_phase_cycles - 129.75).abs() < 1.0e-9, "{first:?}");
        assert!((second.carrier_phase_cycles - 131.25).abs() < 1.0e-9, "{second:?}");
        assert!(
            (second.carrier_phase_cycles - first.carrier_phase_cycles - 1.5).abs() < 1.0e-9,
            "{first:?} {second:?}"
        );
    }

    #[test]
    fn apply_carrier_loop_preserves_continuous_phase_for_negative_doppler() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: -850.0,
            current_carrier_phase_cycles: 512.875,
            epoch_len_samples: 8_184,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s: 0.002,
            pll_bw_hz: 8.0,
            pll_err_rad: 0.0,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });

        assert!((update.carrier_hz + 850.0).abs() < 1.0e-9, "{update:?}");
        assert!((update.carrier_phase_cycles - 511.175).abs() < 1.0e-9, "{update:?}");
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
            apply_pll_frequency: true,
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
            apply_pll_frequency: true,
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
        let code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &frame,
                code_phase_chips,
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
    fn detect_sample_rate_mismatch_flags_catastrophic_pull_in_phase_jump() {
        let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
        let epochs = [0.0, 2.5, 4.0, 4_452.0, 4_452.5]
            .into_iter()
            .enumerate()
            .map(|(index, code_phase_samples)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: (index as u64) * 5_050,
                sat,
                code_phase_samples: Chips(code_phase_samples),
                cn0_dbhz: 48.0,
                lock: true,
                fll_lock: index > 0,
                lock_state: ChannelState::PullIn.to_string(),
                lock_state_reason: Some("carrier_pull_in".to_string()),
                ..TrackEpoch::default()
            })
            .collect::<Vec<_>>();

        let diagnostic =
            super::detect_sample_rate_mismatch(&epochs, 5_050).expect("diagnostic must exist");
        assert_eq!(diagnostic.first_unstable_epoch_index, 3);
        assert!(diagnostic.max_abs_phase_step_samples >= 500.0);
    }

    #[test]
    fn dll_lock_threshold_relaxes_for_subsample_early_late_spacing() {
        assert_eq!(super::dll_lock_threshold(1.0, 0.5), 0.6);
        assert_eq!(super::dll_lock_threshold(4.0, 0.5), 0.2);
    }

    #[test]
    fn clean_seeded_tracking_clears_false_lock_once_loops_converge() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 18.0,
            fll_bw_hz: 12.0,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                doppler_hz: -750.0,
                code_phase_chips: 211.25,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                data_bit_flip: false,
            },
            0x710C_A000,
            0.012,
        );
        let code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples(&config, &frame, 211.25);
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let tracks = tracking.track_from_acquisition(
            &frame,
            &[bijux_gnss_core::api::AcqResult {
                sat,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: bijux_gnss_core::api::Hertz(-750.0),
                carrier_hz: bijux_gnss_core::api::Hertz(-750.0),
                code_phase_samples,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 20.0,
                peak_second_ratio: 10.0,
                cn0_proxy: 52.0,
                score: 1.0,
                hypothesis: bijux_gnss_core::api::AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: Some("clean_seeded_tracking".to_string()),
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            }],
        );
        let epochs = &tracks.first().expect("track").epochs;

        assert!(
            epochs
                .iter()
                .skip(4)
                .filter(|epoch| epoch.pll_lock && epoch.fll_lock)
                .all(|epoch| !epoch.anti_false_lock),
            "clean, converged tracking epochs must not remain marked as false lock: {epochs:?}"
        );
    }

    #[test]
    fn correlate_epoch_honors_receiver_code_phase_seed_at_low_sample_rate() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 2_046_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 18.0,
            fll_bw_hz: 12.0,
            ..ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                data_bit_flip: false,
            },
            0x330C_2000,
            0.04,
        );
        let refined_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &frame,
                200.25,
            );
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let correlator = tracking.correlate_epoch(
            &frame,
            sat,
            750.0,
            0.0,
            1_023_000.0,
            refined_code_phase_samples,
            0.5,
        );

        assert!(
            correlator.prompt.norm() > 10_000.0,
            "receiver-seeded code phase must produce a strong prompt correlation: {:?}",
            correlator.prompt
        );
        assert!(
            correlator.prompt.re.abs() > correlator.prompt.im.abs() * 100.0,
            "receiver-seeded code phase must align carrier phase near the real axis: {:?}",
            correlator.prompt
        );
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
            signal_delay_alignment: None,
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

    #[test]
    fn begin_incremental_tracking_prioritizes_stronger_trackable_acquisitions() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            channels: 2,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisitions = vec![
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                signal_band: SignalBand::L1,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 6.0,
                peak_second_ratio: 2.0,
                cn0_proxy: 38.0,
                score: 1.4,
                hypothesis: AcqHypothesis::Ambiguous,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 7 },
                signal_band: SignalBand::L1,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 8.0,
                peak_second_ratio: 3.0,
                cn0_proxy: 52.0,
                score: 5.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
            bijux_gnss_core::api::AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 23 },
                signal_band: SignalBand::L1,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                carrier_hz: Hertz(0.0),
                code_phase_samples: 0,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.01,
                peak_mean_ratio: 9.0,
                peak_second_ratio: 3.5,
                cn0_proxy: 48.0,
                score: 3.5,
                hypothesis: AcqHypothesis::Ambiguous,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: None,
                doppler_refinement: None,
                code_phase_refinement: None,
                signal_delay_alignment: None,
                uncertainty: None,
            },
        ];

        let incremental = tracking.begin_incremental_tracking(&acquisitions);
        let selected_prns = incremental
            .channels
            .iter()
            .map(|channel| channel.sat.prn)
            .collect::<Vec<_>>();

        assert_eq!(selected_prns, vec![7, 23]);
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
            let mut degraded_epochs = 0u16;
            for event in &fixture.events {
                let decision = super::deterministic_transition_rule(
                    state,
                    event.lock,
                    event.lock,
                    event.anti_false_lock,
                    event.cycle_slip.then_some(super::LossOfLockCause::PhaseJump),
                    unlocked,
                    degraded_epochs,
                    100,
                    false,
                );
                state = decision.to_state;
                unlocked = decision.next_unlocked_count;
                degraded_epochs = decision.next_degraded_epochs;
            }
            assert_eq!(
                state,
                parse_state(&fixture.expected_final_state),
                "fixture {} final state mismatch",
                fixture.id
            );
        }
    }

    fn track_epoch_with_state(
        epoch_index: u32,
        lock: bool,
        lock_state: &str,
        lock_state_reason: Option<&str>,
    ) -> TrackEpoch {
        TrackEpoch {
            epoch: Epoch { index: epoch_index as u64 },
            sample_index: epoch_index as u64,
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            carrier_hz: Hertz(epoch_index as f64),
            code_phase_samples: Chips(epoch_index as f64 + 0.5),
            lock,
            lock_state: lock_state.to_string(),
            lock_state_reason: lock_state_reason.map(str::to_string),
            ..TrackEpoch::default()
        }
    }

    fn tracking_result_with_epochs(
        channel_id: u8,
        sat: SatId,
        epochs: Vec<TrackEpoch>,
    ) -> super::TrackingResult {
        let epochs = epochs
            .into_iter()
            .map(|mut epoch| {
                epoch.channel_id = Some(channel_id);
                epoch.channel_uid = super::tracking_channel_uid(sat, channel_id);
                epoch.sat = sat;
                epoch
            })
            .collect();
        super::TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "accepted".to_string(),
            epochs,
            transitions: Vec::new(),
        }
    }

    #[test]
    fn tracking_channel_state_report_emits_unique_steady_states_and_reacquired_marker() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            2,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, true, "tracking", Some("carrier_converged")),
                track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
                track_epoch_with_state(3, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(4, true, "tracking", Some("reacquired")),
                track_epoch_with_state(5, true, "tracking", Some("stable_tracking")),
            ],
        ));

        let emitted_states =
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
        assert_eq!(
            emitted_states,
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Lost,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Reacquired,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Locked);
        assert_eq!(report.final_reason.as_deref(), Some("stable_tracking"));
    }

    #[test]
    fn tracking_channel_state_report_marks_cn0_lock_refusal() {
        let sat = SatId { constellation: Constellation::Gps, prn: 16 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            1,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            ],
        ));

        let emitted_states =
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
        assert_eq!(
            emitted_states,
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::PullIn,
                super::TrackingChannelState::Refused,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Refused);
        assert_eq!(report.final_reason.as_deref(), Some("cn0_below_tracking_lock_floor"));
    }

    #[test]
    fn tracking_channel_state_report_keeps_degraded_final_state() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            3,
            sat,
            vec![
                track_epoch_with_state(0, true, "tracking", Some("carrier_converged")),
                track_epoch_with_state(1, true, "degraded", Some("signal_fade")),
            ],
        ));

        assert_eq!(
            report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>(),
            vec![
                super::TrackingChannelState::Acquired,
                super::TrackingChannelState::Locked,
                super::TrackingChannelState::Degraded,
            ]
        );
        assert_eq!(report.final_state, super::TrackingChannelState::Degraded);
        assert_eq!(report.final_reason.as_deref(), Some("signal_fade"));
    }

    #[test]
    fn tracking_channel_state_report_suppresses_duplicate_refused_markers() {
        let sat = SatId { constellation: Constellation::Gps, prn: 16 };
        let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
            1,
            sat,
            vec![
                track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
                track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
                track_epoch_with_state(3, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            ],
        ));

        let refused_count = report
            .emitted_states
            .iter()
            .filter(|event| event.state == super::TrackingChannelState::Refused)
            .count();
        assert_eq!(refused_count, 1, "{report:?}");
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
            "Degraded" => ChannelState::Degraded,
            "Lost" => ChannelState::Lost,
            _ => panic!("unsupported state {value}"),
        }
    }
}
