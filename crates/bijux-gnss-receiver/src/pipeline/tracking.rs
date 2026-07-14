#![allow(missing_docs)]

use std::collections::VecDeque;

use num_complex::Complex;
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    AcqHypothesis, AcqTrackingSeed, AcqUncertainty, Chips, Constellation, Cycles, FreqHz, Hertz,
    ReceiverSampleTrace, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, SignalBand,
    SignalCode, SignalComponentRole, SignalDelayAlignment, SignalSecondaryCodeSpec, SignalSpec,
    SignalSubcarrierSpec, TrackEpoch, TrackTransition, TrackingAssumptions, TrackingUncertainty,
    GPS_L1_CA_CARRIER_HZ,
};

use crate::engine::receiver_config::{ReceiverPipelineConfig, TrackingParams};
use crate::engine::runtime::{ReceiverRuntime, TraceRecord};
use crate::engine::signal_selection::default_signal_code_for_band;
use bijux_gnss_core::api::Sample;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::{
    advance_tracking_adaptation, anti_false_lock_detected as signal_anti_false_lock_detected,
    apply_carrier_tracking_loop as signal_apply_carrier_tracking_loop,
    apply_code_loop as signal_apply_code_loop, calibrated_lock_detector_thresholds,
    carrier_frequency_error_hz_from_phase_delta, carrier_phase_offset_radians, code_value_at_phase,
    coherent_integration_seconds as signal_coherent_integration_seconds,
    correlate_early_prompt_late, default_local_code_model_for_signal, discriminators,
    double_delta_dll_discriminator, epoch_start_code_phase_samples_from_receiver_phase,
    estimate_cn0_dbhz, estimate_tracking_uncertainty as signal_estimate_tracking_uncertainty,
    galileo_e5a_q_epoch_symbol, galileo_e5a_q_secondary_code, galileo_e5b_q_epoch_symbol,
    galileo_e5b_q_secondary_code, generate_galileo_e5a_q_code, generate_galileo_e5b_q_code,
    generate_gps_l2c_cl_code, gps_l5_q_epoch_symbol, normalize_dll_discriminator,
    prompt_power_ratio as signal_prompt_power_ratio,
    push_tracking_uncertainty_sample as signal_push_tracking_uncertainty_sample,
    refresh_lock_reference_cn0_dbhz as signal_refresh_lock_reference_cn0_dbhz,
    refresh_prompt_power_reference as signal_refresh_prompt_power_reference,
    resolved_signal_registry_entry, shared_path_code_rate_hz,
    update_windowed_tracking_cn0_estimate as signal_update_windowed_tracking_cn0_estimate,
    wrap_code_phase_samples, wrap_phase_cycles_signed, wrapped_code_phase_delta_samples,
    wrapped_phase_delta_cycles, LocalCodeModel, LockDetectorCalibrationInput,
    LockDetectorThresholds, TrackingAdaptationInput as SignalTrackingAdaptationInput,
    TrackingAdaptationState as SignalTrackingAdaptationState,
    TrackingLoopProfile as SignalTrackingLoopProfile, TrackingQualityClass,
    TrackingUncertaintyInputs as SignalTrackingUncertaintyInputs,
    GALILEO_E5A_Q_SECONDARY_CODE_CHIPS, GALILEO_E5B_Q_SECONDARY_CODE_CHIPS,
    GPS_L5_Q_PRIMARY_EPOCHS_PER_SYMBOL,
};

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
const DLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const PLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const FLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const PROMPT_POWER_DROP_RATIO_THRESHOLD: f32 = 0.2;
const DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO: f32 = 0.5;
const DISCRIMINATOR_INSTABILITY_REQUIRED_EPOCHS: u8 = 2;
const DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS: u16 = 2;
const SHORT_FADE_RECOVERY_GRACE_EPOCHS: u16 = 5;
const NARROW_BPSK_EARLY_LATE_SPACING_CHIPS: f64 = 0.25;
const NARROW_HIGH_RATE_EARLY_LATE_SPACING_CHIPS: f64 = 0.10;
const NARROW_SUBCARRIER_EARLY_LATE_SPACING_CHIPS: f64 = 0.25;
const SECONDARY_CODE_EARLY_LATE_SPACING_CHIPS: f64 = 0.5;
// If prompt energy returns near the short-fade boundary, the loops may need a
// few extra epochs to re-enter tracking without opening a long interruption gap.
const SHORT_FADE_RELOCK_EVIDENCE_GRACE_EPOCHS: u16 = 3;
// Enter steady tracking only after the carrier/code loops stay jointly locked
// across a short sustained window instead of a single optimistic epoch.
const PULL_IN_REQUIRED_STABLE_EPOCHS: u8 = 3;
const REACQUISITION_REQUIRED_LOST_EPOCHS: usize = 3;
const REACQUISITION_CONFIRMATION_EPOCHS: u8 = 2;
const REACQUISITION_PULL_IN_EPOCH_BUDGET: u8 = 20;
const REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ: f64 = 12.0;
const REACQUISITION_STABLE_TRACKING_EPOCHS: u8 = 5;
const TRACKING_CN0_WINDOW_EPOCHS: usize = 8;
const TRACKING_CN0_MIN_WINDOW_EPOCHS: usize = 4;
const TRACKING_UNCERTAINTY_WINDOW_EPOCHS: usize = 8;
const VECTOR_TRACKING_MIN_CONTRIBUTORS: usize = 2;
const VECTOR_TRACKING_MIN_CN0_DBHZ: f64 = 35.0;
const VECTOR_TRACKING_HISTORY_SECONDS: f64 = 0.050;
const VECTOR_TRACKING_MAX_CARRIER_AID_HZ: f64 = 25.0;
const VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ: f64 = 2.0;
const VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES: f64 = 2.0;
const VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S: f64 = 1_000.0;
const SAMPLE_RATE_MISMATCH_CATASTROPHIC_PHASE_STEP_MULTIPLIER: f64 = 8.0;
const LOW_RESOLUTION_DLL_MIN_SAMPLE_SEPARATION: f64 = 1.0;
const JOINT_COMPONENT_MIN_PROMPT_RATIO: f32 = 0.35;
const SECONDARY_CODE_SYNC_MIN_CONFIDENCE: f64 = 0.02;
const SECONDARY_CODE_SYNC_MIN_OBSERVED_CHIPS: usize = 4;
const CARRIER_AID_MIN_DOPPLER_WINDOW_HZ: f64 = 25_000.0;
const CARRIER_AID_DOPPLER_WINDOW_MARGIN_HZ: f64 = 500.0;
const SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER: f32 = 0.05;
const SUBCARRIER_AMBIGUITY_GUARD_OFFSETS_CHIPS: [f64; 2] = [-0.5, 0.5];
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

pub type CorrelatorOutput = bijux_gnss_signal::api::EarlyPromptLateCorrelation;

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
    carrier_rate_hz_per_s: f64,
    code_rate_hz: f64,
    code_rate_reference_hz: f64,
    code_phase_samples: f64,
    tracking_adaptation_state: SignalTrackingAdaptationState,
    tracking_loop_profile: SignalTrackingLoopProfile,
    signal_delay_alignment: Option<SignalDelayAlignment>,
    subcarrier_code_phase_refined: bool,
    acquisition_cn0_proxy_dbhz: f64,
    lock_reference_cn0_dbhz: f64,
    prev_prompt: Option<Complex<f32>>,
    prev_prompt_phase_cycles: Option<f64>,
    secondary_code_prompt_history: VecDeque<SecondaryCodePromptSample>,
    secondary_code_sync: Option<SecondaryCodeSyncResult>,
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
    vector_state: VectorTrackingState,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingMeasurement {
    sat: SatId,
    channel_id: u8,
    epoch_idx: u64,
    sample_index: u64,
    cn0_dbhz: f64,
    dll_error_samples: f64,
    pll_error_rad: f64,
    fll_error_hz: f64,
    code_rate_error_hz: f64,
    carrier_rate_hz_per_s: f64,
    prompt_locked: bool,
    dll_locked: bool,
    pll_locked: bool,
    fll_locked: bool,
    channel_state: ChannelState,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingPrediction {
    sample_index: u64,
    contributor_count: usize,
    mean_cn0_dbhz: f64,
    receiver_position_code_phase_error_samples: f64,
    receiver_clock_frequency_error_hz: f64,
    receiver_code_rate_error_hz: f64,
    receiver_motion_frequency_rate_hz_per_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct VectorTrackingApplication {
    prediction: VectorTrackingPrediction,
    carrier_frequency_correction_hz: f64,
    code_rate_correction_hz: f64,
    code_phase_correction_samples: f64,
    carrier_rate_correction_hz_per_s: f64,
}

#[derive(Debug, Clone, Default)]
struct VectorTrackingState {
    measurements: VecDeque<VectorTrackingMeasurement>,
}

impl VectorTrackingState {
    fn record(&mut self, measurement: VectorTrackingMeasurement, sample_rate_hz: f64) {
        if !vector_tracking_measurement_is_usable(measurement) {
            return;
        }
        self.measurements.push_back(measurement);
        let history_samples = vector_tracking_history_samples(sample_rate_hz);
        let latest_sample_index = measurement.sample_index;
        while self.measurements.front().is_some_and(|oldest| {
            latest_sample_index.saturating_sub(oldest.sample_index) > history_samples
        }) {
            self.measurements.pop_front();
        }
    }

    fn prediction_for(
        &self,
        sample_index: u64,
        sample_rate_hz: f64,
    ) -> Option<VectorTrackingPrediction> {
        let history_samples = vector_tracking_history_samples(sample_rate_hz);
        let mut latest_by_channel = Vec::new();
        for measurement in self.measurements.iter().rev().copied() {
            if measurement.sample_index > sample_index {
                continue;
            }
            if sample_index.saturating_sub(measurement.sample_index) > history_samples {
                continue;
            }
            if latest_by_channel.iter().any(|candidate: &VectorTrackingMeasurement| {
                candidate.channel_id == measurement.channel_id
            }) {
                continue;
            }
            latest_by_channel.push(measurement);
        }
        vector_tracking_prediction(&latest_by_channel)
    }
}

fn vector_tracking_history_samples(sample_rate_hz: f64) -> u64 {
    ((sample_rate_hz.max(1.0) * VECTOR_TRACKING_HISTORY_SECONDS).round() as u64).max(1)
}

fn vector_tracking_measurement_is_usable(measurement: VectorTrackingMeasurement) -> bool {
    let carrier_reliable = measurement.prompt_locked
        && measurement.fll_locked
        && measurement.cn0_dbhz >= VECTOR_TRACKING_MIN_CN0_DBHZ;
    let state_reliable = match measurement.channel_state {
        ChannelState::PullIn => carrier_reliable,
        ChannelState::Tracking | ChannelState::Degraded => {
            carrier_reliable && measurement.dll_locked && measurement.pll_locked
        }
        ChannelState::Idle | ChannelState::Acquired | ChannelState::Lost => false,
    };
    state_reliable
        && measurement.cn0_dbhz.is_finite()
        && measurement.dll_error_samples.is_finite()
        && measurement.pll_error_rad.is_finite()
        && measurement.fll_error_hz.is_finite()
        && measurement.code_rate_error_hz.is_finite()
        && measurement.carrier_rate_hz_per_s.is_finite()
}

fn vector_tracking_prediction(
    measurements: &[VectorTrackingMeasurement],
) -> Option<VectorTrackingPrediction> {
    if measurements.len() < VECTOR_TRACKING_MIN_CONTRIBUTORS {
        return None;
    }
    let mut weighted_clock_error_hz = 0.0;
    let mut weighted_position_error_samples = 0.0;
    let mut weighted_code_error_hz = 0.0;
    let mut weighted_motion_rate_hz_per_s = 0.0;
    let mut weighted_cn0_dbhz = 0.0;
    let mut weight_sum = 0.0;
    let mut code_weight_sum = 0.0;
    let mut sample_index = 0;
    for measurement in measurements {
        let weight = vector_tracking_cn0_weight(measurement.cn0_dbhz);
        weighted_clock_error_hz += measurement.fll_error_hz * weight;
        if measurement.dll_locked {
            weighted_position_error_samples += measurement.dll_error_samples * weight;
            weighted_code_error_hz += measurement.code_rate_error_hz * weight;
            code_weight_sum += weight;
        }
        weighted_motion_rate_hz_per_s += measurement.carrier_rate_hz_per_s * weight;
        weighted_cn0_dbhz += measurement.cn0_dbhz * weight;
        weight_sum += weight;
        sample_index = sample_index.max(measurement.sample_index);
    }
    if weight_sum <= f64::EPSILON {
        return None;
    }
    Some(VectorTrackingPrediction {
        sample_index,
        contributor_count: measurements.len(),
        mean_cn0_dbhz: weighted_cn0_dbhz / weight_sum,
        receiver_position_code_phase_error_samples: if code_weight_sum > f64::EPSILON {
            weighted_position_error_samples / code_weight_sum
        } else {
            0.0
        },
        receiver_clock_frequency_error_hz: weighted_clock_error_hz / weight_sum,
        receiver_code_rate_error_hz: if code_weight_sum > f64::EPSILON {
            weighted_code_error_hz / code_weight_sum
        } else {
            0.0
        },
        receiver_motion_frequency_rate_hz_per_s: weighted_motion_rate_hz_per_s / weight_sum,
    })
}

fn vector_tracking_cn0_weight(cn0_dbhz: f64) -> f64 {
    let relative_db = (cn0_dbhz - VECTOR_TRACKING_MIN_CN0_DBHZ).clamp(0.0, 15.0);
    10.0_f64.powf(relative_db / 10.0)
}

fn vector_tracking_application(
    prediction: VectorTrackingPrediction,
    state: &LoopState,
) -> Option<VectorTrackingApplication> {
    let gain = vector_tracking_channel_gain(state.state);
    if gain <= f64::EPSILON {
        return None;
    }
    Some(VectorTrackingApplication {
        prediction,
        carrier_frequency_correction_hz: prediction
            .receiver_clock_frequency_error_hz
            .clamp(-VECTOR_TRACKING_MAX_CARRIER_AID_HZ, VECTOR_TRACKING_MAX_CARRIER_AID_HZ)
            * gain,
        code_rate_correction_hz: prediction
            .receiver_code_rate_error_hz
            .clamp(-VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ, VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ)
            * gain,
        code_phase_correction_samples: prediction.receiver_position_code_phase_error_samples.clamp(
            -VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES,
            VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES,
        ) * gain,
        carrier_rate_correction_hz_per_s: prediction.receiver_motion_frequency_rate_hz_per_s.clamp(
            -VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S,
            VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S,
        ) * gain,
    })
}

fn vector_tracking_channel_gain(state: ChannelState) -> f64 {
    match state {
        ChannelState::Acquired | ChannelState::PullIn => 0.35,
        ChannelState::Degraded => 0.20,
        ChannelState::Tracking => 0.05,
        ChannelState::Idle | ChannelState::Lost => 0.0,
    }
}

fn vector_tracking_aiding_mode_label(signal_model: &TrackingSignalModel) -> String {
    match signal_model.aiding_mode {
        TrackingAidingMode::None => "vector_receiver_state".to_string(),
        TrackingAidingMode::PilotCarrier => "pilot_carrier+vector_receiver_state".to_string(),
    }
}

fn vector_tracking_provenance(application: VectorTrackingApplication) -> String {
    format!(
        "vector_tracking=applied vector_prediction_sample_index={} vector_contributors={} vector_mean_cn0_dbhz={:.3} vector_position_code_phase_error_samples={:.6} vector_clock_frequency_error_hz={:.6} vector_code_rate_error_hz={:.6} vector_motion_frequency_rate_hz_per_s={:.6} vector_carrier_frequency_correction_hz={:.6} vector_code_rate_correction_hz={:.6} vector_code_phase_correction_samples={:.6} vector_carrier_rate_correction_hz_per_s={:.6}",
        application.prediction.sample_index,
        application.prediction.contributor_count,
        application.prediction.mean_cn0_dbhz,
        application.prediction.receiver_position_code_phase_error_samples,
        application.prediction.receiver_clock_frequency_error_hz,
        application.prediction.receiver_code_rate_error_hz,
        application.prediction.receiver_motion_frequency_rate_hz_per_s,
        application.carrier_frequency_correction_hz,
        application.code_rate_correction_hz,
        application.code_phase_correction_samples,
        application.carrier_rate_correction_hz_per_s,
    )
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
    subcarrier_code_phase_refined: bool,
    acq_to_track_state: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CarrierAidingReference {
    tracked_carrier_hz: f64,
    tracked_doppler_hz: f64,
    code_rate_hz: f64,
}

#[derive(Debug, Clone)]
struct TrackingSignalModel {
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    signal_spec: SignalSpec,
    component_role: SignalComponentRole,
    secondary_code: Option<SignalSecondaryCodeSpec>,
    code_rate_hz: f64,
    code_length: usize,
    local_code_model: LocalCodeModel,
    discriminator_family: TrackingDiscriminatorFamily,
    phase_transition_source: TrackingPhaseTransitionSource,
    aiding_mode: TrackingAidingMode,
    pilot_component: Option<TrackingComponentModel>,
    data_symbol_component: Option<TrackingComponentModel>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingAidingMode {
    None,
    PilotCarrier,
}

impl TrackingAidingMode {
    fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::PilotCarrier => "pilot_carrier",
        }
    }
}

#[derive(Debug, Clone)]
struct TrackingComponentModel {
    role: SignalComponentRole,
    code_length: usize,
    phase_transition_source: TrackingPhaseTransitionSource,
    local_code_model: TrackingComponentLocalCodeModel,
}

impl TrackingComponentModel {
    fn sample_value_from_primary_phase(
        &self,
        primary_chip_phase: f64,
        primary_code_period_index: usize,
        primary_code_length: usize,
    ) -> f32 {
        let total_chip_phase =
            primary_code_period_index as f64 * primary_code_length as f64 + primary_chip_phase;
        let component_code_length = self.code_length.max(1) as f64;
        let component_chip_phase = total_chip_phase.rem_euclid(component_code_length);
        let component_primary_code_period_index = if total_chip_phase <= 0.0 {
            0
        } else {
            (total_chip_phase / component_code_length).floor() as usize
        };
        self.local_code_model
            .sample_tracking_value(component_chip_phase, component_primary_code_period_index)
            .unwrap_or(0.0)
    }

    fn secondary_code_period(&self) -> Option<usize> {
        self.local_code_model.secondary_code_period()
    }

    fn secondary_code_symbol(&self, primary_code_period_index: usize) -> Option<i8> {
        self.local_code_model.secondary_code_symbol(primary_code_period_index)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodePromptSample {
    primary_code_period_index: usize,
    prompt: Complex<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodePhaseScore {
    phase_periods: usize,
    likelihood: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodeSyncResult {
    phase_periods: usize,
    confidence: f64,
    best_likelihood: f64,
    next_best_likelihood: f64,
    observed_periods: usize,
    accepted: bool,
}

#[derive(Debug, Clone, PartialEq)]
enum TrackingComponentLocalCodeModel {
    Local(LocalCodeModel),
    GpsL2cCl { code: Vec<i8> },
    GalileoE5aQ { code: Vec<i8>, secondary_code: [i8; 100] },
    GalileoE5bQ { code: Vec<i8>, secondary_code: [i8; 100] },
}

impl TrackingComponentLocalCodeModel {
    fn sample_tracking_value(
        &self,
        chip_phase: f64,
        primary_code_period_index: usize,
    ) -> Result<f32, bijux_gnss_signal::api::SignalError> {
        match self {
            Self::Local(local_code_model) => {
                local_code_model.sample_tracking_value(chip_phase, primary_code_period_index)
            }
            Self::GpsL2cCl { code } => code_value_at_phase(code, chip_phase),
            Self::GalileoE5aQ { code, secondary_code } => {
                Ok(code_value_at_phase(code, chip_phase)?
                    * galileo_e5a_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
            Self::GalileoE5bQ { code, secondary_code } => {
                Ok(code_value_at_phase(code, chip_phase)?
                    * galileo_e5b_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
        }
    }

    fn secondary_code_period(&self) -> Option<usize> {
        match self {
            Self::Local(LocalCodeModel::GpsL5Q { .. }) => Some(GPS_L5_Q_PRIMARY_EPOCHS_PER_SYMBOL),
            Self::GalileoE5aQ { .. } => Some(GALILEO_E5A_Q_SECONDARY_CODE_CHIPS),
            Self::GalileoE5bQ { .. } => Some(GALILEO_E5B_Q_SECONDARY_CODE_CHIPS),
            Self::Local(_) | Self::GpsL2cCl { .. } => None,
        }
    }

    fn secondary_code_symbol(&self, primary_code_period_index: usize) -> Option<i8> {
        match self {
            Self::Local(LocalCodeModel::GpsL5Q { .. }) => {
                Some(gps_l5_q_epoch_symbol(primary_code_period_index))
            }
            Self::GalileoE5aQ { secondary_code, .. } => {
                Some(galileo_e5a_q_epoch_symbol(secondary_code, primary_code_period_index))
            }
            Self::GalileoE5bQ { secondary_code, .. } => {
                Some(galileo_e5b_q_epoch_symbol(secondary_code, primary_code_period_index))
            }
            Self::Local(_) | Self::GpsL2cCl { .. } => None,
        }
    }
}

fn secondary_code_sync_from_prompt_history(
    component: &TrackingComponentModel,
    prompt_history: &[SecondaryCodePromptSample],
) -> Option<SecondaryCodeSyncResult> {
    let period = component.secondary_code_period()?;
    if period == 0 || prompt_history.len() < SECONDARY_CODE_SYNC_MIN_OBSERVED_CHIPS {
        return None;
    }
    let mut scores = (0..period)
        .map(|phase| secondary_code_phase_score(component, prompt_history, phase))
        .collect::<Vec<_>>();
    scores.sort_by(|left, right| {
        right
            .likelihood
            .partial_cmp(&left.likelihood)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.phase_periods.cmp(&right.phase_periods))
    });
    let best = scores.first().copied()?;
    let next_best = scores
        .get(1)
        .copied()
        .unwrap_or(SecondaryCodePhaseScore { phase_periods: best.phase_periods, likelihood: 0.0 });
    let confidence = secondary_code_sync_confidence(best.likelihood, next_best.likelihood);
    Some(SecondaryCodeSyncResult {
        phase_periods: best.phase_periods,
        confidence,
        best_likelihood: best.likelihood,
        next_best_likelihood: next_best.likelihood,
        observed_periods: prompt_history.len(),
        accepted: confidence >= SECONDARY_CODE_SYNC_MIN_CONFIDENCE,
    })
}

fn secondary_code_phase_score(
    component: &TrackingComponentModel,
    prompt_history: &[SecondaryCodePromptSample],
    phase_periods: usize,
) -> SecondaryCodePhaseScore {
    let mut accumulated_alignment = 0.0_f64;
    let mut transition_count = 0usize;
    let mut previous_prompt: Option<Complex<f64>> = None;
    let reference_period =
        prompt_history.first().map(|sample| sample.primary_code_period_index).unwrap_or_default();
    for sample in prompt_history {
        let relative_period = sample.primary_code_period_index.saturating_sub(reference_period);
        let Some(observed_symbol) =
            component.secondary_code_symbol(sample.primary_code_period_index)
        else {
            continue;
        };
        let Some(candidate_symbol) =
            component.secondary_code_symbol(phase_periods + relative_period)
        else {
            continue;
        };
        let raw_prompt = sample.prompt * observed_symbol as f32;
        let candidate_prompt = Complex::new(
            (raw_prompt.re * candidate_symbol as f32) as f64,
            (raw_prompt.im * candidate_symbol as f32) as f64,
        );
        if let Some(previous_prompt) = previous_prompt {
            let norm_product = previous_prompt.norm() * candidate_prompt.norm();
            if norm_product > f64::EPSILON {
                let alignment = (previous_prompt.conj() * candidate_prompt).re / norm_product;
                accumulated_alignment += ((alignment + 1.0) * 0.5).clamp(0.0, 1.0);
                transition_count += 1;
            }
        }
        previous_prompt = Some(candidate_prompt);
    }
    let likelihood =
        if transition_count > 0 { accumulated_alignment / transition_count as f64 } else { 0.0 };
    SecondaryCodePhaseScore { phase_periods, likelihood }
}

fn secondary_code_sync_confidence(best_likelihood: f64, next_best_likelihood: f64) -> f64 {
    if best_likelihood <= f64::EPSILON {
        return 0.0;
    }
    ((best_likelihood - next_best_likelihood.max(0.0)) / best_likelihood).clamp(0.0, 1.0)
}

fn secondary_code_sync_component(
    signal_model: &TrackingSignalModel,
) -> Option<TrackingComponentModel> {
    let component = signal_model.carrier_component();
    (component.phase_transition_source == TrackingPhaseTransitionSource::SecondaryCode
        && component.secondary_code_period().is_some())
    .then_some(component)
}

fn update_secondary_code_synchronization(
    signal_model: &TrackingSignalModel,
    state: &mut LoopState,
    primary_code_period_index: usize,
    carrier_prompt: Complex<f32>,
) -> Option<SecondaryCodeSyncResult> {
    let component = secondary_code_sync_component(signal_model)?;
    let period = component.secondary_code_period()?;
    if !carrier_prompt.re.is_finite() || !carrier_prompt.im.is_finite() {
        return state.secondary_code_sync;
    }
    state
        .secondary_code_prompt_history
        .push_back(SecondaryCodePromptSample { primary_code_period_index, prompt: carrier_prompt });
    while state.secondary_code_prompt_history.len() > period {
        state.secondary_code_prompt_history.pop_front();
    }
    let prompt_history = state.secondary_code_prompt_history.iter().copied().collect::<Vec<_>>();
    state.secondary_code_sync = select_secondary_code_synchronization(
        state.secondary_code_sync,
        secondary_code_sync_from_prompt_history(&component, &prompt_history),
    );
    state.secondary_code_sync
}

fn select_secondary_code_synchronization(
    current: Option<SecondaryCodeSyncResult>,
    candidate: Option<SecondaryCodeSyncResult>,
) -> Option<SecondaryCodeSyncResult> {
    match (current, candidate) {
        (None, candidate) => candidate,
        (current @ Some(_), None) => current,
        (Some(current), Some(candidate)) if candidate.accepted && !current.accepted => {
            Some(candidate)
        }
        (Some(current), Some(candidate))
            if candidate.accepted == current.accepted
                && candidate.confidence > current.confidence =>
        {
            Some(candidate)
        }
        (Some(current), Some(_)) => Some(current),
    }
}

fn carrier_phase_transition_source_for_prompt(
    signal_model: &TrackingSignalModel,
    secondary_code_sync: Option<SecondaryCodeSyncResult>,
) -> TrackingPhaseTransitionSource {
    let transition_source = signal_model.carrier_phase_transition_source();
    if transition_source != TrackingPhaseTransitionSource::SecondaryCode {
        return transition_source;
    }
    if secondary_code_sync.is_some_and(|sync| sync.accepted) {
        TrackingPhaseTransitionSource::SecondaryCode
    } else {
        TrackingPhaseTransitionSource::None
    }
}

fn secondary_code_sync_provenance(
    signal_model: &TrackingSignalModel,
    sync: Option<SecondaryCodeSyncResult>,
) -> Option<String> {
    secondary_code_sync_component(signal_model)?;
    Some(match sync {
        Some(sync) => format!(
            " secondary_code_sync={} secondary_code_phase_periods={} secondary_code_sync_confidence={:.6} secondary_code_best_likelihood={:.6} secondary_code_next_likelihood={:.6} secondary_code_observed_periods={}",
            if sync.accepted { "accepted" } else { "rejected" },
            sync.phase_periods,
            sync.confidence,
            sync.best_likelihood,
            sync.next_best_likelihood,
            sync.observed_periods
        ),
        None => " secondary_code_sync=insufficient".to_string(),
    })
}

fn prompt_center_primary_code_period_index(
    epoch_primary_code_period_index: usize,
    base_chip_phase: f64,
    tracked_chips_per_sample: f64,
    coherent_samples: usize,
    primary_code_length: usize,
) -> usize {
    let code_length = primary_code_length.max(1) as f64;
    let prompt_center_chip_phase =
        base_chip_phase + tracked_chips_per_sample * coherent_samples as f64 * 0.5;
    let period_offset = (prompt_center_chip_phase / code_length).floor().max(0.0) as usize;
    epoch_primary_code_period_index.saturating_add(period_offset)
}

#[derive(Debug, Clone, Copy)]
struct TrackingEpochCorrelation {
    primary: CorrelatorOutput,
    double_delta_outer: Option<CorrelatorOutput>,
    carrier_prompt: Complex<f32>,
    carrier_prompt_source: CarrierPromptSource,
    data_prompt: Option<Complex<f32>>,
    secondary_code_prompt_period_index: usize,
    subcarrier_ambiguity_guard: Option<SubcarrierAmbiguityGuard>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarrierPromptSource {
    Primary,
    Pilot,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SubcarrierAmbiguityGuard {
    prompt_power: f32,
    strongest_alternate_power: f32,
    strongest_alternate_offset_chips: f64,
    prompt_relative_power: f32,
}

impl SubcarrierAmbiguityGuard {
    fn detected(self) -> bool {
        !self.prompt_relative_power.is_finite()
            || self.prompt_relative_power < SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingDiscriminatorFamily {
    EarlyPromptLate,
    BocEarlyPromptLate,
    CbocEarlyPromptLate,
}

impl TrackingDiscriminatorFamily {
    fn label(self) -> &'static str {
        match self {
            Self::EarlyPromptLate => "early_prompt_late",
            Self::BocEarlyPromptLate => "unambiguous_boc_early_prompt_late",
            Self::CbocEarlyPromptLate => "unambiguous_cboc_early_prompt_late",
        }
    }

    fn requires_unambiguous_code_lock(self) -> bool {
        matches!(self, Self::BocEarlyPromptLate | Self::CbocEarlyPromptLate)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CodeDiscriminatorMode {
    EarlyPromptLate,
    DoubleDeltaEarlyPromptLate,
}

impl CodeDiscriminatorMode {
    fn label(self) -> &'static str {
        match self {
            Self::EarlyPromptLate => "early_prompt_late",
            Self::DoubleDeltaEarlyPromptLate => "double_delta_early_prompt_late",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingPhaseTransitionSource {
    None,
    DataSymbol,
    SecondaryCode,
}

impl TrackingPhaseTransitionSource {
    fn allows_half_cycle_transition(self) -> bool {
        !matches!(self, Self::None)
    }

    fn reports_navigation_bit_lock(self) -> bool {
        matches!(self, Self::DataSymbol)
    }

    fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::DataSymbol => "data_symbol",
            Self::SecondaryCode => "secondary_code",
        }
    }
}

impl TrackingSignalModel {
    fn for_sat(config: &ReceiverPipelineConfig, sat: SatId) -> Self {
        match sat.constellation {
            Constellation::Gps => {
                Self::for_sat_signal_band(config, sat, SignalBand::L1, SignalCode::Ca, None)
            }
            Constellation::Galileo => {
                Self::for_sat_signal_band(config, sat, SignalBand::E1, SignalCode::E1B, None)
            }
            Constellation::Beidou => {
                Self::for_sat_signal_band(config, sat, SignalBand::B1, SignalCode::B1I, None)
            }
            Constellation::Glonass => {
                Self::for_sat_signal_band(config, sat, SignalBand::L1, SignalCode::Unknown, None)
            }
            _ => Self::fallback(config, sat, SignalBand::L1),
        }
    }

    fn for_sat_signal_band(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
        glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    ) -> Self {
        let signal_code = if signal_code == SignalCode::Unknown {
            default_signal_code_for_band(sat.constellation, signal_band)
        } else {
            signal_code
        };
        let registry_entry = resolved_signal_registry_entry(
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        )
        .ok()
        .flatten();
        match (
            default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten(),
            registry_entry,
        ) {
            (Some(local_code_model), Some(registry_entry)) => {
                let component = registry_entry
                    .default_component()
                    .expect("tracking registry entry must expose a default component");
                let (aiding_mode, pilot_component, data_symbol_component) =
                    joint_tracking_components(
                        sat,
                        signal_band,
                        signal_code,
                        glonass_frequency_channel,
                        &registry_entry,
                        component,
                    );
                Self {
                    signal_band,
                    signal_code,
                    glonass_frequency_channel: ((sat.constellation == Constellation::Glonass)
                        && (signal_band == SignalBand::L1))
                        .then_some(glonass_frequency_channel)
                        .flatten(),
                    signal_spec: registry_entry.spec,
                    component_role: component.role,
                    secondary_code: component.secondary_code,
                    code_rate_hz: component.primary_code_rate_hz,
                    code_length: component.primary_code_chips as usize,
                    local_code_model,
                    discriminator_family: tracking_discriminator_family(component.subcarrier),
                    phase_transition_source: tracking_phase_transition_source(component),
                    aiding_mode,
                    pilot_component,
                    data_symbol_component,
                }
            }
            _ => Self::fallback(config, sat, signal_band),
        }
    }

    fn fallback(config: &ReceiverPipelineConfig, sat: SatId, signal_band: SignalBand) -> Self {
        let local_code_model = if sat.constellation == Constellation::Gps {
            LocalCodeModel::gps_l1_ca_or_ones(sat.prn)
        } else {
            LocalCodeModel::ones(config.code_length.max(1), config.code_freq_basis_hz)
        };
        Self {
            signal_band,
            signal_code: default_signal_code_for_band(sat.constellation, signal_band),
            glonass_frequency_channel: None,
            signal_spec: SignalSpec {
                constellation: sat.constellation,
                band: signal_band,
                code: default_signal_code_for_band(sat.constellation, signal_band),
                code_rate_hz: config.code_freq_basis_hz,
                carrier_hz: FreqHz::new(0.0),
            },
            component_role: SignalComponentRole::Data,
            secondary_code: None,
            code_rate_hz: config.code_freq_basis_hz,
            code_length: config.code_length.max(1),
            local_code_model,
            discriminator_family: TrackingDiscriminatorFamily::EarlyPromptLate,
            phase_transition_source: TrackingPhaseTransitionSource::None,
            aiding_mode: TrackingAidingMode::None,
            pilot_component: None,
            data_symbol_component: None,
        }
    }

    fn supports_tracking(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
        glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    ) -> bool {
        default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten().is_some()
            && resolved_signal_registry_entry(
                sat,
                signal_band,
                signal_code,
                glonass_frequency_channel,
            )
            .ok()
            .flatten()
            .and_then(|entry| entry.default_component().copied())
            .is_some()
    }

    fn samples_per_code(&self, sample_rate_hz: f64) -> usize {
        samples_per_code(sample_rate_hz, self.code_rate_hz, self.code_length)
    }

    fn tracking_epoch_samples(
        &self,
        sample_rate_hz: f64,
        tracking_params: TrackingParams,
    ) -> usize {
        self.samples_per_code(sample_rate_hz)
            .saturating_mul(tracking_params.integration_ms.max(1) as usize)
    }

    fn nominal_chips_per_sample(&self, sample_rate_hz: f64) -> f64 {
        self.code_rate_hz / sample_rate_hz
    }

    fn value_at_phase(&self, chip_phase: f64, epoch_primary_code_period_index: usize) -> f32 {
        let code_length = self.code_length.max(1) as f64;
        let period_offset = (chip_phase / code_length).floor() as i64;
        let primary_code_period_index =
            (epoch_primary_code_period_index as i64 + period_offset).max(0) as usize;
        let wrapped_chip_phase = chip_phase.rem_euclid(code_length);
        self.local_code_model
            .sample_tracking_value(wrapped_chip_phase, primary_code_period_index)
            .unwrap_or(0.0)
    }

    fn nominal_carrier_hz(&self) -> f64 {
        self.signal_spec.carrier_hz.0
    }

    fn carrier_phase_transition_source(&self) -> TrackingPhaseTransitionSource {
        self.carrier_component().phase_transition_source
    }

    fn carrier_component(&self) -> TrackingComponentModel {
        self.pilot_component.clone().unwrap_or_else(|| TrackingComponentModel {
            role: self.component_role,
            code_length: self.code_length,
            phase_transition_source: self.phase_transition_source,
            local_code_model: TrackingComponentLocalCodeModel::Local(self.local_code_model.clone()),
        })
    }

    fn data_symbol_component(&self) -> Option<&TrackingComponentModel> {
        self.data_symbol_component.as_ref()
    }

    fn supports_epoch_data_symbol_sign_recovery(&self) -> bool {
        self.data_symbol_component.is_some() && !self.supports_navigation_bit_sign_recovery()
    }

    fn supports_navigation_bit_sign_recovery(&self) -> bool {
        self.signal_spec.constellation == Constellation::Gps
            && self.signal_band == SignalBand::L1
            && self.signal_code == SignalCode::Ca
            && self.phase_transition_source.reports_navigation_bit_lock()
    }
}

fn signal_default_early_late_spacing_chips(signal_model: &TrackingSignalModel) -> f64 {
    match signal_model.discriminator_family {
        TrackingDiscriminatorFamily::BocEarlyPromptLate
        | TrackingDiscriminatorFamily::CbocEarlyPromptLate => {
            NARROW_SUBCARRIER_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.phase_transition_source
                == TrackingPhaseTransitionSource::SecondaryCode =>
        {
            SECONDARY_CODE_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.code_rate_hz >= 10_000_000.0 - f64::EPSILON =>
        {
            NARROW_HIGH_RATE_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate => NARROW_BPSK_EARLY_LATE_SPACING_CHIPS,
    }
}

fn resolve_signal_tracking_params(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
) -> TrackingParams {
    let mut params = config.tracking_params(signal_model.signal_band);
    if config.tracking_per_band.iter().any(|profile| profile.band == signal_model.signal_band) {
        return params;
    }
    params.early_late_spacing_chips =
        params.early_late_spacing_chips.min(signal_default_early_late_spacing_chips(signal_model));
    params
}

fn code_discriminator_mode(signal_model: &TrackingSignalModel) -> CodeDiscriminatorMode {
    match signal_model.discriminator_family {
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.phase_transition_source
                != TrackingPhaseTransitionSource::SecondaryCode =>
        {
            CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate
        }
        TrackingDiscriminatorFamily::EarlyPromptLate => CodeDiscriminatorMode::EarlyPromptLate,
        TrackingDiscriminatorFamily::BocEarlyPromptLate
        | TrackingDiscriminatorFamily::CbocEarlyPromptLate => {
            CodeDiscriminatorMode::EarlyPromptLate
        }
    }
}

fn tracking_dll_discriminator(correlation: &TrackingEpochCorrelation) -> f32 {
    let (early_late_dll_err, _, _, _) = discriminators(
        correlation.primary.early,
        correlation.primary.prompt,
        correlation.primary.late,
        None,
    );
    if let Some(outer) = correlation.double_delta_outer {
        let double_delta_dll_err = double_delta_dll_discriminator(
            correlation.primary.early,
            correlation.primary.late,
            outer.early,
            outer.late,
        );
        if double_delta_dll_err.abs() < early_late_dll_err.abs() {
            return double_delta_dll_err;
        }
        return early_late_dll_err;
    }
    early_late_dll_err
}

fn joint_tracking_components(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    registry_entry: &bijux_gnss_core::api::SignalRegistryEntry,
    primary_component: &bijux_gnss_core::api::SignalComponentSpec,
) -> (TrackingAidingMode, Option<TrackingComponentModel>, Option<TrackingComponentModel>) {
    let data_component = registry_entry.component(SignalComponentRole::Data);
    let pilot_component = registry_entry.component(SignalComponentRole::Pilot);

    if data_component.is_some() && pilot_component.is_some() {
        let built_pilot = (primary_component.role != SignalComponentRole::Pilot)
            .then(|| {
                tracking_component_model_for_signal_role(
                    sat,
                    signal_band,
                    signal_code,
                    SignalComponentRole::Pilot,
                    glonass_frequency_channel,
                )
            })
            .flatten();
        let built_data = (primary_component.role == SignalComponentRole::Pilot)
            .then(|| {
                tracking_component_model_for_signal_role(
                    sat,
                    signal_band,
                    signal_code,
                    SignalComponentRole::Data,
                    glonass_frequency_channel,
                )
            })
            .flatten()
            .or_else(|| {
                (primary_component.role == SignalComponentRole::Data).then(|| {
                    TrackingComponentModel {
                        role: SignalComponentRole::Data,
                        code_length: primary_component.primary_code_chips as usize,
                        phase_transition_source: tracking_phase_transition_source(
                            primary_component,
                        ),
                        local_code_model: TrackingComponentLocalCodeModel::Local(
                            default_local_code_model_for_signal(sat, signal_band, signal_code)
                                .ok()
                                .flatten()
                                .expect("primary data component local code model"),
                        ),
                    }
                })
            });
        return (TrackingAidingMode::PilotCarrier, built_pilot, built_data);
    }

    let companion_signal_code = complementary_joint_tracking_signal_code(signal_band, signal_code);
    let Some(companion_signal_code) = companion_signal_code else {
        return (TrackingAidingMode::None, None, None);
    };
    let companion_role = if primary_component.role == SignalComponentRole::Data {
        SignalComponentRole::Pilot
    } else {
        SignalComponentRole::Data
    };
    let companion_component = tracking_component_model_for_signal_role(
        sat,
        signal_band,
        companion_signal_code,
        companion_role,
        glonass_frequency_channel,
    );
    match (primary_component.role, companion_component) {
        (SignalComponentRole::Data, Some(pilot_component)) => (
            TrackingAidingMode::PilotCarrier,
            Some(pilot_component),
            default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten().map(
                |local_code_model| TrackingComponentModel {
                    role: SignalComponentRole::Data,
                    code_length: primary_component.primary_code_chips as usize,
                    phase_transition_source: tracking_phase_transition_source(primary_component),
                    local_code_model: TrackingComponentLocalCodeModel::Local(local_code_model),
                },
            ),
        ),
        (SignalComponentRole::Pilot, Some(data_component)) => {
            (TrackingAidingMode::PilotCarrier, None, Some(data_component))
        }
        _ => (TrackingAidingMode::None, None, None),
    }
}

fn complementary_joint_tracking_signal_code(
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> Option<SignalCode> {
    match (signal_band, signal_code) {
        (SignalBand::L5, SignalCode::L5I) => Some(SignalCode::L5Q),
        (SignalBand::L5, SignalCode::L5Q) => Some(SignalCode::L5I),
        _ => None,
    }
}

fn tracking_component_model_for_signal_role(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    role: SignalComponentRole,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
) -> Option<TrackingComponentModel> {
    let registry_entry =
        resolved_signal_registry_entry(sat, signal_band, signal_code, glonass_frequency_channel)
            .ok()
            .flatten()?;
    let component = registry_entry.component(role)?;
    let local_code_model =
        tracking_component_local_code_model_for_signal_role(sat, signal_band, signal_code, role)?;
    Some(TrackingComponentModel {
        role,
        code_length: component.primary_code_chips as usize,
        phase_transition_source: tracking_phase_transition_source(component),
        local_code_model,
    })
}

fn tracking_component_local_code_model_for_signal_role(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    role: SignalComponentRole,
) -> Option<TrackingComponentLocalCodeModel> {
    match (sat.constellation, signal_band, signal_code, role) {
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GpsL2cCl {
                code: generate_gps_l2c_cl_code(sat.prn).ok()?,
            })
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GalileoE5aQ {
                code: generate_galileo_e5a_q_code(sat.prn).ok()?,
                secondary_code: galileo_e5a_q_secondary_code(sat.prn).ok()?,
            })
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5b, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GalileoE5bQ {
                code: generate_galileo_e5b_q_code(sat.prn).ok()?,
                secondary_code: galileo_e5b_q_secondary_code(sat.prn).ok()?,
            })
        }
        _ => default_local_code_model_for_signal(sat, signal_band, signal_code)
            .ok()
            .flatten()
            .map(TrackingComponentLocalCodeModel::Local),
    }
}

fn tracking_discriminator_family(subcarrier: SignalSubcarrierSpec) -> TrackingDiscriminatorFamily {
    match subcarrier {
        SignalSubcarrierSpec::None => TrackingDiscriminatorFamily::EarlyPromptLate,
        SignalSubcarrierSpec::Boc { .. } => TrackingDiscriminatorFamily::BocEarlyPromptLate,
        SignalSubcarrierSpec::Cboc { .. } => TrackingDiscriminatorFamily::CbocEarlyPromptLate,
    }
}

fn subcarrier_ambiguity_guard(
    signal_model: &TrackingSignalModel,
    samples: &[Complex<f32>],
    sample_rate_hz: f64,
    carrier_freq_hz: f64,
    carrier_phase_cycles: f64,
    base_chip_phase: f64,
    tracked_chips_per_sample: f64,
    epoch_primary_code_period_index: usize,
    prompt: Complex<f32>,
) -> Option<SubcarrierAmbiguityGuard> {
    if !signal_model.discriminator_family.requires_unambiguous_code_lock() {
        return None;
    }

    let prompt_power = prompt.norm_sqr();
    if !prompt_power.is_finite() {
        return Some(SubcarrierAmbiguityGuard {
            prompt_power,
            strongest_alternate_power: f32::INFINITY,
            strongest_alternate_offset_chips: 0.0,
            prompt_relative_power: 0.0,
        });
    }

    let mut strongest_alternate_power = 0.0_f32;
    let mut strongest_alternate_offset_chips = 0.0_f64;
    for offset_chips in SUBCARRIER_AMBIGUITY_GUARD_OFFSETS_CHIPS {
        let alternate = correlate_early_prompt_late(
            samples,
            sample_rate_hz,
            carrier_freq_hz,
            carrier_phase_offset_radians(carrier_phase_cycles),
            base_chip_phase + offset_chips,
            tracked_chips_per_sample,
            0.0,
            |chip_phase| signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index),
        )
        .prompt;
        let alternate_power = alternate.norm_sqr();
        if alternate_power.is_finite() && alternate_power > strongest_alternate_power {
            strongest_alternate_power = alternate_power;
            strongest_alternate_offset_chips = offset_chips;
        }
    }

    let prompt_relative_power = if strongest_alternate_power > f32::EPSILON {
        prompt_power / strongest_alternate_power
    } else if prompt_power.is_finite() {
        f32::INFINITY
    } else {
        0.0
    };

    Some(SubcarrierAmbiguityGuard {
        prompt_power,
        strongest_alternate_power,
        strongest_alternate_offset_chips,
        prompt_relative_power,
    })
}

fn subcarrier_ambiguity_detected(guard: Option<SubcarrierAmbiguityGuard>) -> bool {
    guard.is_some_and(SubcarrierAmbiguityGuard::detected)
}

fn subcarrier_ambiguity_provenance(guard: Option<SubcarrierAmbiguityGuard>) -> Option<String> {
    let guard = guard?;
    Some(format!(
        " subcarrier_ambiguity_guard=side_peak_guard prompt_relative_power={:.6} strongest_alternate_offset_chips={:.6} strongest_alternate_power={:.6}",
        guard.prompt_relative_power,
        guard.strongest_alternate_offset_chips,
        guard.strongest_alternate_power,
    ))
}

fn tracking_phase_transition_source(
    component: &bijux_gnss_core::api::SignalComponentSpec,
) -> TrackingPhaseTransitionSource {
    if component.secondary_code.is_some() {
        TrackingPhaseTransitionSource::SecondaryCode
    } else if component.symbol_period_s.is_some() && component.role == SignalComponentRole::Data {
        TrackingPhaseTransitionSource::DataSymbol
    } else {
        TrackingPhaseTransitionSource::None
    }
}

fn select_carrier_prompt(
    primary_prompt: Complex<f32>,
    pilot_prompt: Option<Complex<f32>>,
    aiding_mode: TrackingAidingMode,
    require_pilot: bool,
) -> (Complex<f32>, CarrierPromptSource) {
    if aiding_mode != TrackingAidingMode::PilotCarrier {
        return (primary_prompt, CarrierPromptSource::Primary);
    }
    let Some(pilot_prompt) = pilot_prompt else {
        return (primary_prompt, CarrierPromptSource::Primary);
    };
    let primary_norm = primary_prompt.norm();
    let pilot_norm = pilot_prompt.norm();
    if require_pilot && pilot_norm > f32::EPSILON {
        return (pilot_prompt, CarrierPromptSource::Pilot);
    }
    if primary_norm <= f32::EPSILON {
        return (pilot_prompt, CarrierPromptSource::Pilot);
    }
    if pilot_norm >= primary_norm * JOINT_COMPONENT_MIN_PROMPT_RATIO {
        (pilot_prompt, CarrierPromptSource::Pilot)
    } else {
        (primary_prompt, CarrierPromptSource::Primary)
    }
}

pub(crate) fn supports_tracking_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> bool {
    supports_tracking_signal_with_channel(sat, signal_band, signal_code, None)
}

pub(crate) fn supports_tracking_signal_with_channel(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
) -> bool {
    TrackingSignalModel::supports_tracking(sat, signal_band, signal_code, glonass_frequency_channel)
}

#[derive(Debug, Clone)]
struct IncrementalTrackingChannel {
    sat: SatId,
    channel_id: u8,
    start_source_time: ReceiverSampleTrace,
    signal_band: SignalBand,
    signal_model: TrackingSignalModel,
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

type CodeLoopUpdate = bijux_gnss_signal::api::CodeLoopUpdate;
type CarrierLoopUpdate = bijux_gnss_signal::api::CarrierTrackingLoopUpdate;
type CodeLoopInput = bijux_gnss_signal::api::CodeLoopInput;
type CarrierLoopInput = bijux_gnss_signal::api::CarrierTrackingLoopInput;

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

    fn tracking_loop_profile(tracking_params: TrackingParams) -> SignalTrackingLoopProfile {
        SignalTrackingLoopProfile {
            dll_bw_hz: tracking_params.dll_bw_hz,
            pll_bw_hz: tracking_params.pll_bw_hz,
            fll_bw_hz: tracking_params.fll_bw_hz,
            integration_ms: tracking_params.integration_ms.max(1),
        }
    }

    fn tracking_params_for_state(
        &self,
        base_tracking_params: TrackingParams,
        state: &LoopState,
    ) -> TrackingParams {
        if !self.config.adaptive_tracking_enabled {
            return base_tracking_params;
        }
        TrackingParams {
            early_late_spacing_chips: base_tracking_params.early_late_spacing_chips,
            dll_bw_hz: state.tracking_loop_profile.dll_bw_hz,
            pll_bw_hz: state.tracking_loop_profile.pll_bw_hz,
            fll_bw_hz: state.tracking_loop_profile.fll_bw_hz,
            integration_ms: state.tracking_loop_profile.integration_ms.max(1),
        }
    }

    fn initial_loop_state(
        &self,
        signal_model: &TrackingSignalModel,
        carrier_hz: f64,
        code_phase_samples: f64,
        acquisition_cn0_proxy_dbhz: f64,
        signal_delay_alignment: Option<SignalDelayAlignment>,
        subcarrier_code_phase_refined: bool,
        tracking_params: TrackingParams,
        reacquisition_pending: bool,
    ) -> LoopState {
        let code_rate_reference_hz =
            carrier_aided_code_rate_hz(&self.config, signal_model, carrier_hz);
        LoopState {
            carrier_hz,
            carrier_phase_cycles: pilot_carrier_phase_offset_cycles(signal_model),
            carrier_rate_hz_per_s: 0.0,
            code_rate_hz: code_rate_reference_hz,
            code_rate_reference_hz,
            code_phase_samples,
            tracking_adaptation_state: SignalTrackingAdaptationState::default(),
            tracking_loop_profile: Self::tracking_loop_profile(tracking_params),
            signal_delay_alignment,
            subcarrier_code_phase_refined,
            acquisition_cn0_proxy_dbhz,
            lock_reference_cn0_dbhz: acquisition_cn0_proxy_dbhz,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            secondary_code_prompt_history: VecDeque::new(),
            secondary_code_sync: None,
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
            reacquisition_pending,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
        }
    }

    fn tracking_start_context(
        &self,
        acquisition: &bijux_gnss_core::api::AcqResult,
    ) -> Option<TrackingStartContext> {
        if !matches!(acquisition.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous) {
            return None;
        }
        let signal_model = TrackingSignalModel::for_sat_signal_band(
            &self.config,
            acquisition.sat,
            acquisition.signal_band,
            acquisition.signal_code,
            acquisition.glonass_frequency_channel,
        );
        let acquisition_carrier_hz =
            normalize_acquisition_carrier_hz(&self.config, &signal_model, acquisition);
        if carrier_aiding_validation_required(&signal_model) {
            if let Err(reason) =
                carrier_aiding_reference(&self.config, &signal_model, acquisition_carrier_hz)
            {
                self.runtime.trace.record(TraceRecord {
                    name: "tracking_acquisition_refused",
                    fields: vec![
                        ("constellation", format!("{:?}", acquisition.sat.constellation)),
                        ("prn", acquisition.sat.prn.to_string()),
                        ("signal_band", format!("{:?}", acquisition.signal_band)),
                        ("signal_code", format!("{:?}", acquisition.signal_code)),
                        ("reason", reason.to_string()),
                        ("acquisition_carrier_hz", format!("{acquisition_carrier_hz:.3}")),
                        ("acquisition_doppler_hz", format!("{:.3}", acquisition.doppler_hz.0)),
                    ],
                });
                return None;
            }
        }
        Some(TrackingStartContext {
            seed: acquisition.tracking_seed(),
            acquisition_hypothesis: acquisition.hypothesis.to_string(),
            acquisition_hypothesis_rank: acquisition_hypothesis_rank(acquisition.hypothesis),
            acquisition_score: acquisition.score,
            acquisition_code_phase_samples: acquisition.code_phase_samples,
            acquisition_carrier_hz,
            acquisition_cn0_proxy_dbhz: acquisition.cn0_proxy as f64,
            subcarrier_code_phase_refined: acquisition.code_phase_refinement.is_some(),
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
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        self.correlate_epoch_range_with_signal_model(
            frame,
            0,
            frame.len(),
            frame.t0.sample_index,
            &signal_model,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        )
    }

    fn correlate_epoch_range_with_signal_model(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        sample_index: u64,
        signal_model: &TrackingSignalModel,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> CorrelatorOutput {
        self.tracking_epoch_correlation(
            frame,
            start,
            end,
            sample_index,
            signal_model,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        )
        .primary
    }

    fn tracking_epoch_correlation(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        sample_index: u64,
        signal_model: &TrackingSignalModel,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> TrackingEpochCorrelation {
        let sample_rate_hz = self.config.sampling_freq_hz;
        let samples = &frame.iq[start..end];
        let primary_code_period_samples = signal_model.samples_per_code(sample_rate_hz);
        let nominal_chips_per_sample = signal_model.nominal_chips_per_sample(sample_rate_hz);
        let tracked_chips_per_sample = code_rate_hz / sample_rate_hz;
        let epoch_primary_code_period_index =
            sample_index as usize / primary_code_period_samples.max(1);
        let epoch_start_code_phase_samples = epoch_start_code_phase_samples_from_receiver_phase(
            code_phase_samples,
            primary_code_period_samples,
        );
        let base_chip_phase = epoch_start_code_phase_samples * nominal_chips_per_sample;
        let primary = correlate_early_prompt_late(
            samples,
            sample_rate_hz,
            carrier_freq_hz,
            carrier_phase_offset_radians(carrier_phase_cycles),
            base_chip_phase,
            tracked_chips_per_sample,
            early_late_spacing_chips,
            |chip_phase| signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index),
        );
        let double_delta_outer = (code_discriminator_mode(signal_model)
            == CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate)
            .then(|| {
                correlate_early_prompt_late(
                    samples,
                    sample_rate_hz,
                    carrier_freq_hz,
                    carrier_phase_offset_radians(carrier_phase_cycles),
                    base_chip_phase,
                    tracked_chips_per_sample,
                    early_late_spacing_chips * 2.0,
                    |chip_phase| {
                        signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index)
                    },
                )
            });
        let subcarrier_ambiguity_guard = subcarrier_ambiguity_guard(
            signal_model,
            samples,
            sample_rate_hz,
            carrier_freq_hz,
            carrier_phase_cycles,
            base_chip_phase,
            tracked_chips_per_sample,
            epoch_primary_code_period_index,
            primary.prompt,
        );
        let pilot_prompt = signal_model.pilot_component.as_ref().map(|component| {
            correlate_early_prompt_late(
                samples,
                sample_rate_hz,
                carrier_freq_hz,
                carrier_phase_offset_radians(carrier_phase_cycles),
                base_chip_phase,
                tracked_chips_per_sample,
                0.0,
                |chip_phase| {
                    component.sample_value_from_primary_phase(
                        chip_phase,
                        epoch_primary_code_period_index,
                        signal_model.code_length,
                    )
                },
            )
            .prompt
        });
        let data_prompt = signal_model.data_symbol_component().map(|component| {
            if component.role == signal_model.component_role {
                primary.prompt
            } else {
                correlate_early_prompt_late(
                    samples,
                    sample_rate_hz,
                    carrier_freq_hz,
                    carrier_phase_offset_radians(carrier_phase_cycles),
                    base_chip_phase,
                    tracked_chips_per_sample,
                    0.0,
                    |chip_phase| {
                        component.sample_value_from_primary_phase(
                            chip_phase,
                            epoch_primary_code_period_index,
                            signal_model.code_length,
                        )
                    },
                )
                .prompt
            }
        });
        let (carrier_prompt, carrier_prompt_source) = select_carrier_prompt(
            primary.prompt,
            pilot_prompt,
            signal_model.aiding_mode,
            requires_dedicated_pilot_carrier(signal_model),
        );
        let secondary_code_prompt_period_index = prompt_center_primary_code_period_index(
            epoch_primary_code_period_index,
            base_chip_phase,
            tracked_chips_per_sample,
            samples.len(),
            signal_model.code_length,
        );
        TrackingEpochCorrelation {
            primary,
            double_delta_outer,
            carrier_prompt,
            carrier_prompt_source,
            data_prompt,
            secondary_code_prompt_period_index,
            subcarrier_ambiguity_guard,
        }
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
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        let (epoch, correlation) = self.track_epoch_range_with_signal_model(
            frame,
            0,
            frame.len(),
            channel_id,
            sat,
            &signal_model,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        );
        (epoch, correlation.primary)
    }

    fn track_epoch_range_with_signal_model(
        &self,
        frame: &SamplesFrame,
        start: usize,
        end: usize,
        channel_id: u8,
        sat: SatId,
        signal_model: &TrackingSignalModel,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (TrackEpoch, TrackingEpochCorrelation) {
        let sample_index = frame.t0.sample_index + start as u64;
        let source_time = SampleTime { sample_index, sample_rate_hz: frame.t0.sample_rate_hz };
        let clock = SampleClock::new(self.config.sampling_freq_hz);
        let epoch = clock.epoch_from_samples(sample_index);
        let correlation = self.tracking_epoch_correlation(
            frame,
            start,
            end,
            sample_index,
            signal_model,
            carrier_freq_hz,
            carrier_phase_cycles,
            code_rate_hz,
            code_phase_samples,
            early_late_spacing_chips,
        );
        let coherent_samples = end.saturating_sub(start);
        let cn0_dbhz = estimate_cn0_dbhz(
            correlation.primary.prompt,
            correlation.primary.early - correlation.primary.late,
            self.config.sampling_freq_hz,
            coherent_samples as f64,
            correlation.primary.early_late_noise_weight_energy,
        );
        if !correlation.primary.prompt.re.is_finite() || !correlation.primary.prompt.im.is_finite()
        {
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
            signal_band: signal_model.signal_band,
            signal_code: signal_model.signal_code,
            glonass_frequency_channel: signal_model.glonass_frequency_channel,
            prompt_i: correlation.primary.prompt.re,
            prompt_q: correlation.primary.prompt.im,
            early_i: correlation.primary.early.re,
            early_q: correlation.primary.early.im,
            late_i: correlation.primary.late.re,
            late_q: correlation.primary.late.im,
            carrier_hz: Hertz(carrier_freq_hz),
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            code_rate_hz: Hertz(code_rate_hz),
            code_phase_samples: Chips(code_phase_samples),
            lock: correlation.primary.prompt.norm() > 0.0,
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
            tracking_assumptions: Some(default_tracking_assumptions(&self.config, signal_model)),
            tracking_uncertainty: None,
            signal_delay_alignment: None,
            processing_ms: None,
        };
        (track_epoch, correlation)
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
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        let tracking_params = resolve_signal_tracking_params(&self.config, &signal_model);
        let epochs = self.track_epochs(
            &frame,
            0,
            sat,
            &signal_model,
            0.0,
            0.0,
            f64::INFINITY,
            tracking_params,
            5,
        );
        vec![TrackingResult {
            sat,
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
                    "carrier_pull_in" | "carrier_converged" | "fade_recovered" | "signal_fade"
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
        signal_model: &TrackingSignalModel,
        carrier_hz: f64,
        code_phase_samples: f64,
        acquisition_cn0_proxy_dbhz: f64,
        tracking_params: TrackingParams,
        epochs: usize,
    ) -> (Vec<TrackEpoch>, Vec<TrackTransition>) {
        let mut state = self.initial_loop_state(
            signal_model,
            carrier_hz,
            code_phase_samples,
            acquisition_cn0_proxy_dbhz,
            None,
            false,
            tracking_params,
            false,
        );

        let mut out = Vec::new();
        let mut transitions = Vec::new();
        self.append_tracked_epochs(
            frame,
            channel_id,
            sat,
            signal_model,
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
                let signal_model = TrackingSignalModel::for_sat_signal_band(
                    &self.config,
                    context.seed.sat,
                    context.seed.signal_band,
                    context.seed.signal_code,
                    context.seed.glonass_frequency_channel,
                );
                let tracking_params = resolve_signal_tracking_params(&self.config, &signal_model);
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
                    state: self.initial_loop_state(
                        &signal_model,
                        context.acquisition_carrier_hz,
                        context.seed.code_phase_samples.0,
                        context.acquisition_cn0_proxy_dbhz,
                        context.seed.signal_delay_alignment.clone(),
                        context.subcarrier_code_phase_refined,
                        tracking_params,
                        false,
                    ),
                    sat: context.seed.sat,
                    channel_id,
                    start_source_time: context.seed.source_time,
                    signal_band: context.seed.signal_band,
                    signal_model: signal_model.clone(),
                    acquisition_uncertainty: context.seed.uncertainty.clone(),
                    acquisition_hypothesis: context.acquisition_hypothesis,
                    acquisition_score: context.acquisition_score,
                    acquisition_code_phase_samples: context.acquisition_code_phase_samples,
                    acquisition_doppler_hz: context.seed.doppler_hz.0,
                    acquisition_resolved_code_phase_samples: context.seed.code_phase_samples.0,
                    acquisition_carrier_hz: context.acquisition_carrier_hz,
                    acq_to_track_state: context.acq_to_track_state,
                    tracking_params,
                    epochs: Vec::new(),
                    transitions: Vec::new(),
                }
            })
            .collect();
        IncrementalTrackingState { channels, vector_state: VectorTrackingState::default() }
    }

    pub(crate) fn track_incremental_frame(
        &self,
        tracking: &mut IncrementalTrackingState,
        frame: &SamplesFrame,
    ) {
        let vector_state = &mut tracking.vector_state;
        for channel in &mut tracking.channels {
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
            let mut epoch_start = channel_frame_start;
            while epoch_start < frame.len() {
                let tracking_params =
                    self.tracking_params_for_state(channel.tracking_params, &channel.state);
                let samples_per_epoch = channel
                    .signal_model
                    .tracking_epoch_samples(self.config.sampling_freq_hz, tracking_params);
                let epoch_end = (epoch_start + samples_per_epoch).min(frame.len());
                let reacquisition_outcome = if channel.state.state == ChannelState::Lost {
                    let epoch_frame = frame_slice(frame, epoch_start, epoch_end);
                    self.try_reacquire_channel(channel, &epoch_frame)
                } else {
                    ReacquisitionOutcome::NotNeeded
                };
                let epoch_count_before = channel.epochs.len();
                let transition_count_before = channel.transitions.len();
                let vector_prediction = if self.config.vector_tracking_enabled {
                    vector_state.prediction_for(
                        frame.t0.sample_index + epoch_start as u64,
                        self.config.sampling_freq_hz,
                    )
                } else {
                    None
                };
                let measurement = self.append_tracked_epoch_range(
                    frame,
                    epoch_start,
                    epoch_end,
                    channel.channel_id,
                    channel.sat,
                    &channel.signal_model,
                    channel.tracking_params,
                    tracking_params,
                    vector_prediction,
                    &mut channel.state,
                    &mut channel.epochs,
                    &mut channel.transitions,
                );
                if self.config.vector_tracking_enabled {
                    if let Some(measurement) = measurement {
                        vector_state.record(measurement, self.config.sampling_freq_hz);
                    }
                }
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
                    let runtime_tracking_provenance = epoch.tracking_provenance.clone();
                    epoch.tracking_provenance = format!(
                        "acq_hypothesis={} acq_score={:.6} acq_signal_band={:?} acq_doppler_hz={:.3} acq_carrier_hz={:.3} acq_code_phase_samples={} acq_resolved_code_phase_samples={:.6} acq_start_sample_index={} track_component_role={:?} nominal_carrier_hz={:.3} secondary_code={} discriminator_family={} code_discriminator={} phase_transition_source={} aiding_mode={} pilot_component={} data_symbol_component={}",
                        channel.acquisition_hypothesis,
                        channel.acquisition_score,
                        channel.signal_band,
                        channel.acquisition_doppler_hz,
                        channel.acquisition_carrier_hz,
                        channel.acquisition_code_phase_samples,
                        channel.acquisition_resolved_code_phase_samples,
                        channel.start_source_time.sample_index,
                        channel.signal_model.component_role,
                        channel.signal_model.nominal_carrier_hz(),
                        channel.signal_model.secondary_code.is_some(),
                        channel.signal_model.discriminator_family.label(),
                        code_discriminator_mode(&channel.signal_model).label(),
                        channel.signal_model.phase_transition_source.label(),
                        channel.signal_model.aiding_mode.label(),
                        channel.signal_model.pilot_component.is_some(),
                        channel.signal_model.data_symbol_component.is_some(),
                    );
                    epoch.tracking_provenance.push_str(&format!(
                        " code_rate_reference={} carrier_aiding_doppler_window_hz={:.3}",
                        code_rate_reference_label(&self.config, &channel.signal_model, epoch.carrier_hz.0),
                        carrier_aiding_doppler_window_hz(&self.config),
                    ));
                    if let Some(secondary_code_sync_provenance) = secondary_code_sync_provenance(
                        &channel.signal_model,
                        channel.state.secondary_code_sync,
                    ) {
                        epoch.tracking_provenance.push_str(&secondary_code_sync_provenance);
                    }
                    if runtime_tracking_provenance.starts_with("vector_tracking=applied") {
                        epoch.tracking_provenance.push(' ');
                        epoch.tracking_provenance.push_str(&runtime_tracking_provenance);
                    } else if let Some(subcarrier_ambiguity_provenance) =
                        runtime_tracking_provenance.find("subcarrier_ambiguity_guard=").map(
                            |start| runtime_tracking_provenance[start..].to_string(),
                        )
                    {
                        epoch.tracking_provenance.push(' ');
                        epoch.tracking_provenance.push_str(&subcarrier_ambiguity_provenance);
                    }
                }
                self.apply_sample_rate_mismatch_diagnostic(
                    channel.sat,
                    channel.acquisition_uncertainty.as_ref(),
                    &mut channel.epochs,
                );
                annotate_navigation_bit_signs(&channel.signal_model, &mut channel.epochs);
                stabilize_joint_navigation_bit_signs(&channel.signal_model, &mut channel.epochs);
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
        signal_model: &TrackingSignalModel,
        base_tracking_params: TrackingParams,
        epochs: usize,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) {
        let mut start = 0usize;
        for _ in 0..epochs {
            let tracking_params = self.tracking_params_for_state(base_tracking_params, state);
            let samples_per_epoch =
                signal_model.tracking_epoch_samples(self.config.sampling_freq_hz, tracking_params);
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
                signal_model,
                base_tracking_params,
                tracking_params,
                None,
                state,
                out,
                transitions,
            );
            start = end;
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
        signal_model: &TrackingSignalModel,
        base_tracking_params: TrackingParams,
        tracking_params: TrackingParams,
        vector_prediction: Option<VectorTrackingPrediction>,
        state: &mut LoopState,
        out: &mut Vec<TrackEpoch>,
        transitions: &mut Vec<TrackTransition>,
    ) -> Option<VectorTrackingMeasurement> {
        let samples_per_code = signal_model.samples_per_code(self.config.sampling_freq_hz);
        let samples_per_chip = samples_per_code as f64 / signal_model.code_length as f64;
        let vector_application =
            vector_prediction.and_then(|prediction| vector_tracking_application(prediction, state));
        let aided_carrier_hz = state.carrier_hz
            + vector_application
                .map(|application| application.carrier_frequency_correction_hz)
                .unwrap_or_default();
        let aided_code_rate_hz = state.code_rate_hz
            + vector_application
                .map(|application| application.code_rate_correction_hz)
                .unwrap_or_default();
        let aided_code_phase_samples = state.code_phase_samples
            + vector_application
                .map(|application| application.code_phase_correction_samples)
                .unwrap_or_default();
        let aided_carrier_rate_hz_per_s = state.carrier_rate_hz_per_s
            + vector_application
                .map(|application| application.carrier_rate_correction_hz_per_s)
                .unwrap_or_default();
        let alloc_before = crate::engine::alloc::allocation_count();
        let (mut track_epoch, correlation) = self.track_epoch_range_with_signal_model(
            frame,
            start,
            end,
            channel_id,
            sat,
            signal_model,
            aided_carrier_hz,
            state.carrier_phase_cycles,
            aided_code_rate_hz,
            aided_code_phase_samples,
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

        let primary_correlator = correlation.primary;
        let carrier_prompt = correlation.carrier_prompt;
        update_secondary_code_synchronization(
            signal_model,
            state,
            correlation.secondary_code_prompt_period_index,
            carrier_prompt,
        );
        let dll_err = tracking_dll_discriminator(&correlation);
        let dll_err =
            normalize_dll_discriminator(dll_err, tracking_params.early_late_spacing_chips);
        let (_raw_pll_err, raw_fll_err, lock) =
            carrier_prompt_discriminators(carrier_prompt, state.prev_prompt);
        state.prev_prompt = Some(carrier_prompt);
        let phase_cycles = carrier_prompt.arg() as f64 / (2.0 * std::f64::consts::PI);
        let phase_transition_source =
            carrier_phase_transition_source_for_prompt(signal_model, state.secondary_code_sync);
        let phase_decision = classify_prompt_phase(
            phase_cycles,
            state.prev_prompt_phase_cycles,
            state.nav_bit_phase_offset_cycles,
            phase_transition_source,
        );
        state.prev_prompt_phase_cycles = Some(phase_decision.aligned_phase_cycles);
        state.nav_bit_phase_offset_cycles = phase_decision.nav_bit_phase_offset_cycles;
        let pll_err = (phase_decision.aligned_phase_cycles * std::f64::consts::TAU) as f32;
        if phase_decision.nav_bit_transition {
            state.nav_bit_transition_count = state.nav_bit_transition_count.saturating_add(1);
        }

        let cycle_slip = phase_decision.cycle_slip;
        let cycle_slip_reason = cycle_slip.then(|| LossOfLockCause::PhaseJump.reason().to_string());
        let prompt_power = carrier_prompt.norm();

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
        let dll_bw = tracking_params.dll_bw_hz;
        let pll_bw = tracking_params.pll_bw_hz;
        let fll_bw = tracking_params.fll_bw_hz;
        let coherent_integration_s =
            coherent_integration_seconds(epoch_len_samples, self.config.sampling_freq_hz);
        let raw_fll_err_hz =
            carrier_frequency_error_hz_from_phase_delta(raw_fll_err as f64, coherent_integration_s);
        let nav_bit_aware_fll_err_hz = carrier_frequency_error_hz_from_phase_delta(
            phase_decision.aligned_phase_delta_cycles * std::f64::consts::TAU,
            coherent_integration_s,
        );
        let use_nav_bit_aware_fll = phase_transition_source.allows_half_cycle_transition()
            || phase_decision.nav_bit_transition
            || state.nav_bit_phase_offset_cycles.abs() > f64::EPSILON
            || phase_decision.nav_bit_phase_offset_cycles.abs() > f64::EPSILON;
        let fll_err_hz =
            if use_nav_bit_aware_fll { nav_bit_aware_fll_err_hz } else { raw_fll_err_hz } as f32;
        let from_state = state.state;
        let lock_detector_thresholds = tracking_lock_detector_thresholds(
            cn0_dbhz,
            coherent_integration_s,
            samples_per_chip,
            tracking_params,
            state.carrier_rate_hz_per_s,
        );
        let raw_dll_lock = dll_err.abs() < lock_detector_thresholds.dll_lock;
        let subcarrier_ambiguity =
            subcarrier_ambiguity_detected(correlation.subcarrier_ambiguity_guard);
        let subcarrier_code_phase_unambiguous =
            !signal_model.discriminator_family.requires_unambiguous_code_lock()
                || state.subcarrier_code_phase_refined;
        let fll_enabled = fll_bw > f64::EPSILON;
        let raw_pll_lock = pll_err.abs() < lock_detector_thresholds.pll_lock_rad;
        let raw_fll_lock =
            !fll_enabled || (fll_err_hz as f64).abs() <= lock_detector_thresholds.fll_lock_hz;
        let sustained_dll_lock = raw_dll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && dll_err.abs() < lock_detector_thresholds.dll_hold);
        let sustained_pll_lock = raw_pll_lock
            || (matches!(
                from_state,
                ChannelState::PullIn | ChannelState::Tracking | ChannelState::Degraded
            ) && pll_err.abs() < lock_detector_thresholds.pll_hold_rad);
        let anti_false_lock = anti_false_lock_detected(
            primary_correlator.early,
            primary_correlator.prompt,
            primary_correlator.late,
        ) && !sustained_pll_lock;
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
            ) && prompt_power_ratio
                .is_some_and(|ratio| ratio >= DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO)
                && !cycle_slip
                && !anti_false_lock);
        let sustained_code_lock = sustained_dll_lock
            || low_resolution_code_lock(
                samples_per_chip,
                tracking_params.early_late_spacing_chips,
                sustained_prompt_lock,
                sustained_pll_lock,
                raw_fll_lock,
                cycle_slip,
                anti_false_lock,
            );
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
            sustained_code_lock,
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
                && sustained_code_lock
                && raw_pll_lock
                && raw_fll_lock
                && !cycle_slip
                && !anti_false_lock
        } else {
            cn0_supports_lock
                && sustained_prompt_lock
                && sustained_code_lock
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
                dll_locked: state.state != ChannelState::Lost
                    && sustained_code_lock
                    && subcarrier_code_phase_unambiguous
                    && !subcarrier_ambiguity,
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
        let dll_lock = tracking_state_locked
            && sustained_code_lock
            && subcarrier_code_phase_unambiguous
            && !subcarrier_ambiguity;
        let pll_lock = tracking_state_locked
            && cn0_supports_lock
            && sustained_prompt_lock
            && sustained_pll_lock
            && !cycle_slip;
        let fll_lock = state.state != ChannelState::Lost
            && (raw_fll_lock || (tracking_state_locked && pll_lock));
        let navigation_bit_sign = recover_epoch_navigation_bit_sign(
            signal_model,
            correlation.data_prompt,
            carrier_prompt,
            correlation.carrier_prompt_source,
            phase_decision.nav_bit_phase_offset_cycles,
            tracking_state_locked && dll_lock && (pll_lock || fll_lock),
        );
        let nav_bit_lock = navigation_bit_sign.is_some()
            || (signal_model.phase_transition_source.reports_navigation_bit_lock()
                && state.nav_bit_transition_count > 0);
        if track_epoch.early_i.is_infinite() || track_epoch.late_i.is_infinite() {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "TRACK_NUMERIC_INVALID",
                "infinite tracking sample in early/late output",
            ));
        }

        let apply_fll = fll_bw > 0.0 && should_apply_fll(state.state, raw_fll_lock);
        let tracked_center_hz =
            tracked_signal_center_hz(self.config.intermediate_freq_hz, signal_model.signal_spec);
        let current_carrier_doppler_hz = tracked_signal_doppler_hz(
            self.config.intermediate_freq_hz,
            aided_carrier_hz,
            signal_model.signal_spec,
        );
        let carrier_loop = apply_carrier_loop(CarrierLoopInput {
            current_carrier_hz: current_carrier_doppler_hz,
            current_carrier_phase_cycles: state.carrier_phase_cycles,
            current_carrier_rate_hz_per_s: aided_carrier_rate_hz_per_s,
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
        let tracked_carrier_hz = tracked_center_hz + carrier_loop.carrier_hz;
        let code_rate_reference_hz = next_code_rate_reference_hz(
            &self.config,
            signal_model,
            tracked_carrier_hz,
            state.code_rate_reference_hz,
            raw_fll_lock || sustained_pll_lock,
        );
        let code_loop = apply_dll_code_loop(CodeLoopInput {
            current_code_rate_hz: aided_code_rate_hz,
            previous_reference_code_rate_hz: state.code_rate_reference_hz,
            reference_code_rate_hz: code_rate_reference_hz,
            current_code_phase_samples: aided_code_phase_samples,
            epoch_len_samples,
            coherent_integration_s,
            nominal_code_rate_hz: signal_model.code_rate_hz,
            dll_bw_hz: dll_bw,
            dll_err,
            samples_per_chip,
            samples_per_code,
        });
        state.carrier_hz = tracked_carrier_hz;
        state.carrier_phase_cycles = carrier_loop.carrier_phase_cycles;
        state.carrier_rate_hz_per_s = carrier_loop.carrier_rate_hz_per_s;
        state.code_rate_hz = code_loop.code_rate_hz;
        state.code_rate_reference_hz = code_rate_reference_hz;
        state.code_phase_samples = code_loop.code_phase_samples;
        if self.config.adaptive_tracking_enabled {
            let adaptation = advance_tracking_adaptation(
                Self::tracking_loop_profile(base_tracking_params),
                state.tracking_adaptation_state,
                SignalTrackingAdaptationInput {
                    cn0_dbhz,
                    fll_error_hz: fll_err_hz as f64,
                    carrier_rate_hz_per_s: carrier_loop.carrier_rate_hz_per_s,
                    carrier_lock_ready: raw_fll_lock || sustained_pll_lock,
                    steady_state_lock: matches!(
                        state.state,
                        ChannelState::Tracking | ChannelState::Degraded
                    ) && sustained_prompt_lock
                        && sustained_code_lock,
                    discriminator_stable: state.unstable_discriminator_epochs == 0
                        && !cycle_slip
                        && !anti_false_lock,
                },
            );
            state.tracking_adaptation_state = adaptation.state;
            state.tracking_loop_profile = adaptation.profile;
        }

        if state.state != from_state {
            transitions.push(TrackTransition {
                sat,
                channel_id,
                epoch_idx: track_epoch.epoch.index,
                sample_index: track_epoch.sample_index,
                from_state: from_state.to_string(),
                to_state: state.state.to_string(),
                reason: lock_state_reason.clone().unwrap_or_else(|| "state_transition".to_string()),
                lock_quality: epoch_lock_quality(
                    sustained_prompt_lock,
                    pll_lock,
                    dll_lock,
                    fll_lock,
                    cn0_dbhz,
                ),
            });
        }

        let measurement = VectorTrackingMeasurement {
            sat,
            channel_id,
            epoch_idx: track_epoch.epoch.index,
            sample_index: track_epoch.sample_index,
            cn0_dbhz,
            dll_error_samples: (dll_err as f64) * samples_per_chip,
            pll_error_rad: pll_err as f64,
            fll_error_hz: fll_err_hz as f64,
            code_rate_error_hz: state.code_rate_hz - signal_model.code_rate_hz,
            carrier_rate_hz_per_s: state.carrier_rate_hz_per_s,
            prompt_locked: channel_locked,
            dll_locked: dll_lock,
            pll_locked: pll_lock,
            fll_locked: fll_lock,
            channel_state: state.state,
        };
        let mut epoch_assumptions = tracking_assumptions(signal_model, tracking_params);
        if vector_application.is_some() {
            epoch_assumptions.aiding_mode = vector_tracking_aiding_mode_label(signal_model);
        }
        let vector_provenance = vector_application.map(vector_tracking_provenance);
        let mut tracking_provenance =
            vector_provenance.unwrap_or_else(|| track_epoch.tracking_provenance.clone());
        if let Some(subcarrier_ambiguity_provenance) =
            subcarrier_ambiguity_provenance(correlation.subcarrier_ambiguity_guard)
        {
            tracking_provenance.push_str(&subcarrier_ambiguity_provenance);
        }
        if signal_model.discriminator_family.requires_unambiguous_code_lock() {
            tracking_provenance.push_str(if state.subcarrier_code_phase_refined {
                " subcarrier_code_phase_handoff=refined"
            } else {
                " subcarrier_code_phase_handoff=coarse"
            });
        }
        tracking_provenance.push_str(&lock_detector_provenance(lock_detector_thresholds));

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
            nav_bit_lock,
            navigation_bit_sign,
            dll_err,
            pll_err,
            fll_err: fll_err_hz,
            anti_false_lock,
            cycle_slip_reason,
            lock_state,
            lock_state_reason,
            tracking_provenance,
            tracking_assumptions: Some(epoch_assumptions),
            signal_delay_alignment: state.signal_delay_alignment.clone(),
            tracking_uncertainty,
            ..track_epoch
        });
        Some(measurement)
    }
}

#[cfg(feature = "nav")]
const NAV_SYMBOL_SYNC_MIN_CONFIDENCE: f64 = 0.05;
#[cfg(feature = "nav")]
const NAV_SYMBOL_SYNC_MIN_COMPLETE_WINDOWS: usize = 2;
#[cfg(feature = "nav")]
const NAV_SYMBOL_BIT_MIN_CONFIDENCE: f64 = 0.75;

#[cfg(feature = "nav")]
fn annotate_navigation_bit_signs(signal_model: &TrackingSignalModel, epochs: &mut [TrackEpoch]) {
    if !signal_model.supports_navigation_bit_sign_recovery() || epochs.is_empty() {
        return;
    }

    let prompt_history = epochs.iter().map(|epoch| epoch.prompt_i).collect::<Vec<_>>();
    let demodulation = bijux_gnss_nav::api::demodulate_gps_l1ca_navigation_bits(&prompt_history);
    if !navigation_symbol_sync_is_confident(&demodulation) {
        return;
    }

    for bit in demodulation.bits {
        if bit.confidence < NAV_SYMBOL_BIT_MIN_CONFIDENCE {
            continue;
        }
        for epoch in epochs[bit.start_prompt_index..bit.end_prompt_index_exclusive].iter_mut() {
            epoch.navigation_bit_sign = Some(bit.sign);
            epoch.nav_bit_lock = true;
            annotate_navigation_symbol_sync_provenance(
                epoch,
                demodulation.bit_start_ms,
                demodulation.sync_confidence,
                bit.confidence,
            );
        }
    }
}

#[cfg(feature = "nav")]
fn annotate_navigation_symbol_sync_provenance(
    epoch: &mut TrackEpoch,
    bit_start_ms: usize,
    sync_confidence: f64,
    bit_confidence: f64,
) {
    epoch.tracking_provenance.push_str(&format!(
        " nav_symbol_sync=confident nav_symbol_start_ms={} nav_symbol_sync_confidence={:.6} nav_symbol_bit_confidence={:.6}",
        bit_start_ms, sync_confidence, bit_confidence
    ));
}

#[cfg(feature = "nav")]
fn navigation_symbol_sync_is_confident(
    demodulation: &bijux_gnss_nav::api::GpsL1CaNavigationBits,
) -> bool {
    demodulation.complete_window_count >= NAV_SYMBOL_SYNC_MIN_COMPLETE_WINDOWS
        && demodulation.sync_confidence >= NAV_SYMBOL_SYNC_MIN_CONFIDENCE
}

#[cfg(not(feature = "nav"))]
fn annotate_navigation_bit_signs(_signal_model: &TrackingSignalModel, _epochs: &mut [TrackEpoch]) {}

fn stabilize_joint_navigation_bit_signs(
    signal_model: &TrackingSignalModel,
    epochs: &mut [TrackEpoch],
) {
    if signal_model.aiding_mode != TrackingAidingMode::PilotCarrier
        || !signal_model.supports_epoch_data_symbol_sign_recovery()
        || epochs.len() < 3
    {
        return;
    }

    let stable_tracking_epoch = |epoch: &TrackEpoch| {
        epoch.lock_state == "tracking" && epoch.pll_lock && epoch.dll_lock && !epoch.cycle_slip
    };
    let mut smoothed_signs = Vec::new();
    for index in 1..epochs.len() - 1 {
        let previous = &epochs[index - 1];
        let current = &epochs[index];
        let next = &epochs[index + 1];
        if !stable_tracking_epoch(previous)
            || !stable_tracking_epoch(current)
            || !stable_tracking_epoch(next)
        {
            continue;
        }
        let Some(previous_sign) = previous.navigation_bit_sign else {
            continue;
        };
        let Some(next_sign) = next.navigation_bit_sign else {
            continue;
        };
        if previous_sign != next_sign {
            continue;
        }
        match current.navigation_bit_sign {
            None => smoothed_signs.push((index, previous_sign)),
            Some(current_sign) if current_sign != previous_sign => {
                smoothed_signs.push((index, previous_sign));
            }
            _ => {}
        }
    }

    for (index, sign) in smoothed_signs {
        epochs[index].navigation_bit_sign = Some(sign);
        epochs[index].nav_bit_lock = true;
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
        .then_with(|| right.acquisition_cn0_proxy_dbhz.total_cmp(&left.acquisition_cn0_proxy_dbhz))
        .then_with(|| left.seed.sat.constellation.cmp(&right.seed.sat.constellation))
        .then_with(|| left.seed.sat.prn.cmp(&right.seed.sat.prn))
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

fn tracking_quality_class(channel_state: ChannelState) -> TrackingQualityClass {
    match channel_state {
        ChannelState::Tracking => TrackingQualityClass::Tracking,
        ChannelState::Degraded => TrackingQualityClass::Degraded,
        ChannelState::PullIn | ChannelState::Acquired => TrackingQualityClass::PullIn,
        ChannelState::Lost | ChannelState::Idle => TrackingQualityClass::Lost,
    }
}

fn refresh_prompt_power_reference(
    current_reference: f32,
    prompt_power: f32,
    state: ChannelState,
    anti_false_lock: bool,
) -> f32 {
    signal_refresh_prompt_power_reference(
        current_reference,
        prompt_power,
        tracking_quality_class(state),
        anti_false_lock,
    )
}

fn refresh_lock_reference_cn0_dbhz(
    current_reference: f64,
    cn0_dbhz: f64,
    reliable_tracking_lock: bool,
) -> f64 {
    signal_refresh_lock_reference_cn0_dbhz(current_reference, cn0_dbhz, reliable_tracking_lock)
}

fn prompt_power_ratio(prompt_power: f32, prompt_power_reference: f32) -> Option<f32> {
    signal_prompt_power_ratio(prompt_power, prompt_power_reference)
}

fn anti_false_lock_detected(early: Complex<f32>, prompt: Complex<f32>, late: Complex<f32>) -> bool {
    signal_anti_false_lock_detected(early, prompt, late)
}

fn should_apply_fll(state: ChannelState, raw_fll_lock: bool) -> bool {
    matches!(state, ChannelState::PullIn | ChannelState::Degraded) || !raw_fll_lock
}

fn update_windowed_tracking_cn0_estimate(
    prompt_cn0_window: &mut VecDeque<f64>,
    epoch_cn0_dbhz: f64,
) -> Option<f64> {
    signal_update_windowed_tracking_cn0_estimate(
        prompt_cn0_window,
        epoch_cn0_dbhz,
        TRACKING_CN0_WINDOW_EPOCHS,
        TRACKING_CN0_MIN_WINDOW_EPOCHS,
    )
}

fn push_tracking_uncertainty_sample(window: &mut VecDeque<f64>, value: f64) {
    signal_push_tracking_uncertainty_sample(window, value, TRACKING_UNCERTAINTY_WINDOW_EPOCHS)
}

fn estimate_tracking_uncertainty(
    state: &LoopState,
    input: TrackingUncertaintyInputs,
) -> TrackingUncertainty {
    signal_estimate_tracking_uncertainty(
        &state.code_error_window_samples,
        &state.carrier_phase_error_window_cycles,
        &state.doppler_error_window_hz,
        &state.cn0_estimate_window_dbhz,
        SignalTrackingUncertaintyInputs {
            samples_per_chip: input.samples_per_chip,
            dll_err: input.dll_err,
            pll_err_rad: input.pll_err_rad,
            fll_err_hz: input.fll_err_hz,
            cn0_dbhz: input.cn0_dbhz,
            cn0_reference_dbhz: input.cn0_reference_dbhz,
            integration_ms: input.integration_ms,
            channel_locked: input.channel_locked,
            dll_locked: input.dll_locked,
            anti_false_lock: input.anti_false_lock,
            cycle_slip: input.cycle_slip,
            quality_class: tracking_quality_class(input.channel_state),
        },
    )
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

fn default_tracking_assumptions(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
) -> TrackingAssumptions {
    tracking_assumptions(signal_model, resolve_signal_tracking_params(config, signal_model))
}

fn tracking_assumptions(
    signal_model: &TrackingSignalModel,
    params: TrackingParams,
) -> TrackingAssumptions {
    TrackingAssumptions {
        integration_ms: params.integration_ms,
        early_late_spacing_chips: params.early_late_spacing_chips,
        dll_bw_hz: params.dll_bw_hz,
        pll_bw_hz: params.pll_bw_hz,
        fll_bw_hz: params.fll_bw_hz,
        discriminator_family: signal_model.discriminator_family.label().to_string(),
        aiding_mode: signal_model.aiding_mode.label().to_string(),
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

fn tracked_signal_center_hz(intermediate_freq_hz: f64, signal: SignalSpec) -> f64 {
    intermediate_freq_hz + (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())
}

fn tracked_signal_doppler_hz(
    intermediate_freq_hz: f64,
    tracked_carrier_hz: f64,
    signal: SignalSpec,
) -> f64 {
    tracked_carrier_hz - tracked_signal_center_hz(intermediate_freq_hz, signal)
}

fn normalize_acquisition_carrier_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    acquisition: &bijux_gnss_core::api::AcqResult,
) -> f64 {
    let acquisition_carrier_hz = acquisition.carrier_hz.0;
    let tracked_center_hz =
        tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec);
    let looks_like_doppler_only_seed = tracked_center_hz.abs() > 1.0
        && (acquisition_carrier_hz - acquisition.doppler_hz.0).abs() <= 1.0e-9;
    if looks_like_doppler_only_seed {
        tracked_center_hz + acquisition.doppler_hz.0
    } else {
        acquisition_carrier_hz
    }
}

fn carrier_aided_code_rate_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> f64 {
    carrier_aiding_reference(config, signal_model, tracked_carrier_hz)
        .map(|reference| reference.code_rate_hz)
        .unwrap_or(signal_model.code_rate_hz)
}

fn next_code_rate_reference_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
    previous_reference_hz: f64,
    carrier_lock_ready: bool,
) -> f64 {
    if carrier_lock_ready {
        carrier_aided_code_rate_hz(config, signal_model, tracked_carrier_hz)
    } else {
        previous_reference_hz
    }
}

fn carrier_aiding_reference(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> Result<CarrierAidingReference, &'static str> {
    let tracked_doppler_hz = tracked_signal_doppler_hz(
        config.intermediate_freq_hz,
        tracked_carrier_hz,
        signal_model.signal_spec,
    );
    if !tracked_carrier_hz.is_finite() || !tracked_doppler_hz.is_finite() {
        return Err("carrier_aiding_non_finite_seed");
    }
    if tracked_doppler_hz.abs() > carrier_aiding_doppler_window_hz(config) {
        return Err("carrier_aiding_incompatible_signal_center");
    }
    let Some(code_rate_hz) = shared_path_code_rate_hz(
        tracked_doppler_hz,
        signal_model.signal_spec,
        signal_model.signal_spec,
    ) else {
        return Err("carrier_aiding_invalid_signal_metadata");
    };
    if !code_rate_hz.is_finite() || code_rate_hz <= 0.0 {
        return Err("carrier_aiding_invalid_code_rate");
    }
    Ok(CarrierAidingReference { tracked_carrier_hz, tracked_doppler_hz, code_rate_hz })
}

fn carrier_aiding_validation_required(signal_model: &TrackingSignalModel) -> bool {
    !matches!(signal_model.signal_spec.constellation, Constellation::Glonass)
}

fn code_rate_reference_label(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> &'static str {
    if carrier_aiding_reference(config, signal_model, tracked_carrier_hz).is_ok() {
        "carrier_doppler"
    } else {
        "nominal"
    }
}

fn carrier_aiding_doppler_window_hz(config: &ReceiverPipelineConfig) -> f64 {
    (config.acquisition_doppler_search_hz.unsigned_abs() as f64
        + CARRIER_AID_DOPPLER_WINDOW_MARGIN_HZ)
        .max(CARRIER_AID_MIN_DOPPLER_WINDOW_HZ)
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
    phase_transition_source: TrackingPhaseTransitionSource,
) -> PromptPhaseDecision {
    let same_offset_phase =
        wrap_phase_cycles_signed(raw_phase_cycles - nav_bit_phase_offset_cycles);
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
    let flipped_offset =
        wrap_phase_cycles_signed(nav_bit_phase_offset_cycles + NAV_BIT_PHASE_STEP_CYCLES);
    let flipped_phase = wrap_phase_cycles_signed(raw_phase_cycles - flipped_offset);
    let flipped_delta = wrapped_phase_delta_cycles(flipped_phase, previous_aligned_phase_cycles);

    let nav_bit_transition = phase_transition_source.allows_half_cycle_transition()
        && same_delta.abs() > CYCLE_SLIP_PHASE_DELTA_CYCLES
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

fn carrier_prompt_discriminators(
    prompt: Complex<f32>,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, bool) {
    let pll = prompt.im.atan2(prompt.re);
    let fll = if let Some(prev) = prev_prompt {
        let dot = prompt.re * prev.re + prompt.im * prev.im;
        let det = prompt.im * prev.re - prompt.re * prev.im;
        det.atan2(dot)
    } else {
        0.0
    };
    let lock = prompt.norm() > 0.1;
    (pll, fll, lock)
}

fn recover_epoch_navigation_bit_sign(
    signal_model: &TrackingSignalModel,
    data_prompt: Option<Complex<f32>>,
    carrier_prompt: Complex<f32>,
    carrier_prompt_source: CarrierPromptSource,
    carrier_phase_offset_cycles: f64,
    allow_decision: bool,
) -> Option<i8> {
    if !allow_decision || !signal_model.supports_epoch_data_symbol_sign_recovery() {
        return None;
    }
    let prompt = data_prompt?;
    let prompt_norm = prompt.norm();
    if !prompt_norm.is_finite() || prompt_norm <= f32::EPSILON {
        return None;
    }
    let carrier_reference =
        align_prompt_with_phase_offset(carrier_prompt, carrier_phase_offset_cycles);
    let carrier_norm = carrier_reference.norm();
    if !carrier_norm.is_finite() || carrier_norm <= f32::EPSILON {
        return None;
    }
    let data_axis_reference = align_prompt_with_phase_offset(
        carrier_reference,
        carrier_to_data_phase_offset_cycles(signal_model, carrier_prompt_source),
    );
    let aligned_data_prompt = prompt * data_axis_reference.conj();
    if !aligned_data_prompt.re.is_finite() || !aligned_data_prompt.im.is_finite() {
        return None;
    }
    (aligned_data_prompt.re.abs() >= aligned_data_prompt.im.abs())
        .then_some(if aligned_data_prompt.re >= 0.0 { 1 } else { -1 })
}

fn carrier_to_data_phase_offset_cycles(
    signal_model: &TrackingSignalModel,
    carrier_prompt_source: CarrierPromptSource,
) -> f64 {
    if carrier_prompt_source != CarrierPromptSource::Pilot {
        return 0.0;
    }
    pilot_carrier_phase_offset_cycles(signal_model)
}

fn requires_dedicated_pilot_carrier(signal_model: &TrackingSignalModel) -> bool {
    pilot_carrier_phase_offset_cycles(signal_model).abs() > f64::EPSILON
}

fn pilot_carrier_phase_offset_cycles(signal_model: &TrackingSignalModel) -> f64 {
    match signal_model.pilot_component.as_ref().map(|component| &component.local_code_model) {
        Some(
            TrackingComponentLocalCodeModel::GalileoE5aQ { .. }
            | TrackingComponentLocalCodeModel::GalileoE5bQ { .. },
        ) => 0.25,
        _ => 0.0,
    }
}

fn align_prompt_with_phase_offset(prompt: Complex<f32>, phase_offset_cycles: f64) -> Complex<f32> {
    let rotation_radians = -carrier_phase_offset_radians(phase_offset_cycles) as f32;
    let rotation = Complex::new(rotation_radians.cos(), rotation_radians.sin());
    prompt * rotation
}

fn apply_dll_code_loop(input: CodeLoopInput) -> CodeLoopUpdate {
    let update = signal_apply_code_loop(bijux_gnss_signal::api::CodeLoopInput {
        current_code_rate_hz: input.current_code_rate_hz,
        previous_reference_code_rate_hz: input.previous_reference_code_rate_hz,
        reference_code_rate_hz: input.reference_code_rate_hz,
        current_code_phase_samples: input.current_code_phase_samples,
        epoch_len_samples: input.epoch_len_samples,
        coherent_integration_s: input.coherent_integration_s,
        nominal_code_rate_hz: input.nominal_code_rate_hz,
        dll_bw_hz: input.dll_bw_hz,
        dll_err: input.dll_err,
        samples_per_chip: input.samples_per_chip,
        samples_per_code: input.samples_per_code,
    });
    CodeLoopUpdate {
        code_rate_hz: update.code_rate_hz,
        code_phase_samples: update.code_phase_samples,
    }
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
            let phase_step_samples = wrapped_code_phase_delta_samples(
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
        instability_markers.iter().copied().find(|(_, abs_phase_step_samples, _, supported)| {
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

#[cfg(test)]
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
    signal_coherent_integration_seconds(epoch_len_samples, sample_rate_hz)
}

fn tracking_lock_detector_thresholds(
    cn0_dbhz: f64,
    coherent_integration_s: f64,
    samples_per_chip: f64,
    tracking_params: TrackingParams,
    carrier_rate_hz_per_s: f64,
) -> LockDetectorThresholds {
    calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz,
        coherent_integration_s,
        samples_per_chip,
        early_late_spacing_chips: tracking_params.early_late_spacing_chips,
        dll_false_unlock_probability: DLL_FALSE_UNLOCK_PROBABILITY,
        pll_false_unlock_probability: PLL_FALSE_UNLOCK_PROBABILITY,
        fll_false_unlock_probability: FLL_FALSE_UNLOCK_PROBABILITY,
        dynamic_stress_hz: carrier_rate_hz_per_s.abs() * coherent_integration_s.max(0.0),
    })
}

fn lock_detector_provenance(thresholds: LockDetectorThresholds) -> String {
    format!(
        " lock_detector=statistical lock_detector_coherent_snr={:.6} dll_lock_threshold={:.6} dll_hold_threshold={:.6} pll_lock_threshold_rad={:.6} pll_hold_threshold_rad={:.6} fll_lock_threshold_hz={:.6} fll_sigma_hz={:.6}",
        thresholds.distributions.coherent_snr_linear,
        thresholds.dll_lock,
        thresholds.dll_hold,
        thresholds.pll_lock_rad,
        thresholds.pll_hold_rad,
        thresholds.fll_lock_hz,
        thresholds.distributions.fll_sigma_hz,
    )
}

fn low_resolution_code_lock(
    samples_per_chip: f64,
    early_late_spacing_chips: f64,
    prompt_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
    anti_false_lock: bool,
) -> bool {
    let effective_sample_separation = samples_per_chip * early_late_spacing_chips.abs();
    effective_sample_separation + f64::EPSILON < LOW_RESOLUTION_DLL_MIN_SAMPLE_SEPARATION
        && prompt_lock
        && (pll_lock || fll_lock)
        && !cycle_slip
        && !anti_false_lock
}

fn short_fade_epoch_budget(tracking_params: TrackingParams) -> u16 {
    let integration_ms = tracking_params.integration_ms.max(1) as f64;
    ((SHORT_FADE_MAX_DURATION_S * 1000.0) / integration_ms).ceil() as u16
        + SHORT_FADE_RECOVERY_GRACE_EPOCHS
}

fn update_pull_in_stable_epochs(
    current_stable_epochs: u8,
    prompt_lock: bool,
    dll_lock: bool,
    pll_lock: bool,
    fll_lock: bool,
    cycle_slip: bool,
) -> u8 {
    if cycle_slip || !prompt_lock || !dll_lock || (!pll_lock && !fll_lock) {
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

fn reacquisition_min_cn0_dbhz(lock_reference_cn0_dbhz: f64) -> f64 {
    if !lock_reference_cn0_dbhz.is_finite() || lock_reference_cn0_dbhz <= 0.0 {
        return TRACKING_LOCK_MIN_CN0_DBHZ;
    }
    (lock_reference_cn0_dbhz - REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ)
        .max(TRACKING_LOCK_MIN_CN0_DBHZ)
}

fn apply_carrier_loop(input: CarrierLoopInput) -> CarrierLoopUpdate {
    let update =
        signal_apply_carrier_tracking_loop(bijux_gnss_signal::api::CarrierTrackingLoopInput {
            current_carrier_hz: input.current_carrier_hz,
            current_carrier_phase_cycles: input.current_carrier_phase_cycles,
            current_carrier_rate_hz_per_s: input.current_carrier_rate_hz_per_s,
            epoch_len_samples: input.epoch_len_samples,
            sample_rate_hz: input.sample_rate_hz,
            coherent_integration_s: input.coherent_integration_s,
            pll_bw_hz: input.pll_bw_hz,
            pll_err_rad: input.pll_err_rad,
            fll_bw_hz: input.fll_bw_hz,
            fll_err_hz: input.fll_err_hz,
            apply_fll: input.apply_fll,
            apply_pll_frequency: input.apply_pll_frequency,
        });
    CarrierLoopUpdate {
        carrier_hz: update.carrier_hz,
        carrier_phase_cycles: update.carrier_phase_cycles,
        carrier_rate_hz_per_s: update.carrier_rate_hz_per_s,
    }
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

        channel.state = self.initial_loop_state(
            &channel.signal_model,
            seed.carrier_hz,
            seed.code_phase_samples,
            seed.cn0_dbhz,
            channel.state.signal_delay_alignment.clone(),
            channel.state.subcarrier_code_phase_refined,
            channel.tracking_params,
            true,
        );
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
    use crate::engine::receiver_config::{
        BandTrackingSpec, ReceiverPipelineConfig, TrackingParams,
    };
    use crate::engine::runtime::ReceiverRuntime;
    use crate::sim::synthetic::{generate_l1_ca, SyntheticSignalParams};
    use bijux_gnss_core::api::{
        AcqHypothesis, AcqUncertainty, Chips, Constellation, Epoch, Hertz, ReceiverSampleTrace,
        SampleTime, SamplesFrame, SatId, Seconds, SignalBand, SignalCode, SignalComponentRole,
        TrackEpoch, GPS_L1_CA_CARRIER_HZ,
    };
    use bijux_gnss_signal::api::TrackingLoopProfile as SignalTrackingLoopProfile;
    use bijux_gnss_signal::api::{
        advance_code_phase_seconds, delay_lock_loop_coefficients, discriminators,
        first_order_angular_loop_coefficients, normalize_dll_discriminator,
        phase_lock_loop_coefficients, sample_ca_code, sample_galileo_e1_cboc, samples_per_code,
        shared_path_code_rate_hz, signal_spec_gps_l5_i, LocalCodeModel, Prn,
    };
    use num_complex::Complex;
    use serde::Deserialize;

    fn vector_measurement(
        channel_id: u8,
        sample_index: u64,
        cn0_dbhz: f64,
        fll_error_hz: f64,
        code_rate_error_hz: f64,
        carrier_rate_hz_per_s: f64,
    ) -> super::VectorTrackingMeasurement {
        super::VectorTrackingMeasurement {
            sat: SatId { constellation: Constellation::Gps, prn: channel_id + 1 },
            channel_id,
            epoch_idx: sample_index / 5_000,
            sample_index,
            cn0_dbhz,
            dll_error_samples: 0.05,
            pll_error_rad: 0.02,
            fll_error_hz,
            code_rate_error_hz,
            carrier_rate_hz_per_s,
            prompt_locked: true,
            dll_locked: true,
            pll_locked: true,
            fll_locked: true,
            channel_state: ChannelState::Tracking,
        }
    }

    #[derive(Debug, Clone, Copy)]
    struct DelayedPathProfile {
        delay_chips: f64,
        relative_amplitude: f32,
        carrier_phase_rad: f32,
    }

    #[derive(Debug, Clone, Copy)]
    enum CodeBiasDiscriminator {
        StandardEarlyLate,
        NarrowEarlyLate,
        DoubleDelta,
    }

    impl CodeBiasDiscriminator {
        fn early_late_spacing_chips(self) -> f64 {
            match self {
                Self::StandardEarlyLate => 0.5,
                Self::NarrowEarlyLate | Self::DoubleDelta => 0.25,
            }
        }
    }

    #[derive(Debug, Clone, Copy)]
    struct MultipathBiasObservation {
        delayed_path: DelayedPathProfile,
        standard_bias_chips: f64,
        narrow_bias_chips: f64,
        double_delta_bias_chips: f64,
    }

    #[derive(Debug, Clone, Copy)]
    struct CleanMultipathBiasBaseline {
        standard_zero_chips: f64,
        narrow_zero_chips: f64,
        double_delta_zero_chips: f64,
    }

    fn multipath_tracking_config() -> ReceiverPipelineConfig {
        ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            tracking_budget_ms: 100.0,
            tracking_over_budget_action: "continue".to_string(),
            ..ReceiverPipelineConfig::default()
        }
    }

    fn multipath_bias_frame(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        direct_code_phase_chips: f64,
        delayed_path: DelayedPathProfile,
    ) -> SamplesFrame {
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let direct = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            direct_code_phase_chips,
            sample_count,
        )
        .expect("direct path code");
        let reflected = sample_ca_code(
            Prn(sat.prn),
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            direct_code_phase_chips - delayed_path.delay_chips,
            sample_count,
        )
        .expect("delayed path code");
        let iq = direct
            .into_iter()
            .zip(reflected)
            .map(|(direct, reflected)| {
                let phase = Complex::new(
                    delayed_path.carrier_phase_rad.cos(),
                    delayed_path.carrier_phase_rad.sin(),
                );
                Complex::new(direct, 0.0) + phase * delayed_path.relative_amplitude * reflected
            })
            .collect();
        SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            iq,
        )
    }

    fn measured_code_zero_crossing_chips(
        config: &ReceiverPipelineConfig,
        tracking: &Tracking,
        frame: &SamplesFrame,
        sat: SatId,
        direct_code_phase_chips: f64,
        discriminator: CodeBiasDiscriminator,
    ) -> f64 {
        let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
        let early_late_spacing_chips = discriminator.early_late_spacing_chips();
        let direct_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                config,
                frame,
                direct_code_phase_chips,
            );
        let mut best_abs_discriminator = f32::INFINITY;
        let mut best_offset_chips = 0.0;
        let signal_model = super::TrackingSignalModel::for_sat(config, sat);
        for offset_index in -50..=50 {
            let offset_chips = offset_index as f64 * 0.01;
            let correlation = tracking.tracking_epoch_correlation(
                frame,
                0,
                frame.len(),
                frame.t0.sample_index,
                &signal_model,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                direct_code_phase_samples + offset_chips * samples_per_chip,
                early_late_spacing_chips,
            );
            let dll_err = match discriminator {
                CodeBiasDiscriminator::DoubleDelta => {
                    super::tracking_dll_discriminator(&correlation)
                }
                CodeBiasDiscriminator::StandardEarlyLate
                | CodeBiasDiscriminator::NarrowEarlyLate => {
                    let (dll_err, _, _, _) = discriminators(
                        correlation.primary.early,
                        correlation.primary.prompt,
                        correlation.primary.late,
                        None,
                    );
                    dll_err
                }
            };
            let dll_err = normalize_dll_discriminator(dll_err, early_late_spacing_chips);
            let abs_discriminator = dll_err.abs();
            if abs_discriminator < best_abs_discriminator {
                best_abs_discriminator = abs_discriminator;
                best_offset_chips = offset_chips;
            }
        }
        best_offset_chips
    }

    fn measured_early_late_zero_crossing_chips(
        config: &ReceiverPipelineConfig,
        tracking: &Tracking,
        frame: &SamplesFrame,
        sat: SatId,
        direct_code_phase_chips: f64,
        early_late_spacing_chips: f64,
    ) -> f64 {
        let discriminator = if (early_late_spacing_chips - 0.5).abs() <= f64::EPSILON {
            CodeBiasDiscriminator::StandardEarlyLate
        } else {
            CodeBiasDiscriminator::NarrowEarlyLate
        };
        measured_code_zero_crossing_chips(
            config,
            tracking,
            frame,
            sat,
            direct_code_phase_chips,
            discriminator,
        )
    }

    fn multipath_bias_observation(
        config: &ReceiverPipelineConfig,
        tracking: &Tracking,
        sat: SatId,
        direct_code_phase_chips: f64,
        baseline: CleanMultipathBiasBaseline,
        delayed_path: DelayedPathProfile,
    ) -> MultipathBiasObservation {
        let frame = multipath_bias_frame(config, sat, direct_code_phase_chips, delayed_path);
        let standard_zero_chips = measured_code_zero_crossing_chips(
            config,
            tracking,
            &frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::StandardEarlyLate,
        );
        let narrow_zero_chips = measured_code_zero_crossing_chips(
            config,
            tracking,
            &frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::NarrowEarlyLate,
        );
        let double_delta_zero_chips = measured_code_zero_crossing_chips(
            config,
            tracking,
            &frame,
            sat,
            direct_code_phase_chips,
            CodeBiasDiscriminator::DoubleDelta,
        );
        MultipathBiasObservation {
            delayed_path,
            standard_bias_chips: (standard_zero_chips - baseline.standard_zero_chips).abs(),
            narrow_bias_chips: (narrow_zero_chips - baseline.narrow_zero_chips).abs(),
            double_delta_bias_chips: (double_delta_zero_chips - baseline.double_delta_zero_chips)
                .abs(),
        }
    }

    fn multipath_bias_observations(
        config: &ReceiverPipelineConfig,
        tracking: &Tracking,
        sat: SatId,
        direct_code_phase_chips: f64,
        delayed_paths: &[DelayedPathProfile],
    ) -> Vec<MultipathBiasObservation> {
        let clean_frame = multipath_bias_frame(
            config,
            sat,
            direct_code_phase_chips,
            DelayedPathProfile {
                delay_chips: 0.0,
                relative_amplitude: 0.0,
                carrier_phase_rad: 0.0,
            },
        );
        let baseline = CleanMultipathBiasBaseline {
            standard_zero_chips: measured_code_zero_crossing_chips(
                config,
                tracking,
                &clean_frame,
                sat,
                direct_code_phase_chips,
                CodeBiasDiscriminator::StandardEarlyLate,
            ),
            narrow_zero_chips: measured_code_zero_crossing_chips(
                config,
                tracking,
                &clean_frame,
                sat,
                direct_code_phase_chips,
                CodeBiasDiscriminator::NarrowEarlyLate,
            ),
            double_delta_zero_chips: measured_code_zero_crossing_chips(
                config,
                tracking,
                &clean_frame,
                sat,
                direct_code_phase_chips,
                CodeBiasDiscriminator::DoubleDelta,
            ),
        };
        delayed_paths
            .iter()
            .copied()
            .map(|delayed_path| {
                multipath_bias_observation(
                    config,
                    tracking,
                    sat,
                    direct_code_phase_chips,
                    baseline,
                    delayed_path,
                )
            })
            .collect()
    }

    #[test]
    fn narrow_correlator_reduces_delayed_path_code_bias() {
        let config = multipath_tracking_config();
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let direct_code_phase_chips = 245.25;
        let clean_frame = multipath_bias_frame(
            &config,
            sat,
            direct_code_phase_chips,
            DelayedPathProfile {
                delay_chips: 0.0,
                relative_amplitude: 0.0,
                carrier_phase_rad: 0.0,
            },
        );
        let clean_standard_zero = measured_early_late_zero_crossing_chips(
            &config,
            &tracking,
            &clean_frame,
            sat,
            direct_code_phase_chips,
            0.5,
        );
        let clean_narrow_zero = measured_early_late_zero_crossing_chips(
            &config,
            &tracking,
            &clean_frame,
            sat,
            direct_code_phase_chips,
            0.25,
        );
        let delayed_paths = [
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.65,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
        ];

        for delayed_path in delayed_paths {
            let frame = multipath_bias_frame(&config, sat, direct_code_phase_chips, delayed_path);
            let standard_zero_chips = measured_early_late_zero_crossing_chips(
                &config,
                &tracking,
                &frame,
                sat,
                direct_code_phase_chips,
                0.5,
            );
            let narrow_zero_chips = measured_early_late_zero_crossing_chips(
                &config,
                &tracking,
                &frame,
                sat,
                direct_code_phase_chips,
                0.25,
            );
            let standard_bias_chips = (standard_zero_chips - clean_standard_zero).abs();
            let narrow_bias_chips = (narrow_zero_chips - clean_narrow_zero).abs();

            assert!(
                standard_bias_chips > 0.0 && narrow_bias_chips < standard_bias_chips,
                "narrow correlator must reduce delayed-path code bias: delayed_path={delayed_path:?} standard_bias_chips={standard_bias_chips:.6} narrow_bias_chips={narrow_bias_chips:.6}",
            );
        }
    }

    #[test]
    fn double_delta_multipath_bias_sweep_quantifies_benefit_and_failure_region() {
        let config = multipath_tracking_config();
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let direct_code_phase_chips = 245.25;
        let delayed_paths = [
            DelayedPathProfile {
                delay_chips: 0.20,
                relative_amplitude: 0.20,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.20,
                relative_amplitude: 0.50,
                carrier_phase_rad: std::f32::consts::FRAC_PI_2,
            },
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.20,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.35,
                carrier_phase_rad: std::f32::consts::FRAC_PI_2,
            },
            DelayedPathProfile {
                delay_chips: 0.45,
                relative_amplitude: 0.50,
                carrier_phase_rad: std::f32::consts::PI,
            },
            DelayedPathProfile {
                delay_chips: 0.65,
                relative_amplitude: 0.35,
                carrier_phase_rad: 0.0,
            },
            DelayedPathProfile {
                delay_chips: 0.65,
                relative_amplitude: 0.50,
                carrier_phase_rad: std::f32::consts::PI,
            },
        ];

        let observations = multipath_bias_observations(
            &config,
            &tracking,
            sat,
            direct_code_phase_chips,
            &delayed_paths,
        );
        let benefit_count = observations
            .iter()
            .filter(|observation| {
                observation.double_delta_bias_chips < observation.narrow_bias_chips
            })
            .count();
        let failure_count = observations
            .iter()
            .filter(|observation| {
                observation.double_delta_bias_chips > observation.narrow_bias_chips
            })
            .count();
        let neutral_count = observations.len() - benefit_count - failure_count;
        let standard_biased_count =
            observations.iter().filter(|observation| observation.standard_bias_chips > 0.0).count();

        assert_eq!(observations.len(), 7);
        assert_eq!(benefit_count, 1, "observations={observations:#?}");
        assert_eq!(failure_count, 4, "observations={observations:#?}");
        assert_eq!(neutral_count, 2, "observations={observations:#?}");
        assert_eq!(standard_biased_count, 3, "observations={observations:#?}");
        assert!(observations.iter().any(|observation| {
            (observation.delayed_path.delay_chips - 0.45).abs() <= f64::EPSILON
                && (observation.delayed_path.relative_amplitude - 0.50).abs() <= f32::EPSILON
                && (observation.delayed_path.carrier_phase_rad - std::f32::consts::PI).abs()
                    <= f32::EPSILON
                && observation.double_delta_bias_chips < observation.narrow_bias_chips
        }));
        assert!(observations.iter().any(|observation| {
            (observation.delayed_path.delay_chips - 0.65).abs() <= f64::EPSILON
                && (observation.delayed_path.relative_amplitude - 0.50).abs() <= f32::EPSILON
                && (observation.delayed_path.carrier_phase_rad - std::f32::consts::PI).abs()
                    <= f32::EPSILON
                && observation.double_delta_bias_chips > observation.narrow_bias_chips
        }));
    }

    #[test]
    fn tracking_dll_discriminator_uses_double_delta_outer_pair() {
        let correlation = super::TrackingEpochCorrelation {
            primary: super::CorrelatorOutput {
                early: Complex::new(8.0, 0.0),
                prompt: Complex::new(10.0, 0.0),
                late: Complex::new(4.0, 0.0),
                early_late_noise_weight_energy: 0.0,
            },
            double_delta_outer: Some(super::CorrelatorOutput {
                early: Complex::new(6.0, 0.0),
                prompt: Complex::new(10.0, 0.0),
                late: Complex::new(2.0, 0.0),
                early_late_noise_weight_energy: 0.0,
            }),
            carrier_prompt: Complex::new(10.0, 0.0),
            carrier_prompt_source: super::CarrierPromptSource::Primary,
            data_prompt: None,
            secondary_code_prompt_period_index: 0,
            subcarrier_ambiguity_guard: None,
        };

        assert!(
            (super::tracking_dll_discriminator(&correlation) - 0.166_666_67).abs() <= f32::EPSILON
        );
    }

    #[test]
    fn tracking_dll_discriminator_keeps_inner_pair_when_double_delta_is_larger() {
        let correlation = super::TrackingEpochCorrelation {
            primary: super::CorrelatorOutput {
                early: Complex::new(5.0, 0.0),
                prompt: Complex::new(10.0, 0.0),
                late: Complex::new(5.0, 0.0),
                early_late_noise_weight_energy: 0.0,
            },
            double_delta_outer: Some(super::CorrelatorOutput {
                early: Complex::new(2.0, 0.0),
                prompt: Complex::new(10.0, 0.0),
                late: Complex::new(8.0, 0.0),
                early_late_noise_weight_energy: 0.0,
            }),
            carrier_prompt: Complex::new(10.0, 0.0),
            carrier_prompt_source: super::CarrierPromptSource::Primary,
            data_prompt: None,
            secondary_code_prompt_period_index: 0,
            subcarrier_ambiguity_guard: None,
        };

        assert_eq!(super::tracking_dll_discriminator(&correlation), 0.0);
    }

    fn galileo_e1_subcarrier_guard_config() -> ReceiverPipelineConfig {
        ReceiverPipelineConfig {
            sampling_freq_hz: 4_092_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 4092,
            channels: 1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            ..ReceiverPipelineConfig::default()
        }
    }

    fn galileo_e1_subcarrier_guard_for_seed(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        code_phase_chips: f64,
        seed_offset_chips: f64,
    ) -> super::SubcarrierAmbiguityGuard {
        let sample_count = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = SamplesFrame::new(
            SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            sample_galileo_e1_cboc(
                sat.prn,
                config.sampling_freq_hz,
                code_phase_chips,
                sample_count,
                0,
                1,
            )
            .expect("Galileo E1 CBOC samples")
            .into_iter()
            .map(|value| Complex::new(value, 0.0))
            .collect(),
        );
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
        let code_phase_samples = (code_phase_chips + seed_offset_chips) * samples_per_chip;
        tracking
            .tracking_epoch_correlation(
                &frame,
                0,
                frame.len(),
                0,
                &signal_model,
                0.0,
                0.0,
                config.code_freq_basis_hz,
                code_phase_samples,
                config.early_late_spacing_chips,
            )
            .subcarrier_ambiguity_guard
            .expect("Galileo E1 requires subcarrier ambiguity guard")
    }

    fn gps_l5q_secondary_code_component() -> super::TrackingComponentModel {
        super::TrackingComponentModel {
            role: SignalComponentRole::Pilot,
            code_length: 10_230,
            phase_transition_source: super::TrackingPhaseTransitionSource::SecondaryCode,
            local_code_model: super::TrackingComponentLocalCodeModel::Local(
                LocalCodeModel::gps_l5_q(18).expect("GPS L5Q local code"),
            ),
        }
    }

    fn secondary_code_prompt_history(
        component: &super::TrackingComponentModel,
        phase_periods: usize,
        observed_periods: usize,
        prompt: Complex<f32>,
    ) -> Vec<super::SecondaryCodePromptSample> {
        (0..observed_periods)
            .map(|period_offset| {
                let primary_code_period_index = phase_periods + period_offset;
                assert!(component.secondary_code_symbol(primary_code_period_index).is_some());
                super::SecondaryCodePromptSample { primary_code_period_index, prompt }
            })
            .collect()
    }

    #[test]
    fn secondary_code_sync_accepts_best_likelihood_phase() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 7, 20, Complex::new(2.0, 0.25));

        let sync =
            super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

        assert!(sync.accepted, "sync result should be accepted: {sync:?}");
        assert_eq!(sync.phase_periods, 7);
        assert_eq!(sync.observed_periods, 20);
        assert!(sync.best_likelihood > sync.next_best_likelihood);
        assert!(sync.confidence >= super::SECONDARY_CODE_SYNC_MIN_CONFIDENCE);
    }

    #[test]
    fn secondary_code_sync_scores_incorrect_phase_lower() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 11, 20, Complex::new(1.5, -0.5));

        let correct = super::secondary_code_phase_score(&component, &history, 11);
        let incorrect = super::secondary_code_phase_score(&component, &history, 12);

        assert!(correct.likelihood > incorrect.likelihood);
    }

    #[test]
    fn secondary_code_sync_waits_for_enough_observations() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 3, 3, Complex::new(1.0, 0.0));

        assert!(super::secondary_code_sync_from_prompt_history(&component, &history).is_none());
    }

    #[test]
    fn secondary_code_sync_rejects_zero_energy_prompt_history() {
        let component = gps_l5q_secondary_code_component();
        let history = secondary_code_prompt_history(&component, 5, 20, Complex::new(0.0, 0.0));

        let sync =
            super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

        assert!(!sync.accepted);
        assert_eq!(sync.confidence, 0.0);
    }

    #[test]
    fn secondary_code_sync_updates_from_pilot_carrier_prompt_history() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let mut state = empty_loop_state();

        for period_offset in 0..20 {
            super::update_secondary_code_synchronization(
                &signal_model,
                &mut state,
                7 + period_offset,
                Complex::new(1.0, 0.125),
            );
        }

        let sync = state.secondary_code_sync.expect("secondary code sync");
        assert!(sync.accepted, "pilot prompt sync should be accepted: {sync:?}");
        assert_eq!(sync.phase_periods, 7);
        assert_eq!(sync.observed_periods, 20);
        assert_eq!(state.secondary_code_prompt_history.len(), 20);
    }

    #[test]
    fn secondary_code_sync_accepts_galileo_e5_pilot_phase() {
        let config = ReceiverPipelineConfig::default();
        for (signal_code, phase_periods) in [(SignalCode::E5a, 37), (SignalCode::E5b, 61)] {
            let signal_model = super::TrackingSignalModel::for_sat_signal_band(
                &config,
                SatId { constellation: Constellation::Galileo, prn: 11 },
                SignalBand::E5,
                signal_code,
                None,
            );
            let component = signal_model.carrier_component();
            let history = secondary_code_prompt_history(
                &component,
                phase_periods,
                100,
                Complex::new(1.0, 0.2),
            );

            let sync =
                super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

            assert!(sync.accepted, "Galileo {signal_code:?} sync should be accepted: {sync:?}");
            assert_eq!(sync.phase_periods, phase_periods);
            assert!(sync.best_likelihood > sync.next_best_likelihood);
        }
    }

    #[test]
    fn secondary_code_sync_ignores_signals_without_secondary_code() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat(
            &config,
            SatId { constellation: Constellation::Gps, prn: 1 },
        );
        let mut state = empty_loop_state();

        let sync = super::update_secondary_code_synchronization(
            &signal_model,
            &mut state,
            0,
            Complex::new(1.0, 0.0),
        );

        assert!(sync.is_none());
        assert!(state.secondary_code_sync.is_none());
        assert!(state.secondary_code_prompt_history.is_empty());
    }

    #[test]
    fn secondary_code_sync_provenance_reports_accepted_phase() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let sync = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.5,
            best_likelihood: 1.0,
            next_best_likelihood: 0.5,
            observed_periods: 20,
            accepted: true,
        };

        let provenance = super::secondary_code_sync_provenance(&signal_model, Some(sync))
            .expect("secondary-code provenance");

        assert!(provenance.contains("secondary_code_sync=accepted"));
        assert!(provenance.contains("secondary_code_phase_periods=7"));
        assert!(provenance.contains("secondary_code_sync_confidence=0.500000"));
        assert!(provenance.contains("secondary_code_observed_periods=20"));
    }

    #[test]
    fn secondary_code_sync_provenance_reports_insufficient_evidence() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let provenance =
            super::secondary_code_sync_provenance(&signal_model, None).expect("provenance");

        assert_eq!(provenance, " secondary_code_sync=insufficient");
    }

    #[test]
    fn secondary_code_sync_keeps_accepted_phase_over_weaker_candidate() {
        let accepted = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.25,
            best_likelihood: 1.0,
            next_best_likelihood: 0.75,
            observed_periods: 20,
            accepted: true,
        };
        let rejected = super::SecondaryCodeSyncResult {
            phase_periods: 3,
            confidence: 0.01,
            best_likelihood: 0.6,
            next_best_likelihood: 0.594,
            observed_periods: 20,
            accepted: false,
        };

        let selected = super::select_secondary_code_synchronization(Some(accepted), Some(rejected))
            .expect("selected sync");

        assert_eq!(selected, accepted);
    }

    #[test]
    fn secondary_code_sync_replaces_rejected_phase_with_accepted_candidate() {
        let rejected = super::SecondaryCodeSyncResult {
            phase_periods: 3,
            confidence: 0.01,
            best_likelihood: 0.6,
            next_best_likelihood: 0.594,
            observed_periods: 20,
            accepted: false,
        };
        let accepted = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 0.25,
            best_likelihood: 1.0,
            next_best_likelihood: 0.75,
            observed_periods: 20,
            accepted: true,
        };

        let selected = super::select_secondary_code_synchronization(Some(rejected), Some(accepted))
            .expect("selected sync");

        assert_eq!(selected, accepted);
    }

    #[test]
    fn prompt_center_period_index_follows_code_phase_across_epoch_boundary() {
        assert_eq!(
            super::prompt_center_primary_code_period_index(0, 100.0, 1.0, 10_230, 10_230),
            0
        );
        assert_eq!(
            super::prompt_center_primary_code_period_index(0, 8_182.0, 1.0, 10_230, 10_230),
            1
        );
    }

    #[cfg(feature = "nav")]
    fn gps_l1ca_tracking_signal_model() -> super::TrackingSignalModel {
        let config = ReceiverPipelineConfig::default();
        super::TrackingSignalModel::for_sat(
            &config,
            SatId { constellation: Constellation::Gps, prn: 1 },
        )
    }

    #[cfg(feature = "nav")]
    fn prompt_history_epochs(prompt_i: &[f32]) -> Vec<TrackEpoch> {
        prompt_i
            .iter()
            .enumerate()
            .map(|(index, prompt_i)| TrackEpoch {
                epoch: Epoch { index: index as u64 },
                sample_index: index as u64,
                sat: SatId { constellation: Constellation::Gps, prn: 1 },
                prompt_i: *prompt_i,
                tracking_provenance: "tracking".to_string(),
                ..TrackEpoch::default()
            })
            .collect()
    }

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

    #[cfg(feature = "nav")]
    #[test]
    fn annotate_navigation_bit_signs_waits_for_boundary_confidence() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let prompt = vec![1.0_f32; 60];
        let mut epochs = prompt_history_epochs(&prompt);

        super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

        assert!(
            epochs.iter().all(|epoch| epoch.navigation_bit_sign.is_none()),
            "ambiguous symbol boundaries must not emit bit signs: {epochs:?}"
        );
        assert!(epochs.iter().all(|epoch| !epoch.nav_bit_lock));
        assert!(epochs
            .iter()
            .all(|epoch| !epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
    }

    #[cfg(feature = "nav")]
    #[test]
    fn annotate_navigation_bit_signs_emits_confident_symbol_windows() {
        let signal_model = gps_l1ca_tracking_signal_model();
        let mut prompt = vec![0.2_f32; 6];
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        let mut epochs = prompt_history_epochs(&prompt);

        super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

        assert!(epochs[..6].iter().all(|epoch| epoch.navigation_bit_sign.is_none()));
        assert!(epochs[6..26]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
        assert!(epochs[26..46]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(-1) && epoch.nav_bit_lock));
        assert!(epochs[46..66]
            .iter()
            .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
        assert!(epochs[6..66]
            .iter()
            .all(|epoch| epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
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
        let uncertainty = AcqUncertainty {
            doppler_hz: 250.0,
            code_phase_samples: 0.75,
            doppler_rate_hz_per_s: None,
            covariance: None,
        };

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
    fn code_value_at_phase_wraps_fractional_chip_phases() {
        let code = vec![1_i8, -1, 1, -1];

        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 0.25).unwrap(), 1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 1.75).unwrap(), -1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, 4.10).unwrap(), 1.0);
        assert_eq!(bijux_gnss_signal::api::code_value_at_phase(&code, -0.10).unwrap(), -1.0);
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
        let decision = super::classify_prompt_phase(
            -0.39,
            Some(0.11),
            0.0,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );

        assert!(decision.nav_bit_transition);
        assert!(!decision.cycle_slip);
        assert!((decision.aligned_phase_cycles - 0.11).abs() <= 0.02);
        assert!(decision.aligned_phase_delta_cycles.abs() <= 0.02);
        assert!((decision.nav_bit_phase_offset_cycles - 0.5).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_preserves_cycle_slip_for_non_nav_jump() {
        let decision = super::classify_prompt_phase(
            0.36,
            Some(0.0),
            0.0,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );

        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
        assert!((decision.aligned_phase_delta_cycles - 0.36).abs() <= 1.0e-9);
        assert!((decision.nav_bit_phase_offset_cycles - 0.0).abs() <= f64::EPSILON);
    }

    #[test]
    fn classify_prompt_phase_keeps_half_cycle_jump_as_slip_without_transition_metadata() {
        let decision = super::classify_prompt_phase(
            -0.39,
            Some(0.11),
            0.0,
            super::TrackingPhaseTransitionSource::None,
        );

        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
        assert!(decision.aligned_phase_delta_cycles.abs() > 0.35);
    }

    #[test]
    fn secondary_code_phase_transition_waits_for_accepted_sync() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let transition_source =
            super::carrier_phase_transition_source_for_prompt(&signal_model, None);
        let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

        assert_eq!(transition_source, super::TrackingPhaseTransitionSource::None);
        assert!(!decision.nav_bit_transition);
        assert!(decision.cycle_slip);
    }

    #[test]
    fn secondary_code_phase_transition_accepts_synchronized_half_cycle_jump() {
        let config = ReceiverPipelineConfig::default();
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Gps, prn: 18 },
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let sync = super::SecondaryCodeSyncResult {
            phase_periods: 7,
            confidence: 1.0,
            best_likelihood: 1.0,
            next_best_likelihood: 0.0,
            observed_periods: 20,
            accepted: true,
        };

        let transition_source =
            super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
        let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

        assert_eq!(transition_source, super::TrackingPhaseTransitionSource::SecondaryCode);
        assert!(decision.nav_bit_transition);
        assert!(!decision.cycle_slip);
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 70.0,
                navigation_data: true.into(),
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
                super::TrackingPhaseTransitionSource::DataSymbol,
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
        let next = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 137.5,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 4_092_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        })
        .code_phase_samples;
        assert!((next - 137.5).abs() < 1.0e-9, "next={next}");
    }

    #[test]
    fn advance_code_phase_samples_applies_dll_correction() {
        let next = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 4_092_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.4,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        })
        .code_phase_samples;
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

    #[test]
    fn tracking_params_for_state_uses_adapted_profile_when_enabled() {
        let tracking = Tracking::new(
            ReceiverPipelineConfig {
                adaptive_tracking_enabled: true,
                ..ReceiverPipelineConfig::default()
            },
            ReceiverRuntime::default(),
        );
        let base_tracking_params = TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let mut state = empty_loop_state();
        state.tracking_loop_profile = SignalTrackingLoopProfile {
            dll_bw_hz: 1.2,
            pll_bw_hz: 8.25,
            fll_bw_hz: 5.0,
            integration_ms: 5,
        };

        let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

        assert_eq!(
            effective.early_late_spacing_chips,
            base_tracking_params.early_late_spacing_chips
        );
        assert_eq!(effective.dll_bw_hz, 1.2);
        assert_eq!(effective.pll_bw_hz, 8.25);
        assert_eq!(effective.fll_bw_hz, 5.0);
        assert_eq!(effective.integration_ms, 5);
    }

    #[test]
    fn tracking_params_for_state_ignores_adapted_profile_when_disabled() {
        let tracking = Tracking::new(
            ReceiverPipelineConfig {
                adaptive_tracking_enabled: false,
                ..ReceiverPipelineConfig::default()
            },
            ReceiverRuntime::default(),
        );
        let base_tracking_params = TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let mut state = empty_loop_state();
        state.tracking_loop_profile = SignalTrackingLoopProfile {
            dll_bw_hz: 1.2,
            pll_bw_hz: 8.25,
            fll_bw_hz: 5.0,
            integration_ms: 5,
        };

        let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

        assert_eq!(effective.dll_bw_hz, base_tracking_params.dll_bw_hz);
        assert_eq!(effective.pll_bw_hz, base_tracking_params.pll_bw_hz);
        assert_eq!(effective.fll_bw_hz, base_tracking_params.fll_bw_hz);
        assert_eq!(effective.integration_ms, base_tracking_params.integration_ms);
    }

    fn empty_loop_state() -> super::LoopState {
        super::LoopState {
            carrier_hz: 0.0,
            carrier_phase_cycles: 0.0,
            carrier_rate_hz_per_s: 0.0,
            code_rate_hz: 0.0,
            code_rate_reference_hz: 0.0,
            code_phase_samples: 0.0,
            tracking_adaptation_state: Default::default(),
            tracking_loop_profile: SignalTrackingLoopProfile {
                dll_bw_hz: 2.0,
                pll_bw_hz: 15.0,
                fll_bw_hz: 10.0,
                integration_ms: 1,
            },
            signal_delay_alignment: None,
            subcarrier_code_phase_refined: false,
            acquisition_cn0_proxy_dbhz: 45.0,
            lock_reference_cn0_dbhz: 45.0,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            secondary_code_prompt_history: std::collections::VecDeque::new(),
            secondary_code_sync: None,
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
    fn vector_tracking_rejects_weak_or_incomplete_evidence() {
        let weak = vector_measurement(0, 5_000, 34.9, 1.0, 0.1, 0.0);
        let mut unlocked = vector_measurement(1, 5_000, 42.0, 1.0, 0.1, 0.0);
        unlocked.pll_locked = false;
        let strong = vector_measurement(2, 5_000, 42.0, 1.0, 0.1, 0.0);

        assert!(!super::vector_tracking_measurement_is_usable(weak));
        assert!(!super::vector_tracking_measurement_is_usable(unlocked));
        assert!(super::vector_tracking_measurement_is_usable(strong));
        assert!(super::vector_tracking_prediction(&[strong]).is_none());
    }

    #[test]
    fn vector_tracking_prediction_uses_latest_measurement_per_channel() {
        let mut state = super::VectorTrackingState::default();
        state.record(vector_measurement(0, 100, 42.0, 1.0, 0.10, 4.0), 1_000.0);
        state.record(vector_measurement(0, 110, 42.0, 5.0, 0.50, 8.0), 1_000.0);
        state.record(vector_measurement(1, 108, 42.0, 7.0, 0.70, 12.0), 1_000.0);

        let prediction = state.prediction_for(120, 1_000.0).expect("vector prediction");

        assert_eq!(prediction.contributor_count, 2);
        assert_eq!(prediction.sample_index, 110);
        assert!((prediction.receiver_position_code_phase_error_samples - 0.05).abs() < 1.0e-9);
        assert!((prediction.receiver_clock_frequency_error_hz - 6.0).abs() < 1.0e-9);
        assert!((prediction.receiver_code_rate_error_hz - 0.60).abs() < 1.0e-9);
        assert!((prediction.receiver_motion_frequency_rate_hz_per_s - 10.0).abs() < 1.0e-9);
    }

    #[test]
    fn vector_tracking_application_bounds_weak_channel_aid() {
        let config = ReceiverPipelineConfig::default();
        let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
        let mut loop_state = tracking.initial_loop_state(
            &signal_model,
            1500.0,
            0.0,
            42.0,
            None,
            false,
            config.tracking_params(SignalBand::L1),
            false,
        );
        loop_state.state = ChannelState::PullIn;
        let prediction = super::VectorTrackingPrediction {
            sample_index: 10_000,
            contributor_count: 3,
            mean_cn0_dbhz: 44.0,
            receiver_position_code_phase_error_samples: 10.0,
            receiver_clock_frequency_error_hz: 1_000.0,
            receiver_code_rate_error_hz: 50.0,
            receiver_motion_frequency_rate_hz_per_s: 5_000.0,
        };

        let application =
            super::vector_tracking_application(prediction, &loop_state).expect("vector aid");

        assert!(
            (application.carrier_frequency_correction_hz
                - super::VECTOR_TRACKING_MAX_CARRIER_AID_HZ * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.code_rate_correction_hz
                - super::VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.code_phase_correction_samples
                - super::VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES * 0.35)
                .abs()
                < 1.0e-9
        );
        assert!(
            (application.carrier_rate_correction_hz_per_s
                - super::VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S * 0.35)
                .abs()
                < 1.0e-9
        );
        loop_state.state = ChannelState::Lost;
        assert!(super::vector_tracking_application(prediction, &loop_state).is_none());
    }

    #[test]
    fn apply_dll_code_loop_updates_code_rate_from_discriminator() {
        let coherent_integration_s = 5_000.0 / 1_023_000.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
            + delay_lock_loop_coefficients(2.0, coherent_integration_s).rate_gain_hz_per_chip
                * 0.25;
        assert!((update.code_rate_hz - expected).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_dll_code_loop_follows_reference_code_rate_step_without_dll_error() {
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_450.0,
            current_code_phase_samples: 250.0,
            epoch_len_samples: 5_000,
            coherent_integration_s: 5_000.0 / 1_023_000.0,
            nominal_code_rate_hz: 1_023_000.0,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip: 4.887585532746823,
            samples_per_code: 5_000,
        });

        assert!((update.code_rate_hz - 1_023_450.0).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_dll_code_loop_uses_coherent_interval_to_scale_gain() {
        let short = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
            (long.code_rate_hz - 1_023_000.0).abs() > (short.code_rate_hz - 1_023_000.0).abs(),
            "short={short:?} long={long:?}"
        );
    }

    #[test]
    fn apply_dll_code_loop_pulls_positive_code_error_toward_prompt() {
        let current_code_phase_samples = 250.0;
        let update = super::apply_dll_code_loop(super::CodeLoopInput {
            current_code_rate_hz: 1_023_000.0,
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
            previous_reference_code_rate_hz: 1_023_000.0,
            reference_code_rate_hz: 1_023_000.0,
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
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
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
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 0.0,
                carrier_phase_rad,
                cn0_db_hz: 70.0,
                navigation_data: false.into(),
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: carrier_hz,
                code_phase_chips: 0.0,
                carrier_phase_rad: initial_phase_cycles * std::f64::consts::TAU,
                cn0_db_hz: 90.0,
                navigation_data: false.into(),
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
            current_carrier_rate_hz_per_s: 0.0,
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
        assert!(
            (update.carrier_rate_hz_per_s
                - pll_coefficients.frequency_rate_gain_hz_per_s_per_rad * 0.25)
                .abs()
                < 1.0e-9,
            "{update:?}"
        );
        let expected_phase_cycles = 12.0
            + (1_000.0 + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)) * 0.0005
            + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
        assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}",);
    }

    #[test]
    fn apply_carrier_loop_uses_fll_correction_during_pull_in() {
        let update = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 80.0,
            current_carrier_phase_cycles: 12.0,
            current_carrier_rate_hz_per_s: 0.0,
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
        let expected_carrier_hz = 80.0 + (30.0 * fll_coefficients.error_blend).clamp(-40.0, 40.0);
        assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
    }

    #[test]
    fn apply_carrier_loop_accumulates_unwrapped_phase_across_epochs() {
        let first = super::apply_carrier_loop(super::CarrierLoopInput {
            current_carrier_hz: 1_500.0,
            current_carrier_phase_cycles: 128.25,
            current_carrier_rate_hz_per_s: 0.0,
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
            current_carrier_rate_hz_per_s: first.carrier_rate_hz_per_s,
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
            current_carrier_rate_hz_per_s: 0.0,
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
            current_carrier_rate_hz_per_s: 0.0,
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
            current_carrier_rate_hz_per_s: 0.0,
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
        let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
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
            previous_reference_code_rate_hz: config.code_freq_basis_hz,
            reference_code_rate_hz: config.code_freq_basis_hz,
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
    fn tracking_lock_detector_thresholds_derive_from_spacing_and_dynamics() {
        let tracking_params = super::TrackingParams {
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        };
        let high_resolution =
            super::tracking_lock_detector_thresholds(45.0, 0.001, 4.0, tracking_params, 0.0);
        let low_resolution =
            super::tracking_lock_detector_thresholds(45.0, 0.001, 1.0, tracking_params, 25_000.0);

        assert!(low_resolution.dll_lock > high_resolution.dll_lock);
        assert!(low_resolution.fll_lock_hz > high_resolution.fll_lock_hz);
        assert!(high_resolution.pll_hold_rad > high_resolution.pll_lock_rad);
    }

    #[test]
    fn low_resolution_code_lock_retains_supported_tracking_when_carrier_is_stable() {
        assert!(super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, false));
        assert!(super::low_resolution_code_lock(1.0, 0.5, true, false, true, false, false));
    }

    #[test]
    fn low_resolution_code_lock_requires_prompt_and_lock_safety_guards() {
        assert!(!super::low_resolution_code_lock(1.0, 0.5, false, true, true, false, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, false, false, false, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, true, false));
        assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, true));
        assert!(!super::low_resolution_code_lock(4.0, 0.5, true, true, true, false, false));
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: -750.0,
                code_phase_chips: 211.25,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 52.0,
                navigation_data: false.into(),
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
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: bijux_gnss_core::api::Hertz(-750.0),
                doppler_rate_hz_per_s: 0.0,
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 750.0,
                code_phase_chips: 200.25,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 58.0,
                navigation_data: false.into(),
            },
            0x330C_2000,
            0.04,
        );
        let refined_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config, &frame, 200.25,
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
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz: 1_000.0,
                code_phase_chips: 10.0,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 48.0,
                navigation_data: false.into(),
            },
            7,
            duration_s,
        );
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(1_000.0),
            doppler_rate_hz_per_s: 0.0,
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
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
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
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
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
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                glonass_frequency_channel: None,
                source_time: ReceiverSampleTrace::default(),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(0.0),
                doppler_rate_hz_per_s: 0.0,
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
        let selected_prns =
            incremental.channels.iter().map(|channel| channel.sat.prn).collect::<Vec<_>>();

        assert_eq!(selected_prns, vec![7, 23]);
    }

    #[test]
    fn begin_incremental_tracking_preserves_glonass_channel_and_carrier_seed() {
        let channel = bijux_gnss_core::api::GlonassFrequencyChannel::new(-4)
            .expect("channel -4 must be valid");
        let carrier_hz = 2_048_125.0;
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 511_000.0,
            code_length: 511,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Glonass, prn: 8 },
            signal_band: SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: Some(channel),
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(125.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(carrier_hz),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
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
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);
        let channel_state = incremental.channels.first().expect("GLONASS tracking channel");

        assert_eq!(channel_state.signal_model.signal_band, SignalBand::L1);
        assert_eq!(channel_state.signal_model.glonass_frequency_channel, Some(channel));
        assert_eq!(channel_state.state.carrier_hz, carrier_hz);
        assert!((channel_state.signal_model.code_rate_hz - 511_000.0).abs() <= f64::EPSILON);
        assert_eq!(channel_state.signal_model.code_length, 511);
    }

    #[test]
    fn begin_incremental_tracking_seeds_code_rate_from_carrier_aid() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(1_500.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(1_500.0),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
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
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);
        let channel_state = incremental.channels.first().expect("GPS L5 tracking channel");
        let expected_code_rate_hz =
            shared_path_code_rate_hz(1_500.0, signal_spec_gps_l5_i(), signal_spec_gps_l5_i())
                .expect("carrier-aided code rate");

        assert!((channel_state.state.code_rate_hz - expected_code_rate_hz).abs() <= 1.0e-9);
        assert!(
            (channel_state.state.code_rate_reference_hz - expected_code_rate_hz).abs() <= 1.0e-9
        );
    }

    #[test]
    fn begin_incremental_tracking_refuses_incompatible_carrier_aiding_seed() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let tracking = Tracking::new(config, ReceiverRuntime::default());
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat: SatId { constellation: Constellation::Gps, prn: 18 },
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(1_500.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(GPS_L1_CA_CARRIER_HZ.value() + 1_500.0),
            code_phase_samples: 37,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 8.0,
            peak_second_ratio: 3.0,
            cn0_proxy: 48.0,
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
        };

        let incremental = tracking.begin_incremental_tracking(&[acquisition]);

        assert!(
            incremental.channels.is_empty(),
            "wrong-band carrier seeds must not enter carrier-aided tracking: {incremental:?}",
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_gps_l5q() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5Q,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Pilot);
        assert!(signal_model.secondary_code.is_some());
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::EarlyPromptLate
        );
        assert!((signal_model.nominal_carrier_hz() - 1_176_450_000.0).abs() <= f64::EPSILON);
        assert!(!signal_model.supports_navigation_bit_sign_recovery());
    }

    #[test]
    fn tracked_signal_center_hz_rebases_non_l1_signals_into_receiver_if() {
        let signal = signal_spec_gps_l5_i();
        let center_hz = super::tracked_signal_center_hz(0.0, signal);

        assert!(
            (center_hz - (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())).abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn tracked_signal_doppler_hz_removes_signal_specific_center_offset() {
        let signal = signal_spec_gps_l5_i();
        let tracked_carrier_hz = super::tracked_signal_center_hz(0.0, signal) + 875.0;

        assert!(
            (super::tracked_signal_doppler_hz(0.0, tracked_carrier_hz, signal) - 875.0).abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn normalize_acquisition_carrier_hz_rebases_doppler_only_l5_seed() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: SignalBand::L5,
            signal_code: SignalCode::L5I,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(180.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(180.0),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 60.0,
            score: 1.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("seeded_joint_component_tracking".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        };

        let normalized =
            super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition);

        assert!(
            (normalized
                - (super::tracked_signal_center_hz(
                    config.intermediate_freq_hz,
                    signal_model.signal_spec
                ) + acquisition.doppler_hz.0))
                .abs()
                <= f64::EPSILON
        );
    }

    #[test]
    fn normalize_acquisition_carrier_hz_keeps_absolute_wideband_carrier() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );
        let absolute_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 250.0;
        let acquisition = bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: SignalBand::E5,
            signal_code: SignalCode::E5b,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(250.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(absolute_carrier_hz),
            code_phase_samples: 0,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 60.0,
            score: 1.0,
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

        assert_eq!(
            super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition),
            absolute_carrier_hz
        );
    }

    #[test]
    fn carrier_aided_code_rate_hz_uses_tracked_signal_center_and_code_rate() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let tracked_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 875.0;
        let aided_code_rate_hz =
            super::carrier_aided_code_rate_hz(&config, &signal_model, tracked_carrier_hz);
        let expected = signal_model.code_rate_hz
            + (875.0 * signal_model.code_rate_hz / signal_model.signal_spec.carrier_hz.value());

        assert!((aided_code_rate_hz - expected).abs() <= 1.0e-9);
    }

    #[test]
    fn carrier_aiding_reference_rejects_cross_band_center() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
                - signal_spec_gps_l5_i().carrier_hz.value(),
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let error = super::carrier_aiding_reference(
            &config,
            &signal_model,
            GPS_L1_CA_CARRIER_HZ.value() + 875.0,
        )
        .expect_err("GPS L1 carrier must not aid a GPS L5 code loop");

        assert_eq!(error, "carrier_aiding_incompatible_signal_center");
    }

    #[test]
    fn next_code_rate_reference_hz_holds_previous_value_until_carrier_lock_is_ready() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );
        let previous_reference_hz = signal_model.code_rate_hz;
        let tracked_carrier_hz =
            super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
                + 875.0;

        assert_eq!(
            super::next_code_rate_reference_hz(
                &config,
                &signal_model,
                tracked_carrier_hz,
                previous_reference_hz,
                false,
            ),
            previous_reference_hz,
        );
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_gps_l2c() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 511_500.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 12 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L2,
            SignalCode::L2C,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::None
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_gps_l5i() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn tracking_signal_model_resolves_joint_components_for_galileo_e5b() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.pilot_component.is_some());
        assert!(signal_model.data_symbol_component.is_some());
        assert_eq!(
            signal_model.carrier_phase_transition_source(),
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
    }

    #[test]
    fn select_carrier_prompt_prefers_pilot_when_it_is_strong_enough() {
        let (selected, source) = super::select_carrier_prompt(
            Complex::new(0.25, 0.05),
            Some(Complex::new(0.60, -0.10)),
            super::TrackingAidingMode::PilotCarrier,
            false,
        );

        assert_eq!(selected, Complex::new(0.60, -0.10));
        assert_eq!(source, super::CarrierPromptSource::Pilot);
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_reads_data_prompt_polarity_for_joint_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.75, 0.05)),
                Complex::new(0.80, 0.02),
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.75, 0.05)),
                Complex::new(0.80, 0.02),
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn select_carrier_prompt_keeps_dedicated_galileo_pilot_even_when_primary_is_stronger() {
        let (selected, source) = super::select_carrier_prompt(
            Complex::new(0.90, 0.05),
            Some(Complex::new(0.20, 0.70)),
            super::TrackingAidingMode::PilotCarrier,
            true,
        );

        assert_eq!(selected, Complex::new(0.20, 0.70));
        assert_eq!(source, super::CarrierPromptSource::Pilot);
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_compensates_half_cycle_pilot_flips() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.72, -0.08)),
                Complex::new(-0.83, 0.04),
                super::CarrierPromptSource::Primary,
                0.5,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.72, 0.08)),
                Complex::new(-0.83, 0.04),
                super::CarrierPromptSource::Primary,
                0.5,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_projects_onto_carrier_reference() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        let carrier_reference = Complex::new(0.50, 0.50);
        let positive_data = Complex::new(0.62, 0.58);
        let negative_data = -positive_data;

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(positive_data),
                carrier_reference,
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(negative_data),
                carrier_reference,
                super::CarrierPromptSource::Primary,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn recover_epoch_navigation_bit_sign_rotates_galileo_pilot_axis_into_data_axis() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 10_230_000.0,
            code_length: 10_230,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E5,
            SignalCode::E5b,
            None,
        );

        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(0.70, 0.02)),
                Complex::new(0.01, 0.80),
                super::CarrierPromptSource::Pilot,
                0.0,
                true,
            ),
            Some(1)
        );
        assert_eq!(
            super::recover_epoch_navigation_bit_sign(
                &signal_model,
                Some(Complex::new(-0.70, -0.02)),
                Complex::new(0.01, 0.80),
                super::CarrierPromptSource::Pilot,
                0.0,
                true,
            ),
            Some(-1)
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_galileo_e1b() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.secondary_code.is_none());
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::CbocEarlyPromptLate
        );
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::DataSymbol
        );
    }

    #[test]
    fn galileo_e1_subcarrier_guard_accepts_main_lobe_prompt() {
        let config = galileo_e1_subcarrier_guard_config();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let code_phase_chips = 321.375;
        let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.0);

        assert!(!guard.detected(), "guard={guard:?}");
        assert!(
            guard.prompt_relative_power >= super::SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER,
            "guard={guard:?}",
        );
    }

    #[test]
    fn galileo_e1_subcarrier_guard_rejects_half_chip_side_lobe_prompt() {
        let config = galileo_e1_subcarrier_guard_config();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let code_phase_chips = 321.375;
        let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.5);

        assert!(guard.detected(), "guard={guard:?}");
        assert!(
            guard.strongest_alternate_power > guard.prompt_power,
            "guard should find stronger half-chip evidence than the side-lobe prompt: {guard:?}",
        );
    }

    #[test]
    fn tracking_signal_model_uses_registry_metadata_for_beidou_b1i() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            code_freq_basis_hz: 2_046_000.0,
            code_length: 2046,
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };

        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        assert_eq!(signal_model.component_role, SignalComponentRole::Data);
        assert!(signal_model.secondary_code.is_some());
        assert_eq!(
            signal_model.discriminator_family,
            super::TrackingDiscriminatorFamily::EarlyPromptLate
        );
        assert_eq!(
            signal_model.phase_transition_source,
            super::TrackingPhaseTransitionSource::SecondaryCode
        );
        assert!((signal_model.nominal_carrier_hz() - 1_561_098_000.0).abs() <= f64::EPSILON);
        assert!(!signal_model.supports_navigation_bit_sign_recovery());
    }

    #[test]
    fn signal_tracking_params_use_narrow_spacing_for_gps_l1_ca_defaults() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.25);
    }

    #[test]
    fn code_discriminator_mode_uses_double_delta_for_bpsk_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate
        );
    }

    #[test]
    fn code_discriminator_mode_keeps_subcarrier_tracking_unambiguous() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::EarlyPromptLate
        );
    }

    #[test]
    fn code_discriminator_mode_keeps_secondary_code_tracking_unambiguous() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        assert_eq!(
            super::code_discriminator_mode(&signal_model),
            super::CodeDiscriminatorMode::EarlyPromptLate
        );
    }

    #[test]
    fn signal_tracking_params_keep_configured_spacing_for_secondary_code_tracking() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::B1,
            SignalCode::B1I,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
    }

    #[test]
    fn signal_tracking_params_use_tighter_spacing_for_high_rate_signals() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Gps, prn: 18 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L5,
            SignalCode::L5I,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.10);
    }

    #[test]
    fn signal_tracking_params_preserve_per_band_spacing_overrides() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig {
            tracking_per_band: vec![BandTrackingSpec {
                band: SignalBand::L1,
                early_late_spacing_chips: 0.5,
                dll_bw_hz: 2.0,
                pll_bw_hz: 15.0,
                fll_bw_hz: 10.0,
                integration_ms: 1,
            }],
            ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
        };
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::L1,
            SignalCode::Ca,
            None,
        );

        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
    }

    #[test]
    fn tracking_assumptions_follow_signal_metadata() {
        let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            sat,
            SignalBand::E1,
            SignalCode::E1B,
            None,
        );
        let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

        let assumptions = super::tracking_assumptions(&signal_model, tracking_params);

        assert_eq!(assumptions.discriminator_family, "unambiguous_cboc_early_prompt_late");
        assert_eq!(assumptions.early_late_spacing_chips, 0.25);
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
