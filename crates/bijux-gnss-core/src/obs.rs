#![allow(missing_docs)]
#![allow(dead_code)]

use crate::api::SignalCode;
use crate::api::{
    Chips, Constellation, Cycles, Epoch, GlonassFrequencyChannel, Hertz, Meters,
    ReceiverSampleTrace, SampleTime, SatId, Seconds, SigId, SignalBand, SignalSpec,
};
use num_complex::Complex;
use serde::{Deserialize, Serialize};

fn default_start_sample() -> usize {
    0
}

fn default_phase_step_samples() -> usize {
    1
}

fn default_phase_search_mode() -> String {
    "full_code".to_string()
}

fn default_candidate_rank() -> u8 {
    1
}

fn default_is_primary_candidate() -> bool {
    true
}

fn default_signal_band() -> SignalBand {
    SignalBand::L1
}

fn default_cycles_zero() -> Cycles {
    Cycles(0.0)
}

pub const TRACKING_STATE_MODEL_VERSION: u32 = 1;
pub const OBSERVATION_MODEL_VERSION: u32 = 3;
pub const OBSERVATION_DOWNSTREAM_PROFILE_VERSION: u32 = 1;
pub const NAV_SOLUTION_MODEL_VERSION: u32 = 4;
pub const NAV_OUTPUT_STABILITY_SIGNATURE_VERSION: u32 = 2;
pub const OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET: &str =
    "tracked_carrier_hz_minus_intermediate_freq";

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum TrackingLifecycleState {
    Init,
    PullIn,
    Lock,
    Degraded,
    Lost,
    Inactive,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackingAssumptions {
    pub integration_ms: u32,
    #[serde(default)]
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub discriminator_family: String,
    pub aiding_mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TrackingUncertainty {
    pub code_phase_samples: f64,
    pub carrier_phase_cycles: f64,
    pub doppler_hz: f64,
    pub cn0_dbhz: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum SupportStatus {
    Supported,
    Unsupported,
    Planned,
    Deprecated,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct SignalStageSupport {
    pub acquisition: SupportStatus,
    pub tracking: SupportStatus,
    pub data_decoding: SupportStatus,
    pub observations: SupportStatus,
    pub positioning: SupportStatus,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalSupportRow {
    pub constellation: Constellation,
    pub band: SignalBand,
    pub code: SignalCode,
    pub stage_support: SignalStageSupport,
    pub requirements: Vec<String>,
    pub status: SupportStatus,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupportMatrix {
    pub schema_version: u32,
    pub rows: Vec<SignalSupportRow>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqThresholdProvenance {
    pub coherent_ms: u32,
    pub noncoherent: u32,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    pub peak_mean_threshold: f32,
    pub peak_second_threshold: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqExplainCandidate {
    pub rank: u8,
    pub code_phase_samples: usize,
    pub carrier_hz: f64,
    pub peak: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    pub second_peak_ratio: f32,
    pub mean: f32,
    pub hypothesis: AcqHypothesis,
    pub score: f32,
    pub threshold_hit: bool,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqExplain {
    pub sat: SatId,
    pub selected_rank: Option<u8>,
    pub selected_reason: String,
    pub candidate_count: usize,
    pub candidates: Vec<AcqExplainCandidate>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackTransition {
    pub sat: SatId,
    pub channel_id: u8,
    pub epoch_idx: u64,
    pub sample_index: u64,
    pub from_state: String,
    pub to_state: String,
    pub reason: String,
    pub lock_quality: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpochManifest {
    pub version: u32,
    pub artifact_id: String,
    pub epoch_id: String,
    pub source_epoch_idx: u64,
    pub source_sample_index: u64,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub decision: ObservationEpochDecision,
    #[serde(default = "default_observation_downstream_profile_version")]
    pub downstream_profile_version: u32,
}

fn default_observation_downstream_profile_version() -> u32 {
    OBSERVATION_DOWNSTREAM_PROFILE_VERSION
}

fn default_observation_lock_state() -> String {
    "inactive".to_string()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatObservationDecision {
    pub sat: SatId,
    pub status: ObservationStatus,
    pub reasons: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsDecisionArtifact {
    pub artifact_id: String,
    pub epoch_idx: u64,
    pub decision: ObservationEpochDecision,
    pub reasons: Vec<String>,
    pub accepted_sats: Vec<SatObservationDecision>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationStatus {
    Accepted,
    Missing,
    Weak,
    Inconsistent,
    Rejected,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationSupportClass {
    Supported,
    Degraded,
    Unsupported,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationUncertaintyClass {
    Low,
    Medium,
    High,
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum NavUncertaintyClass {
    Low,
    Medium,
    High,
    #[default]
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum NavRefusalClass {
    InsufficientGeometry,
    InvalidSatelliteTime,
    InvalidEphemeris,
    InconsistentObservations,
    SolverFailure,
    UnsupportedConstellation,
    MixedConstellationInput,
    PartialDecodedNavigationState,
    ScientificPrerequisitesTooWeak,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavAssumptions {
    #[serde(default = "default_nav_time_system")]
    pub time_system: String,
    #[serde(default = "default_nav_reference_frame")]
    pub reference_frame: String,
    #[serde(default = "default_nav_clock_model")]
    pub clock_model: String,
    pub ephemeris_source: String,
    pub frame_decode_mode: String,
    #[serde(default = "default_nav_ephemeris_completeness")]
    pub ephemeris_completeness: String,
    pub ephemeris_count: usize,
}

fn default_nav_time_system() -> String {
    "gps".to_string()
}

fn default_nav_reference_frame() -> String {
    "ecef_wgs84".to_string()
}

fn default_nav_clock_model() -> String {
    "receiver_clock_bias_drift_linear".to_string()
}

fn default_nav_ephemeris_completeness() -> String {
    "unknown".to_string()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavProvenance {
    pub solver_family: String,
    pub weighting_mode: String,
    pub robust_solver: bool,
    pub raim_enabled: bool,
    pub satellites_used: Vec<SatId>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum ObservationEpochDecision {
    Accepted,
    Rejected,
}

impl Default for ObservationStatus {
    fn default() -> Self {
        Self::Accepted
    }
}

impl Default for ObservationEpochDecision {
    fn default() -> Self {
        Self::Accepted
    }
}

pub type Sample = Complex<f32>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplesFrame {
    pub t0: SampleTime,
    pub dt_s: Seconds,
    pub iq: Vec<Sample>,
}

impl SamplesFrame {
    pub fn new(t0: SampleTime, dt_s: Seconds, iq: Vec<Sample>) -> Self {
        Self { t0, dt_s, iq }
    }

    pub fn len(&self) -> usize {
        self.iq.len()
    }

    pub fn is_empty(&self) -> bool {
        self.iq.is_empty()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct AcqRequest {
    pub sat: SatId,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    pub coherent_ms: u32,
    pub noncoherent: u32,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AcqHypothesis {
    Accepted,
    Ambiguous,
    Rejected,
    Deferred,
}

impl Default for AcqHypothesis {
    fn default() -> Self {
        Self::Deferred
    }
}

impl std::fmt::Display for AcqHypothesis {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let value = match self {
            Self::Accepted => "accepted",
            Self::Ambiguous => "ambiguous",
            Self::Rejected => "rejected",
            Self::Deferred => "deferred",
        };
        write!(f, "{value}")
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqAssumptions {
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    pub coherent_ms: u32,
    pub noncoherent: u32,
    pub samples_per_code: usize,
    pub frame_samples: usize,
    #[serde(default = "default_start_sample")]
    pub code_phase_search_start_sample: usize,
    #[serde(default = "default_phase_step_samples")]
    pub code_phase_search_step_samples: usize,
    #[serde(default)]
    pub code_phase_search_bins: usize,
    #[serde(default = "default_phase_search_mode")]
    pub code_phase_search_mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqEvidence {
    pub rank: u8,
    pub code_phase_samples: usize,
    pub doppler_hz: f64,
    pub peak: f32,
    pub second_peak: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    pub mean: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqDopplerRefinement {
    pub method: String,
    pub coarse_carrier_hz: Hertz,
    pub offset_hz: f64,
    pub offset_bins: f64,
    pub left_peak_mean_ratio: f32,
    pub center_peak_mean_ratio: f32,
    pub right_peak_mean_ratio: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqCodePhaseRefinement {
    pub method: String,
    pub offset_samples: f64,
    pub refined_code_phase_samples: f64,
    pub left_correlation_norm: f32,
    pub center_correlation_norm: f32,
    pub right_correlation_norm: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqUncertainty {
    pub doppler_hz: f64,
    pub code_phase_samples: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SignalDelayAlignment {
    pub whole_code_periods: u64,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqTrackingSeed {
    pub sat: SatId,
    pub signal_band: SignalBand,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub source_time: ReceiverSampleTrace,
    pub doppler_hz: Hertz,
    pub code_phase_samples: Chips,
    #[serde(default)]
    pub signal_delay_alignment: Option<SignalDelayAlignment>,
    pub uncertainty: Option<AcqUncertainty>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqResult {
    pub sat: SatId,
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    #[serde(default = "default_candidate_rank")]
    pub candidate_rank: u8,
    #[serde(default = "default_is_primary_candidate")]
    pub is_primary_candidate: bool,
    pub doppler_hz: Hertz,
    pub carrier_hz: Hertz,
    pub code_phase_samples: usize,
    pub peak: f32,
    pub second_peak: f32,
    pub mean: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    pub cn0_proxy: f32,
    #[serde(default)]
    pub score: f32,
    #[serde(default)]
    pub hypothesis: AcqHypothesis,
    #[serde(default)]
    pub assumptions: Option<AcqAssumptions>,
    #[serde(default)]
    pub evidence: Vec<AcqEvidence>,
    #[serde(default)]
    pub threshold_provenance: Option<AcqThresholdProvenance>,
    #[serde(default)]
    pub explain_selection_reason: Option<String>,
    #[serde(default)]
    pub doppler_refinement: Option<AcqDopplerRefinement>,
    #[serde(default)]
    pub code_phase_refinement: Option<AcqCodePhaseRefinement>,
    #[serde(default)]
    pub signal_delay_alignment: Option<SignalDelayAlignment>,
    #[serde(default)]
    pub uncertainty: Option<AcqUncertainty>,
}

impl AcqResult {
    pub fn resolved_code_phase_samples(&self) -> f64 {
        self.code_phase_refinement
            .as_ref()
            .map(|refinement| refinement.refined_code_phase_samples)
            .unwrap_or(self.code_phase_samples as f64)
    }

    pub fn tracking_seed(&self) -> AcqTrackingSeed {
        AcqTrackingSeed {
            sat: self.sat,
            signal_band: self.signal_band,
            glonass_frequency_channel: self.glonass_frequency_channel,
            source_time: self.source_time,
            doppler_hz: self.doppler_hz,
            code_phase_samples: Chips(self.resolved_code_phase_samples()),
            signal_delay_alignment: self.signal_delay_alignment.clone(),
            uncertainty: self.uncertainty.clone(),
        }
    }
}

pub fn trackable_acq_tracking_seeds(results: &[AcqResult]) -> Vec<AcqTrackingSeed> {
    results
        .iter()
        .filter(|result| {
            matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous)
        })
        .map(AcqResult::tracking_seed)
        .collect()
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct AcqSearchSummary {
    pub searched_satellites: usize,
    pub accepted: usize,
    pub ambiguous: usize,
    pub rejected: usize,
    pub deferred: usize,
}

impl AcqSearchSummary {
    pub fn from_results(results: &[AcqResult]) -> Self {
        let mut summary = Self { searched_satellites: results.len(), ..Self::default() };
        for result in results {
            match result.hypothesis {
                AcqHypothesis::Accepted => summary.accepted += 1,
                AcqHypothesis::Ambiguous => summary.ambiguous += 1,
                AcqHypothesis::Rejected => summary.rejected += 1,
                AcqHypothesis::Deferred => summary.deferred += 1,
            }
        }
        summary
    }
}

pub fn acq_result_stability_key(result: &AcqResult) -> String {
    let doppler_refinement_key = result
        .doppler_refinement
        .as_ref()
        .map(|refinement| {
            format!(
                "|{}|{:.3}|{:.6}",
                refinement.method, refinement.coarse_carrier_hz.0, refinement.offset_bins
            )
        })
        .unwrap_or_default();
    let code_phase_refinement_key = result
        .code_phase_refinement
        .as_ref()
        .map(|refinement| {
            format!(
                "|{}|{:.6}|{:.6}",
                refinement.method, refinement.offset_samples, refinement.refined_code_phase_samples
            )
        })
        .unwrap_or_default();
    let uncertainty_key = result
        .uncertainty
        .as_ref()
        .map(|uncertainty| {
            format!("|{:.6}|{:.6}", uncertainty.doppler_hz, uncertainty.code_phase_samples)
        })
        .unwrap_or_default();
    let signal_delay_alignment_key = result
        .signal_delay_alignment
        .as_ref()
        .map(|alignment| format!("|{}|{}", alignment.whole_code_periods, alignment.source))
        .unwrap_or_default();
    let glonass_frequency_channel_key = result
        .glonass_frequency_channel
        .map(|channel| format!("|{}", channel.value()))
        .unwrap_or_default();
    format!(
        "{:?}-{:02}|{}|{}{}|{:.3}|{}|{:.6}|{:.6}|{:.6}|{}{}{}{}{}",
        result.sat.constellation,
        result.sat.prn,
        result.candidate_rank,
        result.is_primary_candidate,
        glonass_frequency_channel_key,
        result.carrier_hz.0,
        result.code_phase_samples,
        result.peak_mean_ratio,
        result.peak_second_ratio,
        result.score,
        result.hypothesis,
        doppler_refinement_key,
        code_phase_refinement_key,
        uncertainty_key,
        signal_delay_alignment_key,
    )
}

pub fn stable_acq_result_keys(results: &[AcqResult]) -> Vec<String> {
    let mut keys = results.iter().map(acq_result_stability_key).collect::<Vec<_>>();
    keys.sort();
    keys
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackEpoch {
    pub epoch: Epoch,
    pub sample_index: u64,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub sat: SatId,
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub prompt_i: f32,
    pub prompt_q: f32,
    #[serde(default)]
    pub early_i: f32,
    #[serde(default)]
    pub early_q: f32,
    #[serde(default)]
    pub late_i: f32,
    #[serde(default)]
    pub late_q: f32,
    pub carrier_hz: Hertz,
    #[serde(default = "default_cycles_zero")]
    pub carrier_phase_cycles: Cycles,
    pub code_rate_hz: Hertz,
    pub code_phase_samples: Chips,
    pub lock: bool,
    pub cn0_dbhz: f64,
    pub pll_lock: bool,
    pub dll_lock: bool,
    pub fll_lock: bool,
    pub cycle_slip: bool,
    pub nav_bit_lock: bool,
    #[serde(default)]
    pub navigation_bit_sign: Option<i8>,
    pub dll_err: f32,
    pub pll_err: f32,
    pub fll_err: f32,
    #[serde(default)]
    pub anti_false_lock: bool,
    #[serde(default)]
    pub cycle_slip_reason: Option<String>,
    #[serde(default)]
    pub lock_state: String,
    #[serde(default)]
    pub lock_state_reason: Option<String>,
    #[serde(default)]
    pub channel_id: Option<u8>,
    #[serde(default)]
    pub channel_uid: String,
    #[serde(default)]
    pub tracking_provenance: String,
    #[serde(default)]
    pub tracking_assumptions: Option<TrackingAssumptions>,
    #[serde(default)]
    pub signal_delay_alignment: Option<SignalDelayAlignment>,
    #[serde(default)]
    pub tracking_uncertainty: Option<TrackingUncertainty>,
    #[serde(default)]
    pub processing_ms: Option<f64>,
}

impl Default for TrackEpoch {
    fn default() -> Self {
        Self {
            epoch: Epoch { index: 0 },
            sample_index: 0,
            source_time: ReceiverSampleTrace::default(),
            sat: SatId { constellation: Constellation::Unknown, prn: 0 },
            signal_band: SignalBand::L1,
            glonass_frequency_channel: None,
            prompt_i: 0.0,
            prompt_q: 0.0,
            early_i: 0.0,
            early_q: 0.0,
            late_i: 0.0,
            late_q: 0.0,
            carrier_hz: Hertz(0.0),
            carrier_phase_cycles: Cycles(0.0),
            code_rate_hz: Hertz(0.0),
            code_phase_samples: Chips(0.0),
            lock: false,
            cn0_dbhz: 0.0,
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
            lock_state: "inactive".to_string(),
            lock_state_reason: None,
            channel_id: None,
            channel_uid: String::new(),
            tracking_provenance: String::new(),
            tracking_assumptions: None,
            signal_delay_alignment: None,
            tracking_uncertainty: None,
            processing_ms: None,
        }
    }
}

impl TrackEpoch {
    pub fn lifecycle_state(&self) -> TrackingLifecycleState {
        match self.lock_state.as_str() {
            "tracking" => TrackingLifecycleState::Lock,
            "acquired" => TrackingLifecycleState::Init,
            "pull_in" => TrackingLifecycleState::PullIn,
            "lost" => TrackingLifecycleState::Lost,
            "degraded" => TrackingLifecycleState::Degraded,
            _ => TrackingLifecycleState::Inactive,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LockFlags {
    pub code_lock: bool,
    pub carrier_lock: bool,
    pub bit_lock: bool,
    pub cycle_slip: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsMetadata {
    pub tracking_mode: String,
    pub integration_ms: u32,
    pub lock_quality: f64,
    pub smoothing_window: u32,
    pub smoothing_age: u32,
    pub smoothing_resets: u32,
    pub signal: SignalSpec,
    #[serde(default)]
    pub acquisition_hypothesis: String,
    #[serde(default)]
    pub acquisition_score: f32,
    #[serde(default)]
    pub acquisition_code_phase_samples: usize,
    #[serde(default)]
    pub acquisition_carrier_hz: f64,
    #[serde(default)]
    pub acq_to_track_state: String,
    #[serde(default)]
    pub tracking_state: String,
    #[serde(default)]
    pub tracking_lock_state: String,
    #[serde(default = "default_observation_lock_state")]
    pub observation_lock_state: String,
    #[serde(default)]
    pub observation_lock_reason: Option<String>,
    #[serde(default)]
    pub tracking_lock_quality: f64,
    #[serde(default)]
    pub observation_status: String,
    #[serde(default)]
    pub observation_reject_reasons: Vec<String>,
    #[serde(default)]
    pub observation_epoch_id: String,
    #[serde(default)]
    pub observation_support_class: String,
    #[serde(default)]
    pub observation_uncertainty_class: String,
    #[serde(default)]
    pub pseudorange_model: String,
    #[serde(default)]
    pub carrier_phase_model: String,
    #[serde(default)]
    pub doppler_model: String,
    #[serde(default)]
    pub carrier_phase_continuity: String,
    #[serde(default)]
    pub carrier_phase_arc_start_epoch_idx: u64,
    #[serde(default)]
    pub carrier_phase_arc_start_sample_index: u64,
    #[serde(default)]
    pub signal_delay_alignment_source: String,
    #[serde(default)]
    pub time_tag_source: String,
    #[serde(default)]
    pub time_tag_sample_index: u64,
    #[serde(default)]
    pub time_tag_sample_rate_hz: f64,
    #[serde(default)]
    pub tracking_uncertainty: Option<TrackingUncertainty>,
}

impl Default for ObsMetadata {
    fn default() -> Self {
        Self {
            tracking_mode: "scalar".to_string(),
            integration_ms: 0,
            lock_quality: 0.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: SignalSpec {
                constellation: Constellation::Unknown,
                band: SignalBand::Unknown,
                code: crate::api::SignalCode::Unknown,
                code_rate_hz: 0.0,
                carrier_hz: crate::api::GPS_L1_CA_CARRIER_HZ,
            },
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: String::new(),
            tracking_state: String::new(),
            tracking_lock_state: String::new(),
            observation_lock_state: default_observation_lock_state(),
            observation_lock_reason: None,
            tracking_lock_quality: 0.0,
            observation_status: "accepted".to_string(),
            observation_reject_reasons: Vec::new(),
            observation_epoch_id: String::new(),
            observation_support_class: "supported".to_string(),
            observation_uncertainty_class: "unknown".to_string(),
            pseudorange_model: "receiver_epoch_fallback".to_string(),
            carrier_phase_model: "tracked_carrier_cycles".to_string(),
            doppler_model: OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET.to_string(),
            carrier_phase_continuity: "unusable".to_string(),
            carrier_phase_arc_start_epoch_idx: 0,
            carrier_phase_arc_start_sample_index: 0,
            signal_delay_alignment_source: String::new(),
            time_tag_source: String::new(),
            time_tag_sample_index: 0,
            time_tag_sample_rate_hz: 0.0,
            tracking_uncertainty: None,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasurementErrorModel {
    pub thermal_noise_m: Meters,
    pub tracking_jitter_m: Meters,
    pub multipath_proxy_m: Meters,
    pub clock_error_m: Meters,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ObsSignalTiming {
    pub signal_travel_time_s: Seconds,
    pub transmit_gps_time: crate::api::GpsTime,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsSatellite {
    pub signal_id: SigId,
    pub pseudorange_m: Meters,
    pub pseudorange_var_m2: f64,
    pub carrier_phase_cycles: Cycles,
    pub carrier_phase_var_cycles2: f64,
    pub doppler_hz: Hertz,
    pub doppler_var_hz2: f64,
    pub cn0_dbhz: f64,
    pub lock_flags: LockFlags,
    pub multipath_suspect: bool,
    #[serde(default)]
    pub observation_status: ObservationStatus,
    #[serde(default)]
    pub observation_reject_reasons: Vec<String>,
    pub elevation_deg: Option<f64>,
    pub azimuth_deg: Option<f64>,
    pub weight: Option<f64>,
    #[serde(default)]
    pub timing: Option<ObsSignalTiming>,
    pub error_model: Option<MeasurementErrorModel>,
    pub metadata: ObsMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsEpoch {
    pub t_rx_s: Seconds,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub gps_week: Option<u32>,
    pub tow_s: Option<Seconds>,
    pub epoch_idx: u64,
    pub discontinuity: bool,
    #[serde(default)]
    pub valid: bool,
    #[serde(default)]
    pub processing_ms: Option<f64>,
    pub role: ReceiverRole,
    pub sats: Vec<ObsSatellite>,
    #[serde(default)]
    pub decision: ObservationEpochDecision,
    #[serde(default)]
    pub decision_reason: Option<String>,
    #[serde(default)]
    pub manifest: Option<ObsEpochManifest>,
}

pub fn obs_epoch_stability_key(epoch: &ObsEpoch) -> String {
    let mut sat_keys = epoch
        .sats
        .iter()
        .map(|sat| {
            let timing_key = sat
                .timing
                .map(|timing| {
                    format!(
                        ":{:.12}:{}:{:.12}",
                        timing.signal_travel_time_s.0,
                        timing.transmit_gps_time.week,
                        timing.transmit_gps_time.tow_s
                    )
                })
                .unwrap_or_default();
            format!(
                "{:?}-{:02}:{:?}:{:?}:{:.3}:{:.6}:{:.3}:{}:{:.6}:{}:{}:{}{}",
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.signal_id.band,
                sat.signal_id.code,
                sat.pseudorange_m.0,
                sat.carrier_phase_cycles.0,
                sat.doppler_hz.0,
                sat.metadata.doppler_model,
                sat.cn0_dbhz,
                sat.metadata.carrier_phase_continuity,
                sat.metadata.carrier_phase_arc_start_epoch_idx,
                sat.metadata.carrier_phase_arc_start_sample_index,
                timing_key
            )
        })
        .collect::<Vec<_>>();
    sat_keys.sort();
    format!(
        "epoch:{}|sample:{}|t:{:.9}|decision:{:?}|sats:{}",
        epoch.epoch_idx,
        epoch.source_time.sample_index,
        epoch.t_rx_s.0,
        epoch.decision,
        sat_keys.join(";")
    )
}

impl ObsEpoch {
    pub fn gps_time(&self) -> Option<crate::api::GpsTime> {
        Some(crate::api::GpsTime { week: self.gps_week?, tow_s: self.tow_s?.0 })
    }

    /// Validate basic physics sanity checks for this epoch.
    pub fn validate_physics(&self) -> Vec<crate::api::DiagnosticEvent> {
        let mut events = crate::api::check_obs_epoch_sanity(self);
        for sat in &self.sats {
            if !sat.pseudorange_m.0.is_finite()
                || !sat.carrier_phase_cycles.0.is_finite()
                || !sat.doppler_hz.0.is_finite()
            {
                events.push(crate::api::DiagnosticEvent::new(
                    crate::api::DiagnosticSeverity::Error,
                    "GNSS_OBS_NUMERIC_INVALID",
                    "obs contains NaN/Inf",
                ));
            }
        }
        events
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum ReceiverRole {
    Base,
    Rover,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct ObsStreamTag {
    pub role: ReceiverRole,
    pub source_id: String,
    pub sample_rate_hz: Hertz,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolutionStatus {
    Unavailable,
    Refused,
    Degraded,
    IntegrityFailed,
    Diverged,
    CodeOnly,
    Float,
    Fixed,
}

impl SolutionStatus {
    pub fn is_valid(self) -> bool {
        !matches!(
            self,
            SolutionStatus::Unavailable
                | SolutionStatus::Refused
                | SolutionStatus::IntegrityFailed
                | SolutionStatus::Diverged
        )
    }

    pub fn lifecycle_state(self) -> NavLifecycleState {
        match self {
            SolutionStatus::Unavailable => NavLifecycleState::Unavailable,
            SolutionStatus::Refused => NavLifecycleState::Refused,
            SolutionStatus::Degraded => NavLifecycleState::Degraded,
            SolutionStatus::IntegrityFailed => NavLifecycleState::IntegrityFailed,
            SolutionStatus::Diverged => NavLifecycleState::Diverged,
            SolutionStatus::CodeOnly => NavLifecycleState::CodeOnly,
            SolutionStatus::Float => NavLifecycleState::Float,
            SolutionStatus::Fixed => NavLifecycleState::Fixed,
        }
    }

    pub fn decision_label(self) -> &'static str {
        match self {
            SolutionStatus::Unavailable => "unavailable",
            SolutionStatus::Refused => "refused",
            SolutionStatus::Degraded => "degraded",
            SolutionStatus::IntegrityFailed => "integrity_failed",
            SolutionStatus::Diverged => "diverged",
            SolutionStatus::CodeOnly | SolutionStatus::Float | SolutionStatus::Fixed => "accepted",
        }
    }

    pub fn quality_flag(self) -> NavQualityFlag {
        match self {
            SolutionStatus::Unavailable | SolutionStatus::Refused => {
                NavQualityFlag::NoFix
            }
            SolutionStatus::Degraded
            | SolutionStatus::IntegrityFailed
            | SolutionStatus::Diverged => NavQualityFlag::Degraded,
            SolutionStatus::CodeOnly | SolutionStatus::Float => {
                NavQualityFlag::Float
            }
            SolutionStatus::Fixed => NavQualityFlag::Fix,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavLifecycleState {
    #[default]
    Unavailable,
    Refused,
    Degraded,
    IntegrityFailed,
    Diverged,
    CodeOnly,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavQualityFlag {
    #[default]
    NoFix,
    Float,
    Fix,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SolutionValidity {
    #[default]
    Invalid,
    Coarse,
    Converging,
    Stable,
    Diverging,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasurementRejectReason {
    Outlier,
    Geometry,
    CycleSlip,
    InvalidEphemeris,
    TimeInconsistency,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NavHealthEvent {
    CovarianceSymmetrized,
    CovarianceClamped {
        min_eigenvalue: f64,
    },
    CovarianceDiverged {
        max_variance: f64,
    },
    InnovationRejected {
        reason: String,
    },
    InnovationConsistencyAnomaly {
        normalized_innovation_squared: f64,
        lower_bound: f64,
        upper_bound: f64,
        measurement_dimension: usize,
    },
    CommonCodeDopplerAnomaly {
        common_code_step_m: f64,
        common_doppler_step_hz: f64,
        matched_satellite_count: usize,
        aligned_satellite_count: usize,
        code_step_threshold_m: f64,
        doppler_step_threshold_hz: f64,
    },
    ReplayTimingAnomaly {
        common_delay_step_m: f64,
        centered_delay_rms_m: f64,
        max_centered_delay_m: f64,
        matched_satellite_count: usize,
        positive_step_satellite_count: usize,
        common_delay_step_threshold_m: f64,
        centered_delay_rms_threshold_m: f64,
    },
    ConstellationClockInconsistency {
        constellation: Constellation,
        previous_bias_s: f64,
        current_bias_s: f64,
        bias_step_m: f64,
        bias_step_threshold_m: f64,
        supporting_satellite_count: usize,
    },
    ResidualTemporalCorrelation {
        lag1_correlation: f64,
        correlation_threshold: f64,
        matched_satellite_count: usize,
        previous_centered_rms_m: f64,
        current_centered_rms_m: f64,
        persistent_suspect_epochs: usize,
    },
    ImpossibleGeometry {
        receiver_radius_m: f64,
        altitude_m: f64,
        used_satellite_count: usize,
        min_receiver_radius_m: f64,
        max_receiver_radius_m: f64,
        min_altitude_m: f64,
        max_altitude_m: f64,
    },
    SatelliteClockAnomaly {
        sat: SatId,
        persistent_suspect_epochs: usize,
        max_solution_separation_m: f64,
        separation_threshold_m: f64,
    },
    ZtdClamped {
        before_m: f64,
        after_m: f64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AmbiguityId {
    pub sig: SigId,
    pub signal: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AmbiguityStatus {
    Unknown,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbiguityState {
    pub id: AmbiguityId,
    pub float_cycles: Cycles,
    pub variance: f64,
    pub status: AmbiguityStatus,
    pub last_update_epoch: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct CarrierPhaseTerms {
    pub range_m: Meters,
    pub receiver_clock_s: Seconds,
    pub sat_clock_s: Seconds,
    pub tropo_m: Meters,
    pub iono_m: Meters,
    pub wavelength_m: Meters,
    pub ambiguity_cycles: Cycles,
}

pub(crate) fn carrier_phase_cycles(terms: &CarrierPhaseTerms) -> f64 {
    let corrected_m = terms.range_m.0
        + 299_792_458.0 * (terms.receiver_clock_s.0 - terms.sat_clock_s.0)
        + terms.tropo_m.0
        - terms.iono_m.0;
    corrected_m / terms.wavelength_m.0 + terms.ambiguity_cycles.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDifference {
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoubleDifference {
    pub ref_sig: SigId,
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub canceled: Vec<AmbiguityId>,
}

pub(crate) fn geometry_free_phase_m(
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    phase1_cycles * lambda1_m - phase2_cycles * lambda2_m
}

pub(crate) fn melbourne_wubbena_m(
    code1_m: f64,
    code2_m: f64,
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    let phi1 = phase1_cycles * lambda1_m;
    let phi2 = phase2_cycles * lambda2_m;
    (phi1 - phi2) - (code1_m - code2_m)
}

// Errors moved to crate::error.

#[cfg(test)]
mod tests {
    use super::{
        acq_result_stability_key, obs_epoch_stability_key, ObsEpoch, ObsMetadata, ObsSatellite,
        SignalDelayAlignment, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
    };
    use crate::api::{
        trackable_acq_tracking_seeds, AcqCodePhaseRefinement, AcqHypothesis, AcqResult,
        AcqSearchSummary, AcqUncertainty, Constellation, Cycles, GlonassFrequencyChannel, Hertz,
        LeapSeconds, LockFlags, NavLifecycleState, NavQualityFlag, ObservationEpochDecision,
        ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, SignalBand, SolutionStatus,
        UtcTime,
    };
    use crate::time::utc_to_gps;

    #[test]
    fn leap_second_table_validates() {
        let table = LeapSeconds::default_table();
        table.validate().expect("leap second table valid");
        assert_eq!(table.latest_offset(), 18);
    }

    #[test]
    fn leap_second_offset_lookup() {
        let table = LeapSeconds::default_table();
        assert_eq!(table.offset_at_utc(0.0), 0);
        assert_eq!(table.offset_at_utc(362_793_600.0), 1);
        assert_eq!(table.offset_at_utc(1_483_228_800.0), 18);
    }

    #[test]
    fn utc_to_gps_uses_leap_offset() {
        let table = LeapSeconds::default_table();
        let utc = UtcTime { unix_s: 1_700_000_000.0 };
        let gps = utc_to_gps(utc, &table);
        let offset = table.offset_at_utc(utc.unix_s);
        let expected = utc.unix_s - 315_964_800.0 + offset as f64;
        assert!((gps.to_seconds() - expected).abs() < 1e-6);
    }

    #[test]
    fn utc_to_gps_anchors_week_zero_to_gps_epoch() {
        let table = LeapSeconds::default_table();
        let gps = utc_to_gps(UtcTime { unix_s: 315_964_800.0 }, &table);

        assert_eq!(gps.week, 0);
        assert_eq!(gps.tow_s, 0.0);
    }

    #[test]
    fn precise_solution_status_validity_matches_runtime_semantics() {
        assert!(!SolutionStatus::Unavailable.is_valid());
        assert!(!SolutionStatus::Refused.is_valid());
        assert!(SolutionStatus::Degraded.is_valid());
        assert!(!SolutionStatus::IntegrityFailed.is_valid());
        assert!(!SolutionStatus::Diverged.is_valid());
        assert!(SolutionStatus::CodeOnly.is_valid());
        assert!(SolutionStatus::Float.is_valid());
        assert!(SolutionStatus::Fixed.is_valid());
    }

    #[test]
    fn precise_solution_status_quality_flags_distinguish_no_fix_and_degraded_cases() {
        assert_eq!(SolutionStatus::Unavailable.quality_flag(), NavQualityFlag::NoFix);
        assert_eq!(SolutionStatus::Refused.quality_flag(), NavQualityFlag::NoFix);
        assert_eq!(SolutionStatus::Degraded.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::IntegrityFailed.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::Diverged.quality_flag(), NavQualityFlag::Degraded);
        assert_eq!(SolutionStatus::CodeOnly.quality_flag(), NavQualityFlag::Float);
        assert_eq!(SolutionStatus::Float.quality_flag(), NavQualityFlag::Float);
        assert_eq!(SolutionStatus::Fixed.quality_flag(), NavQualityFlag::Fix);
    }

    #[test]
    fn precise_solution_status_lifecycle_state_tracks_status_exactly() {
        assert_eq!(
            SolutionStatus::Unavailable.lifecycle_state(),
            NavLifecycleState::Unavailable
        );
        assert_eq!(SolutionStatus::Refused.lifecycle_state(), NavLifecycleState::Refused);
        assert_eq!(SolutionStatus::Degraded.lifecycle_state(), NavLifecycleState::Degraded);
        assert_eq!(
            SolutionStatus::IntegrityFailed.lifecycle_state(),
            NavLifecycleState::IntegrityFailed
        );
        assert_eq!(SolutionStatus::Diverged.lifecycle_state(), NavLifecycleState::Diverged);
        assert_eq!(SolutionStatus::CodeOnly.lifecycle_state(), NavLifecycleState::CodeOnly);
        assert_eq!(SolutionStatus::Float.lifecycle_state(), NavLifecycleState::Float);
        assert_eq!(SolutionStatus::Fixed.lifecycle_state(), NavLifecycleState::Fixed);
    }

    #[test]
    fn precise_solution_status_decision_labels_match_public_status_words() {
        assert_eq!(SolutionStatus::Unavailable.decision_label(), "unavailable");
        assert_eq!(SolutionStatus::Refused.decision_label(), "refused");
        assert_eq!(SolutionStatus::Degraded.decision_label(), "degraded");
        assert_eq!(
            SolutionStatus::IntegrityFailed.decision_label(),
            "integrity_failed"
        );
        assert_eq!(SolutionStatus::Diverged.decision_label(), "diverged");
        assert_eq!(SolutionStatus::CodeOnly.decision_label(), "accepted");
        assert_eq!(SolutionStatus::Float.decision_label(), "accepted");
        assert_eq!(SolutionStatus::Fixed.decision_label(), "accepted");
    }

    #[test]
    fn acq_search_summary_counts_each_decision() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let results = vec![
            acq_result_for_summary(sat, AcqHypothesis::Accepted),
            acq_result_for_summary(sat, AcqHypothesis::Ambiguous),
            acq_result_for_summary(sat, AcqHypothesis::Rejected),
            acq_result_for_summary(sat, AcqHypothesis::Deferred),
        ];

        let summary = AcqSearchSummary::from_results(&results);

        assert_eq!(
            summary,
            AcqSearchSummary {
                searched_satellites: 4,
                accepted: 1,
                ambiguous: 1,
                rejected: 1,
                deferred: 1,
            }
        );
    }

    #[test]
    fn acq_tracking_seed_uses_explicit_tracking_start_fields() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let result = AcqResult {
            sat,
            signal_band: SignalBand::L1,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::from_sample_index(8_184, 4_092_000.0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(750.0),
            carrier_hz: Hertz(750.0),
            code_phase_samples: 100,
            peak: 0.0,
            second_peak: 0.0,
            mean: 0.0,
            peak_mean_ratio: 0.0,
            peak_second_ratio: 0.0,
            cn0_proxy: 0.0,
            score: 1.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: Some(AcqCodePhaseRefinement {
                method: "parabolic_code_peak".to_string(),
                offset_samples: 0.125,
                refined_code_phase_samples: 100.125,
                left_correlation_norm: 0.8,
                center_correlation_norm: 1.0,
                right_correlation_norm: 0.7,
            }),
            signal_delay_alignment: Some(SignalDelayAlignment {
                whole_code_periods: 68,
                source: "synthetic_truth".to_string(),
            }),
            uncertainty: Some(AcqUncertainty { doppler_hz: 125.0, code_phase_samples: 0.25 }),
        };

        let seed = result.tracking_seed();

        assert_eq!(seed.sat, sat);
        assert_eq!(seed.signal_band, SignalBand::L1);
        assert_eq!(seed.glonass_frequency_channel, None);
        assert_eq!(seed.source_time, result.source_time);
        assert_eq!(seed.doppler_hz.0, 750.0);
        assert!((seed.code_phase_samples.0 - 100.125).abs() <= f64::EPSILON);
        assert_eq!(
            seed.signal_delay_alignment,
            Some(SignalDelayAlignment {
                whole_code_periods: 68,
                source: "synthetic_truth".to_string(),
            })
        );
        assert_eq!(seed.uncertainty.as_ref().map(|u| u.doppler_hz), Some(125.0));
    }

    #[test]
    fn acq_result_stability_key_includes_signal_delay_alignment() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 68,
            source: "synthetic_truth".to_string(),
        });
        let mut changed = base.clone();
        changed.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 69,
            source: "synthetic_truth".to_string(),
        });

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn acq_tracking_seed_preserves_glonass_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let channel =
            GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid for GLONASS");
        let mut result = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        result.glonass_frequency_channel = Some(channel);

        let seed = result.tracking_seed();

        assert_eq!(seed.sat, sat);
        assert_eq!(seed.glonass_frequency_channel, Some(channel));
    }

    #[test]
    fn acq_result_stability_key_includes_glonass_frequency_channel() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
        let lower = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
        let upper = GlonassFrequencyChannel::new(5).expect("channel 5 must be valid");
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.glonass_frequency_channel = Some(lower);
        let mut changed = base.clone();
        changed.glonass_frequency_channel = Some(upper);

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn obs_metadata_defaults_carrier_phase_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.carrier_phase_model, "tracked_carrier_cycles");
        assert_eq!(metadata.carrier_phase_continuity, "unusable");
        assert_eq!(metadata.carrier_phase_arc_start_epoch_idx, 0);
        assert_eq!(metadata.carrier_phase_arc_start_sample_index, 0);
    }

    #[test]
    fn obs_metadata_defaults_doppler_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.doppler_model, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET);
    }

    #[test]
    fn obs_metadata_defaults_observation_lock_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.observation_lock_state, "inactive");
        assert_eq!(metadata.observation_lock_reason, None);
    }

    #[test]
    fn obs_epoch_stability_key_includes_carrier_phase_arc_metadata() {
        let sat_id = SatId { constellation: Constellation::Gps, prn: 4 };
        let base = ObsEpoch {
            t_rx_s: crate::api::Seconds(0.07),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: crate::api::SigId {
                    sat: sat_id,
                    band: SignalBand::L1,
                    code: crate::api::SignalCode::Ca,
                },
                pseudorange_m: crate::api::Meters(21_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(12.25),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(125.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    carrier_phase_continuity: "continuous".to_string(),
                    carrier_phase_arc_start_epoch_idx: 70,
                    carrier_phase_arc_start_sample_index: 286_440,
                    ..ObsMetadata::default()
                },
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };
        let mut changed = base.clone();
        changed.sats[0].metadata.carrier_phase_continuity = "reset_after_cycle_slip".to_string();
        changed.sats[0].metadata.carrier_phase_arc_start_epoch_idx = 73;
        changed.sats[0].metadata.carrier_phase_arc_start_sample_index = 298_716;

        assert_ne!(obs_epoch_stability_key(&base), obs_epoch_stability_key(&changed));
    }

    #[test]
    fn obs_epoch_stability_key_includes_doppler_model() {
        let sat_id = SatId { constellation: Constellation::Gps, prn: 4 };
        let base = ObsEpoch {
            t_rx_s: crate::api::Seconds(0.07),
            source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
            gps_week: None,
            tow_s: None,
            epoch_idx: 70,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![ObsSatellite {
                signal_id: crate::api::SigId {
                    sat: sat_id,
                    band: SignalBand::L1,
                    code: crate::api::SignalCode::Ca,
                },
                pseudorange_m: crate::api::Meters(21_000_000.0),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(12.25),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(125.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata::default(),
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };
        let mut changed = base.clone();
        changed.sats[0].metadata.doppler_model = "legacy_absolute_doppler_hz".to_string();

        assert_ne!(obs_epoch_stability_key(&base), obs_epoch_stability_key(&changed));
    }

    #[test]
    fn trackable_acq_tracking_seeds_keep_only_trackable_results() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let accepted = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        let ambiguous = acq_result_for_summary(sat, AcqHypothesis::Ambiguous);
        let rejected = acq_result_for_summary(sat, AcqHypothesis::Rejected);
        let deferred = acq_result_for_summary(sat, AcqHypothesis::Deferred);

        let seeds = trackable_acq_tracking_seeds(&[accepted, ambiguous, rejected, deferred]);

        assert_eq!(seeds.len(), 2);
        assert!(seeds.iter().all(|seed| seed.signal_band == SignalBand::L1));
    }

    fn acq_result_for_summary(sat: SatId, hypothesis: AcqHypothesis) -> AcqResult {
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            carrier_hz: Hertz(0.0),
            code_phase_samples: 0,
            peak: 0.0,
            second_peak: 0.0,
            mean: 0.0,
            peak_mean_ratio: 0.0,
            peak_second_ratio: 0.0,
            cn0_proxy: 0.0,
            score: 0.0,
            hypothesis,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }
    }
}
