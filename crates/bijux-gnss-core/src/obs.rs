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

fn default_signal_code() -> SignalCode {
    SignalCode::Unknown
}

fn default_doppler_center_hz() -> f64 {
    0.0
}

fn default_doppler_rate_center_hz_per_s() -> f64 {
    0.0
}

fn default_cycles_zero() -> Cycles {
    Cycles(0.0)
}

fn default_seconds_zero() -> Seconds {
    Seconds(0.0)
}

pub const TRACKING_STATE_MODEL_VERSION: u32 = 1;
pub const OBSERVATION_MODEL_VERSION: u32 = 3;
pub const OBSERVATION_DOWNSTREAM_PROFILE_VERSION: u32 = 1;
pub const NAV_SOLUTION_MODEL_VERSION: u32 = 4;
pub const NAV_OUTPUT_STABILITY_SIGNATURE_VERSION: u32 = 2;
pub const OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET: &str =
    "tracked_carrier_hz_minus_intermediate_freq";
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const CARRIER_DOPPLER_CORRELATION_COEFFICIENT: f64 = 0.5;

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
#[serde(rename_all = "snake_case")]
pub enum ObservationCovarianceStatus {
    PositiveSemidefinite,
    MissingEvidence,
    InvalidEvidence,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct ObservationMeasurementCovariance {
    pub code_phase_m2: f64,
    pub code_carrier_m2: f64,
    pub code_doppler_m_hz: f64,
    pub carrier_code_m2: f64,
    pub carrier_phase_m2: f64,
    pub carrier_doppler_m_hz: f64,
    pub doppler_code_hz_m: f64,
    pub doppler_carrier_hz_m: f64,
    pub doppler_hz2: f64,
    pub status: ObservationCovarianceStatus,
}

impl ObservationMeasurementCovariance {
    pub fn matrix(self) -> [[f64; 3]; 3] {
        [
            [self.code_phase_m2, self.code_carrier_m2, self.code_doppler_m_hz],
            [self.carrier_code_m2, self.carrier_phase_m2, self.carrier_doppler_m_hz],
            [self.doppler_code_hz_m, self.doppler_carrier_hz_m, self.doppler_hz2],
        ]
    }

    pub fn pseudorange_sigma_m(self) -> Option<f64> {
        (self.status == ObservationCovarianceStatus::PositiveSemidefinite
            && self.code_phase_m2.is_finite()
            && self.code_phase_m2 > 0.0)
            .then(|| self.code_phase_m2.sqrt())
    }
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
    #[serde(default = "default_acquisition_threshold_mode")]
    pub mode: String,
    pub coherent_ms: u32,
    pub noncoherent: u32,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    #[serde(default)]
    pub doppler_rate_search_hz_per_s: i32,
    #[serde(default)]
    pub doppler_rate_step_hz_per_s: i32,
    pub peak_mean_threshold: f32,
    pub peak_second_threshold: f32,
    #[serde(default)]
    pub false_alarm_probability: Option<f64>,
    #[serde(default)]
    pub calibration_trial_count: Option<usize>,
    #[serde(default)]
    pub calibration_confidence_level: Option<f64>,
    #[serde(default)]
    pub calibration_false_alarm_rate: Option<f64>,
    #[serde(default)]
    pub calibration_false_alarm_interval_low: Option<f64>,
    #[serde(default)]
    pub calibration_false_alarm_interval_high: Option<f64>,
}

fn default_acquisition_threshold_mode() -> String {
    "fixed_ratio".to_string()
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct AcqAssistanceBounds {
    pub expected_code_phase_samples: f64,
    pub time_uncertainty_s: f64,
    pub position_uncertainty_m: f64,
    pub oscillator_uncertainty_hz: f64,
    pub approximate_velocity_uncertainty_mps: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct AcqRequest {
    pub sat: SatId,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    #[serde(default = "default_doppler_center_hz")]
    pub doppler_center_hz: f64,
    #[serde(default = "default_doppler_rate_center_hz_per_s")]
    pub doppler_rate_center_hz_per_s: f64,
    #[serde(default)]
    pub expected_line_of_sight_doppler_hz: Option<f64>,
    #[serde(default)]
    pub assistance_bounds: Option<AcqAssistanceBounds>,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    #[serde(default)]
    pub doppler_rate_search_hz_per_s: i32,
    #[serde(default)]
    pub doppler_rate_step_hz_per_s: i32,
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
    #[serde(default = "default_doppler_center_hz")]
    pub doppler_center_hz: f64,
    #[serde(default = "default_doppler_rate_center_hz_per_s")]
    pub doppler_rate_center_hz_per_s: f64,
    #[serde(default)]
    pub expected_line_of_sight_doppler_hz: Option<f64>,
    #[serde(default)]
    pub assistance_bounds: Option<AcqAssistanceBounds>,
    pub doppler_search_hz: i32,
    pub doppler_step_hz: i32,
    #[serde(default)]
    pub doppler_rate_search_hz_per_s: i32,
    #[serde(default)]
    pub doppler_rate_step_hz_per_s: i32,
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
pub struct AcqComponentStatistic {
    pub role: crate::api::SignalComponentRole,
    pub peak: f32,
    pub second_peak: f32,
    pub mean: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    #[serde(default)]
    pub secondary_code_phase_periods: Option<u32>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum AcqComponentCombinationMode {
    SingleComponent,
    NoncoherentComponentSum,
    CoherentComponentSum,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqComponentProvenance {
    pub combination_mode: AcqComponentCombinationMode,
    pub components: Vec<AcqComponentStatistic>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqEvidence {
    pub rank: u8,
    pub code_phase_samples: usize,
    pub doppler_hz: f64,
    #[serde(default = "default_doppler_rate_center_hz_per_s")]
    pub doppler_rate_hz_per_s: f64,
    pub peak: f32,
    pub second_peak: f32,
    pub peak_mean_ratio: f32,
    pub peak_second_ratio: f32,
    pub mean: f32,
    #[serde(default)]
    pub component_provenance: Option<AcqComponentProvenance>,
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
pub struct AcqUncertaintyCovariance {
    pub doppler_variance_hz2: f64,
    pub doppler_code_phase_covariance_hz_samples: f64,
    pub code_phase_variance_samples2: f64,
    #[serde(default)]
    pub doppler_rate_variance_hz2_per_s2: Option<f64>,
    #[serde(default)]
    pub doppler_doppler_rate_covariance_hz2_per_s: Option<f64>,
    #[serde(default)]
    pub code_phase_doppler_rate_covariance_samples_hz_per_s: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqUncertainty {
    pub doppler_hz: f64,
    pub code_phase_samples: f64,
    #[serde(default)]
    pub doppler_rate_hz_per_s: Option<f64>,
    #[serde(default)]
    pub covariance: Option<AcqUncertaintyCovariance>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SignalDelayAlignment {
    pub whole_code_periods: u64,
    #[serde(default)]
    pub sample_delay_samples: u64,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TrackingTransmitTime {
    pub transmit_gps_time: crate::api::GpsTime,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcqTrackingSeed {
    pub sat: SatId,
    pub signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
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
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    #[serde(default = "default_candidate_rank")]
    pub candidate_rank: u8,
    #[serde(default = "default_is_primary_candidate")]
    pub is_primary_candidate: bool,
    pub doppler_hz: Hertz,
    #[serde(default = "default_doppler_rate_center_hz_per_s")]
    pub doppler_rate_hz_per_s: f64,
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
    pub fn component_provenance(&self) -> Option<&AcqComponentProvenance> {
        self.evidence.iter().find_map(|evidence| evidence.component_provenance.as_ref())
    }

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
            signal_code: self.signal_code,
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
            let covariance_key = uncertainty
                .covariance
                .as_ref()
                .map(|covariance| {
                    format!(
                        "|{:.6}|{:.6}|{:.6}|{}|{}|{}",
                        covariance.doppler_variance_hz2,
                        covariance.doppler_code_phase_covariance_hz_samples,
                        covariance.code_phase_variance_samples2,
                        covariance
                            .doppler_rate_variance_hz2_per_s2
                            .map_or_else(|| "none".to_string(), |value| format!("{value:.6}")),
                        covariance
                            .doppler_doppler_rate_covariance_hz2_per_s
                            .map_or_else(|| "none".to_string(), |value| format!("{value:.6}")),
                        covariance
                            .code_phase_doppler_rate_covariance_samples_hz_per_s
                            .map_or_else(|| "none".to_string(), |value| format!("{value:.6}")),
                    )
                })
                .unwrap_or_default();
            format!(
                "|{:.6}|{:.6}|{}{}",
                uncertainty.doppler_hz,
                uncertainty.code_phase_samples,
                uncertainty
                    .doppler_rate_hz_per_s
                    .map_or_else(|| "none".to_string(), |value| format!("{value:.6}")),
                covariance_key,
            )
        })
        .unwrap_or_default();
    let signal_delay_alignment_key = result
        .signal_delay_alignment
        .as_ref()
        .map(|alignment| {
            format!(
                "|{}|{}|{}",
                alignment.whole_code_periods, alignment.sample_delay_samples, alignment.source
            )
        })
        .unwrap_or_default();
    let glonass_frequency_channel_key = result
        .glonass_frequency_channel
        .map(|channel| format!("|{}", channel.value()))
        .unwrap_or_default();
    let component_provenance_key = result
        .component_provenance()
        .map(|provenance| {
            let components = provenance
                .components
                .iter()
                .map(|component| {
                    let secondary_code_phase_periods = component
                        .secondary_code_phase_periods
                        .map(|phase| phase.to_string())
                        .unwrap_or_else(|| "none".to_string());
                    format!(
                        "{:?}:{:.6}:{:.6}:{:.6}:{:.6}:{:.6}:{}",
                        component.role,
                        component.peak,
                        component.second_peak,
                        component.mean,
                        component.peak_mean_ratio,
                        component.peak_second_ratio,
                        secondary_code_phase_periods
                    )
                })
                .collect::<Vec<_>>()
                .join(",");
            format!("|{:?}|{}", provenance.combination_mode, components)
        })
        .unwrap_or_default();
    format!(
        "{:?}-{:02}|{}|{}{}|{:?}|{:.3}|{}|{:.6}|{:.6}|{:.6}|{}{}{}{}{}{}",
        result.sat.constellation,
        result.sat.prn,
        result.candidate_rank,
        result.is_primary_candidate,
        glonass_frequency_channel_key,
        result.signal_code,
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
        component_provenance_key,
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
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
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
    pub transmit_time: Option<TrackingTransmitTime>,
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
            signal_code: SignalCode::Unknown,
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
            transmit_time: None,
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct CodeCarrierDivergence {
    pub raw_m: f64,
    pub jump_m: f64,
    pub expected_ionosphere_m: f64,
    pub receiver_clock_m: f64,
    pub oscillator_m: f64,
    pub smoothing_transient_m: f64,
    pub multipath_m: f64,
    pub unexplained_m: f64,
}

impl CodeCarrierDivergence {
    pub fn from_terms(
        raw_m: f64,
        jump_m: f64,
        expected_ionosphere_m: f64,
        receiver_clock_m: f64,
        oscillator_m: f64,
        smoothing_transient_m: f64,
        multipath_m: f64,
    ) -> Self {
        let explained_m = expected_ionosphere_m
            + receiver_clock_m
            + oscillator_m
            + smoothing_transient_m
            + multipath_m;
        let unexplained_m = jump_m - explained_m;
        Self {
            raw_m,
            jump_m,
            expected_ionosphere_m,
            receiver_clock_m,
            oscillator_m,
            smoothing_transient_m,
            multipath_m,
            unexplained_m,
        }
    }

    pub fn unexplained_abs_m(self) -> f64 {
        self.unexplained_m.abs()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[serde(rename_all = "snake_case")]
pub enum CycleSlipDetector {
    TrackingLock,
    DopplerPredictedPhase,
    GeometryFreePhase,
    MelbourneWubbena,
    DataGap,
    PhaseInnovation,
    CodeCarrierDivergence,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CycleSlipDetectorEvidence {
    pub detector: CycleSlipDetector,
    pub triggered: bool,
    pub value: Option<f64>,
    pub threshold: Option<f64>,
    pub units: String,
    pub reason: String,
}

impl CycleSlipDetectorEvidence {
    pub fn new(
        detector: CycleSlipDetector,
        triggered: bool,
        value: Option<f64>,
        threshold: Option<f64>,
        units: impl Into<String>,
        reason: impl Into<String>,
    ) -> Self {
        Self {
            detector,
            triggered,
            value: value.filter(|value| value.is_finite()),
            threshold: threshold.filter(|threshold| threshold.is_finite()),
            units: units.into(),
            reason: reason.into(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CycleSlipDecisionEvidence {
    pub detected: bool,
    pub primary_reason: Option<String>,
    pub contributors: Vec<CycleSlipDetectorEvidence>,
    pub detection_probability_budget: f64,
    pub false_alarm_probability_budget: f64,
}

impl CycleSlipDecisionEvidence {
    pub fn from_contributors(
        contributors: Vec<CycleSlipDetectorEvidence>,
        detection_probability_budget: f64,
        false_alarm_probability_budget: f64,
    ) -> Self {
        let detected = contributors.iter().any(|contributor| contributor.triggered);
        let primary_reason = contributors
            .iter()
            .find(|contributor| contributor.triggered)
            .map(|contributor| contributor.reason.clone());
        Self {
            detected,
            primary_reason,
            contributors,
            detection_probability_budget,
            false_alarm_probability_budget,
        }
    }

    pub fn triggered_detectors(&self) -> Vec<CycleSlipDetector> {
        self.contributors
            .iter()
            .filter(|contributor| contributor.triggered)
            .map(|contributor| contributor.detector)
            .collect()
    }

    pub fn upsert_contributor(&mut self, contributor: CycleSlipDetectorEvidence) {
        if let Some(existing) =
            self.contributors.iter_mut().find(|existing| existing.detector == contributor.detector)
        {
            *existing = contributor;
        } else {
            self.contributors.push(contributor);
            self.contributors.sort_by_key(|contributor| contributor.detector);
        }
        self.detected = self.contributors.iter().any(|contributor| contributor.triggered);
        self.primary_reason = self
            .contributors
            .iter()
            .find(|contributor| contributor.triggered)
            .map(|contributor| contributor.reason.clone());
    }
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
    pub pseudorange_time_source: String,
    #[serde(default)]
    pub pseudorange_integer_code_periods: Option<u64>,
    #[serde(default)]
    pub pseudorange_code_delay_s: Option<Seconds>,
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
    #[serde(default = "default_seconds_zero")]
    pub receiver_clock_bias_s: Seconds,
    #[serde(default)]
    pub receiver_clock_frequency_bias_hz: f64,
    #[serde(default = "default_seconds_zero")]
    pub receiver_clock_bias_sigma_s: Seconds,
    #[serde(default)]
    pub receiver_clock_source: String,
    #[serde(default)]
    pub tracking_uncertainty: Option<TrackingUncertainty>,
    #[serde(default)]
    pub code_carrier_divergence: Option<CodeCarrierDivergence>,
    #[serde(default)]
    pub cycle_slip_evidence: Option<CycleSlipDecisionEvidence>,
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
            pseudorange_time_source: String::new(),
            pseudorange_integer_code_periods: None,
            pseudorange_code_delay_s: None,
            carrier_phase_model: "tracked_carrier_cycles".to_string(),
            doppler_model: OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET.to_string(),
            carrier_phase_continuity: "unusable".to_string(),
            carrier_phase_arc_start_epoch_idx: 0,
            carrier_phase_arc_start_sample_index: 0,
            signal_delay_alignment_source: String::new(),
            time_tag_source: String::new(),
            time_tag_sample_index: 0,
            time_tag_sample_rate_hz: 0.0,
            receiver_clock_bias_s: Seconds(0.0),
            receiver_clock_frequency_bias_hz: 0.0,
            receiver_clock_bias_sigma_s: Seconds(0.0),
            receiver_clock_source: String::new(),
            tracking_uncertainty: None,
            code_carrier_divergence: None,
            cycle_slip_evidence: None,
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

impl ObsSatellite {
    pub fn measurement_covariance(&self) -> Option<ObservationMeasurementCovariance> {
        let code_phase_m2 = positive_finite_variance(self.pseudorange_var_m2)?;
        let carrier_wavelength_m = self.carrier_wavelength_m()?;
        let carrier_phase_m2 = positive_finite_variance(self.carrier_phase_var_cycles2)?
            * carrier_wavelength_m.powi(2);
        let doppler_hz2 = positive_finite_variance(self.doppler_var_hz2)?;

        let common_clock_m2 = self
            .error_model
            .as_ref()
            .map(|model| model.clock_error_m.0.powi(2))
            .filter(|variance| variance.is_finite() && *variance >= 0.0)
            .unwrap_or(0.0);
        let code_carrier_m2 = bounded_covariance(
            common_clock_m2.min(code_phase_m2.min(carrier_phase_m2)),
            code_phase_m2,
            carrier_phase_m2,
        );
        let carrier_doppler_m_hz = self
            .metadata
            .tracking_uncertainty
            .as_ref()
            .and_then(|uncertainty| {
                let carrier_sigma_m = uncertainty.carrier_phase_cycles * carrier_wavelength_m;
                let doppler_sigma_hz = uncertainty.doppler_hz;
                (carrier_sigma_m.is_finite()
                    && carrier_sigma_m > 0.0
                    && doppler_sigma_hz.is_finite()
                    && doppler_sigma_hz > 0.0)
                    .then_some(
                        CARRIER_DOPPLER_CORRELATION_COEFFICIENT
                            * carrier_sigma_m
                            * doppler_sigma_hz,
                    )
            })
            .map(|covariance| bounded_covariance(covariance, carrier_phase_m2, doppler_hz2))
            .unwrap_or(0.0);

        let covariance = ObservationMeasurementCovariance {
            code_phase_m2,
            code_carrier_m2,
            code_doppler_m_hz: 0.0,
            carrier_code_m2: code_carrier_m2,
            carrier_phase_m2,
            carrier_doppler_m_hz,
            doppler_code_hz_m: 0.0,
            doppler_carrier_hz_m: carrier_doppler_m_hz,
            doppler_hz2,
            status: ObservationCovarianceStatus::PositiveSemidefinite,
        };
        covariance_is_positive_semidefinite(covariance.matrix()).then_some(covariance)
    }

    pub fn covariance_pseudorange_sigma_m(&self) -> Option<f64> {
        self.measurement_covariance()?.pseudorange_sigma_m()
    }

    fn carrier_wavelength_m(&self) -> Option<f64> {
        let carrier_hz = self.metadata.signal.carrier_hz.0;
        (carrier_hz.is_finite() && carrier_hz > 0.0).then_some(SPEED_OF_LIGHT_MPS / carrier_hz)
    }
}

fn positive_finite_variance(value: f64) -> Option<f64> {
    (value.is_finite() && value > 0.0).then_some(value)
}

fn bounded_covariance(covariance: f64, left_variance: f64, right_variance: f64) -> f64 {
    if !covariance.is_finite() || left_variance <= 0.0 || right_variance <= 0.0 {
        return 0.0;
    }
    let bound = (left_variance * right_variance).sqrt();
    covariance.clamp(-bound, bound)
}

fn covariance_is_positive_semidefinite(covariance: [[f64; 3]; 3]) -> bool {
    if covariance.iter().flatten().any(|value| !value.is_finite()) {
        return false;
    }
    let leading_minor_1 = covariance[0][0];
    let leading_minor_2 = covariance[0][0] * covariance[1][1] - covariance[0][1].powi(2);
    let determinant = covariance[0][0]
        * (covariance[1][1] * covariance[2][2] - covariance[1][2].powi(2))
        - covariance[0][1]
            * (covariance[1][0] * covariance[2][2] - covariance[1][2] * covariance[2][0])
        + covariance[0][2]
            * (covariance[1][0] * covariance[2][1] - covariance[1][1] * covariance[2][0]);
    let tolerance = 1.0e-9;
    leading_minor_1 >= -tolerance && leading_minor_2 >= -tolerance && determinant >= -tolerance
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
            SolutionStatus::Unavailable | SolutionStatus::Refused => NavQualityFlag::NoFix,
            SolutionStatus::Degraded
            | SolutionStatus::IntegrityFailed
            | SolutionStatus::Diverged => NavQualityFlag::Degraded,
            SolutionStatus::CodeOnly | SolutionStatus::Float => NavQualityFlag::Float,
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
        acq_result_stability_key, obs_epoch_stability_key, CodeCarrierDivergence,
        CycleSlipDecisionEvidence, CycleSlipDetector, CycleSlipDetectorEvidence,
        MeasurementErrorModel, ObsEpoch, ObsMetadata, ObsSatellite, ObservationCovarianceStatus,
        SignalDelayAlignment, TrackingUncertainty,
        OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
    };
    use crate::api::{
        trackable_acq_tracking_seeds, AcqCodePhaseRefinement, AcqComponentCombinationMode,
        AcqComponentProvenance, AcqComponentStatistic, AcqEvidence, AcqHypothesis, AcqResult,
        AcqSearchSummary, AcqUncertainty, AcqUncertaintyCovariance, Constellation, Cycles,
        GlonassFrequencyChannel, Hertz, LeapSeconds, LockFlags, NavLifecycleState, NavQualityFlag,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        SignalBand, SignalCode, SignalComponentRole, SignalSpec, SolutionStatus, UtcTime,
        GPS_L1_CA_CARRIER_HZ,
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
        assert_eq!(SolutionStatus::Unavailable.lifecycle_state(), NavLifecycleState::Unavailable);
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
        assert_eq!(SolutionStatus::IntegrityFailed.decision_label(), "integrity_failed");
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
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::from_sample_index(8_184, 4_092_000.0),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(750.0),
            doppler_rate_hz_per_s: 0.0,
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
                sample_delay_samples: 0,
                source: "synthetic_truth".to_string(),
            }),
            uncertainty: Some(AcqUncertainty {
                doppler_hz: 125.0,
                code_phase_samples: 0.25,
                doppler_rate_hz_per_s: None,
                covariance: Some(AcqUncertaintyCovariance {
                    doppler_variance_hz2: 15_625.0,
                    doppler_code_phase_covariance_hz_samples: -3.0,
                    code_phase_variance_samples2: 0.0625,
                    doppler_rate_variance_hz2_per_s2: None,
                    doppler_doppler_rate_covariance_hz2_per_s: None,
                    code_phase_doppler_rate_covariance_samples_hz_per_s: None,
                }),
            }),
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
                sample_delay_samples: 0,
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
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        });
        let mut changed = base.clone();
        changed.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: 68,
            sample_delay_samples: 4,
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
    fn acq_result_component_provenance_reads_primary_evidence() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut result = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        result.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: result.code_phase_samples,
            doppler_hz: result.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: result.peak,
            second_peak: result.second_peak,
            peak_mean_ratio: result.peak_mean_ratio,
            peak_second_ratio: result.peak_second_ratio,
            mean: result.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                components: vec![
                    AcqComponentStatistic {
                        role: SignalComponentRole::Data,
                        peak: 12.0,
                        second_peak: 3.0,
                        mean: 1.5,
                        peak_mean_ratio: 8.0,
                        peak_second_ratio: 4.0,
                        secondary_code_phase_periods: None,
                    },
                    AcqComponentStatistic {
                        role: SignalComponentRole::Pilot,
                        peak: 11.0,
                        second_peak: 2.5,
                        mean: 1.4,
                        peak_mean_ratio: 7.857143,
                        peak_second_ratio: 4.4,
                        secondary_code_phase_periods: None,
                    },
                ],
            }),
        }];

        let provenance =
            result.component_provenance().expect("component provenance must be discoverable");

        assert_eq!(provenance.combination_mode, AcqComponentCombinationMode::CoherentComponentSum);
        assert_eq!(provenance.components.len(), 2);
        assert_eq!(provenance.components[0].role, SignalComponentRole::Data);
        assert_eq!(provenance.components[1].role, SignalComponentRole::Pilot);
    }

    #[test]
    fn acq_result_stability_key_includes_component_provenance() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: base.code_phase_samples,
            doppler_hz: base.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: base.peak,
            second_peak: base.second_peak,
            peak_mean_ratio: base.peak_mean_ratio,
            peak_second_ratio: base.peak_second_ratio,
            mean: base.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::NoncoherentComponentSum,
                components: vec![AcqComponentStatistic {
                    role: SignalComponentRole::Data,
                    peak: 12.0,
                    second_peak: 3.0,
                    mean: 1.5,
                    peak_mean_ratio: 8.0,
                    peak_second_ratio: 4.0,
                    secondary_code_phase_periods: None,
                }],
            }),
        }];
        let mut changed = base.clone();
        changed.evidence[0].component_provenance = Some(AcqComponentProvenance {
            combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
            components: vec![AcqComponentStatistic {
                role: SignalComponentRole::Data,
                peak: 12.0,
                second_peak: 3.0,
                mean: 1.5,
                peak_mean_ratio: 8.0,
                peak_second_ratio: 4.0,
                secondary_code_phase_periods: None,
            }],
        });

        assert_ne!(acq_result_stability_key(&base), acq_result_stability_key(&changed));
    }

    #[test]
    fn acq_result_stability_key_includes_component_secondary_code_phase() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
        let mut base = acq_result_for_summary(sat, AcqHypothesis::Accepted);
        base.evidence = vec![AcqEvidence {
            rank: 1,
            code_phase_samples: base.code_phase_samples,
            doppler_hz: base.carrier_hz.0,
            doppler_rate_hz_per_s: 0.0,
            peak: base.peak,
            second_peak: base.second_peak,
            peak_mean_ratio: base.peak_mean_ratio,
            peak_second_ratio: base.peak_second_ratio,
            mean: base.mean,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                components: vec![AcqComponentStatistic {
                    role: SignalComponentRole::Pilot,
                    peak: 12.0,
                    second_peak: 3.0,
                    mean: 1.5,
                    peak_mean_ratio: 8.0,
                    peak_second_ratio: 4.0,
                    secondary_code_phase_periods: Some(17),
                }],
            }),
        }];
        let mut changed = base.clone();
        changed.evidence[0]
            .component_provenance
            .as_mut()
            .expect("component provenance must exist")
            .components[0]
            .secondary_code_phase_periods = Some(18);

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
    fn obs_metadata_defaults_receiver_clock_contract() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.receiver_clock_bias_s, crate::api::Seconds(0.0));
        assert_eq!(metadata.receiver_clock_frequency_bias_hz, 0.0);
        assert_eq!(metadata.receiver_clock_bias_sigma_s, crate::api::Seconds(0.0));
        assert_eq!(metadata.receiver_clock_source, "");
    }

    #[test]
    fn code_carrier_divergence_residual_tracks_unexplained_component() {
        let divergence = CodeCarrierDivergence::from_terms(130.0, 12.5, 7.0, 1.5, -0.25, 0.75, 2.0);

        assert_eq!(divergence.raw_m, 130.0);
        assert_eq!(divergence.jump_m, 12.5);
        assert_eq!(divergence.expected_ionosphere_m, 7.0);
        assert_eq!(divergence.receiver_clock_m, 1.5);
        assert_eq!(divergence.oscillator_m, -0.25);
        assert_eq!(divergence.smoothing_transient_m, 0.75);
        assert_eq!(divergence.multipath_m, 2.0);
        assert_eq!(divergence.unexplained_m, 1.5);
        assert_eq!(divergence.unexplained_abs_m(), 1.5);
    }

    #[test]
    fn obs_metadata_defaults_do_not_invent_code_carrier_divergence() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.code_carrier_divergence, None);
    }

    #[test]
    fn cycle_slip_decision_records_triggered_detector_budget() {
        let mut decision = CycleSlipDecisionEvidence::from_contributors(
            vec![
                CycleSlipDetectorEvidence::new(
                    CycleSlipDetector::TrackingLock,
                    false,
                    None,
                    None,
                    "",
                    "tracking_lock_continuous",
                ),
                CycleSlipDetectorEvidence::new(
                    CycleSlipDetector::PhaseInnovation,
                    true,
                    Some(0.42),
                    Some(0.15),
                    "cycles",
                    "phase_residual",
                ),
            ],
            0.99,
            1.0e-3,
        );

        assert!(decision.detected);
        assert_eq!(decision.primary_reason.as_deref(), Some("phase_residual"));
        assert_eq!(decision.triggered_detectors(), vec![CycleSlipDetector::PhaseInnovation]);
        assert_eq!(decision.detection_probability_budget, 0.99);
        assert_eq!(decision.false_alarm_probability_budget, 1.0e-3);

        decision.upsert_contributor(CycleSlipDetectorEvidence::new(
            CycleSlipDetector::TrackingLock,
            true,
            None,
            None,
            "",
            "loss_of_lock",
        ));

        assert_eq!(
            decision.triggered_detectors(),
            vec![CycleSlipDetector::TrackingLock, CycleSlipDetector::PhaseInnovation]
        );
        assert_eq!(decision.primary_reason.as_deref(), Some("loss_of_lock"));
    }

    #[test]
    fn obs_metadata_defaults_do_not_invent_cycle_slip_evidence() {
        let metadata = ObsMetadata::default();

        assert_eq!(metadata.cycle_slip_evidence, None);
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

    fn covariance_satellite(
        pseudorange_var_m2: f64,
        carrier_phase_var_cycles2: f64,
        doppler_var_hz2: f64,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: crate::api::SigId {
                sat: SatId { constellation: Constellation::Gps, prn: 4 },
                band: SignalBand::L1,
                code: crate::api::SignalCode::Ca,
            },
            pseudorange_m: crate::api::Meters(21_000_000.0),
            pseudorange_var_m2,
            carrier_phase_cycles: Cycles(12.25),
            carrier_phase_var_cycles2,
            doppler_hz: Hertz(125.0),
            doppler_var_hz2,
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
            error_model: Some(MeasurementErrorModel {
                thermal_noise_m: crate::api::Meters(0.0),
                tracking_jitter_m: crate::api::Meters(pseudorange_var_m2.sqrt()),
                multipath_proxy_m: crate::api::Meters(0.0),
                clock_error_m: crate::api::Meters(0.5),
            }),
            metadata: ObsMetadata {
                signal: SignalSpec {
                    constellation: Constellation::Gps,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: GPS_L1_CA_CARRIER_HZ,
                },
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.01,
                    carrier_phase_cycles: carrier_phase_var_cycles2.sqrt(),
                    doppler_hz: doppler_var_hz2.sqrt(),
                    cn0_dbhz: 0.5,
                }),
                ..ObsMetadata::default()
            },
        }
    }

    #[test]
    fn observation_measurement_covariance_is_symmetric_positive_semidefinite() {
        let sat = covariance_satellite(4.0, 0.01, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");
        let matrix = covariance.matrix();

        assert_eq!(covariance.status, ObservationCovarianceStatus::PositiveSemidefinite);
        assert_eq!(matrix[0][1], matrix[1][0]);
        assert_eq!(matrix[1][2], matrix[2][1]);
        assert!(matrix[0][0] > 0.0);
        assert!(matrix[1][1] > 0.0);
        assert!(matrix[2][2] > 0.0);
        let leading_minor = matrix[0][0] * matrix[1][1] - matrix[0][1].powi(2);
        let determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2].powi(2))
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
        assert!(leading_minor >= 0.0);
        assert!(determinant >= -1.0e-12);
    }

    #[test]
    fn observation_measurement_covariance_uses_common_clock_error() {
        let sat = covariance_satellite(4.0, 10.0, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");

        assert!((covariance.code_carrier_m2 - 0.25).abs() < 1.0e-12);
        assert_eq!(covariance.code_doppler_m_hz, 0.0);
    }

    #[test]
    fn observation_measurement_covariance_uses_carrier_loop_evidence() {
        let sat = covariance_satellite(4.0, 0.01, 9.0);

        let covariance = sat.measurement_covariance().expect("measurement covariance");

        assert!(covariance.carrier_doppler_m_hz > 0.0);
        assert_eq!(covariance.carrier_doppler_m_hz, covariance.doppler_carrier_hz_m);
    }

    #[test]
    fn observation_measurement_covariance_requires_positive_variances() {
        let sat = covariance_satellite(0.0, 0.01, 9.0);

        assert!(sat.measurement_covariance().is_none());
        assert!(sat.covariance_pseudorange_sigma_m().is_none());
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
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(0.0),
            doppler_rate_hz_per_s: 0.0,
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
