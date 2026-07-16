use super::{default_signal_band, default_signal_code, SignalDelayAlignment};
use crate::api::SignalCode;
use crate::api::{Chips, GlonassFrequencyChannel, Hertz, ReceiverSampleTrace, SatId, SignalBand};
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

fn default_doppler_center_hz() -> f64 {
    0.0
}

fn default_doppler_rate_center_hz_per_s() -> f64 {
    0.0
}

fn default_acquisition_threshold_mode() -> String {
    "fixed_ratio".to_string()
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
