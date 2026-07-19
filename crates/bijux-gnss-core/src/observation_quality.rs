#![allow(missing_docs)]

use crate::ids::{Constellation, SigId, SignalBand, SignalCode, SignalSpec, GPS_L1_CA_CARRIER_HZ};
use crate::observation::epochs::ObservationStatus;
use crate::observation::tracking::TrackingUncertainty;
use crate::observation::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET;
use crate::time::GpsTime;
use crate::units::{Cycles, Hertz, Meters, Seconds};
use serde::{Deserialize, Serialize};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const CARRIER_DOPPLER_CORRELATION_COEFFICIENT: f64 = 0.5;

fn default_observation_lock_state() -> String {
    "inactive".to_string()
}

fn default_seconds_zero() -> Seconds {
    Seconds(0.0)
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

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CarrierPhaseArc {
    pub id: String,
    pub signal_id: SigId,
    pub start_epoch_idx: u64,
    pub start_sample_index: u64,
    pub start_reason: String,
    pub valid_for_smoothing: bool,
    pub valid_for_ambiguity: bool,
}

impl CarrierPhaseArc {
    pub fn new(
        signal_id: SigId,
        start_epoch_idx: u64,
        start_sample_index: u64,
        start_reason: impl Into<String>,
    ) -> Self {
        Self {
            id: carrier_phase_arc_id(signal_id, start_epoch_idx, start_sample_index),
            signal_id,
            start_epoch_idx,
            start_sample_index,
            start_reason: start_reason.into(),
            valid_for_smoothing: true,
            valid_for_ambiguity: true,
        }
    }

    pub fn invalid_boundary(signal_id: SigId, boundary_reason: impl Into<String>) -> Self {
        Self {
            id: carrier_phase_arc_id(signal_id, 0, 0),
            signal_id,
            start_epoch_idx: 0,
            start_sample_index: 0,
            start_reason: boundary_reason.into(),
            valid_for_smoothing: false,
            valid_for_ambiguity: false,
        }
    }
}

fn carrier_phase_arc_id(signal_id: SigId, start_epoch_idx: u64, start_sample_index: u64) -> String {
    format!(
        "{:?}-{:02}-{:?}-{:?}-e{start_epoch_idx:010}-s{start_sample_index:012}",
        signal_id.sat.constellation, signal_id.sat.prn, signal_id.band, signal_id.code
    )
    .to_ascii_lowercase()
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
    pub carrier_phase_arc: Option<CarrierPhaseArc>,
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
                code: SignalCode::Unknown,
                code_rate_hz: 0.0,
                carrier_hz: GPS_L1_CA_CARRIER_HZ,
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
            carrier_phase_arc: None,
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
    pub transmit_gps_time: GpsTime,
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
