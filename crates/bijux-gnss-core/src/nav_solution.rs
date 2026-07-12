#![allow(missing_docs)]

use crate::ids::{Constellation, SatId, SignalBand};
use crate::obs::{
    MeasurementRejectReason, NavAssumptions, NavHealthEvent, NavLifecycleState, NavProvenance,
    NavQualityFlag, NavRefusalClass, NavUncertaintyClass, SolutionStatus, SolutionValidity,
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
};
use crate::time::{Epoch, ReceiverSampleTrace};
use crate::units::{Meters, Seconds};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavResidual {
    pub sat: SatId,
    pub residual_m: Meters,
    pub rejected: bool,
    #[serde(default)]
    pub weight: Option<f64>,
    #[serde(default)]
    pub reject_reason: Option<MeasurementRejectReason>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavConstellationResidualRms {
    pub constellation: Constellation,
    #[serde(default)]
    pub pre_fit_rms_m: Option<Meters>,
    #[serde(default)]
    pub post_fit_rms_m: Option<Meters>,
    #[serde(default)]
    pub pre_fit_sat_count: usize,
    #[serde(default)]
    pub post_fit_sat_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterSystemBias {
    pub constellation: Constellation,
    pub band: Option<SignalBand>,
    pub bias_s: Seconds,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavSolutionEpoch {
    pub epoch: Epoch,
    pub t_rx_s: Seconds,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub ecef_x_m: Meters,
    pub ecef_y_m: Meters,
    pub ecef_z_m: Meters,
    #[serde(default)]
    pub position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: Meters,
    pub clock_bias_s: Seconds,
    #[serde(default = "default_nav_clock_bias_m")]
    pub clock_bias_m: Meters,
    #[serde(default)]
    pub clock_drift_s_per_s: f64,
    pub pdop: f64,
    #[serde(default)]
    pub pre_fit_residual_rms_m: Option<Meters>,
    #[serde(default)]
    pub post_fit_residual_rms_m: Option<Meters>,
    pub rms_m: Meters,
    pub status: SolutionStatus,
    #[serde(default)]
    pub quality: NavQualityFlag,
    #[serde(default)]
    pub validity: SolutionValidity,
    pub valid: bool,
    #[serde(default)]
    pub processing_ms: Option<f64>,
    pub residuals: Vec<NavResidual>,
    #[serde(default)]
    pub constellation_residual_rms: Vec<NavConstellationResidualRms>,
    #[serde(default)]
    pub health: Vec<NavHealthEvent>,
    pub isb: Vec<InterSystemBias>,
    pub sigma_h_m: Option<Meters>,
    pub sigma_v_m: Option<Meters>,
    #[serde(default)]
    pub innovation_rms_m: Option<f64>,
    #[serde(default)]
    pub normalized_innovation_rms: Option<f64>,
    #[serde(default)]
    pub normalized_innovation_max: Option<f64>,
    pub ekf_innovation_rms: Option<f64>,
    pub ekf_condition_number: Option<f64>,
    pub ekf_whiteness_ratio: Option<f64>,
    pub ekf_predicted_variance: Option<f64>,
    pub ekf_observed_variance: Option<f64>,
    #[serde(default)]
    pub integrity_hpl_m: Option<f64>,
    #[serde(default)]
    pub integrity_vpl_m: Option<f64>,
    #[serde(default = "default_nav_solution_model_version")]
    pub model_version: u32,
    #[serde(default)]
    pub lifecycle_state: NavLifecycleState,
    #[serde(default)]
    pub uncertainty_class: NavUncertaintyClass,
    #[serde(default)]
    pub assumptions: Option<NavAssumptions>,
    #[serde(default)]
    pub refusal_class: Option<NavRefusalClass>,
    #[serde(default)]
    pub artifact_id: String,
    #[serde(default)]
    pub source_observation_epoch_id: String,
    #[serde(default)]
    pub explain_decision: String,
    #[serde(default)]
    pub explain_reasons: Vec<String>,
    #[serde(default)]
    pub provenance: Option<NavProvenance>,
    #[serde(default)]
    pub sat_count: usize,
    #[serde(default)]
    pub used_sat_count: usize,
    #[serde(default)]
    pub rejected_sat_count: usize,
    #[serde(default)]
    pub hdop: Option<f64>,
    #[serde(default)]
    pub vdop: Option<f64>,
    #[serde(default)]
    pub gdop: Option<f64>,
    #[serde(default)]
    pub tdop: Option<f64>,
    #[serde(default)]
    pub stability_signature: String,
    #[serde(default = "default_nav_output_stability_signature_version")]
    pub stability_signature_version: u32,
}

fn default_nav_solution_model_version() -> u32 {
    NAV_SOLUTION_MODEL_VERSION
}

fn default_nav_output_stability_signature_version() -> u32 {
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION
}

fn default_nav_clock_bias_m() -> Meters {
    Meters(0.0)
}
