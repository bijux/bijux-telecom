#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, SatId, SigId};
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity};

use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider};
use crate::corrections::phase_windup::PhaseWindupState;
use crate::corrections::CorrectionContext;
use crate::estimation::ekf::state::Ekf;
use crate::estimation::position::solver::weighting::WeightingConfig;
use crate::formats::precise_products::PreciseProductDiscontinuityKind;
use crate::models::antenna::{ReceiverAntennaCalibrations, SatelliteAntennaCalibrations};
use crate::models::atmosphere::TroposphereMeteorology;
use crate::models::ocean_tide_loading::OceanTideLoadingModel;
use crate::models::solid_earth_tide::SolidEarthTideModel;

pub const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct PppProcessNoise {
    pub position_m: f64,
    pub velocity_mps: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
    pub inter_system_bias_s: f64,
    pub ztd_m: f64,
    pub iono_m: f64,
    pub ambiguity_cycles: f64,
}

impl Default for PppProcessNoise {
    fn default() -> Self {
        Self {
            position_m: 0.02,
            velocity_mps: 0.005,
            clock_bias_s: 1e-7,
            clock_drift_s: 1e-5,
            inter_system_bias_s: 1e-9,
            ztd_m: 0.01,
            iono_m: 0.1,
            ambiguity_cycles: 0.05,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PppConvergenceConfig {
    pub min_time_s: f64,
    pub pos_rate_mps: f64,
    pub sigma_h_m: f64,
    pub sigma_v_m: f64,
}

#[derive(Debug, Clone)]
pub struct PppMeasurementNoise {
    pub code_floor_m: f64,
    pub phase_floor_cycles: f64,
    pub orbit_sigma_scale: f64,
    pub clock_sigma_scale: f64,
    pub troposphere_residual_m: f64,
    pub antenna_residual_m: f64,
}

impl Default for PppMeasurementNoise {
    fn default() -> Self {
        Self {
            code_floor_m: 0.3,
            phase_floor_cycles: 0.01,
            orbit_sigma_scale: 1.0,
            clock_sigma_scale: 1.0,
            troposphere_residual_m: 0.05,
            antenna_residual_m: 0.01,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PppConfig {
    pub enable_iono_state: bool,
    pub use_iono_free: bool,
    pub use_doppler: bool,
    pub ar_mode: PppArMode,
    pub ar_ratio_threshold: f64,
    pub ar_stability_epochs: u32,
    pub ar_max_sats: usize,
    pub ar_use_elevation: bool,
    pub prune_after_epochs: u64,
    pub reset_gap_s: f64,
    pub residual_gate_m: f64,
    pub drift_window_epochs: usize,
    pub drift_threshold_m: f64,
    pub checkpoint_interval_epochs: u64,
    pub tropo_pressure_hpa: Option<f64>,
    pub tropo_temperature_k: Option<f64>,
    pub tropo_relative_humidity: Option<f64>,
    pub solid_earth_tide_model: Option<SolidEarthTideModel>,
    pub ocean_tide_loading_model: Option<OceanTideLoadingModel>,
    pub receiver_antenna_type: Option<String>,
    pub receiver_antenna_calibrations: Option<ReceiverAntennaCalibrations>,
    pub satellite_antenna_calibrations: Option<SatelliteAntennaCalibrations>,
    pub process_noise: PppProcessNoise,
    pub measurement_noise: PppMeasurementNoise,
    pub precise_product_policy: PppPreciseProductPolicy,
    pub weighting: WeightingConfig,
    pub convergence: PppConvergenceConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppTroposphereSource {
    StandardAtmosphere,
    LocalMeteorology,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct PppStochasticEvidence {
    pub code_observation_variance_supported: bool,
    pub phase_observation_variance_supported: bool,
    pub satellite_orbit_uncertainty_supported: bool,
    pub satellite_clock_uncertainty_supported: bool,
    pub atmosphere_residual_supported: bool,
    pub antenna_residual_supported: bool,
    pub process_covariance_supported: bool,
}

#[derive(Debug, Clone)]
pub struct PppPreciseProductPolicy {
    pub missing_satellite_action: PppPreciseProductAction,
    pub out_of_coverage_action: PppPreciseProductAction,
    pub insufficient_support_action: PppPreciseProductAction,
    pub orbit_gap_action: PppPreciseProductAction,
    pub orbit_flag_action: PppPreciseProductAction,
    pub clock_gap_action: PppPreciseProductAction,
    pub clock_jump_action: PppPreciseProductAction,
    pub satellite_state_inflation: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppPreciseProductAction {
    BridgeWithBroadcast,
    InflateSatelliteState,
    ResetSatelliteState,
    RefuseSatellite,
}

impl Default for PppPreciseProductPolicy {
    fn default() -> Self {
        Self {
            missing_satellite_action: PppPreciseProductAction::BridgeWithBroadcast,
            out_of_coverage_action: PppPreciseProductAction::BridgeWithBroadcast,
            insufficient_support_action: PppPreciseProductAction::BridgeWithBroadcast,
            orbit_gap_action: PppPreciseProductAction::ResetSatelliteState,
            orbit_flag_action: PppPreciseProductAction::RefuseSatellite,
            clock_gap_action: PppPreciseProductAction::ResetSatelliteState,
            clock_jump_action: PppPreciseProductAction::ResetSatelliteState,
            satellite_state_inflation: 100.0,
        }
    }
}

impl PppPreciseProductPolicy {
    pub fn action_for(&self, kind: PreciseProductDiscontinuityKind) -> PppPreciseProductAction {
        match kind {
            PreciseProductDiscontinuityKind::MissingSatellite => self.missing_satellite_action,
            PreciseProductDiscontinuityKind::OutOfCoverage => self.out_of_coverage_action,
            PreciseProductDiscontinuityKind::InsufficientSupport => {
                self.insufficient_support_action
            }
            PreciseProductDiscontinuityKind::OrbitGap => self.orbit_gap_action,
            PreciseProductDiscontinuityKind::OrbitFlag => self.orbit_flag_action,
            PreciseProductDiscontinuityKind::ClockGap => self.clock_gap_action,
            PreciseProductDiscontinuityKind::ClockJump => self.clock_jump_action,
        }
    }
}

impl Default for PppConfig {
    fn default() -> Self {
        Self {
            enable_iono_state: false,
            use_iono_free: false,
            use_doppler: false,
            ar_mode: PppArMode::FloatPpp,
            ar_ratio_threshold: 3.0,
            ar_stability_epochs: 3,
            ar_max_sats: 8,
            ar_use_elevation: true,
            prune_after_epochs: 200,
            reset_gap_s: 2.0,
            residual_gate_m: 200.0,
            drift_window_epochs: 100,
            drift_threshold_m: 10.0,
            checkpoint_interval_epochs: 0,
            tropo_pressure_hpa: None,
            tropo_temperature_k: None,
            tropo_relative_humidity: None,
            solid_earth_tide_model: None,
            ocean_tide_loading_model: None,
            receiver_antenna_type: None,
            receiver_antenna_calibrations: None,
            satellite_antenna_calibrations: None,
            process_noise: PppProcessNoise::default(),
            measurement_noise: PppMeasurementNoise::default(),
            precise_product_policy: PppPreciseProductPolicy::default(),
            weighting: WeightingConfig::default(),
            convergence: PppConvergenceConfig {
                min_time_s: 60.0,
                pos_rate_mps: 0.1,
                sigma_h_m: 1.0,
                sigma_v_m: 2.0,
            },
        }
    }
}

impl PppConfig {
    pub fn troposphere_source(&self) -> PppTroposphereSource {
        if self.troposphere_meteorology().is_some() {
            PppTroposphereSource::LocalMeteorology
        } else {
            PppTroposphereSource::StandardAtmosphere
        }
    }

    pub(crate) fn troposphere_meteorology(&self) -> Option<TroposphereMeteorology> {
        match (self.tropo_pressure_hpa, self.tropo_temperature_k, self.tropo_relative_humidity) {
            (Some(pressure_hpa), Some(temperature_k), Some(relative_humidity)) => {
                let meteorology =
                    TroposphereMeteorology::new(pressure_hpa, temperature_k, relative_humidity);
                meteorology.is_physical().then_some(meteorology)
            }
            _ => None,
        }
    }

    pub(crate) fn ocean_tide_loading_displacement_m(
        &self,
        receiver_ecef_m: [f64; 3],
        gps_time: Option<bijux_gnss_core::api::GpsTime>,
    ) -> Option<[f64; 3]> {
        self.ocean_tide_loading_model.as_ref()?.displacement_ecef_m(receiver_ecef_m, gps_time?)
    }

    pub(crate) fn solid_earth_tide_displacement_m(
        &self,
        receiver_ecef_m: [f64; 3],
        gps_time: Option<bijux_gnss_core::api::GpsTime>,
    ) -> Option<[f64; 3]> {
        self.solid_earth_tide_model.as_ref()?.displacement_ecef_m(receiver_ecef_m, gps_time?)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppSolutionEpoch {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub ztd_m: f64,
    pub ztd_sigma_m: Option<f64>,
    pub troposphere_source: PppTroposphereSource,
    pub position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    pub sigma_e_m: Option<f64>,
    pub sigma_n_m: Option<f64>,
    pub sigma_u_m: Option<f64>,
    pub horizontal_error_ellipse_major_axis_m: Option<f64>,
    pub horizontal_error_ellipse_minor_axis_m: Option<f64>,
    pub horizontal_error_ellipse_azimuth_deg: Option<f64>,
    pub clock_bias_s: f64,
    pub constellation_clock_state_count: usize,
    pub slant_ionosphere_state_count: usize,
    pub carrier_ambiguity_state_count: usize,
    pub lifecycle_events: Vec<PppLifecycleEvent>,
    pub stochastic_evidence: PppStochasticEvidence,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub innovation_rms: f64,
    pub convergence: PppConvergenceState,
    pub residuals: Vec<(SigId, f64)>,
    pub nis_mean: Option<f64>,
    pub ar_mode: PppArMode,
    pub fixed_wl: usize,
    #[serde(default)]
    pub ambiguity_resolution: PppAmbiguityResolutionEvidence,
    #[serde(default)]
    pub integer_ambiguities: Vec<PppIntegerAmbiguityCandidate>,
}

impl ArtifactPayloadValidate for PppSolutionEpoch {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.ecef_x_m.is_finite()
            || !self.ecef_y_m.is_finite()
            || !self.ecef_z_m.is_finite()
            || !self.ztd_m.is_finite()
            || !self.clock_bias_s.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_EPOCH_INVALID",
                "PPP solution contains NaN/Inf",
            ));
        }
        if self.ztd_sigma_m.is_some_and(|value| !value.is_finite()) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_ZTD_SIGMA_INVALID",
                "PPP zenith troposphere delay sigma contains NaN/Inf",
            ));
        }
        if self.ztd_sigma_m.is_some_and(|value| value < 0.0) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_ZTD_SIGMA_NEGATIVE",
                "PPP zenith troposphere delay sigma must be non-negative",
            ));
        }
        if self.ztd_sigma_m.is_none() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "PPP_ZTD_SIGMA_MISSING",
                "PPP solution should carry zenith troposphere delay uncertainty",
            ));
        }
        if let Some(covariance) = self.position_covariance_ecef_m2 {
            if !covariance.iter().flat_map(|row| row.iter()).all(|value| value.is_finite()) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "PPP_POSITION_COVARIANCE_INVALID",
                    "PPP position covariance contains NaN/Inf",
                ));
            }
            if covariance[0][0] < 0.0 || covariance[1][1] < 0.0 || covariance[2][2] < 0.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "PPP_POSITION_COVARIANCE_NEGATIVE_VARIANCE",
                    "PPP position covariance diagonal must be non-negative",
                ));
            }
        } else {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "PPP_POSITION_COVARIANCE_MISSING",
                "PPP solution should carry an ECEF position covariance matrix",
            ));
        }
        let enu_sigmas = [self.sigma_e_m, self.sigma_n_m, self.sigma_u_m];
        if enu_sigmas.iter().flatten().any(|value| !value.is_finite()) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_POSITION_SIGMA_ENU_INVALID",
                "PPP ENU position standard deviations contain NaN/Inf",
            ));
        }
        if enu_sigmas.iter().flatten().any(|value| *value < 0.0) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_POSITION_SIGMA_ENU_NEGATIVE",
                "PPP ENU position standard deviations must be non-negative",
            ));
        }
        if enu_sigmas.iter().any(|value| value.is_none()) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "PPP_POSITION_SIGMA_ENU_MISSING",
                "PPP solution should carry east, north, and up position standard deviations",
            ));
        }
        let horizontal_error_ellipse = [
            self.horizontal_error_ellipse_major_axis_m,
            self.horizontal_error_ellipse_minor_axis_m,
        ];
        if horizontal_error_ellipse.iter().flatten().any(|value| !value.is_finite())
            || self.horizontal_error_ellipse_azimuth_deg.is_some_and(|value| !value.is_finite())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_HORIZONTAL_ERROR_ELLIPSE_INVALID",
                "PPP horizontal error ellipse contains NaN/Inf",
            ));
        }
        if horizontal_error_ellipse.iter().flatten().any(|value| *value < 0.0) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_HORIZONTAL_ERROR_ELLIPSE_NEGATIVE_AXIS",
                "PPP horizontal error ellipse axes must be non-negative",
            ));
        }
        if self
            .horizontal_error_ellipse_azimuth_deg
            .is_some_and(|value| !(0.0..180.0).contains(&value))
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_HORIZONTAL_ERROR_ELLIPSE_AZIMUTH_RANGE",
                "PPP horizontal error ellipse azimuth must be in [0, 180)",
            ));
        }
        if self.horizontal_error_ellipse_major_axis_m.is_none()
            || self.horizontal_error_ellipse_minor_axis_m.is_none()
            || self.horizontal_error_ellipse_azimuth_deg.is_none()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "PPP_HORIZONTAL_ERROR_ELLIPSE_MISSING",
                "PPP solution should carry horizontal error ellipse axes and azimuth",
            ));
        }
        if self.convergence.converged && !self.convergence.evidence.supports_convergence_claim() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_CONVERGENCE_EVIDENCE_MISSING",
                "PPP convergence claim is missing required numerical evidence",
            ));
        }
        if self.convergence.converged && !self.convergence.missing_reasons.is_empty() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_CONVERGENCE_REASON_INCONSISTENT",
                "PPP convergence claim must not carry unresolved evidence blockers",
            ));
        }
        if self.fixed_wl > 0 && self.ambiguity_resolution.accepted_count == 0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_AR_EVIDENCE_MISSING",
                "PPP fixed ambiguity count requires accepted ambiguity-resolution evidence",
            ));
        }
        if self
            .integer_ambiguities
            .iter()
            .any(|candidate| candidate.accepted && !candidate.phase_bias_provenance_complete)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_AR_PHASE_BIAS_PROVENANCE_MISSING",
                "PPP accepted integer ambiguity requires complete phase-bias provenance",
            ));
        }
        if self.integer_ambiguities.iter().any(|candidate| {
            candidate.accepted
                && (!candidate.ratio.is_finite()
                    || !candidate.float_cycles.is_finite()
                    || !candidate.variance_cycles2.is_finite()
                    || candidate.variance_cycles2 < 0.0)
        }) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "PPP_AR_CANDIDATE_INVALID",
                "PPP accepted integer ambiguity candidate must carry finite float, variance, and ratio evidence",
            ));
        }
        events
    }
}

#[cfg(test)]
mod tests {
    use super::{
        PppAmbiguityResolutionEvidence, PppArMode, PppConfig, PppConvergenceEvidence,
        PppConvergenceState, PppIntegerAmbiguityCandidate, PppIntegerAmbiguityKind,
        PppPreciseProductAction, PppPreciseProductPolicy, PppSolutionEpoch, PppStochasticEvidence,
        PppTroposphereSource,
    };
    use crate::formats::precise_products::PreciseProductDiscontinuityKind;
    use crate::models::ocean_tide_loading::{
        OceanTideConstituent, OceanTideLoadingConstituent, OceanTideLoadingModel,
    };
    use crate::models::solid_earth_tide::SolidEarthTideModel;
    use bijux_gnss_core::api::ArtifactPayloadValidate;

    fn sample_solution() -> PppSolutionEpoch {
        PppSolutionEpoch {
            epoch_idx: 7,
            t_rx_s: 7.0,
            ecef_x_m: 1.0,
            ecef_y_m: 2.0,
            ecef_z_m: 3.0,
            ztd_m: 2.35,
            ztd_sigma_m: Some(0.12),
            troposphere_source: PppTroposphereSource::StandardAtmosphere,
            position_covariance_ecef_m2: Some([
                [4.0, 0.5, 0.25],
                [0.5, 9.0, 0.75],
                [0.25, 0.75, 16.0],
            ]),
            sigma_e_m: Some(0.8),
            sigma_n_m: Some(0.9),
            sigma_u_m: Some(1.2),
            horizontal_error_ellipse_major_axis_m: Some(1.1),
            horizontal_error_ellipse_minor_axis_m: Some(0.7),
            horizontal_error_ellipse_azimuth_deg: Some(32.0),
            clock_bias_s: 1.0e-6,
            constellation_clock_state_count: 1,
            slant_ionosphere_state_count: 1,
            carrier_ambiguity_state_count: 2,
            lifecycle_events: Vec::new(),
            stochastic_evidence: PppStochasticEvidence {
                code_observation_variance_supported: true,
                phase_observation_variance_supported: true,
                satellite_orbit_uncertainty_supported: true,
                satellite_clock_uncertainty_supported: true,
                atmosphere_residual_supported: true,
                antenna_residual_supported: true,
                process_covariance_supported: true,
            },
            rms_m: 1.5,
            sigma_h_m: Some(2.0),
            sigma_v_m: Some(3.0),
            innovation_rms: 1.5,
            convergence: PppConvergenceState {
                converged: false,
                time_to_first_meter_s: None,
                time_to_decimeter_s: None,
                time_to_centimeter_s: None,
                last_position_change_m: None,
                evidence: PppConvergenceEvidence::default(),
                missing_reasons: Vec::new(),
            },
            residuals: Vec::new(),
            nis_mean: None,
            ar_mode: PppArMode::FloatPpp,
            fixed_wl: 0,
            ambiguity_resolution: PppAmbiguityResolutionEvidence::default(),
            integer_ambiguities: Vec::new(),
        }
    }

    #[test]
    fn ppp_solution_validation_warns_on_missing_position_covariance() {
        let mut solution = sample_solution();
        solution.position_covariance_ecef_m2 = None;

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_POSITION_COVARIANCE_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_warns_on_missing_enu_position_sigmas() {
        let mut solution = sample_solution();
        solution.sigma_e_m = None;

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_POSITION_SIGMA_ENU_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_warns_on_missing_horizontal_error_ellipse() {
        let mut solution = sample_solution();
        solution.horizontal_error_ellipse_major_axis_m = None;

        let diagnostics = solution.validate_payload();

        assert!(diagnostics
            .iter()
            .any(|event| event.code == "PPP_HORIZONTAL_ERROR_ELLIPSE_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_rejects_converged_state_without_full_evidence() {
        let mut solution = sample_solution();
        solution.convergence.converged = true;
        solution.convergence.evidence = PppConvergenceEvidence {
            covariance_supported: true,
            residual_supported: true,
            ambiguity_supported: false,
            correction_supported: false,
            integrity_supported: false,
        };
        solution.convergence.missing_reasons = vec![
            "missing_ambiguity_evidence".to_string(),
            "missing_correction_evidence".to_string(),
            "missing_integrity_evidence".to_string(),
        ];

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_CONVERGENCE_EVIDENCE_MISSING"));
        assert!(diagnostics
            .iter()
            .any(|event| event.code == "PPP_CONVERGENCE_REASON_INCONSISTENT"));
    }

    #[test]
    fn ppp_solution_validation_rejects_fixed_count_without_ar_evidence() {
        let mut solution = sample_solution();
        solution.fixed_wl = 1;

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_AR_EVIDENCE_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_rejects_integer_acceptance_without_phase_bias_provenance() {
        let mut solution = sample_solution();
        solution.integer_ambiguities.push(PppIntegerAmbiguityCandidate {
            kind: PppIntegerAmbiguityKind::WideLane,
            sat: bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: 7,
            },
            signal: None,
            float_cycles: 11.05,
            integer_cycles: 11,
            variance_cycles2: 0.01,
            ratio: 12.0,
            accepted: true,
            phase_bias_provenance_complete: false,
            validation_reasons: Vec::new(),
        });
        solution.ambiguity_resolution = PppAmbiguityResolutionEvidence {
            candidate_count: 1,
            accepted_count: 1,
            phase_bias_provenance_complete: false,
            wide_lane_validated: true,
            narrow_lane_validated: false,
            ratio_threshold: 3.0,
            stability_epochs_required: 3,
            stable_epochs: 3,
            missing_reasons: Vec::new(),
        };

        let diagnostics = solution.validate_payload();

        assert!(diagnostics
            .iter()
            .any(|event| event.code == "PPP_AR_PHASE_BIAS_PROVENANCE_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_warns_on_missing_ztd_sigma() {
        let mut solution = sample_solution();
        solution.ztd_sigma_m = None;

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_ZTD_SIGMA_MISSING"));
    }

    #[test]
    fn ppp_solution_validation_rejects_non_finite_position_covariance() {
        let mut solution = sample_solution();
        solution.position_covariance_ecef_m2 =
            Some([[1.0, 0.0, 0.0], [0.0, f64::NAN, 0.0], [0.0, 0.0, 1.0]]);

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_POSITION_COVARIANCE_INVALID"));
    }

    #[test]
    fn ppp_solution_validation_rejects_non_finite_enu_position_sigma() {
        let mut solution = sample_solution();
        solution.sigma_u_m = Some(f64::NAN);

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_POSITION_SIGMA_ENU_INVALID"));
    }

    #[test]
    fn ppp_solution_validation_rejects_non_finite_horizontal_error_ellipse() {
        let mut solution = sample_solution();
        solution.horizontal_error_ellipse_azimuth_deg = Some(f64::NAN);

        let diagnostics = solution.validate_payload();

        assert!(diagnostics
            .iter()
            .any(|event| event.code == "PPP_HORIZONTAL_ERROR_ELLIPSE_INVALID"));
    }

    #[test]
    fn ppp_solution_validation_rejects_non_finite_ztd_sigma() {
        let mut solution = sample_solution();
        solution.ztd_sigma_m = Some(f64::NAN);

        let diagnostics = solution.validate_payload();

        assert!(diagnostics.iter().any(|event| event.code == "PPP_ZTD_SIGMA_INVALID"));
    }

    #[test]
    fn ppp_config_builds_troposphere_meteorology_from_complete_inputs() {
        let config = PppConfig {
            tropo_pressure_hpa: Some(990.0),
            tropo_temperature_k: Some(299.15),
            tropo_relative_humidity: Some(0.65),
            ..PppConfig::default()
        };

        let meteorology = config.troposphere_meteorology().expect("complete meteorology");

        assert_eq!(meteorology.pressure_hpa, 990.0);
        assert_eq!(meteorology.temperature_k, 299.15);
        assert_eq!(meteorology.relative_humidity, 0.65);
        assert_eq!(config.troposphere_source(), PppTroposphereSource::LocalMeteorology);
    }

    #[test]
    fn ppp_config_ignores_partial_or_non_physical_troposphere_inputs() {
        let partial = PppConfig {
            tropo_pressure_hpa: Some(990.0),
            tropo_temperature_k: Some(299.15),
            ..PppConfig::default()
        };
        let non_physical = PppConfig {
            tropo_pressure_hpa: Some(50.0),
            tropo_temperature_k: Some(299.15),
            tropo_relative_humidity: Some(0.65),
            ..PppConfig::default()
        };

        assert!(partial.troposphere_meteorology().is_none());
        assert!(non_physical.troposphere_meteorology().is_none());
        assert_eq!(partial.troposphere_source(), PppTroposphereSource::StandardAtmosphere);
        assert_eq!(non_physical.troposphere_source(), PppTroposphereSource::StandardAtmosphere);
    }

    #[test]
    fn ppp_precise_product_policy_defaults_are_explicit() {
        let policy = PppPreciseProductPolicy::default();

        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::MissingSatellite),
            PppPreciseProductAction::BridgeWithBroadcast
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::OutOfCoverage),
            PppPreciseProductAction::BridgeWithBroadcast
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::InsufficientSupport),
            PppPreciseProductAction::BridgeWithBroadcast
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::OrbitGap),
            PppPreciseProductAction::ResetSatelliteState
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::OrbitFlag),
            PppPreciseProductAction::RefuseSatellite
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::ClockGap),
            PppPreciseProductAction::ResetSatelliteState
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::ClockJump),
            PppPreciseProductAction::ResetSatelliteState
        );
        assert_eq!(policy.satellite_state_inflation, 100.0);
    }

    #[test]
    fn ppp_precise_product_policy_supports_per_discontinuity_actions() {
        let policy = PppPreciseProductPolicy {
            orbit_gap_action: PppPreciseProductAction::InflateSatelliteState,
            clock_jump_action: PppPreciseProductAction::RefuseSatellite,
            ..PppPreciseProductPolicy::default()
        };

        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::OrbitGap),
            PppPreciseProductAction::InflateSatelliteState
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::ClockJump),
            PppPreciseProductAction::RefuseSatellite
        );
        assert_eq!(
            policy.action_for(PreciseProductDiscontinuityKind::ClockGap),
            PppPreciseProductAction::ResetSatelliteState
        );
    }

    #[test]
    fn ppp_config_resolves_ocean_tide_loading_displacement() {
        let config = PppConfig {
            ocean_tide_loading_model: Some(OceanTideLoadingModel {
                reference_time: bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 0.0 },
                constituents: vec![OceanTideLoadingConstituent::new(
                    OceanTideConstituent::M2,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.02,
                    0.0,
                )],
            }),
            ..PppConfig::default()
        };

        let displacement = config
            .ocean_tide_loading_displacement_m(
                [6_378_137.0, 0.0, 0.0],
                Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 0.0 }),
            )
            .expect("ocean tide loading displacement");

        assert!((displacement[0] - 0.02).abs() < 1.0e-9);
        assert!(displacement[1].abs() < 1.0e-9);
        assert!(displacement[2].abs() < 1.0e-9);
    }

    #[test]
    fn ppp_config_resolves_solid_earth_tide_displacement() {
        let config =
            PppConfig { solid_earth_tide_model: Some(SolidEarthTideModel), ..PppConfig::default() };

        let displacement = config
            .solid_earth_tide_displacement_m(
                [4_479_597.678, 628_775.774, 4_489_073.842],
                Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 43_200.0 }),
            )
            .expect("solid Earth tide displacement");

        assert!(displacement.into_iter().all(f64::is_finite));
        let displacement_norm_m = (displacement[0] * displacement[0]
            + displacement[1] * displacement[1]
            + displacement[2] * displacement[2])
            .sqrt();
        assert!(displacement_norm_m > 0.05);
        assert!(displacement_norm_m < 0.6);
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppConvergenceEvidence {
    pub covariance_supported: bool,
    pub residual_supported: bool,
    pub ambiguity_supported: bool,
    pub correction_supported: bool,
    pub integrity_supported: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppIntegerAmbiguityKind {
    WideLane,
    NarrowLane,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PppIntegerAmbiguityCandidate {
    pub kind: PppIntegerAmbiguityKind,
    pub sat: SatId,
    pub signal: Option<SigId>,
    pub float_cycles: f64,
    pub integer_cycles: i64,
    pub variance_cycles2: f64,
    pub ratio: f64,
    pub accepted: bool,
    pub phase_bias_provenance_complete: bool,
    #[serde(default)]
    pub validation_reasons: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PppAmbiguityResolutionEvidence {
    pub candidate_count: usize,
    pub accepted_count: usize,
    pub phase_bias_provenance_complete: bool,
    pub wide_lane_validated: bool,
    pub narrow_lane_validated: bool,
    pub ratio_threshold: f64,
    pub stability_epochs_required: u32,
    pub stable_epochs: u32,
    #[serde(default)]
    pub missing_reasons: Vec<String>,
}

impl Default for PppAmbiguityResolutionEvidence {
    fn default() -> Self {
        Self {
            candidate_count: 0,
            accepted_count: 0,
            phase_bias_provenance_complete: false,
            wide_lane_validated: false,
            narrow_lane_validated: false,
            ratio_threshold: 0.0,
            stability_epochs_required: 0,
            stable_epochs: 0,
            missing_reasons: Vec::new(),
        }
    }
}

impl Default for PppConvergenceEvidence {
    fn default() -> Self {
        Self {
            covariance_supported: false,
            residual_supported: false,
            ambiguity_supported: false,
            correction_supported: false,
            integrity_supported: false,
        }
    }
}

impl PppConvergenceEvidence {
    pub fn missing_reasons(&self) -> Vec<String> {
        let mut reasons = Vec::new();
        if !self.covariance_supported {
            reasons.push("missing_covariance_evidence".to_string());
        }
        if !self.residual_supported {
            reasons.push("missing_residual_evidence".to_string());
        }
        if !self.ambiguity_supported {
            reasons.push("missing_ambiguity_evidence".to_string());
        }
        if !self.correction_supported {
            reasons.push("missing_correction_evidence".to_string());
        }
        if !self.integrity_supported {
            reasons.push("missing_integrity_evidence".to_string());
        }
        reasons
    }

    pub fn supports_convergence_claim(&self) -> bool {
        self.covariance_supported
            && self.residual_supported
            && self.ambiguity_supported
            && self.correction_supported
            && self.integrity_supported
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppConvergenceState {
    pub converged: bool,
    pub time_to_first_meter_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub time_to_centimeter_s: Option<f64>,
    pub last_position_change_m: Option<f64>,
    #[serde(default)]
    pub evidence: PppConvergenceEvidence,
    #[serde(default)]
    pub missing_reasons: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PppArMode {
    FloatPpp,
    PppArWideLane,
    PppArNarrowLane,
}

#[derive(Debug, Clone)]
pub struct PppHealth {
    pub last_reset_reason: Option<String>,
    pub pruned_states: usize,
    pub convergence: PppConvergenceState,
    pub warnings: Vec<String>,
    pub lifecycle_events: Vec<PppLifecycleEvent>,
    pub nis_mean: Option<f64>,
    pub ar_events: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppLifecycleEventKind {
    ReceiverReset,
    CarrierDiscontinuity,
    SatelliteStatePruned,
    ProductSupportChanged,
    PreciseProductDiscontinuity,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PppLifecycleEvent {
    pub kind: PppLifecycleEventKind,
    pub epoch_idx: Option<u64>,
    pub sat: Option<SatId>,
    pub signal: Option<SigId>,
    pub removed_states: Vec<PppStateIdentity>,
    pub reason: String,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PppProductSupport {
    pub precise_orbit: bool,
    pub precise_clock: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PppIndices {
    pub pos: [usize; 3],
    pub vel: [usize; 3],
    pub clock_bias: usize,
    pub clock_drift: usize,
    pub ztd: usize,
    pub isb: BTreeMap<Constellation, usize>,
    pub iono: BTreeMap<SatId, usize>,
    pub ambiguity: BTreeMap<SigId, usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppStateIdentity {
    ReceiverPositionX,
    ReceiverPositionY,
    ReceiverPositionZ,
    ReceiverVelocityX,
    ReceiverVelocityY,
    ReceiverVelocityZ,
    ReceiverClockBias,
    ReceiverClockDrift,
    ZenithTroposphereDelay,
    InterSystemBias(Constellation),
    SlantIonosphere(SatId),
    CarrierAmbiguity(SigId),
}

pub struct PppFilter {
    pub ekf: Ekf,
    pub config: PppConfig,
    pub indices: PppIndices,
    pub state_identities: Vec<PppStateIdentity>,
    pub last_t_rx_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
    pub epoch0_t_s: Option<f64>,
    pub last_seen_iono: BTreeMap<SatId, u64>,
    pub last_seen_amb: BTreeMap<SigId, u64>,
    pub residual_history: BTreeMap<SigId, Vec<f64>>,
    pub drift_history: Vec<[f64; 3]>,
    pub wl_state: BTreeMap<SatId, WlAmbiguity>,
    pub phase_windup: BTreeMap<SatId, PhaseWindupState>,
    pub product_support: BTreeMap<SatId, PppProductSupport>,
    pub ar_stable_epochs: u32,
    pub ar_evidence: PppAmbiguityResolutionEvidence,
    pub ar_integer_ambiguities: Vec<PppIntegerAmbiguityCandidate>,
    pub health: PppHealth,
    pub code_bias: Box<dyn CodeBiasProvider + Send + Sync>,
    pub phase_bias: Box<dyn PhaseBiasProvider + Send + Sync>,
    pub corrections: CorrectionContext,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PppCheckpoint {
    pub x: Vec<f64>,
    pub p: Vec<Vec<f64>>,
    #[serde(default)]
    pub state_identities: Vec<PppStateIdentity>,
    pub indices_isb: Vec<(Constellation, usize)>,
    pub indices_iono: Vec<(SatId, usize)>,
    pub indices_amb: Vec<(SigId, usize)>,
    pub last_t_rx_s: Option<f64>,
    pub epoch0_t_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
    #[serde(default)]
    pub last_seen_iono: Vec<(SatId, u64)>,
    #[serde(default)]
    pub last_seen_amb: Vec<(SigId, u64)>,
    pub phase_windup: Vec<(SatId, PhaseWindupState)>,
    #[serde(default)]
    pub wl_state: Vec<(SatId, WlAmbiguity)>,
    #[serde(default)]
    pub product_support: Vec<(SatId, PppProductSupport)>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct WlAmbiguity {
    pub float_cycles: f64,
    pub variance: f64,
    pub fixed: bool,
    #[serde(default)]
    pub integer_cycles: Option<i64>,
    #[serde(default)]
    pub ratio: Option<f64>,
    #[serde(default)]
    pub phase_bias_provenance_complete: bool,
    pub last_update_epoch: u64,
}
