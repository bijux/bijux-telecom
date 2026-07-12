#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, SatId, SigId};
use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{ArtifactPayloadValidate, DiagnosticEvent, DiagnosticSeverity};

use crate::corrections::biases::{CodeBiasProvider, PhaseBiasProvider};
use crate::corrections::CorrectionContext;
use crate::estimation::ekf::state::Ekf;
use crate::estimation::position::solver::WeightingConfig;
use crate::models::antenna::{ReceiverAntennaCalibrations, SatelliteAntennaCalibrations};
use crate::models::atmosphere::TroposphereMeteorology;
use crate::models::ocean_tide_loading::OceanTideLoadingModel;

pub const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub struct PppProcessNoise {
    pub clock_drift_s: f64,
    pub ztd_m: f64,
    pub iono_m: f64,
    pub ambiguity_cycles: f64,
}

#[derive(Debug, Clone)]
pub struct PppConvergenceConfig {
    pub min_time_s: f64,
    pub pos_rate_mps: f64,
    pub sigma_h_m: f64,
    pub sigma_v_m: f64,
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
    pub ocean_tide_loading_model: Option<OceanTideLoadingModel>,
    pub receiver_antenna_type: Option<String>,
    pub receiver_antenna_calibrations: Option<ReceiverAntennaCalibrations>,
    pub satellite_antenna_calibrations: Option<SatelliteAntennaCalibrations>,
    pub process_noise: PppProcessNoise,
    pub weighting: WeightingConfig,
    pub convergence: PppConvergenceConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PppTroposphereSource {
    StandardAtmosphere,
    LocalMeteorology,
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
            ocean_tide_loading_model: None,
            receiver_antenna_type: None,
            receiver_antenna_calibrations: None,
            satellite_antenna_calibrations: None,
            process_noise: PppProcessNoise {
                clock_drift_s: 1e-5,
                ztd_m: 0.01,
                iono_m: 0.1,
                ambiguity_cycles: 0.05,
            },
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
        match (
            self.tropo_pressure_hpa,
            self.tropo_temperature_k,
            self.tropo_relative_humidity,
        ) {
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
        self.ocean_tide_loading_model
            .as_ref()?
            .displacement_ecef_m(receiver_ecef_m, gps_time?)
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
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub innovation_rms: f64,
    pub convergence: PppConvergenceState,
    pub residuals: Vec<(SigId, f64)>,
    pub nis_mean: Option<f64>,
    pub ar_mode: PppArMode,
    pub fixed_wl: usize,
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
        events
    }
}

#[cfg(test)]
mod tests {
    use super::{
        PppArMode, PppConfig, PppConvergenceState, PppSolutionEpoch, PppTroposphereSource,
    };
    use bijux_gnss_core::api::ArtifactPayloadValidate;
    use crate::models::ocean_tide_loading::{
        OceanTideConstituent, OceanTideLoadingConstituent, OceanTideLoadingModel,
    };

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
            },
            residuals: Vec::new(),
            nis_mean: None,
            ar_mode: PppArMode::FloatPpp,
            fixed_wl: 0,
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
        assert_eq!(
            non_physical.troposphere_source(),
            PppTroposphereSource::StandardAtmosphere
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
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppConvergenceState {
    pub converged: bool,
    pub time_to_first_meter_s: Option<f64>,
    pub time_to_decimeter_s: Option<f64>,
    pub time_to_centimeter_s: Option<f64>,
    pub last_position_change_m: Option<f64>,
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
    pub nis_mean: Option<f64>,
    pub ar_events: Vec<String>,
}

#[derive(Debug, Clone)]
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

pub struct PppFilter {
    pub ekf: Ekf,
    pub config: PppConfig,
    pub indices: PppIndices,
    pub last_t_rx_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
    pub epoch0_t_s: Option<f64>,
    pub last_seen_iono: BTreeMap<SatId, u64>,
    pub last_seen_amb: BTreeMap<SigId, u64>,
    pub residual_history: BTreeMap<SigId, Vec<f64>>,
    pub drift_history: Vec<[f64; 3]>,
    pub wl_state: BTreeMap<SatId, WlAmbiguity>,
    pub ar_stable_epochs: u32,
    pub health: PppHealth,
    pub code_bias: Box<dyn CodeBiasProvider + Send + Sync>,
    pub phase_bias: Box<dyn PhaseBiasProvider + Send + Sync>,
    pub corrections: CorrectionContext,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PppCheckpoint {
    pub x: Vec<f64>,
    pub p: Vec<Vec<f64>>,
    pub indices_isb: Vec<(Constellation, usize)>,
    pub indices_iono: Vec<(SatId, usize)>,
    pub indices_amb: Vec<(SigId, usize)>,
    pub last_t_rx_s: Option<f64>,
    pub epoch0_t_s: Option<f64>,
    pub last_pos: Option<[f64; 3]>,
}

#[derive(Debug, Clone)]
pub struct WlAmbiguity {
    pub float_cycles: f64,
    pub variance: f64,
    pub fixed: bool,
    pub last_update_epoch: u64,
}
