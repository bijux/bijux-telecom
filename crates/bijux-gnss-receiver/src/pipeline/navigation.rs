#![allow(missing_docs)]

use bijux_gnss_core::api::{
    check_nav_solution_sanity, is_solution_valid, obs_epoch_stability_key, Constellation,
    MeasurementRejectReason, Meters, NavAssumptions, NavLifecycleState, NavProvenance,
    NavRefusalClass, NavResidual, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch, ObsSatellite,
    Seconds, SolutionStatus, SolutionValidity, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    NAV_SOLUTION_MODEL_VERSION,
};
use bijux_gnss_nav::api::{
    ecef_to_geodetic, elevation_azimuth_deg, formal_protection_levels,
    position_broadcast_navigation_from_galileo_navigations,
    position_broadcast_navigation_from_gps_ephemerides, position_measurement_weight,
    position_observation_has_valid_satellite_time, sat_state_beidou_b1i_from_observation,
    sat_state_galileo_e1_from_observation, sat_state_glonass_l1_from_observation,
    sat_state_gps_l1ca_from_observation, GalileoBroadcastNavigationData,
    GpsBroadcastNavigationData, GpsEphemeris, KlobucharCoefficients,
    ImpossibleGeometryEvidence, PositionBroadcastNavigation, PositionFilterMotionClass,
    PositionFilterDivergenceReason, PositionObservation, PositionRobustWeighting,
    PositionSolutionSmoother, PositionSolutionSmootherConfig, PositionSolveRefusalKind,
    PositionSolver, PositionWeightingModel, RaimFaultDetectionStatus, WeightingConfig,
};

use crate::engine::receiver_config::{
    NavigationMotionClass, NavigationWeightingMode, ReceiverPipelineConfig,
};
use crate::engine::runtime::ReceiverRuntime;
use crate::pipeline::common_code_doppler_anomaly::detect_common_code_doppler_anomaly;
use crate::pipeline::constellation_clock_inconsistency::{
    detect_constellation_clock_inconsistencies, ConstellationClockInconsistency,
};
use crate::pipeline::replay_timing_anomaly::detect_replay_timing_anomaly;
use crate::pipeline::residual_whiteness::{
    advance_residual_whiteness_suspect_streak, classify_residual_temporal_correlation,
    detect_residual_temporal_correlation, residual_temporal_correlation_is_persistent,
};
use crate::pipeline::satellite_clock_anomaly::{
    advance_satellite_clock_suspect_streak, detect_satellite_clock_anomaly,
};

const FIXED_VALIDATED_SIGMA_H_FLOOR_M: f64 = 0.02;
const FIXED_VALIDATED_SIGMA_V_FLOOR_M: f64 = 0.03;
const FIXED_SIGMA_H_FLOOR_M: f64 = 0.10;
const FIXED_SIGMA_V_FLOOR_M: f64 = 0.15;
const FLOAT_SIGMA_H_FLOOR_M: f64 = 0.25;
const FLOAT_SIGMA_V_FLOOR_M: f64 = 0.40;
const CONVERGED_SIGMA_H_FLOOR_M: f64 = 1.0;
const CONVERGED_SIGMA_V_FLOOR_M: f64 = 2.0;
const CODE_ONLY_SIGMA_H_FLOOR_M: f64 = 3.0;
const CODE_ONLY_SIGMA_V_FLOOR_M: f64 = 5.0;
const DEGRADED_SIGMA_H_FLOOR_M: f64 = 5.0;
const DEGRADED_SIGMA_V_FLOOR_M: f64 = 8.0;

/// Navigation solution derived from observation epochs.
pub struct Navigation {
    #[allow(dead_code)]
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
    solver: PositionSolver,
    position_smoother: Option<PositionSolutionSmoother>,
    clock: ClockModel,
    last_ecef: Option<(f64, f64, f64)>,
    last_solution: Option<NavSolutionEpoch>,
    last_position_observations: Option<Vec<PositionObservation>>,
    last_satellite_clock_suspect: Option<bijux_gnss_core::api::SatId>,
    satellite_clock_suspect_streak: usize,
    residual_whiteness_suspect_streak: usize,
}

#[derive(Debug, Clone)]
struct NavDecision {
    status: SolutionStatus,
    refusal_class: Option<NavRefusalClass>,
    explain_decision: String,
    explain_reasons: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
struct PrecisionReportingPolicy {
    horizontal_sigma_floor_m: f64,
    vertical_sigma_floor_m: f64,
    low_uncertainty_allowed: bool,
    explain_reason: &'static str,
}

#[derive(Debug, Clone)]
enum PositionObservationPreparation {
    Included(PositionObservation),
    InvalidSatelliteTime(bijux_gnss_core::api::SatId),
    ExcludedByElevationMask,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavigationState {
    ColdStart,
    Coarse,
    Converged,
    Degraded,
    Recovered,
}

pub struct NavigationEngine {
    pub state: NavigationState,
}

impl NavigationEngine {
    pub fn new() -> Self {
        Self { state: NavigationState::ColdStart }
    }

    pub fn transition(&mut self, sats: usize, rms_m: f64) {
        self.state = match (self.state, sats, rms_m) {
            (_, s, _) if s < 4 => NavigationState::Degraded,
            (NavigationState::ColdStart, _, r) if r.is_finite() => NavigationState::Coarse,
            (NavigationState::Coarse, _, r) if r < 50.0 => NavigationState::Converged,
            (NavigationState::Converged, _, r) if r > 200.0 => NavigationState::Degraded,
            (NavigationState::Degraded, _, r) if r < 80.0 => NavigationState::Recovered,
            (state, _, _) => state,
        };
    }
}

impl Default for NavigationEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone)]
pub struct EkfState {
    pub position_ecef_m: [f64; 3],
    pub velocity_ecef_mps: [f64; 3],
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
}

impl EkfState {
    pub fn new() -> Self {
        Self {
            position_ecef_m: [0.0; 3],
            velocity_ecef_mps: [0.0; 3],
            clock_bias_s: 0.0,
            clock_drift_s: 0.0,
        }
    }
}

impl Default for EkfState {
    fn default() -> Self {
        Self::new()
    }
}

impl Navigation {
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        let mut solver = PositionSolver::new();
        solver.robust_weighting = if config.robust_solver {
            PositionRobustWeighting::huber(config.huber_k)
        } else {
            PositionRobustWeighting::Disabled
        };
        solver.raim = config.raim;
        solver.apply_troposphere = config.tropo_enable;
        let position_smoother = config.position_solution_smoothing.then(|| {
            PositionSolutionSmoother::new(position_solution_smoother_config(
                config.position_solution_motion_class,
            ))
        });
        Self {
            config,
            runtime,
            solver,
            position_smoother,
            clock: ClockModel::new(),
            last_ecef: None,
            last_solution: None,
            last_position_observations: None,
            last_satellite_clock_suspect: None,
            satellite_clock_suspect_streak: 0,
            residual_whiteness_suspect_streak: 0,
        }
    }

    pub fn solve_epoch(
        &mut self,
        obs: &ObsEpoch,
        eph: &[GpsEphemeris],
    ) -> Option<NavSolutionEpoch> {
        let navigation = position_broadcast_navigation_from_gps_ephemerides(eph);
        self.solve_epoch_with_navigation_data_and_broadcast_ionosphere(obs, &navigation, None)
    }

    pub fn solve_epoch_with_broadcast_ionosphere(
        &mut self,
        obs: &ObsEpoch,
        eph: &[GpsEphemeris],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        let navigation = position_broadcast_navigation_from_gps_ephemerides(eph);
        self.solve_epoch_with_navigation_data_and_broadcast_ionosphere(obs, &navigation, klobuchar)
    }

    pub fn solve_epoch_with_gps_broadcast_navigation(
        &mut self,
        obs: &ObsEpoch,
        navigation: &GpsBroadcastNavigationData,
    ) -> Option<NavSolutionEpoch> {
        let entries = position_broadcast_navigation_from_gps_ephemerides(&navigation.ephemerides);
        self.solve_epoch_with_navigation_data_and_broadcast_ionosphere(
            obs,
            &entries,
            navigation.klobuchar.as_ref(),
        )
    }

    pub fn solve_epoch_with_galileo_broadcast_navigations(
        &mut self,
        obs: &ObsEpoch,
        navigations: &[GalileoBroadcastNavigationData],
    ) -> Option<NavSolutionEpoch> {
        let entries = position_broadcast_navigation_from_galileo_navigations(navigations);
        self.solve_epoch_with_navigation_data_and_broadcast_ionosphere(obs, &entries, None)
    }

    pub fn solve_epoch_with_navigation_data(
        &mut self,
        obs: &ObsEpoch,
        navigation: &[PositionBroadcastNavigation],
    ) -> Option<NavSolutionEpoch> {
        self.solve_epoch_with_navigation_data_and_broadcast_ionosphere(obs, navigation, None)
    }

    pub fn solve_epoch_with_navigation_data_and_broadcast_ionosphere(
        &mut self,
        obs: &ObsEpoch,
        navigation: &[PositionBroadcastNavigation],
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<NavSolutionEpoch> {
        let source_observation_epoch_id = source_observation_epoch_id(obs);
        let nav_artifact_id = nav_artifact_id(obs.epoch_idx, &source_observation_epoch_id);
        let assumptions = nav_assumptions(navigation.len());
        if !obs.valid {
            return Some(apply_atmosphere_explainability(
                invalid_solution_epoch(
                    obs,
                    source_observation_epoch_id,
                    nav_artifact_id,
                    Some(NavRefusalClass::InconsistentObservations),
                    "invalid_observation_epoch".to_string(),
                    vec!["input_observation_marked_invalid".to_string()],
                    assumptions,
                ),
                obs,
                navigation,
                klobuchar,
                self.config.tropo_enable,
            ));
        }
        let input_constellations = obs
            .sats
            .iter()
            .map(|row| row.signal_id.sat.constellation)
            .collect::<std::collections::BTreeSet<_>>();
        let has_supported_constellation =
            input_constellations.iter().copied().any(navigation_supported_constellation);
        let has_unsupported_constellation = input_constellations
            .iter()
            .any(|constellation| !navigation_supported_constellation(*constellation));
        let filtered_by_policy_count = obs
            .sats
            .iter()
            .filter(|sat| {
                navigation_supported_constellation(sat.signal_id.sat.constellation)
                    && !self.config.allows_constellation(sat.signal_id.sat.constellation)
            })
            .count();

        if !has_supported_constellation {
            return Some(apply_atmosphere_explainability(
                invalid_solution_epoch(
                    obs,
                    source_observation_epoch_id,
                    nav_artifact_id,
                    Some(NavRefusalClass::UnsupportedConstellation),
                    "unsupported_constellation_input".to_string(),
                    vec![format!(
                        "constellations={}",
                        input_constellations
                            .iter()
                            .map(|value| format!("{value:?}"))
                            .collect::<Vec<_>>()
                            .join(",")
                    )],
                    assumptions,
                ),
                obs,
                navigation,
                klobuchar,
                self.config.tropo_enable,
            ));
        }
        if has_unsupported_constellation {
            return Some(apply_atmosphere_explainability(
                invalid_solution_epoch(
                    obs,
                    source_observation_epoch_id,
                    nav_artifact_id,
                    Some(NavRefusalClass::MixedConstellationInput),
                    "mixed_constellation_time_handling_refused".to_string(),
                    vec![
                        format!(
                            "supported_constellations={}",
                            input_constellations
                                .iter()
                                .filter(|constellation| navigation_supported_constellation(
                                    **constellation
                                ))
                                .map(|value| format!("{value:?}"))
                                .collect::<Vec<_>>()
                                .join(",")
                        ),
                        format!(
                            "unsupported_constellations={}",
                            input_constellations
                                .iter()
                                .filter(|constellation| !navigation_supported_constellation(
                                    **constellation
                                ))
                                .map(|value| format!("{value:?}"))
                                .collect::<Vec<_>>()
                                .join(",")
                        ),
                        "unknown_inter_system_time_offset".to_string(),
                    ],
                    assumptions,
                ),
                obs,
                navigation,
                klobuchar,
                self.config.tropo_enable,
            ));
        }
        let mut invalid_satellite_time_sats = Vec::new();
        let observations: Vec<PositionObservation> = obs
            .sats
            .iter()
            .filter(|s| navigation_supported_constellation(s.signal_id.sat.constellation))
            .filter(|s| self.config.allows_constellation(s.signal_id.sat.constellation))
            .filter_map(|s| {
                match prepare_position_observation(&self.config, obs, s, navigation, self.last_ecef)
                {
                    PositionObservationPreparation::Included(observation) => Some(observation),
                    PositionObservationPreparation::InvalidSatelliteTime(sat) => {
                        invalid_satellite_time_sats.push(sat);
                        None
                    }
                    PositionObservationPreparation::ExcludedByElevationMask => None,
                }
            })
            .collect();
        if !invalid_satellite_time_sats.is_empty() {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_INVALID_SATELLITE_TIME",
                "excluded observations without valid satellite timing",
            ));
        }
        if observations.len() < 4 {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_INSUFFICIENT_SATS",
                "insufficient satellites for navigation solution",
            ));
            let refusal = if !invalid_satellite_time_sats.is_empty() {
                NavRefusalClass::InvalidSatelliteTime
            } else if has_unsupported_constellation {
                NavRefusalClass::MixedConstellationInput
            } else {
                NavRefusalClass::InsufficientGeometry
            };
            let mut explain_reasons = vec![
                "insufficient_geometry".to_string(),
                format!("supported_satellites={}", observations.len()),
            ];
            if !invalid_satellite_time_sats.is_empty() {
                explain_reasons.push("invalid_satellite_time".to_string());
                explain_reasons.push(format!(
                    "invalid_satellite_time_count={}",
                    invalid_satellite_time_sats.len()
                ));
            }
            if filtered_by_policy_count > 0 {
                explain_reasons.push("constellation_policy_filtered_satellites".to_string());
                explain_reasons.push(format!(
                    "constellation_policy_filtered_count={filtered_by_policy_count}"
                ));
            }
            return Some(apply_atmosphere_explainability(
                self.current_epoch_refusal(
                    obs,
                    source_observation_epoch_id,
                    nav_artifact_id,
                    Some(refusal),
                    "refused".to_string(),
                    explain_reasons,
                    assumptions,
                    observations.len() + invalid_satellite_time_sats.len(),
                    observations.len(),
                    invalid_satellite_time_sats
                        .iter()
                        .copied()
                        .map(|sat| (sat, MeasurementRejectReason::TimeInconsistency))
                        .collect(),
                ),
                obs,
                navigation,
                klobuchar,
                self.config.tropo_enable,
            ));
        }
        let eph_covered_count = observations
            .iter()
            .filter(|row| navigation.iter().any(|entry| entry.sat() == row.sat))
            .count();
        let solution = match self
            .solver
            .try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
                &observations,
                navigation,
                obs.t_rx_s.0,
                klobuchar,
            ) {
            Ok(solution) => solution,
            Err(refusal) => {
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "NAV_SOLVER_FAILED",
                    "nav solver failed to converge",
                ));
                let refusal_class = match refusal.kind {
                    PositionSolveRefusalKind::InsufficientObservations
                    | PositionSolveRefusalKind::InsufficientUsableSatellites
                    | PositionSolveRefusalKind::UnderdeterminedRaimExclusion => {
                        NavRefusalClass::InsufficientGeometry
                    }
                    PositionSolveRefusalKind::InvalidSatelliteTime => {
                        NavRefusalClass::InvalidSatelliteTime
                    }
                    PositionSolveRefusalKind::UnknownInterSystemTimeOffset => {
                        NavRefusalClass::MixedConstellationInput
                    }
                    PositionSolveRefusalKind::InvalidEphemeris => NavRefusalClass::InvalidEphemeris,
                    PositionSolveRefusalKind::SolverFailure => {
                        if eph_covered_count < 4 {
                            NavRefusalClass::InvalidEphemeris
                        } else if eph_covered_count < observations.len() {
                            NavRefusalClass::PartialDecodedNavigationState
                        } else {
                            NavRefusalClass::SolverFailure
                        }
                    }
                    PositionSolveRefusalKind::FilterDivergence(_) => NavRefusalClass::SolverFailure,
                };
                let solution_status = solver_refusal_status(refusal.kind, refusal_class);
                let mut rejected = refusal.rejected;
                rejected.extend(
                    invalid_satellite_time_sats
                        .iter()
                        .copied()
                        .map(|sat| (sat, MeasurementRejectReason::TimeInconsistency)),
                );
                let mut explain_reasons = vec![
                    "position_solver_failed".to_string(),
                    format!("ephemeris_covered_count={eph_covered_count}"),
                ];
                if let PositionSolveRefusalKind::FilterDivergence(reason) = refusal.kind {
                    explain_reasons.push(filter_divergence_explain_reason(reason));
                }
                if refusal.kind == PositionSolveRefusalKind::UnderdeterminedRaimExclusion {
                    explain_reasons.push("raim_exclusion_underdetermined".to_string());
                    if let Some((suspect_sat, _reason)) = rejected
                        .iter()
                        .find(|(_sat, reason)| *reason == MeasurementRejectReason::Outlier)
                    {
                        explain_reasons.push(format!("raim_suspect_prn={}", suspect_sat.prn));
                    }
                    explain_reasons
                        .push(format!("raim_usable_satellites={}", refusal.used_sat_count));
                }
                if refusal.kind == PositionSolveRefusalKind::UnknownInterSystemTimeOffset {
                    explain_reasons.push("unknown_inter_system_time_offset".to_string());
                    let constellations = rejected
                        .iter()
                        .filter(|(_sat, reason)| {
                            *reason == MeasurementRejectReason::TimeInconsistency
                        })
                        .map(|(sat, _reason)| format!("{:?}", sat.constellation))
                        .collect::<std::collections::BTreeSet<_>>()
                        .into_iter()
                        .collect::<Vec<_>>();
                    if !constellations.is_empty() {
                        explain_reasons.push(format!(
                            "unknown_time_offset_constellations={}",
                            constellations.join(",")
                        ));
                    }
                }
                let is_sparse_refusal = matches!(
                    refusal.kind,
                    PositionSolveRefusalKind::InsufficientObservations
                        | PositionSolveRefusalKind::InvalidSatelliteTime
                        | PositionSolveRefusalKind::UnknownInterSystemTimeOffset
                        | PositionSolveRefusalKind::InvalidEphemeris
                        | PositionSolveRefusalKind::InsufficientUsableSatellites
                        | PositionSolveRefusalKind::UnderdeterminedRaimExclusion
                );
                self.last_satellite_clock_suspect = None;
                self.satellite_clock_suspect_streak = 0;
                return Some(apply_atmosphere_explainability(
                    if is_sparse_refusal {
                        let solution = self.current_epoch_refusal(
                            obs,
                            source_observation_epoch_id,
                            nav_artifact_id,
                            Some(refusal_class),
                            "refused".to_string(),
                            explain_reasons,
                            assumptions,
                            refusal.sat_count + invalid_satellite_time_sats.len(),
                            refusal.used_sat_count,
                            rejected,
                        );
                        if solution_status == SolutionStatus::IntegrityFailed {
                            mark_integrity_failure(solution)
                        } else {
                            solution
                        }
                    } else {
                        let solution = self.reuse_previous_solution(
                            obs,
                            source_observation_epoch_id,
                            nav_artifact_id,
                            NavDecision {
                                status: solution_status,
                                refusal_class: Some(refusal_class),
                                explain_decision: "refused".to_string(),
                                explain_reasons,
                            },
                            assumptions,
                        );
                        if solution_status == SolutionStatus::IntegrityFailed {
                            mark_integrity_failure(solution)
                        } else {
                            solution
                        }
                    },
                    obs,
                    navigation,
                    klobuchar,
                    self.config.tropo_enable,
                ));
            }
        };
        let dt_s = self
            .last_solution
            .as_ref()
            .map(|last| (obs.t_rx_s.0 - last.t_rx_s.0).abs())
            .filter(|dt_s| dt_s.is_finite() && *dt_s > 0.0)
            .unwrap_or(1.0);
        let (_smoothed_clock_bias_s, clock_drift_s_per_s) =
            self.clock.update(solution.clock_bias_s, dt_s);
        let smoothed_position = self
            .position_smoother
            .as_mut()
            .map(|smoother| smoother.smooth_position_solution(obs.t_rx_s.0, &solution));
        let (ecef_x_m, ecef_y_m, ecef_z_m) = smoothed_position
            .as_ref()
            .map(|position| (position.ecef_x_m, position.ecef_y_m, position.ecef_z_m))
            .unwrap_or((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        let (latitude_deg, longitude_deg, altitude_m) =
            ecef_to_geodetic(ecef_x_m, ecef_y_m, ecef_z_m);
        self.last_ecef = Some((ecef_x_m, ecef_y_m, ecef_z_m));
        let inter_system_biases = solution.inter_system_biases.clone();
        let mut nav_epoch = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
            t_rx_s: obs.t_rx_s,
            source_time: obs.source_time,
            ecef_x_m: Meters(ecef_x_m),
            ecef_y_m: Meters(ecef_y_m),
            ecef_z_m: Meters(ecef_z_m),
            position_covariance_ecef_m2: smoothed_position
                .as_ref()
                .and_then(|position| position.position_covariance_ecef_m2)
                .or(solution.position_covariance_ecef_m2),
            latitude_deg,
            longitude_deg,
            altitude_m: Meters(altitude_m),
            clock_bias_s: Seconds(solution.clock_bias_s),
            clock_bias_m: Meters(solution.clock_bias_s * 299_792_458.0),
            clock_drift_s_per_s,
            pdop: solution.pdop,
            pre_fit_residual_rms_m: Some(Meters(solution.pre_fit_residual_rms_m)),
            post_fit_residual_rms_m: Some(Meters(solution.post_fit_residual_rms_m)),
            rms_m: Meters(solution.rms_m),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: SolutionValidity::Invalid,
            valid: true,
            processing_ms: None,
            sigma_h_m: solution.sigma_h_m.map(Meters),
            sigma_v_m: solution.sigma_v_m.map(Meters),
            residuals: solution
                .residuals
                .into_iter()
                .map(|(sat, residual_m, weight)| NavResidual {
                    sat,
                    residual_m: Meters(residual_m),
                    rejected: false,
                    weight: Some(weight),
                    reject_reason: None,
                })
                .chain(solution.rejected.into_iter().map(|(sat, reason)| NavResidual {
                    sat,
                    residual_m: Meters(0.0),
                    rejected: true,
                    weight: None,
                    reject_reason: Some(reason),
                }))
                .collect(),
            constellation_residual_rms: solution.constellation_residual_rms.clone(),
            isb: inter_system_biases,
            health: Vec::new(),
            sigma_e_m: smoothed_position
                .as_ref()
                .and_then(|position| position.sigma_e_m)
                .or(solution.sigma_e_m)
                .map(Meters),
            sigma_n_m: smoothed_position
                .as_ref()
                .and_then(|position| position.sigma_n_m)
                .or(solution.sigma_n_m)
                .map(Meters),
            sigma_u_m: smoothed_position
                .as_ref()
                .and_then(|position| position.sigma_u_m)
                .or(solution.sigma_u_m)
                .map(Meters),
            horizontal_error_ellipse_major_axis_m: smoothed_position
                .as_ref()
                .and_then(|position| position.horizontal_error_ellipse_major_axis_m)
                .or(solution.horizontal_error_ellipse_major_axis_m)
                .map(Meters),
            horizontal_error_ellipse_minor_axis_m: smoothed_position
                .as_ref()
                .and_then(|position| position.horizontal_error_ellipse_minor_axis_m)
                .or(solution.horizontal_error_ellipse_minor_axis_m)
                .map(Meters),
            horizontal_error_ellipse_azimuth_deg: smoothed_position
                .as_ref()
                .and_then(|position| position.horizontal_error_ellipse_azimuth_deg)
                .or(solution.horizontal_error_ellipse_azimuth_deg),
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::CodeOnly,
            uncertainty_class: NavUncertaintyClass::Unknown,
            assumptions: Some(assumptions.clone()),
            refusal_class: None,
            artifact_id: nav_artifact_id,
            source_observation_epoch_id,
            explain_decision: String::new(),
            explain_reasons: Vec::new(),
            provenance: None,
            sat_count: solution.sat_count,
            used_sat_count: solution.used_sat_count,
            rejected_sat_count: solution.rejected_sat_count,
            hdop: solution.hdop,
            vdop: solution.vdop,
            gdop: solution.gdop,
            tdop: solution.tdop,
            stability_signature: String::new(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        nav_epoch.sigma_h_m = smoothed_position
            .as_ref()
            .and_then(|position| position.sigma_h_m)
            .or(solution.sigma_h_m)
            .map(Meters);
        nav_epoch.sigma_v_m = smoothed_position
            .as_ref()
            .and_then(|position| position.sigma_v_m)
            .or(solution.sigma_v_m)
            .map(Meters);
        nav_epoch.residuals.extend(invalid_satellite_time_sats.iter().copied().map(|sat| {
            NavResidual {
                sat,
                residual_m: Meters(0.0),
                rejected: true,
                weight: None,
                reject_reason: Some(MeasurementRejectReason::TimeInconsistency),
            }
        }));
        nav_epoch.sat_count += invalid_satellite_time_sats.len();
        nav_epoch.rejected_sat_count += invalid_satellite_time_sats.len();

        if solution.covariance_symmetrized {
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::CovarianceSymmetrized);
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_COV_SYMM",
                "nav covariance symmetrized",
            ));
        }
        if solution.covariance_clamped {
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::CovarianceClamped {
                min_eigenvalue: 0.0,
            });
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_COV_CLAMP",
                "nav covariance clamped",
            ));
        }
        if let Some(max_var) = solution.covariance_max_variance {
            if max_var > 1e6 {
                nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::CovarianceDiverged {
                    max_variance: max_var,
                });
                nav_epoch = override_solution_status(nav_epoch, SolutionStatus::Diverged);
                nav_epoch.refusal_class = Some(NavRefusalClass::SolverFailure);
                nav_epoch.explain_decision = "diverged".to_string();
                if !nav_epoch
                    .explain_reasons
                    .iter()
                    .any(|reason| reason == "covariance_divergence")
                {
                    nav_epoch.explain_reasons.push("covariance_divergence".to_string());
                }
            }
        }

        let sat_count = observations.len();
        let provisional_status = if sat_count < 4 {
            SolutionStatus::Degraded
        } else {
            SolutionStatus::CodeOnly
        };
        nav_epoch.status = deterministic_solution_transition(
            self.last_solution.as_ref().map(|row| row.status),
            provisional_status,
            None,
            false,
        );

        let mut raim_explain_reasons = Vec::new();
        let mut common_anomaly_explain_reasons = Vec::new();
        let mut replay_timing_explain_reasons = Vec::new();
        let mut resolved_by_raim_exclusion = false;
        let mut clock_anomaly_explain_reasons = Vec::new();
        let mut constellation_clock_explain_reasons = Vec::new();
        let mut residual_whiteness_explain_reasons = Vec::new();
        if let Some(solution_separation) = solution.raim_solution_separation.as_ref() {
            raim_explain_reasons.push(format!(
                "raim_solution_separation_reference_satellites={}",
                solution_separation.reference_sat_count
            ));
            raim_explain_reasons.push(format!(
                "raim_solution_separation_compared_subsets={}",
                solution_separation.compared_subset_count()
            ));
            if let Some(max_subset) = solution_separation.max_separation() {
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Info,
                    "NAV_RAIM_SEPARATION",
                    format!(
                        "raim compared {} of {} subset solutions; max separation {:.2} m at PRN {}",
                        solution_separation.compared_subset_count(),
                        solution_separation.reference_sat_count,
                        max_subset.separation_m,
                        max_subset.excluded_sat.prn
                    ),
                ));
            }
        }
        if let Some(raim_fault_exclusion) = solution.raim_fault_exclusion {
            raim_explain_reasons.push("raim_fault_excluded".to_string());
            raim_explain_reasons
                .push(format!("raim_excluded_prn={}", raim_fault_exclusion.excluded_sat.prn));
            resolved_by_raim_exclusion = raim_fault_exclusion.improved();
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Info,
                "NAV_RAIM_SEPARATION",
                format!(
                    "raim excluded PRN {}: residual rms {:.2} m -> {:.2} m",
                    raim_fault_exclusion.excluded_sat.prn,
                    raim_fault_exclusion.pre_exclusion_rms_m,
                    raim_fault_exclusion.post_exclusion_rms_m
                ),
            ));
        }
        if let Some(raim_fault_detection) = solution.raim_fault_detection {
            if raim_fault_detection.status == RaimFaultDetectionStatus::FaultDetected {
                raim_explain_reasons.push("raim_fault_detected".to_string());
                if let Some(suspect_sat) = raim_fault_detection.suspect_sat {
                    let suspect_reason = format!("raim_suspect_prn={}", suspect_sat.prn);
                    raim_explain_reasons.push(suspect_reason);
                    if !resolved_by_raim_exclusion {
                        nav_epoch.status = SolutionStatus::Degraded;
                    }
                    self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                        bijux_gnss_core::api::DiagnosticSeverity::Warning,
                        "NAV_RAIM_SEPARATION",
                        format!(
                            "raim fault detected for PRN {}: {:.2} m > {:.2} m",
                            suspect_sat.prn,
                            raim_fault_detection.max_solution_separation_m,
                            raim_fault_detection.threshold_m
                        ),
                    ));
                }
            }
        }
        let (next_satellite_clock_suspect, next_satellite_clock_suspect_streak) =
            advance_satellite_clock_suspect_streak(
                self.last_satellite_clock_suspect,
                self.satellite_clock_suspect_streak,
                solution.raim_fault_detection,
                solution.raim_fault_exclusion,
            );
        if let Some(clock_anomaly) = detect_satellite_clock_anomaly(
            solution.raim_fault_detection,
            solution.raim_fault_exclusion,
            next_satellite_clock_suspect_streak,
        ) {
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::SatelliteClockAnomaly {
                sat: clock_anomaly.sat,
                persistent_suspect_epochs: clock_anomaly.persistent_suspect_epochs,
                max_solution_separation_m: clock_anomaly.max_solution_separation_m,
                separation_threshold_m: clock_anomaly.separation_threshold_m,
            });
            clock_anomaly_explain_reasons.push("satellite_clock_anomaly".to_string());
            clock_anomaly_explain_reasons
                .push(format!("satellite_clock_anomaly_prn={}", clock_anomaly.sat.prn));
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_SAT_CLOCK_ANOMALY",
                format!(
                    "satellite clock anomaly for PRN {}: unresolved RAIM suspect for {} epochs with solution separation {:.2} m above {:.2} m",
                    clock_anomaly.sat.prn,
                    clock_anomaly.persistent_suspect_epochs,
                    clock_anomaly.max_solution_separation_m,
                    clock_anomaly.separation_threshold_m
                ),
            ));
        }
        if let Some(common_anomaly) = detect_common_code_doppler_anomaly(
            self.last_solution.as_ref(),
            self.last_position_observations.as_deref(),
            &nav_epoch,
            &observations,
        ) {
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::CommonCodeDopplerAnomaly {
                common_code_step_m: common_anomaly.common_code_step_m,
                common_doppler_step_hz: common_anomaly.common_doppler_step_hz,
                matched_satellite_count: common_anomaly.matched_satellite_count,
                aligned_satellite_count: common_anomaly.aligned_satellite_count,
                code_step_threshold_m: common_anomaly.code_step_threshold_m,
                doppler_step_threshold_hz: common_anomaly.doppler_step_threshold_hz,
            });
            common_anomaly_explain_reasons.push("common_code_doppler_anomaly".to_string());
            common_anomaly_explain_reasons
                .push(format!("common_code_step_m={:.3}", common_anomaly.common_code_step_m));
            common_anomaly_explain_reasons.push(format!(
                "common_doppler_step_hz={:.3}",
                common_anomaly.common_doppler_step_hz
            ));
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_COMMON_CODE_DOPPLER_ANOMALY",
                format!(
                    "common code step {:.2} m and common Doppler step {:.2} Hz across {}/{} matched satellites",
                    common_anomaly.common_code_step_m,
                    common_anomaly.common_doppler_step_hz,
                    common_anomaly.aligned_satellite_count,
                    common_anomaly.matched_satellite_count
                ),
            ));
        }
        if let Some(replay_timing_anomaly) = detect_replay_timing_anomaly(
            self.last_solution.as_ref(),
            self.last_position_observations.as_deref(),
            &nav_epoch,
            &observations,
        ) {
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::ReplayTimingAnomaly {
                common_delay_step_m: replay_timing_anomaly.common_delay_step_m,
                centered_delay_rms_m: replay_timing_anomaly.centered_delay_rms_m,
                max_centered_delay_m: replay_timing_anomaly.max_centered_delay_m,
                matched_satellite_count: replay_timing_anomaly.matched_satellite_count,
                positive_step_satellite_count: replay_timing_anomaly.positive_step_satellite_count,
                common_delay_step_threshold_m: replay_timing_anomaly.common_delay_step_threshold_m,
                centered_delay_rms_threshold_m: replay_timing_anomaly
                    .centered_delay_rms_threshold_m,
            });
            replay_timing_explain_reasons.push("replay_timing_anomaly".to_string());
            replay_timing_explain_reasons.push(format!(
                "common_delay_step_m={:.3}",
                replay_timing_anomaly.common_delay_step_m
            ));
            replay_timing_explain_reasons.push(format!(
                "centered_delay_rms_m={:.3}",
                replay_timing_anomaly.centered_delay_rms_m
            ));
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_REPLAY_TIMING_ANOMALY",
                format!(
                    "positive code-delay steps suggest replayed timing: median {:.2} m, centered rms {:.2} m, max {:.2} m across {}/{} matched satellites; clock step {:.2} m",
                    replay_timing_anomaly.common_delay_step_m,
                    replay_timing_anomaly.centered_delay_rms_m,
                    replay_timing_anomaly.max_centered_delay_m,
                    replay_timing_anomaly.positive_step_satellite_count,
                    replay_timing_anomaly.matched_satellite_count,
                    replay_timing_anomaly.clock_step_m
                ),
            ));
        }

        let sanity_events = check_nav_solution_sanity(self.last_solution.as_ref(), &nav_epoch);
        if sanity_events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            nav_epoch = override_solution_status(nav_epoch, SolutionStatus::Diverged);
            nav_epoch.refusal_class = Some(NavRefusalClass::InconsistentObservations);
            nav_epoch.explain_decision = "diverged".to_string();
            if !nav_epoch
                .explain_reasons
                .iter()
                .any(|reason| reason == "navigation_solution_sanity_failed")
            {
                nav_epoch
                    .explain_reasons
                    .push("navigation_solution_sanity_failed".to_string());
            }
            for event in sanity_events {
                self.runtime.logger.event(&event);
            }
        } else {
            nav_epoch.valid = is_solution_valid(nav_epoch.status);
        }
        if let Some(impossible_geometry) = solution.impossible_geometry {
            nav_epoch.health.push(bijux_gnss_core::api::NavHealthEvent::ImpossibleGeometry {
                receiver_radius_m: impossible_geometry.receiver_radius_m,
                altitude_m: impossible_geometry.altitude_m,
                used_satellite_count: impossible_geometry.used_satellite_count,
                min_receiver_radius_m: impossible_geometry.min_receiver_radius_m,
                max_receiver_radius_m: impossible_geometry.max_receiver_radius_m,
                min_altitude_m: impossible_geometry.min_altitude_m,
                max_altitude_m: impossible_geometry.max_altitude_m,
            });
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_IMPOSSIBLE_GEOMETRY",
                format!(
                    "impossible terrestrial geometry: receiver radius {:.2} m, altitude {:.2} m across {} satellites",
                    impossible_geometry.receiver_radius_m,
                    impossible_geometry.altitude_m,
                    impossible_geometry.used_satellite_count
                ),
            ));
            nav_epoch = policy_refusal_epoch(
                nav_epoch,
                self.last_solution.as_ref().map(|row| row.status),
                NavRefusalClass::InconsistentObservations,
                impossible_geometry_explain_reasons(impossible_geometry),
            );
            nav_epoch = mark_integrity_failure(nav_epoch);
        }
        let constellation_clock_inconsistencies = detect_constellation_clock_inconsistencies(
            self.last_solution.as_ref(),
            &nav_epoch,
        );
        if !constellation_clock_inconsistencies.is_empty() {
            for inconsistency in &constellation_clock_inconsistencies {
                nav_epoch
                    .health
                    .push(bijux_gnss_core::api::NavHealthEvent::ConstellationClockInconsistency {
                        constellation: inconsistency.constellation,
                        previous_bias_s: inconsistency.previous_bias_s,
                        current_bias_s: inconsistency.current_bias_s,
                        bias_step_m: inconsistency.bias_step_m,
                        bias_step_threshold_m: inconsistency.bias_step_threshold_m,
                        supporting_satellite_count: inconsistency.supporting_satellite_count,
                    });
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "NAV_CONSTELLATION_CLOCK_INCONSISTENCY",
                    format!(
                        "{:?} clock offset jumped by {:.2} m across {} satellites",
                        inconsistency.constellation,
                        inconsistency.bias_step_m,
                        inconsistency.supporting_satellite_count
                    ),
                ));
            }
            constellation_clock_explain_reasons =
                constellation_clock_inconsistency_explain_reasons(
                    &constellation_clock_inconsistencies,
                );
            nav_epoch = policy_refusal_epoch(
                nav_epoch,
                self.last_solution.as_ref().map(|row| row.status),
                NavRefusalClass::InconsistentObservations,
                constellation_clock_explain_reasons.clone(),
            );
            nav_epoch = mark_integrity_failure(nav_epoch);
        }
        nav_epoch.quality = nav_epoch.status.quality_flag();

        let (innovation_rms, norm_rms, norm_max, predicted_var, observed_var) =
            innovation_stats(&nav_epoch.residuals);
        nav_epoch.innovation_rms_m = innovation_rms;
        nav_epoch.normalized_innovation_rms = norm_rms;
        nav_epoch.normalized_innovation_max = norm_max;
        nav_epoch.ekf_predicted_variance = predicted_var;
        nav_epoch.ekf_observed_variance = observed_var;
        let residual_temporal_correlation_evidence =
            detect_residual_temporal_correlation(self.last_solution.as_ref(), &nav_epoch);
        let next_residual_whiteness_suspect_streak = advance_residual_whiteness_suspect_streak(
            self.residual_whiteness_suspect_streak,
            residual_temporal_correlation_evidence,
        );
        if let Some(evidence) = residual_temporal_correlation_evidence {
            let correlation = classify_residual_temporal_correlation(
                evidence,
                next_residual_whiteness_suspect_streak,
            );
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch
                .health
                .push(bijux_gnss_core::api::NavHealthEvent::ResidualTemporalCorrelation {
                    lag1_correlation: correlation.lag1_correlation,
                    correlation_threshold: correlation.correlation_threshold,
                    matched_satellite_count: correlation.matched_satellite_count,
                    previous_centered_rms_m: correlation.previous_centered_rms_m,
                    current_centered_rms_m: correlation.current_centered_rms_m,
                    persistent_suspect_epochs: correlation.persistent_suspect_epochs,
                });
            residual_whiteness_explain_reasons = residual_temporal_correlation_explain_reasons(
                correlation,
            );
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_RESIDUAL_WHITENESS",
                format!(
                    "residual lag-1 correlation {:.3} across {} matched satellites (centered rms {:.2}/{:.2} m, streak {})",
                    correlation.lag1_correlation,
                    correlation.matched_satellite_count,
                    correlation.previous_centered_rms_m,
                    correlation.current_centered_rms_m,
                    correlation.persistent_suspect_epochs
                ),
            ));
            if residual_temporal_correlation_is_persistent(
                correlation.persistent_suspect_epochs,
            ) {
                nav_epoch = policy_refusal_epoch(
                    nav_epoch,
                    self.last_solution.as_ref().map(|row| row.status),
                    NavRefusalClass::InconsistentObservations,
                    residual_whiteness_explain_reasons.clone(),
                );
                nav_epoch = mark_integrity_failure(nav_epoch);
            }
        }
        nav_epoch.validity = classify_validity(&nav_epoch);
        nav_epoch.lifecycle_state = lifecycle_state_from_status(nav_epoch.status);
        nav_epoch.uncertainty_class = uncertainty_class_from_solution(&nav_epoch);
        nav_epoch.provenance = Some(build_provenance(&self.config, &observations, &nav_epoch));
        let mut decision = decision_for_solution(&nav_epoch);
        let policy_violations =
            scientific_prerequisite_violations(&nav_epoch, &observations, &self.config);
        if decision.explain_decision == "accepted" && !policy_violations.is_empty() {
            let refusal_class = if only_geometry_policy_violations(&policy_violations) {
                NavRefusalClass::InsufficientGeometry
            } else {
                NavRefusalClass::ScientificPrerequisitesTooWeak
            };
            nav_epoch = policy_refusal_epoch(
                nav_epoch,
                self.last_solution.as_ref().map(|row| row.status),
                refusal_class,
                policy_violations.clone(),
            );
            decision = NavDecision {
                status: nav_epoch.status,
                refusal_class: Some(refusal_class),
                explain_decision: "refused".to_string(),
                explain_reasons: policy_violations,
            };
        }
        nav_epoch.explain_decision = decision.explain_decision;
        nav_epoch.explain_reasons = decision.explain_reasons;
        nav_epoch.refusal_class = decision.refusal_class;
        apply_atmosphere_explainability_in_place(
            &mut nav_epoch,
            obs,
            navigation,
            klobuchar,
            self.config.tropo_enable,
        );
        for reason in raim_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        for reason in clock_anomaly_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        for reason in common_anomaly_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        for reason in replay_timing_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        for reason in constellation_clock_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        for reason in residual_whiteness_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        nav_epoch.stability_signature = nav_output_stability_signature(&nav_epoch);
        let protection_levels = nav_epoch.position_covariance_ecef_m2.and_then(|covariance| {
            formal_protection_levels(
                [nav_epoch.ecef_x_m.0, nav_epoch.ecef_y_m.0, nav_epoch.ecef_z_m.0],
                covariance,
            )
        });
        nav_epoch.integrity_hpl_m = protection_levels.map(|levels| levels.horizontal_m);
        nav_epoch.integrity_vpl_m = protection_levels.map(|levels| levels.vertical_m);
        if !is_solution_valid(nav_epoch.status) {
            nav_epoch.integrity_hpl_m = None;
            nav_epoch.integrity_vpl_m = None;
        }

        if let Some(rms) = nav_epoch.innovation_rms_m {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Info,
                "NAV_INNOVATION_RMS",
                format!("innovation rms {:.3} m", rms),
            ));
        }

        self.last_solution = Some(nav_epoch.clone());
        self.last_position_observations = Some(observations.clone());
        self.last_satellite_clock_suspect = next_satellite_clock_suspect;
        self.satellite_clock_suspect_streak = next_satellite_clock_suspect_streak;
        self.residual_whiteness_suspect_streak = next_residual_whiteness_suspect_streak;
        Some(nav_epoch)
    }

    fn reuse_previous_solution(
        &mut self,
        obs: &ObsEpoch,
        source_observation_epoch_id: String,
        nav_artifact_id: String,
        decision: NavDecision,
        assumptions: NavAssumptions,
    ) -> NavSolutionEpoch {
        self.last_satellite_clock_suspect = None;
        self.satellite_clock_suspect_streak = 0;
        let mut solution = self.last_solution.clone().unwrap_or_else(|| {
            invalid_solution_epoch(
                obs,
                source_observation_epoch_id.clone(),
                nav_artifact_id.clone(),
                decision.refusal_class,
                decision.explain_decision.clone(),
                decision.explain_reasons.clone(),
                assumptions.clone(),
            )
        });
        solution.epoch = bijux_gnss_core::api::Epoch { index: obs.epoch_idx };
        solution.t_rx_s = obs.t_rx_s;
        solution.source_time = obs.source_time;
        solution.status = deterministic_solution_transition(
            self.last_solution.as_ref().map(|row| row.status),
            decision.status,
            decision.refusal_class,
            true,
        );
        solution.lifecycle_state = lifecycle_state_from_status(solution.status);
        solution.quality = solution.status.quality_flag();
        solution.valid = is_solution_valid(solution.status);
        solution.validity = SolutionValidity::Coarse;
        solution.processing_ms = None;
        solution.residuals.clear();
        solution.assumptions = Some(assumptions);
        solution.source_observation_epoch_id = source_observation_epoch_id;
        solution.artifact_id = nav_artifact_id;
        solution.explain_decision = decision.explain_decision;
        solution.explain_reasons = decision.explain_reasons;
        solution.refusal_class = decision.refusal_class;
        if matches!(decision.status, SolutionStatus::Diverged | SolutionStatus::IntegrityFailed) {
            solution = override_solution_status(solution, decision.status);
        }
        solution.stability_signature = nav_output_stability_signature(&solution);
        solution
    }

    fn current_epoch_refusal(
        &self,
        obs: &ObsEpoch,
        source_observation_epoch_id: String,
        nav_artifact_id: String,
        refusal_class: Option<NavRefusalClass>,
        explain_decision: String,
        explain_reasons: Vec<String>,
        assumptions: NavAssumptions,
        sat_count: usize,
        used_sat_count: usize,
        rejected: Vec<(bijux_gnss_core::api::SatId, MeasurementRejectReason)>,
    ) -> NavSolutionEpoch {
        sparse_navigation_refusal_epoch(
            obs,
            source_observation_epoch_id,
            nav_artifact_id,
            refusal_class,
            explain_decision,
            explain_reasons,
            assumptions,
            sat_count,
            used_sat_count,
            rejected,
        )
    }
}

fn navigation_supported_constellation(constellation: Constellation) -> bool {
    matches!(
        constellation,
        Constellation::Gps
            | Constellation::Galileo
            | Constellation::Glonass
            | Constellation::Beidou
    )
}

fn apply_atmosphere_explainability(
    mut solution: NavSolutionEpoch,
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) -> NavSolutionEpoch {
    apply_atmosphere_explainability_in_place(&mut solution, obs, navigation, klobuchar, tropo_enabled);
    solution
}

fn apply_atmosphere_explainability_in_place(
    solution: &mut NavSolutionEpoch,
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) {
    for ionosphere_reason in ionosphere_explain_reasons(obs, navigation, klobuchar) {
        if !solution.explain_reasons.iter().any(|existing| existing == ionosphere_reason) {
            solution.explain_reasons.push(ionosphere_reason.to_string());
        }
    }
    let troposphere_reason = if tropo_enabled {
        "troposphere_correction=saastamoinen"
    } else {
        "troposphere_uncorrected"
    };
    if !solution.explain_reasons.iter().any(|existing| existing == troposphere_reason) {
        solution.explain_reasons.push(troposphere_reason.to_string());
    }
}

fn ionosphere_explain_reasons(
    obs: &ObsEpoch,
    navigation: &[PositionBroadcastNavigation],
    klobuchar: Option<&KlobucharCoefficients>,
) -> Vec<&'static str> {
    let has_gps_observations = obs
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.sat.constellation == Constellation::Gps);
    let has_galileo_observations = obs
        .sats
        .iter()
        .any(|satellite| satellite.signal_id.sat.constellation == Constellation::Galileo);
    let has_galileo_navigation = navigation
        .iter()
        .any(|entry| matches!(entry, PositionBroadcastNavigation::Galileo(_)));

    let mut reasons = Vec::new();
    if klobuchar.is_some() && has_gps_observations {
        reasons.push("ionosphere_correction=klobuchar_broadcast");
    }
    if has_galileo_observations && has_galileo_navigation {
        reasons.push("ionosphere_correction=galileo_nequick");
    }
    if reasons.is_empty() {
        reasons.push("ionosphere_uncorrected");
    }
    reasons
}

fn position_solution_smoother_config(
    motion_class: NavigationMotionClass,
) -> PositionSolutionSmootherConfig {
    PositionSolutionSmootherConfig::for_motion_class(match motion_class {
        NavigationMotionClass::Static => PositionFilterMotionClass::Static,
        NavigationMotionClass::Pedestrian => PositionFilterMotionClass::Pedestrian,
        NavigationMotionClass::Vehicle => PositionFilterMotionClass::Vehicle,
        NavigationMotionClass::Airborne => PositionFilterMotionClass::Airborne,
    })
}

impl bijux_gnss_nav::api::NavEngine for Navigation {
    fn update(
        &mut self,
        obs: &bijux_gnss_core::api::ObsEpochV1,
    ) -> bijux_gnss_core::api::NavSolutionEpochV1 {
        let solution = self.solve_epoch(&obs.payload, &[]).unwrap_or_else(|| {
            invalid_solution_epoch(
                &obs.payload,
                source_observation_epoch_id(&obs.payload),
                nav_artifact_id(obs.payload.epoch_idx, &source_observation_epoch_id(&obs.payload)),
                Some(NavRefusalClass::SolverFailure),
                "refused".to_string(),
                vec!["runtime_update_solver_failure".to_string()],
                nav_assumptions(0),
            )
        });
        bijux_gnss_core::api::NavSolutionEpochV1 { header: obs.header.clone(), payload: solution }
    }
}

fn invalid_solution_epoch(
    obs: &ObsEpoch,
    source_observation_epoch_id: String,
    artifact_id: String,
    refusal_class: Option<NavRefusalClass>,
    explain_decision: String,
    explain_reasons: Vec<String>,
    assumptions: NavAssumptions,
) -> NavSolutionEpoch {
    let status = refusal_class.map(refusal_status).unwrap_or(SolutionStatus::Unavailable);
    let mut solution = NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        source_time: obs.source_time,
        ecef_x_m: Meters(0.0),
        ecef_y_m: Meters(0.0),
        ecef_z_m: Meters(0.0),
        position_covariance_ecef_m2: None,
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: Meters(0.0),
        clock_bias_s: Seconds(0.0),
        clock_bias_m: Meters(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 0.0,
        pre_fit_residual_rms_m: None,
        post_fit_residual_rms_m: None,
        rms_m: Meters(0.0),
        status,
        quality: status.quality_flag(),
        validity: SolutionValidity::Invalid,
        valid: false,
        processing_ms: None,
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_e_m: None,
        sigma_n_m: None,
        sigma_u_m: None,
        horizontal_error_ellipse_major_axis_m: None,
        horizontal_error_ellipse_minor_axis_m: None,
        horizontal_error_ellipse_azimuth_deg: None,
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: None,
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: None,
        ekf_condition_number: None,
        ekf_whiteness_ratio: None,
        ekf_predicted_variance: None,
        ekf_observed_variance: None,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: lifecycle_state_from_status(status),
        uncertainty_class: NavUncertaintyClass::Unknown,
        assumptions: Some(assumptions),
        refusal_class,
        artifact_id,
        source_observation_epoch_id,
        explain_decision,
        explain_reasons,
        provenance: None,
        sat_count: 0,
        used_sat_count: 0,
        rejected_sat_count: 0,
        hdop: None,
        vdop: None,
        gdop: None,
        tdop: None,
        stability_signature: String::new(),
        stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    };
    solution.stability_signature = nav_output_stability_signature(&solution);
    solution
}

fn sparse_navigation_refusal_epoch(
    obs: &ObsEpoch,
    source_observation_epoch_id: String,
    artifact_id: String,
    refusal_class: Option<NavRefusalClass>,
    explain_decision: String,
    explain_reasons: Vec<String>,
    assumptions: NavAssumptions,
    sat_count: usize,
    used_sat_count: usize,
    rejected: Vec<(bijux_gnss_core::api::SatId, MeasurementRejectReason)>,
) -> NavSolutionEpoch {
    let mut solution = invalid_solution_epoch(
        obs,
        source_observation_epoch_id,
        artifact_id,
        refusal_class,
        explain_decision,
        explain_reasons,
        assumptions,
    );
    solution.residuals = rejected
        .into_iter()
        .map(|(sat, reject_reason)| NavResidual {
            sat,
            residual_m: Meters(0.0),
            rejected: true,
            weight: None,
            reject_reason: Some(reject_reason),
        })
        .collect();
    solution.sat_count = sat_count;
    solution.used_sat_count = used_sat_count;
    solution.rejected_sat_count = solution.residuals.len();
    solution.stability_signature = nav_output_stability_signature(&solution);
    solution
}

fn policy_refusal_epoch(
    mut solution: NavSolutionEpoch,
    previous_status: Option<SolutionStatus>,
    refusal_class: NavRefusalClass,
    explain_reasons: Vec<String>,
) -> NavSolutionEpoch {
    solution.ecef_x_m = Meters(0.0);
    solution.ecef_y_m = Meters(0.0);
    solution.ecef_z_m = Meters(0.0);
    solution.latitude_deg = 0.0;
    solution.longitude_deg = 0.0;
    solution.altitude_m = Meters(0.0);
    solution.position_covariance_ecef_m2 = None;
    solution.clock_bias_s = Seconds(0.0);
    solution.clock_bias_m = Meters(0.0);
    solution.clock_drift_s_per_s = 0.0;
    solution.sigma_e_m = None;
    solution.sigma_n_m = None;
    solution.sigma_u_m = None;
    solution.horizontal_error_ellipse_major_axis_m = None;
    solution.horizontal_error_ellipse_minor_axis_m = None;
    solution.horizontal_error_ellipse_azimuth_deg = None;
    solution.sigma_h_m = None;
    solution.sigma_v_m = None;
    solution.status = deterministic_solution_transition(
        previous_status,
        solution.status,
        Some(refusal_class),
        false,
    );
    solution.lifecycle_state = lifecycle_state_from_status(solution.status);
    solution.valid = false;
    solution.validity = SolutionValidity::Invalid;
    solution.quality = solution.status.quality_flag();
    solution.refusal_class = Some(refusal_class);
    solution.explain_decision = "refused".to_string();
    solution.explain_reasons = explain_reasons;
    solution
}

fn nav_output_stability_signature(solution: &NavSolutionEpoch) -> String {
    let refusal = solution
        .refusal_class
        .map(|value| format!("{value:?}"))
        .unwrap_or_else(|| "None".to_string());
    format!(
        "navsig:v{}:epoch={}:src={}:status={:?}:lifecycle={:?}:valid={}:sat={}:used={}:rej={}:pdop={:.3}:hdop={}:vdop={}:gdop={}:tdop={}:rms={:.3}:refusal={}:decision={}",
        NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        solution.epoch.index,
        format!(
            "{}@{}",
            short_id(&solution.source_observation_epoch_id),
            solution.source_time.sample_index
        ),
        solution.status,
        solution.lifecycle_state,
        solution.valid,
        solution.sat_count,
        solution.used_sat_count,
        solution.rejected_sat_count,
        solution.pdop,
        format_optional_nav_dop(solution.hdop),
        format_optional_nav_dop(solution.vdop),
        format_optional_nav_dop(solution.gdop),
        format_optional_nav_dop(solution.tdop),
        solution.rms_m.0,
        refusal,
        solution.explain_decision
    )
}

fn format_optional_nav_dop(value: Option<f64>) -> String {
    value.map(|dop| format!("{dop:.3}")).unwrap_or_else(|| "na".to_string())
}

fn source_observation_epoch_id(obs: &ObsEpoch) -> String {
    obs.manifest
        .as_ref()
        .map(|manifest| manifest.epoch_id.clone())
        .unwrap_or_else(|| obs_epoch_stability_key(obs))
}

fn nav_artifact_id(epoch_idx: u64, source_observation_epoch_id: &str) -> String {
    format!("nav-epoch-{epoch_idx:010}-{}", short_id(source_observation_epoch_id))
}

fn short_id(value: &str) -> String {
    value.chars().take(16).collect()
}

fn nav_assumptions(ephemeris_count: usize) -> NavAssumptions {
    let ephemeris_completeness = if ephemeris_count == 0 {
        "none"
    } else if ephemeris_count < 4 {
        "partial"
    } else {
        "sufficient"
    };
    NavAssumptions {
        time_system: "gps".to_string(),
        reference_frame: "ecef_wgs84".to_string(),
        clock_model: "receiver_clock_bias_drift_linear".to_string(),
        ephemeris_source: "broadcast_lnav".to_string(),
        frame_decode_mode: "lnav".to_string(),
        ephemeris_completeness: ephemeris_completeness.to_string(),
        ephemeris_count,
    }
}

fn deterministic_solution_transition(
    previous: Option<SolutionStatus>,
    proposed: SolutionStatus,
    refusal_class: Option<NavRefusalClass>,
    reused_previous: bool,
) -> SolutionStatus {
    if refusal_class.is_some() && !reused_previous {
        return refusal_status(refusal_class.expect("guarded refusal class"));
    }
    if refusal_class.is_some() && reused_previous {
        return SolutionStatus::Degraded;
    }
    match (previous, proposed) {
        (_, SolutionStatus::Invalid) => SolutionStatus::Refused,
        (_, SolutionStatus::Unavailable)
        | (_, SolutionStatus::Refused)
        | (_, SolutionStatus::IntegrityFailed)
        | (_, SolutionStatus::Diverged) => proposed,
        (Some(SolutionStatus::Converged), SolutionStatus::CodeOnly)
        | (Some(SolutionStatus::CodeOnly), SolutionStatus::CodeOnly)
        | (Some(SolutionStatus::Fixed), SolutionStatus::CodeOnly)
        | (Some(SolutionStatus::Float), SolutionStatus::CodeOnly)
        | (Some(SolutionStatus::Converged), SolutionStatus::Coarse)
        | (Some(SolutionStatus::Fixed), SolutionStatus::Coarse) => SolutionStatus::Degraded,
        (_, other) => other,
    }
}

fn refusal_status(refusal_class: NavRefusalClass) -> SolutionStatus {
    match refusal_class {
        NavRefusalClass::UnsupportedConstellation
        | NavRefusalClass::MixedConstellationInput
        | NavRefusalClass::InvalidEphemeris
        | NavRefusalClass::PartialDecodedNavigationState => SolutionStatus::Unavailable,
        NavRefusalClass::InsufficientGeometry
        | NavRefusalClass::InvalidSatelliteTime
        | NavRefusalClass::InconsistentObservations
        | NavRefusalClass::ScientificPrerequisitesTooWeak
        | NavRefusalClass::SolverFailure => SolutionStatus::Refused,
    }
}

fn solver_refusal_status(
    refusal_kind: PositionSolveRefusalKind,
    refusal_class: NavRefusalClass,
) -> SolutionStatus {
    match refusal_kind {
        PositionSolveRefusalKind::UnderdeterminedRaimExclusion => {
            SolutionStatus::IntegrityFailed
        }
        PositionSolveRefusalKind::FilterDivergence(_) => SolutionStatus::Diverged,
        PositionSolveRefusalKind::SolverFailure
            if refusal_class == NavRefusalClass::SolverFailure =>
        {
            SolutionStatus::Diverged
        }
        _ => refusal_status(refusal_class),
    }
}

fn filter_divergence_explain_reason(reason: PositionFilterDivergenceReason) -> String {
    let label = match reason {
        PositionFilterDivergenceReason::InnovationGrowth => "innovation_growth",
        PositionFilterDivergenceReason::CovarianceCollapse => "covariance_collapse",
        PositionFilterDivergenceReason::CovarianceDivergence => "covariance_divergence",
        PositionFilterDivergenceReason::ResidualExplosion => "residual_explosion",
    };
    format!("filter_divergence={label}")
}

fn override_solution_status(
    mut solution: NavSolutionEpoch,
    status: SolutionStatus,
) -> NavSolutionEpoch {
    solution.status = status;
    solution.lifecycle_state = lifecycle_state_from_status(status);
    solution.quality = status.quality_flag();
    solution.valid = is_solution_valid(status);
    solution.validity = if solution.valid {
        solution.validity
    } else {
        SolutionValidity::Invalid
    };
    solution
}

fn mark_integrity_failure(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
    solution = override_solution_status(solution, SolutionStatus::IntegrityFailed);
    solution.explain_decision = "integrity_failed".to_string();
    solution
}

fn lifecycle_state_from_status(status: SolutionStatus) -> NavLifecycleState {
    match status {
        SolutionStatus::Invalid => NavLifecycleState::Invalid,
        SolutionStatus::Unavailable => NavLifecycleState::Unavailable,
        SolutionStatus::Refused => NavLifecycleState::Refused,
        SolutionStatus::Held => NavLifecycleState::Held,
        SolutionStatus::Degraded => NavLifecycleState::Degraded,
        SolutionStatus::IntegrityFailed => NavLifecycleState::IntegrityFailed,
        SolutionStatus::Diverged => NavLifecycleState::Diverged,
        SolutionStatus::CodeOnly => NavLifecycleState::CodeOnly,
        SolutionStatus::Coarse => NavLifecycleState::Coarse,
        SolutionStatus::Converged => NavLifecycleState::Converged,
        SolutionStatus::Float => NavLifecycleState::Float,
        SolutionStatus::Fixed => NavLifecycleState::Fixed,
    }
}

fn uncertainty_class_from_solution(solution: &NavSolutionEpoch) -> NavUncertaintyClass {
fn precision_reporting_policy(solution: &NavSolutionEpoch) -> PrecisionReportingPolicy {
    match solution.status {
        SolutionStatus::Fixed if has_validated_centimeter_precision_support(solution) => {
            PrecisionReportingPolicy {
                horizontal_sigma_floor_m: FIXED_VALIDATED_SIGMA_H_FLOOR_M,
                vertical_sigma_floor_m: FIXED_VALIDATED_SIGMA_V_FLOOR_M,
                low_uncertainty_allowed: true,
                explain_reason: "precision_floor=validated_fixed_solution_minimum",
            }
        }
        SolutionStatus::Fixed => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: FIXED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: FIXED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: true,
            explain_reason: "precision_floor=fixed_solution_without_validated_support",
        },
        SolutionStatus::Float => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: FLOAT_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: FLOAT_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: true,
            explain_reason: "precision_floor=float_solution_without_integer_fix",
        },
        SolutionStatus::Converged => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: CONVERGED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: CONVERGED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: true,
            explain_reason: "precision_floor=code_navigation_without_carrier_ambiguity",
        },
        SolutionStatus::CodeOnly | SolutionStatus::Coarse => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: CODE_ONLY_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: CODE_ONLY_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=code_navigation_without_carrier_ambiguity",
        },
        SolutionStatus::Held | SolutionStatus::Degraded => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: DEGRADED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: DEGRADED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=degraded_navigation_precision",
        },
        SolutionStatus::Invalid
        | SolutionStatus::Unavailable
        | SolutionStatus::Refused
        | SolutionStatus::IntegrityFailed
        | SolutionStatus::Diverged => PrecisionReportingPolicy {
            horizontal_sigma_floor_m: DEGRADED_SIGMA_H_FLOOR_M,
            vertical_sigma_floor_m: DEGRADED_SIGMA_V_FLOOR_M,
            low_uncertainty_allowed: false,
            explain_reason: "precision_floor=invalid_navigation_solution",
        },
    }
}

fn has_validated_centimeter_precision_support(solution: &NavSolutionEpoch) -> bool {
    let corrections_supported = solution.explain_reasons.iter().any(|reason| {
        reason.starts_with("ionosphere_correction=") && reason != "ionosphere_uncorrected"
    }) && solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "troposphere_correction=saastamoinen");
    let validation_supported = solution.valid
        && solution.refusal_class.is_none()
        && solution.integrity_hpl_m.is_some()
        && solution.integrity_vpl_m.is_some()
        && !solution.health.iter().any(|event| matches!(event, bijux_gnss_core::api::NavHealthEvent::CovarianceDiverged { .. } | bijux_gnss_core::api::NavHealthEvent::CommonCodeDopplerAnomaly { .. } | bijux_gnss_core::api::NavHealthEvent::ReplayTimingAnomaly { .. } | bijux_gnss_core::api::NavHealthEvent::ConstellationClockInconsistency { .. } | bijux_gnss_core::api::NavHealthEvent::ResidualTemporalCorrelation { .. } | bijux_gnss_core::api::NavHealthEvent::ImpossibleGeometry { .. } | bijux_gnss_core::api::NavHealthEvent::SatelliteClockAnomaly { .. }));
    corrections_supported && validation_supported
}
    let sigma = solution
        .sigma_h_m
        .map(|value| value.0)
        .or(solution.innovation_rms_m)
        .unwrap_or(f64::INFINITY);
    if !sigma.is_finite() {
        NavUncertaintyClass::Unknown
    } else if sigma <= 3.0 {
        NavUncertaintyClass::Low
    } else if sigma <= 10.0 {
        NavUncertaintyClass::Medium
    } else {
        NavUncertaintyClass::High
    }
}

fn scientific_prerequisite_violations(
    solution: &NavSolutionEpoch,
    observations: &[PositionObservation],
    config: &ReceiverPipelineConfig,
) -> Vec<String> {
    let mut reasons = Vec::new();
    let thresholds = &config.science_thresholds;
    let mean_cn0 = if observations.is_empty() {
        None
    } else {
        Some(observations.iter().map(|row| row.cn0_dbhz).sum::<f64>() / observations.len() as f64)
    };
    if let Some(value) = mean_cn0 {
        if value < thresholds.min_mean_cn0_dbhz {
            reasons.push(format!(
                "mean_cn0_below_threshold:{value:.2}<{:.2}",
                thresholds.min_mean_cn0_dbhz
            ));
        }
    }
    reasons.extend(geometry_threshold_violations(solution, thresholds));
    if solution.rms_m.0 > thresholds.max_residual_rms_m {
        reasons.push(format!(
            "residual_rms_above_threshold:{:.3}>{:.3}",
            solution.rms_m.0, thresholds.max_residual_rms_m
        ));
    }
    reasons
}

fn geometry_threshold_violations(
    solution: &NavSolutionEpoch,
    thresholds: &crate::engine::receiver_config::ScienceThresholdsConfig,
) -> Vec<String> {
    let mut reasons = Vec::new();
    if solution.pdop > thresholds.max_pdop {
        reasons
            .push(format!("pdop_above_threshold:{:.3}>{:.3}", solution.pdop, thresholds.max_pdop));
    }
    if solution.gdop.unwrap_or(f64::INFINITY) > thresholds.max_gdop {
        reasons.push(format!(
            "gdop_above_threshold:{:.3}>{:.3}",
            solution.gdop.unwrap_or(f64::INFINITY),
            thresholds.max_gdop
        ));
    }
    if solution.used_sat_count < thresholds.min_used_satellites {
        reasons.push(format!(
            "used_satellites_below_threshold:{}<{}",
            solution.used_sat_count, thresholds.min_used_satellites
        ));
    }
    reasons
}

fn geometry_violation_reason(reason: &str) -> bool {
    reason.starts_with("pdop_above_threshold:")
        || reason.starts_with("gdop_above_threshold:")
        || reason.starts_with("used_satellites_below_threshold:")
}

fn only_geometry_policy_violations(reasons: &[String]) -> bool {
    !reasons.is_empty() && reasons.iter().all(|reason| geometry_violation_reason(reason))
}

fn prepare_position_observation(
    config: &ReceiverPipelineConfig,
    obs: &ObsEpoch,
    sat: &ObsSatellite,
    navigation: &[PositionBroadcastNavigation],
    last_ecef: Option<(f64, f64, f64)>,
) -> PositionObservationPreparation {
    let mut observation = PositionObservation {
        sat: sat.signal_id.sat,
        pseudorange_m: sat.pseudorange_m.0,
        doppler_hz: Some(sat.doppler_hz.0),
        doppler_var_hz2: Some(sat.doppler_var_hz2),
        cn0_dbhz: sat.cn0_dbhz,
        elevation_deg: sat.elevation_deg,
        weight: sat.weight.unwrap_or(1.0),
        gps_receive_time: obs.gps_time(),
        signal_timing: sat.timing,
        signal_id: Some(sat.signal_id),
    };
    if !position_observation_has_valid_satellite_time(&observation, obs.t_rx_s.0) {
        return PositionObservationPreparation::InvalidSatelliteTime(observation.sat);
    }

    let mut elevation = observation.elevation_deg;
    if elevation.is_none() {
        if let Some((rx_x, rx_y, rx_z)) = last_ecef {
            if let Some((sat_x, sat_y, sat_z)) = navigation_satellite_state(obs, sat, navigation) {
                let (_azimuth_deg, elevation_deg) =
                    elevation_azimuth_deg(rx_x, rx_y, rx_z, sat_x, sat_y, sat_z);
                elevation = Some(elevation_deg);
            }
        }
    }
    if let Some(elevation_deg) = elevation {
        if elevation_deg < config.weighting.elev_mask_deg {
            return PositionObservationPreparation::ExcludedByElevationMask;
        }
    }

    let tracking_mode_weight = if sat.metadata.tracking_mode.contains("vector") {
        config.weighting.tracking_mode_vector_weight
    } else {
        config.weighting.tracking_mode_scalar_weight
    };
    let pseudorange_sigma_m = (sat.pseudorange_var_m2.is_finite() && sat.pseudorange_var_m2 > 0.0)
        .then_some(sat.pseudorange_var_m2.sqrt());
    let weight = position_measurement_weight(
        Some(sat.cn0_dbhz),
        elevation,
        pseudorange_sigma_m,
        WeightingConfig {
            model: match config.weighting.mode {
                NavigationWeightingMode::Elevation => PositionWeightingModel::Elevation,
                NavigationWeightingMode::Cn0 => PositionWeightingModel::Cn0,
                NavigationWeightingMode::ElevationCn0 => PositionWeightingModel::ElevationCn0,
            },
            enabled: config.weighting.enabled,
            min_elev_deg: config.weighting.min_elev_deg,
            elev_exponent: config.weighting.elev_exponent,
            cn0_ref_dbhz: config.weighting.cn0_ref_dbhz,
            min_weight: config.weighting.min_weight,
        },
    );
    observation.elevation_deg = elevation;
    observation.weight = weight * tracking_mode_weight;
    PositionObservationPreparation::Included(observation)
}

fn build_provenance(
    config: &ReceiverPipelineConfig,
    observations: &[PositionObservation],
    solution: &NavSolutionEpoch,
) -> NavProvenance {
    let satellites_used = solution
        .residuals
        .iter()
        .filter(|residual| !residual.rejected)
        .map(|residual| residual.sat)
        .collect::<Vec<_>>();
    let weighting_mode = if !config.weighting.enabled {
        "sigma_weighted"
    } else {
        match config.weighting.mode {
            NavigationWeightingMode::Elevation => "elevation_sigma_weighted",
            NavigationWeightingMode::Cn0 => "cn0_sigma_weighted",
            NavigationWeightingMode::ElevationCn0 => "elevation_cn0_sigma_weighted",
        }
    };
    let solver_family = if observations.iter().any(|row| (row.weight - 1.0).abs() > 1.0e-6) {
        "wls_weighted"
    } else {
        "wls"
    };
    NavProvenance {
        solver_family: solver_family.to_string(),
        weighting_mode: weighting_mode.to_string(),
        robust_solver: config.robust_solver,
        raim_enabled: config.raim,
        satellites_used,
    }
}

fn impossible_geometry_explain_reasons(
    impossible_geometry: ImpossibleGeometryEvidence,
) -> Vec<String> {
    vec![
        "impossible_geometry".to_string(),
        format!(
            "receiver_radius_m={:.3}",
            impossible_geometry.receiver_radius_m
        ),
        format!("altitude_m={:.3}", impossible_geometry.altitude_m),
        format!(
            "used_satellites={}",
            impossible_geometry.used_satellite_count
        ),
        format!(
            "receiver_radius_bounds_m={:.3}..{:.3}",
            impossible_geometry.min_receiver_radius_m,
            impossible_geometry.max_receiver_radius_m
        ),
        format!(
            "altitude_bounds_m={:.3}..{:.3}",
            impossible_geometry.min_altitude_m,
            impossible_geometry.max_altitude_m
        ),
    ]
}

fn constellation_clock_inconsistency_explain_reasons(
    inconsistencies: &[ConstellationClockInconsistency],
) -> Vec<String> {
    let mut reasons = vec!["constellation_clock_inconsistency".to_string()];
    for inconsistency in inconsistencies {
        reasons.push(format!(
            "constellation_clock_inconsistency_constellation={:?}",
            inconsistency.constellation
        ));
        reasons.push(format!(
            "constellation_clock_inconsistency_bias_step_m={:.3}",
            inconsistency.bias_step_m
        ));
        reasons.push(format!(
            "constellation_clock_inconsistency_previous_bias_s={:.12}",
            inconsistency.previous_bias_s
        ));
        reasons.push(format!(
            "constellation_clock_inconsistency_current_bias_s={:.12}",
            inconsistency.current_bias_s
        ));
        reasons.push(format!(
            "constellation_clock_inconsistency_supporting_satellites={}",
            inconsistency.supporting_satellite_count
        ));
    }
    reasons
}

fn residual_temporal_correlation_explain_reasons(
    correlation: crate::pipeline::residual_whiteness::ResidualTemporalCorrelation,
) -> Vec<String> {
    vec![
        "residual_whiteness".to_string(),
        format!("residual_lag1_correlation={:.3}", correlation.lag1_correlation),
        format!(
            "residual_matched_satellites={}",
            correlation.matched_satellite_count
        ),
        format!(
            "residual_temporal_correlation_streak={}",
            correlation.persistent_suspect_epochs
        ),
    ]
}

fn decision_for_solution(solution: &NavSolutionEpoch) -> NavDecision {
    if solution.refusal_class.is_some() || !solution.valid {
        return NavDecision {
            status: solution.status,
            refusal_class: solution.refusal_class,
            explain_decision: "refused".to_string(),
            explain_reasons: if solution.explain_reasons.is_empty() {
                vec!["solution_marked_invalid".to_string()]
            } else {
                solution.explain_reasons.clone()
            },
        };
    }
    if matches!(solution.status, SolutionStatus::Held | SolutionStatus::Degraded) {
        return NavDecision {
            status: solution.status,
            refusal_class: None,
            explain_decision: "degraded".to_string(),
            explain_reasons: vec!["quality_or_geometry_degraded".to_string()],
        };
    }
    NavDecision {
        status: solution.status,
        refusal_class: None,
        explain_decision: "accepted".to_string(),
        explain_reasons: vec!["navigation_solution_usable".to_string()],
    }
}

type InnovationStats = (Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>);

fn innovation_stats(residuals: &[NavResidual]) -> InnovationStats {
    let mut sum = 0.0;
    let mut sum_norm = 0.0;
    let mut max_norm = 0.0;
    let mut sum_pred = 0.0;
    let mut sum_obs = 0.0;
    let mut count = 0.0;
    for res in residuals {
        if res.rejected {
            continue;
        }
        let r = res.residual_m.0;
        sum += r * r;
        if let Some(w) = res.weight {
            let sigma2 = 1.0 / w.max(1e-6);
            sum_pred += sigma2;
            let norm = r / sigma2.sqrt();
            sum_norm += norm * norm;
            if norm.abs() > max_norm {
                max_norm = norm.abs();
            }
        }
        sum_obs += r * r;
        count += 1.0;
    }
    if count == 0.0 {
        return (None, None, None, None, None);
    }
    let rms = (sum / count).sqrt();
    let norm_rms = if sum_norm > 0.0 { Some((sum_norm / count).sqrt()) } else { None };
    let max_norm_opt = if max_norm > 0.0 { Some(max_norm) } else { None };
    let pred_var = if sum_pred > 0.0 { Some(sum_pred / count) } else { None };
    let obs_var = Some(sum_obs / count);
    (Some(rms), norm_rms, max_norm_opt, pred_var, obs_var)
}

fn classify_validity(solution: &NavSolutionEpoch) -> bijux_gnss_core::api::SolutionValidity {
    use bijux_gnss_core::api::SolutionValidity;
    if !solution.valid {
        return SolutionValidity::Invalid;
    }
    if solution.rms_m.0 > 50.0 || solution.pdop > 10.0 {
        return SolutionValidity::Diverging;
    }
    if solution.rms_m.0 > 10.0 {
        return SolutionValidity::Coarse;
    }
    if solution.rms_m.0 > 3.0 {
        return SolutionValidity::Converging;
    }
    SolutionValidity::Stable
}

fn navigation_satellite_state(
    obs: &ObsEpoch,
    sat: &bijux_gnss_core::api::ObsSatellite,
    navigation: &[PositionBroadcastNavigation],
) -> Option<(f64, f64, f64)> {
    let receive_tow_s = obs.gps_time().map(|gps_time| gps_time.tow_s).unwrap_or(obs.t_rx_s.0);
    let navigation = navigation.iter().find(|entry| entry.sat() == sat.signal_id.sat)?;
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                receive_tow_s,
                sat.pseudorange_m.0,
                sat.timing,
            );
            Some((state.x_m, state.y_m, state.z_m))
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let state = sat_state_galileo_e1_from_observation(
                navigation,
                receive_tow_s,
                sat.pseudorange_m.0,
                sat.timing,
            );
            Some((state.x_m, state.y_m, state.z_m))
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let state = sat_state_beidou_b1i_from_observation(
                navigation,
                receive_tow_s,
                sat.pseudorange_m.0,
                sat.timing,
            );
            Some((state.x_m, state.y_m, state.z_m))
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let state = sat_state_glonass_l1_from_observation(
                navigation,
                receive_tow_s,
                sat.pseudorange_m.0,
                sat.timing,
            )?;
            Some((state.x_m, state.y_m, state.z_m))
        }
    }
}

#[derive(Debug, Clone)]
struct ClockModel {
    bias_s: f64,
    drift_s: f64,
}

impl ClockModel {
    fn new() -> Self {
        Self { bias_s: 0.0, drift_s: 0.0 }
    }

    fn update(&mut self, measurement_bias_s: f64, dt_s: f64) -> (f64, f64) {
        let alpha = 0.1;
        let beta = 0.01;
        let mut bias = self.bias_s + self.drift_s * dt_s;
        let residual = measurement_bias_s - bias;
        bias += alpha * residual;
        let drift = self.drift_s + (beta * residual / dt_s.max(1e-6));
        self.bias_s = bias;
        self.drift_s = drift;
        (self.bias_s, self.drift_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::observations::fake_obs_epoch_for_nav_tests;
    use bijux_gnss_core::api::{
        Constellation, GpsTime, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ObservationSupportClass,
        ObservationUncertaintyClass, ReceiverRole, SatId, Seconds, SigId, SignalBand, SignalCode,
        SignalSpec,
    };
    use bijux_gnss_nav::api::{
        ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, sat_state_galileo_e1,
        sat_state_gps_l1ca, GalileoBroadcastNavigationData, GalileoClockCorrection,
        GalileoEphemeris, GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags,
        GalileoSignalHealth, GalileoSystemTime, GpsEphemeris, SaastamoinenModel, TroposphereModel,
    };

    fn sample_last_solution() -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: 0 },
            t_rx_s: Seconds(0.0),
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            ecef_x_m: Meters(1.0),
            ecef_y_m: Meters(2.0),
            ecef_z_m: Meters(3.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_bias_m: Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: Some(Meters(1.0)),
            post_fit_residual_rms_m: Some(Meters(1.0)),
            rms_m: Meters(1.0),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
            validity: bijux_gnss_core::api::SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            health: Vec::new(),
            isb: Vec::new(),
            sigma_e_m: None,
            sigma_n_m: None,
            sigma_u_m: None,
            horizontal_error_ellipse_major_axis_m: None,
            horizontal_error_ellipse_minor_axis_m: None,
            horizontal_error_ellipse_azimuth_deg: None,
            sigma_h_m: None,
            sigma_v_m: None,
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: NavLifecycleState::Converged,
            uncertainty_class: NavUncertaintyClass::Medium,
            assumptions: Some(nav_assumptions(0)),
            refusal_class: None,
            artifact_id: "nav-epoch-0000000000-seed".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000000-seed".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.2),
            gdop: Some(1.5),
            tdop: Some(0.7),
            stability_signature: "navsig:v2:sample".to_string(),
            stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    fn make_eph(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: t_ref_s,
            toc_s: t_ref_s,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0,
            omegadot: 0.0,
            w: 0.0,
            m0,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        }
    }

    fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
        KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        )
    }

    fn sample_pipeline_position_satellite(
        sat: SatId,
        cn0_dbhz: f64,
        elevation_deg: Option<f64>,
        tracking_mode: &str,
    ) -> ObsSatellite {
        let signal_travel_time_s = 24_000_000.0 / 299_792_458.0;
        ObsSatellite {
            signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(24_000_000.0),
            pseudorange_var_m2: 4.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: bijux_gnss_core::api::Hertz(0.0),
            doppler_var_hz2: 4.0,
            cn0_dbhz,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: true,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg,
            azimuth_deg: Some(0.0),
            weight: Some(1.0),
            timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(signal_travel_time_s),
                transmit_gps_time: GpsTime { week: 0, tow_s: 2_000.0 - signal_travel_time_s },
            }),
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: tracking_mode.to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: SignalSpec {
                    constellation: sat.constellation,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
                },
                observation_status: "accepted".to_string(),
                observation_support_class: "supported".to_string(),
                observation_uncertainty_class: "unknown".to_string(),
                ..ObsMetadata::default()
            },
        }
    }

    type NavFixtureEpoch = ObsEpoch;

    fn make_obs_epoch_for_solution(
        epoch_idx: u64,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        ephs: &[GpsEphemeris],
    ) -> NavFixtureEpoch {
        make_obs_epoch_for_solution_with_clock_bias(epoch_idx, t_rx_s, position_ecef, ephs, 0.0)
    }

    fn make_obs_epoch_for_solution_with_clock_bias(
        epoch_idx: u64,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        ephs: &[GpsEphemeris],
        receiver_clock_bias_s: f64,
    ) -> NavFixtureEpoch {
        let sats = ephs
            .iter()
            .map(|eph| {
                let pseudorange_m =
                    synthetic_pseudorange_m(eph, t_rx_s, position_ecef, receiver_clock_bias_s);
                ObsSatellite {
                    signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: Meters(pseudorange_m),
                    pseudorange_var_m2: 4.0,
                    carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
                    carrier_phase_var_cycles2: 1.0,
                    doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                    doppler_var_hz2: 4.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: true,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: Some(45.0),
                    azimuth_deg: Some(0.0),
                    weight: Some(1.0),
                    timing: Some(ObsSignalTiming {
                        signal_travel_time_s: Seconds(pseudorange_m / 299_792_458.0),
                        transmit_gps_time: GpsTime {
                            week: 0,
                            tow_s: t_rx_s - (pseudorange_m / 299_792_458.0),
                        },
                    }),
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "synthetic".to_string(),
                        integration_ms: 1,
                        lock_quality: 1.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal: SignalSpec {
                            constellation: Constellation::Gps,
                            band: SignalBand::L1,
                            code: SignalCode::Ca,
                            code_rate_hz: 1_023_000.0,
                            carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
                        },
                        observation_status: "accepted".to_string(),
                        observation_support_class: match ObservationSupportClass::Supported {
                            ObservationSupportClass::Supported => "supported",
                            ObservationSupportClass::Degraded => "degraded",
                            ObservationSupportClass::Unsupported => "unsupported",
                        }
                        .to_string(),
                        observation_uncertainty_class: match ObservationUncertaintyClass::Unknown {
                            ObservationUncertaintyClass::Low => "low",
                            ObservationUncertaintyClass::Medium => "medium",
                            ObservationUncertaintyClass::High => "high",
                            ObservationUncertaintyClass::Unknown => "unknown",
                        }
                        .to_string(),
                        ..ObsMetadata::default()
                    },
                }
            })
            .collect();
        let mut epoch = crate::pipeline::observations::fake_obs_epoch_for_nav_tests(epoch_idx);
        epoch.t_rx_s = Seconds(t_rx_s);
        epoch.role = ReceiverRole::Rover;
        epoch.sats = sats;
        epoch.decision = ObservationEpochDecision::Accepted;
        epoch.decision_reason = Some("accepted_observables_present".to_string());
        epoch.manifest = Some(bijux_gnss_core::api::ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: format!("obs-epoch-{epoch_idx:010}"),
            epoch_id: format!("obs-epoch-{epoch_idx:010}-synthetic"),
            source_epoch_idx: epoch_idx,
            source_sample_index: epoch_idx,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(
                epoch_idx, 1_000.0,
            ),
            decision: ObservationEpochDecision::Accepted,
            downstream_profile_version:
                bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        });
        epoch.source_time =
            bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0);
        epoch
    }

    fn synthetic_pseudorange_m(
        eph: &GpsEphemeris,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        receiver_clock_bias_s: f64,
    ) -> f64 {
        let c = 299_792_458.0;
        let mut tau = 0.07;
        let mut pseudorange_m = 0.0;
        for _ in 0..10 {
            let sat = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
            let dx = position_ecef.0 - sat.x_m;
            let dy = position_ecef.1 - sat.y_m;
            let dz = position_ecef.2 - sat.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            pseudorange_m = range + receiver_clock_bias_s * c - sat.clock_correction.bias_s * c;
            let next_tau = pseudorange_m / c;
            if (next_tau - tau).abs() < 1e-12 {
                break;
            }
            tau = next_tau;
        }
        let sat = sat_state_gps_l1ca(eph, t_rx_s - tau, tau);
        let (lat_deg, lon_deg, alt_m) =
            ecef_to_geodetic(position_ecef.0, position_ecef.1, position_ecef.2);
        let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
            position_ecef.0,
            position_ecef.1,
            position_ecef.2,
            sat.x_m,
            sat.y_m,
            sat.z_m,
        );
        let receiver = bijux_gnss_core::api::Llh { lat_deg, lon_deg, alt_m };
        let model = SaastamoinenModel;
        pseudorange_m + model.delay_m(receiver, elevation_deg, Seconds(t_rx_s))
    }

    fn make_galileo_navigation(prn: u8, omega0: f64, m0: f64) -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            gst: GalileoSystemTime { week: 2222, tow_s: 100_000 },
            sisa_e1_e5b: 77,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s: 100_000.0,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                bgd_e1_e5a_s: 0.0,
                bgd_e1_e5b_s: 0.0,
            },
            ephemeris: GalileoEphemeris {
                sat: SatId { constellation: Constellation::Galileo, prn },
                iodnav: prn as u16,
                toe_s: 100_000.0,
                sqrt_a: 5_440.612_319,
                e: 0.001_23,
                i0: 0.953,
                idot: -2.1e-10,
                omega0,
                omegadot: -5.8e-9,
                w: -0.37,
                m0,
                delta_n: 4.7e-9,
                cuc: -3.2e-6,
                cus: 4.1e-6,
                crc: 178.0,
                crs: -91.0,
                cic: 1.9e-7,
                cis: -2.4e-7,
            },
            ionosphere: GalileoIonosphericCorrection {
                ai0: 0.0,
                ai1: 0.0,
                ai2: 0.0,
                disturbance_flags: GalileoIonosphericDisturbanceFlags {
                    region_1: false,
                    region_2: false,
                    region_3: false,
                    region_4: false,
                    region_5: false,
                },
            },
        }
    }

    fn make_galileo_nequick_navigation(
        prn: u8,
        omega0: f64,
        m0: f64,
    ) -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav: prn as u16,
            gst: GalileoSystemTime { week: 2222, tow_s: 504_018 },
            sisa_e1_e5b: 77,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s: 504_018.0,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                bgd_e1_e5a_s: 0.0,
                bgd_e1_e5b_s: 0.0,
            },
            ephemeris: GalileoEphemeris {
                sat: SatId { constellation: Constellation::Galileo, prn },
                iodnav: prn as u16,
                toe_s: 504_000.0,
                sqrt_a: 5_440.612_319,
                e: 0.001_23,
                i0: 0.953,
                idot: -2.1e-10,
                omega0,
                omegadot: -5.8e-9,
                w: -0.37,
                m0,
                delta_n: 4.7e-9,
                cuc: -3.2e-6,
                cus: 4.1e-6,
                crc: 178.0,
                crs: -91.0,
                cic: 1.9e-7,
                cis: -2.4e-7,
            },
            ionosphere: GalileoIonosphericCorrection {
                ai0: 82.0,
                ai1: 0.18,
                ai2: -0.01,
                disturbance_flags: GalileoIonosphericDisturbanceFlags {
                    region_1: false,
                    region_2: false,
                    region_3: false,
                    region_4: false,
                    region_5: false,
                },
            },
        }
    }

    fn synthetic_galileo_pseudorange_m(
        navigation: &GalileoBroadcastNavigationData,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        receiver_clock_bias_s: f64,
        galileo_bias_s: f64,
    ) -> f64 {
        let c = 299_792_458.0;
        let mut tau = 0.07;
        let mut pseudorange_m = 0.0;
        for _ in 0..10 {
            let sat = sat_state_galileo_e1(navigation, t_rx_s - tau, tau);
            let dx = position_ecef.0 - sat.x_m;
            let dy = position_ecef.1 - sat.y_m;
            let dz = position_ecef.2 - sat.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            pseudorange_m = range + (receiver_clock_bias_s + galileo_bias_s) * c
                - sat.clock_correction.bias_s * c;
            let next_tau = pseudorange_m / c;
            if (next_tau - tau).abs() < 1e-12 {
                break;
            }
            tau = next_tau;
        }
        pseudorange_m
    }

    fn visible_galileo_navigations(
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        receiver_clock_bias_s: f64,
    ) -> Vec<GalileoBroadcastNavigationData> {
        let mut candidates = [
            (11, 0.17, 0.24),
            (12, 1.17, 0.84),
            (19, -0.83, 1.52),
            (24, 2.11, -0.41),
            (27, -2.34, 0.61),
            (30, 0.48, -1.14),
            (31, 1.84, 2.27),
            (33, -1.47, -2.02),
            (36, 2.73, 1.06),
            (37, -0.26, 2.88),
            (39, 0.95, -2.41),
            (40, -2.88, -0.73),
        ]
        .into_iter()
        .map(|(prn, omega0, m0)| make_galileo_nequick_navigation(prn, omega0, m0))
        .filter_map(|navigation| {
            let pseudorange_m = synthetic_galileo_pseudorange_m(
                &navigation,
                t_rx_s,
                position_ecef,
                receiver_clock_bias_s,
                0.0,
            );
            let signal_travel_time_s = pseudorange_m / 299_792_458.0;
            let state = sat_state_galileo_e1(
                &navigation,
                t_rx_s - signal_travel_time_s,
                signal_travel_time_s,
            );
            let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                position_ecef.0,
                position_ecef.1,
                position_ecef.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            (elevation_deg > 0.0).then_some((elevation_deg, navigation))
        })
        .collect::<Vec<_>>();
        candidates.sort_by(|(left_elevation_deg, _), (right_elevation_deg, _)| {
            right_elevation_deg
                .partial_cmp(left_elevation_deg)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        candidates
            .into_iter()
            .take(6)
            .map(|(_, navigation)| navigation)
            .collect()
    }

    fn make_galileo_obs_epoch_for_solution(
        epoch_idx: u64,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        navigations: &[GalileoBroadcastNavigationData],
        receiver_clock_bias_s: f64,
    ) -> NavFixtureEpoch {
        let sats = navigations
            .iter()
            .map(|navigation| {
                let pseudorange_m = synthetic_galileo_pseudorange_m(
                    navigation,
                    t_rx_s,
                    position_ecef,
                    receiver_clock_bias_s,
                    0.0,
                );
                let signal_travel_time_s = pseudorange_m / 299_792_458.0;
                ObsSatellite {
                    signal_id: SigId {
                        sat: navigation.sat,
                        band: SignalBand::E1,
                        code: SignalCode::E1B,
                    },
                    pseudorange_m: Meters(pseudorange_m),
                    pseudorange_var_m2: 9.0,
                    carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
                    carrier_phase_var_cycles2: 1.0,
                    doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                    doppler_var_hz2: 4.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: true,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: None,
                    azimuth_deg: None,
                    weight: Some(1.0),
                    timing: Some(ObsSignalTiming {
                        signal_travel_time_s: Seconds(signal_travel_time_s),
                        transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
                    }),
                    error_model: None,
                    metadata: ObsMetadata::default(),
                }
            })
            .collect();
        let mut epoch = fake_obs_epoch_for_nav_tests(epoch_idx);
        epoch.t_rx_s = Seconds(t_rx_s);
        epoch.role = ReceiverRole::Rover;
        epoch.sats = sats;
        epoch.decision = ObservationEpochDecision::Accepted;
        epoch.decision_reason = Some("accepted_observables_present".to_string());
        epoch.manifest = Some(bijux_gnss_core::api::ObsEpochManifest {
            version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
            artifact_id: format!("obs-epoch-{epoch_idx:010}"),
            epoch_id: format!("obs-epoch-{epoch_idx:010}-synthetic-galileo"),
            source_epoch_idx: epoch_idx,
            source_sample_index: epoch_idx,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(
                epoch_idx, 1_000.0,
            ),
            decision: ObservationEpochDecision::Accepted,
            downstream_profile_version:
                bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
        });
        epoch.source_time =
            bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0);
        epoch
    }

    fn apply_galileo_nequick_bias(
        epoch: &mut NavFixtureEpoch,
        navigations: &[GalileoBroadcastNavigationData],
        position_ecef: (f64, f64, f64),
    ) {
        let (lat_deg, lon_deg, alt_m) =
            ecef_to_geodetic(position_ecef.0, position_ecef.1, position_ecef.2);
        let receiver = bijux_gnss_core::api::Llh { lat_deg, lon_deg, alt_m };
        for satellite in &mut epoch.sats {
            let navigation = navigations
                .iter()
                .find(|navigation| navigation.sat == satellite.signal_id.sat)
                .expect("matching galileo navigation");
            let signal_travel_time_s =
                satellite.timing.expect("galileo timing").signal_travel_time_s.0;
            let state = sat_state_galileo_e1(
                navigation,
                epoch.t_rx_s.0 - signal_travel_time_s,
                signal_travel_time_s,
            );
            let (sat_lat_deg, sat_lon_deg, sat_alt_m) =
                ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
            let satellite_llh =
                bijux_gnss_core::api::Llh { lat_deg: sat_lat_deg, lon_deg: sat_lon_deg, alt_m: sat_alt_m };
            let gps_time = GpsTime { week: 0, tow_s: epoch.t_rx_s.0 };
            let delay_m = navigation
                .nequick_delay_m(satellite.signal_id, receiver, satellite_llh, gps_time)
                .expect("galileo nequick delay");
            let delay_s = delay_m / 299_792_458.0;
            satellite.pseudorange_m.0 += delay_m;
            if let Some(timing) = satellite.timing.as_mut() {
                timing.signal_travel_time_s.0 += delay_s;
                timing.transmit_gps_time.tow_s -= delay_s;
            }
        }
    }

    fn deterministic_pseudorange_offset_m(epoch_idx: u64, prn: u8) -> f64 {
        let pattern = ((epoch_idx as usize * 17 + prn as usize * 13) % 9) as f64 - 4.0;
        pattern * 0.85
    }

    fn apply_deterministic_pseudorange_offsets(obs: &mut NavFixtureEpoch) {
        for sat in &mut obs.sats {
            let offset_m = deterministic_pseudorange_offset_m(obs.epoch_idx, sat.signal_id.sat.prn);
            sat.pseudorange_m.0 += offset_m;
            if let Some(timing) = sat.timing.as_mut() {
                timing.signal_travel_time_s = Seconds(sat.pseudorange_m.0 / 299_792_458.0);
                timing.transmit_gps_time.tow_s = obs.t_rx_s.0 - timing.signal_travel_time_s.0;
            }
        }
    }

    fn translate_truth_ecef_m(
        truth_ecef_m: (f64, f64, f64),
        truth_velocity_mps: (f64, f64, f64),
        dt_s: f64,
    ) -> (f64, f64, f64) {
        (
            truth_ecef_m.0 + truth_velocity_mps.0 * dt_s,
            truth_ecef_m.1 + truth_velocity_mps.1 * dt_s,
            truth_ecef_m.2 + truth_velocity_mps.2 * dt_s,
        )
    }

    fn solution_error_3d_m(solution: &NavSolutionEpoch, truth_ecef_m: (f64, f64, f64)) -> f64 {
        let dx = solution.ecef_x_m.0 - truth_ecef_m.0;
        let dy = solution.ecef_y_m.0 - truth_ecef_m.1;
        let dz = solution.ecef_z_m.0 - truth_ecef_m.2;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    fn solution_trace_path_length_m(solutions: &[NavSolutionEpoch]) -> f64 {
        solutions
            .windows(2)
            .map(|window| {
                let dx = window[1].ecef_x_m.0 - window[0].ecef_x_m.0;
                let dy = window[1].ecef_y_m.0 - window[0].ecef_y_m.0;
                let dz = window[1].ecef_z_m.0 - window[0].ecef_z_m.0;
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .sum()
    }

    fn root_mean_square(values: &[f64]) -> f64 {
        (values.iter().map(|value| value * value).sum::<f64>() / values.len() as f64).sqrt()
    }

    #[test]
    fn navigation_satellite_state_uses_observation_timing_when_available() {
        let eph = make_eph(6, 1.2, 0.8, 345_600.0);
        let tau_s = 0.074;
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.08 };
        let pseudorange_m = tau_s * 299_792_458.0;
        let sat = ObsSatellite {
            signal_id: SigId { sat: eph.sat, band: SignalBand::L1, code: SignalCode::Ca },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 4.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: bijux_gnss_core::api::Hertz(0.0),
            doppler_var_hz2: 4.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: true,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: Some(1.0),
            timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(tau_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-tau_s),
            }),
            error_model: None,
            metadata: ObsMetadata::default(),
        };
        let obs = ObsEpoch {
            t_rx_s: Seconds(1.0),
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(0, 1_000.0),
            gps_week: Some(receive_gps_time.week),
            tow_s: Some(Seconds(receive_gps_time.tow_s)),
            epoch_idx: 1,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![sat.clone()],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        };

        let navigation = vec![PositionBroadcastNavigation::Gps(eph.clone())];
        let from_navigation =
            navigation_satellite_state(&obs, &sat, &navigation).expect("navigation state");
        let from_timing =
            sat_state_gps_l1ca(&eph, sat.timing.expect("timing").transmit_gps_time.tow_s, tau_s);
        let uncorrected = sat_state_gps_l1ca(&eph, receive_gps_time.tow_s, 0.0);

        assert!((from_navigation.0 - from_timing.x_m).abs() < 1.0e-9);
        assert!((from_navigation.1 - from_timing.y_m).abs() < 1.0e-9);
        assert!((from_navigation.2 - from_timing.z_m).abs() < 1.0e-9);
        assert!((from_navigation.0 - uncorrected.x_m).abs() > 0.01);
        assert!((from_navigation.1 - uncorrected.y_m).abs() > 0.01);
    }

    #[test]
    fn prepare_position_observation_downweights_low_elevation_satellites() {
        let sat = SatId { constellation: Constellation::Gps, prn: 8 };
        let mut obs_epoch = fake_obs_epoch_for_nav_tests(0);
        obs_epoch.t_rx_s = Seconds(2_000.0);
        obs_epoch.gps_week = Some(0);
        obs_epoch.tow_s = Some(Seconds(2_000.0));
        let config = ReceiverPipelineConfig::default();
        let high_elevation = sample_pipeline_position_satellite(sat, 45.0, Some(75.0), "scalar");
        let low_elevation = sample_pipeline_position_satellite(sat, 45.0, Some(10.0), "scalar");

        let PositionObservationPreparation::Included(high_observation) =
            prepare_position_observation(&config, &obs_epoch, &high_elevation, &[], None)
        else {
            panic!("high-elevation observation should be retained");
        };
        let PositionObservationPreparation::Included(low_observation) =
            prepare_position_observation(&config, &obs_epoch, &low_elevation, &[], None)
        else {
            panic!("low-elevation observation should be retained");
        };

        assert!(low_observation.weight.is_finite());
        assert!(high_observation.weight.is_finite());
        assert!(low_observation.weight < high_observation.weight);
    }

    #[test]
    fn prepare_position_observation_excludes_satellite_below_elevation_mask() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let mut obs_epoch = fake_obs_epoch_for_nav_tests(0);
        obs_epoch.t_rx_s = Seconds(2_000.0);
        obs_epoch.gps_week = Some(0);
        obs_epoch.tow_s = Some(Seconds(2_000.0));
        let mut config = ReceiverPipelineConfig::default();
        config.weighting.elev_mask_deg = 15.0;
        let low_elevation = sample_pipeline_position_satellite(sat, 45.0, Some(10.0), "scalar");

        let preparation =
            prepare_position_observation(&config, &obs_epoch, &low_elevation, &[], None);

        assert!(matches!(preparation, PositionObservationPreparation::ExcludedByElevationMask));
    }

    #[test]
    fn prepare_position_observation_downweights_low_cn0_signals_in_cn0_mode() {
        let sat = SatId { constellation: Constellation::Gps, prn: 10 };
        let mut obs_epoch = fake_obs_epoch_for_nav_tests(0);
        obs_epoch.t_rx_s = Seconds(2_000.0);
        obs_epoch.gps_week = Some(0);
        obs_epoch.tow_s = Some(Seconds(2_000.0));
        let mut config = ReceiverPipelineConfig::default();
        config.weighting.mode = NavigationWeightingMode::Cn0;
        let strong_signal = sample_pipeline_position_satellite(sat, 48.0, Some(45.0), "scalar");
        let weak_signal = sample_pipeline_position_satellite(sat, 28.0, Some(45.0), "scalar");

        let PositionObservationPreparation::Included(strong_observation) =
            prepare_position_observation(&config, &obs_epoch, &strong_signal, &[], None)
        else {
            panic!("strong-signal observation should be retained");
        };
        let PositionObservationPreparation::Included(weak_observation) =
            prepare_position_observation(&config, &obs_epoch, &weak_signal, &[], None)
        else {
            panic!("weak-signal observation should be retained");
        };

        assert!(weak_observation.weight.is_finite());
        assert!(strong_observation.weight.is_finite());
        assert!(weak_observation.weight < strong_observation.weight);
    }

    #[test]
    fn prepare_position_observation_downweights_low_elevation_and_low_cn0_signals() {
        let sat = SatId { constellation: Constellation::Gps, prn: 10 };
        let mut obs_epoch = fake_obs_epoch_for_nav_tests(0);
        obs_epoch.t_rx_s = Seconds(2_000.0);
        obs_epoch.gps_week = Some(0);
        obs_epoch.tow_s = Some(Seconds(2_000.0));
        let mut config = ReceiverPipelineConfig::default();
        config.weighting.mode = NavigationWeightingMode::ElevationCn0;
        let weak_low_signal = sample_pipeline_position_satellite(sat, 28.0, Some(10.0), "scalar");
        let strong_low_signal = sample_pipeline_position_satellite(sat, 48.0, Some(10.0), "scalar");
        let weak_high_signal = sample_pipeline_position_satellite(sat, 28.0, Some(75.0), "scalar");

        let PositionObservationPreparation::Included(weak_low_observation) =
            prepare_position_observation(&config, &obs_epoch, &weak_low_signal, &[], None)
        else {
            panic!("weak low-elevation observation should be retained");
        };
        let PositionObservationPreparation::Included(strong_low_observation) =
            prepare_position_observation(&config, &obs_epoch, &strong_low_signal, &[], None)
        else {
            panic!("strong low-elevation observation should be retained");
        };
        let PositionObservationPreparation::Included(weak_high_observation) =
            prepare_position_observation(&config, &obs_epoch, &weak_high_signal, &[], None)
        else {
            panic!("weak high-elevation observation should be retained");
        };

        assert!(weak_low_observation.weight.is_finite());
        assert!(strong_low_observation.weight.is_finite());
        assert!(weak_high_observation.weight.is_finite());
        assert!(weak_low_observation.weight < strong_low_observation.weight);
        assert!(weak_low_observation.weight < weak_high_observation.weight);
    }

    #[test]
    fn navigation_refuses_epochs_without_ephemeris_even_with_last_solution() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation {
            config,
            runtime: crate::engine::runtime::ReceiverRuntime::default(),
            solver: PositionSolver::new(),
            position_smoother: None,
            clock: ClockModel::new(),
            last_ecef: Some((1.0, 2.0, 3.0)),
            last_solution: Some(sample_last_solution()),
            last_position_observations: None,
            last_satellite_clock_suspect: None,
            satellite_clock_suspect_streak: 0,
            residual_whiteness_suspect_streak: 0,
        };
        let obs = fake_obs_epoch_for_nav_tests(10);
        let solution = nav.solve_epoch(&obs, &[]).expect("degraded solution");
        assert_eq!(solution.status, SolutionStatus::Unavailable);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InvalidEphemeris));
        assert_eq!(solution.explain_decision, "refused");
        assert_eq!(solution.epoch.index, 10);
    }

    #[test]
    fn invalid_observation_epoch_is_refused_with_explain_surface() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let mut obs = fake_obs_epoch_for_nav_tests(11);
        obs.valid = false;
        obs.decision = ObservationEpochDecision::Rejected;
        obs.decision_reason = Some("malformed_observation_set".to_string());
        let solution = nav.solve_epoch(&obs, &[]).expect("invalid solution");
        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InconsistentObservations));
        assert_eq!(solution.explain_decision, "invalid_observation_epoch");
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "input_observation_marked_invalid"));
        assert_eq!(solution.lifecycle_state, NavLifecycleState::Refused);
        assert_eq!(solution.model_version, NAV_SOLUTION_MODEL_VERSION);
    }

    #[test]
    fn navigation_refuses_observations_without_valid_satellite_time() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
        ];
        let mut obs = make_obs_epoch_for_solution(13, t_rx_s, truth, &ephs);
        for sat in &mut obs.sats {
            let transmit_gps_time = sat.timing.expect("timing").transmit_gps_time;
            sat.timing =
                Some(ObsSignalTiming { signal_travel_time_s: Seconds(0.0), transmit_gps_time });
        }
        let solution = nav.solve_epoch(&obs, &ephs).expect("timing refusal");
        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InvalidSatelliteTime));
        assert!(!solution.valid);
        assert_eq!(solution.used_sat_count, 0);
        assert!(solution.explain_reasons.iter().any(|reason| reason == "invalid_satellite_time"));
    }

    #[test]
    fn unknown_constellation_input_is_refused_explicitly() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let mut obs = fake_obs_epoch_for_nav_tests(15);
        for sat in &mut obs.sats {
            sat.signal_id.sat.constellation = Constellation::Unknown;
        }
        let solution = nav.solve_epoch(&obs, &[]).expect("unsupported constellation solution");
        assert_eq!(solution.status, SolutionStatus::Unavailable);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::UnsupportedConstellation));
        assert_eq!(solution.explain_decision, "unsupported_constellation_input");
    }

    #[test]
    fn mixed_supported_and_unsupported_constellations_are_refused_explicitly() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let mut obs = fake_obs_epoch_for_nav_tests(16);
        for sat in obs.sats.iter_mut().skip(1) {
            sat.signal_id.sat.constellation = Constellation::Unknown;
        }
        let solution = nav.solve_epoch(&obs, &[]).expect("mixed constellation refusal");
        assert_eq!(solution.status, SolutionStatus::Unavailable);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::MixedConstellationInput));
        assert_eq!(solution.explain_decision, "mixed_constellation_time_handling_refused");
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "unknown_inter_system_time_offset"));
    }

    #[test]
    fn glonass_only_input_is_treated_as_supported_constellation() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let mut obs = fake_obs_epoch_for_nav_tests(16);
        for sat in &mut obs.sats {
            sat.signal_id.sat.constellation = Constellation::Glonass;
        }
        let solution = nav.solve_epoch(&obs, &[]).expect("glonass-only solution");
        assert_eq!(solution.status, SolutionStatus::Unavailable);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InvalidEphemeris));
        assert_eq!(solution.explain_decision, "refused");
        assert!(!solution.valid);
    }

    #[test]
    fn mixed_supported_constellation_without_navigation_refuses_invalid_ephemeris() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let mut obs = fake_obs_epoch_for_nav_tests(17);
        for sat in obs.sats.iter_mut().skip(1) {
            sat.signal_id.sat.constellation = Constellation::Galileo;
        }
        let solution = nav.solve_epoch(&obs, &[]).expect("mixed constellation solution");
        assert_eq!(solution.status, SolutionStatus::Unavailable);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InvalidEphemeris));
        assert_eq!(solution.explain_decision, "refused");
        assert!(!solution.valid);
    }

    #[test]
    fn mixed_gps_galileo_navigation_solves_and_emits_galileo_bias() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let receiver_clock_bias_s = 2.75e-4;
        let galileo_bias_s = -1.15e-6;
        let gps_ephemerides = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
        ];
        let galileo_navigation =
            vec![make_galileo_navigation(19, 1.17, 0.84), make_galileo_navigation(24, -0.83, 1.52)];
        let mut obs = make_obs_epoch_for_solution_with_clock_bias(
            17,
            t_rx_s,
            truth,
            &gps_ephemerides,
            receiver_clock_bias_s,
        );
        obs.sats.extend(galileo_navigation.iter().map(|navigation| {
            let pseudorange_m = synthetic_galileo_pseudorange_m(
                navigation,
                t_rx_s,
                truth,
                receiver_clock_bias_s,
                galileo_bias_s,
            );
            let signal_travel_time_s = pseudorange_m / 299_792_458.0;
            ObsSatellite {
                signal_id: SigId {
                    sat: navigation.sat,
                    band: SignalBand::E1,
                    code: SignalCode::E1B,
                },
                pseudorange_m: Meters(pseudorange_m),
                pseudorange_var_m2: 9.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(0.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: Some(1.0),
                timing: Some(ObsSignalTiming {
                    signal_travel_time_s: Seconds(signal_travel_time_s),
                    transmit_gps_time: GpsTime { week: 0, tow_s: t_rx_s - signal_travel_time_s },
                }),
                error_model: None,
                metadata: ObsMetadata::default(),
            }
        }));

        let mut navigation = position_broadcast_navigation_from_gps_ephemerides(&gps_ephemerides);
        navigation
            .extend(galileo_navigation.iter().cloned().map(PositionBroadcastNavigation::Galileo));

        let solution = nav
            .solve_epoch_with_navigation_data(&obs, &navigation)
            .expect("mixed gps+galileo solution");

        assert!(
            solution.valid,
            "unexpected navigation solution status={:?} refusal={:?} reasons={:?}",
            solution.status,
            solution.refusal_class,
            solution.explain_reasons
        );
        assert_eq!(solution.status, SolutionStatus::CodeOnly);
        assert_eq!(solution.refusal_class, None);
        assert_eq!(solution.isb.len(), 1);
        assert_eq!(solution.isb[0].constellation, Constellation::Galileo);
        assert!((solution.isb[0].bias_s.0 - galileo_bias_s).abs() < 5.0e-9);
        assert!((solution.clock_bias_s.0 - receiver_clock_bias_s).abs() < 1.0e-8);
        assert_eq!(solution.used_sat_count, 6);
    }

    #[test]
    fn sparse_current_epoch_refusal_does_not_reuse_last_solution_position() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation {
            config,
            runtime: crate::engine::runtime::ReceiverRuntime::default(),
            solver: PositionSolver::new(),
            position_smoother: None,
            clock: ClockModel::new(),
            last_ecef: Some((10.0, 20.0, 30.0)),
            last_solution: Some(sample_last_solution()),
            last_position_observations: None,
            last_satellite_clock_suspect: None,
            satellite_clock_suspect_streak: 0,
            residual_whiteness_suspect_streak: 0,
        };
        let mut obs = fake_obs_epoch_for_nav_tests(18);
        obs.sats.truncate(3);

        let solution = nav.solve_epoch(&obs, &[]).expect("sparse refusal");

        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InsufficientGeometry));
        assert_eq!(solution.epoch.index, 18);
        assert_eq!(solution.used_sat_count, 3);
        assert_eq!(solution.sat_count, 3);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert_eq!(solution.ecef_y_m.0, 0.0);
        assert_eq!(solution.ecef_z_m.0, 0.0);
        assert!(!solution.valid);
    }

    #[test]
    fn synthetic_solution_populates_provenance_counts_and_dops() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(12, t_rx_s, truth, &ephs);
        let solution = nav.solve_epoch(&obs, &ephs).expect("synthetic solution");

        assert_eq!(solution.model_version, NAV_SOLUTION_MODEL_VERSION);
        assert!(solution.artifact_id.starts_with("nav-epoch-"));
        assert!(solution.source_observation_epoch_id.starts_with("obs-epoch-"));
        assert!(solution.sat_count >= 4);
        assert!(solution.used_sat_count >= 4);
        assert_eq!(solution.sat_count, solution.used_sat_count + solution.rejected_sat_count);
        assert!(solution.hdop.is_some());
        assert!(solution.vdop.is_some());
        assert!(solution.gdop.is_some());
        assert!(solution.tdop.is_some());
        assert!(solution.provenance.is_some());
        assert_eq!(solution.explain_decision, "accepted");
        assert_eq!(solution.refusal_class, None);
        assert!(
            solution.valid,
            "unexpected navigation solution status={:?} refusal={:?} reasons={:?}",
            solution.status,
            solution.refusal_class,
            solution.explain_reasons
        );
        assert!(solution.explain_reasons.iter().any(|reason| reason == "ionosphere_uncorrected"));
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
        assert!(solution.stability_signature.starts_with("navsig:v2:"));
        assert!(solution.stability_signature.contains(":tdop="));
        assert_eq!(solution.stability_signature_version, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION);
    }

    #[test]
    fn synthetic_solution_emits_ecef_position_covariance() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(20, t_rx_s, truth, &ephs);
        let solution = nav.solve_epoch(&obs, &ephs).expect("synthetic solution");
        let covariance = solution
            .position_covariance_ecef_m2
            .expect("navigation solution should emit position covariance");

        for row in covariance {
            for value in row {
                assert!(value.is_finite());
            }
        }
        assert!(covariance[0][0] > 0.0);
        assert!(covariance[1][1] > 0.0);
        assert!(covariance[2][2] > 0.0);
    }

    #[test]
    fn synthetic_solution_emits_enu_position_sigmas() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(21, t_rx_s, truth, &ephs);
        let solution = nav.solve_epoch(&obs, &ephs).expect("synthetic solution");

        assert!(solution.sigma_e_m.expect("east sigma").0.is_finite());
        assert!(solution.sigma_n_m.expect("north sigma").0.is_finite());
        assert!(solution.sigma_u_m.expect("up sigma").0.is_finite());
        assert!(solution
            .horizontal_error_ellipse_major_axis_m
            .expect("ellipse major axis")
            .0
            .is_finite());
        assert!(solution
            .horizontal_error_ellipse_minor_axis_m
            .expect("ellipse minor axis")
            .0
            .is_finite());
        assert!(solution
            .horizontal_error_ellipse_azimuth_deg
            .expect("ellipse azimuth")
            .is_finite());
    }

    #[test]
    fn synthetic_solution_reports_combined_weighting_provenance() {
        let mut config = ReceiverPipelineConfig::default();
        config.weighting.mode = NavigationWeightingMode::ElevationCn0;
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(19, t_rx_s, truth, &ephs);
        let solution = nav.solve_epoch(&obs, &ephs).expect("synthetic solution");
        let provenance = solution.provenance.expect("navigation provenance");

        assert_eq!(provenance.weighting_mode, "elevation_cn0_sigma_weighted");
    }

    #[test]
    fn synthetic_solution_marks_broadcast_ionosphere_correction_when_available() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(18, t_rx_s, truth, &ephs);
        let solution = nav
            .solve_epoch_with_broadcast_ionosphere(
                &obs,
                &ephs,
                Some(&sample_klobuchar_coefficients()),
            )
            .expect("synthetic solution");

        assert!(solution.valid);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
    }

    #[test]
    fn synthetic_solution_uses_gps_broadcast_navigation_klobuchar_payload() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(23, t_rx_s, truth, &ephs);
        let navigation = GpsBroadcastNavigationData {
            ephemerides: ephs,
            klobuchar: Some(sample_klobuchar_coefficients()),
        };

        let solution = nav
            .solve_epoch_with_gps_broadcast_navigation(&obs, &navigation)
            .expect("synthetic solution");

        assert!(solution.valid);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
    }

    #[test]
    fn synthetic_solution_uses_galileo_broadcast_navigation_payload() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 10.0);
        let receiver_clock_bias_s = 0.0;
        let t_rx_s = 504_018.07;
        let navigations = visible_galileo_navigations(t_rx_s, truth, receiver_clock_bias_s);
        assert!(
            navigations.len() >= 4,
            "expected at least four visible galileo satellites"
        );
        let mut obs = make_galileo_obs_epoch_for_solution(
            24,
            t_rx_s,
            truth,
            &navigations,
            receiver_clock_bias_s,
        );
        apply_galileo_nequick_bias(&mut obs, &navigations, truth);

        let solution = nav
            .solve_epoch_with_galileo_broadcast_navigations(&obs, &navigations)
            .expect("synthetic solution");

        assert!(
            solution.valid,
            "unexpected navigation solution status={:?} refusal={:?} reasons={:?}",
            solution.status,
            solution.refusal_class,
            solution.explain_reasons
        );
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_correction=galileo_nequick"));
        assert!(!solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "ionosphere_uncorrected"));
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "troposphere_correction=saastamoinen"));
    }

    #[test]
    fn synthetic_solution_marks_troposphere_uncorrected_when_disabled() {
        let mut config = ReceiverPipelineConfig::default();
        config.tropo_enable = false;
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_000.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(19, t_rx_s, truth, &ephs);
        let solution = nav.solve_epoch(&obs, &ephs).expect("synthetic solution");

        assert!(solution.explain_reasons.iter().any(|reason| reason == "troposphere_uncorrected"));
    }

    #[test]
    fn synthetic_solution_preserves_solved_receiver_clock_bias() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let receiver_clock_bias_s = 2.75e-4;
        let t_rx_s = 100_000.0 + receiver_clock_bias_s;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution_with_clock_bias(
            14,
            t_rx_s,
            truth,
            &ephs,
            receiver_clock_bias_s,
        );
        let solution = nav.solve_epoch(&obs, &ephs).expect("biased synthetic solution");

        assert!((solution.ecef_x_m.0 - truth.0).abs() < 5.0);
        assert!((solution.ecef_y_m.0 - truth.1).abs() < 5.0);
        assert!((solution.ecef_z_m.0 - truth.2).abs() < 5.0);
        assert!((solution.clock_bias_s.0 - receiver_clock_bias_s).abs() < 1.0e-9);
        assert!((solution.clock_bias_m.0 - receiver_clock_bias_s * 299_792_458.0).abs() < 1.0e-6);
    }

    #[test]
    fn synthetic_solution_applies_troposphere_correction_when_enabled() {
        let corrected_config = ReceiverPipelineConfig::default();
        let mut corrected_nav =
            Navigation::new(corrected_config, crate::engine::runtime::ReceiverRuntime::default());
        let mut uncorrected_config = ReceiverPipelineConfig::default();
        uncorrected_config.tropo_enable = false;
        let mut uncorrected_nav =
            Navigation::new(uncorrected_config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_050.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(20, t_rx_s, truth, &ephs);

        let corrected = corrected_nav.solve_epoch(&obs, &ephs).expect("corrected solution");
        let uncorrected = uncorrected_nav.solve_epoch(&obs, &ephs).expect("uncorrected solution");
        let corrected_error_m = ((corrected.ecef_x_m.0 - truth.0).powi(2)
            + (corrected.ecef_y_m.0 - truth.1).powi(2)
            + (corrected.ecef_z_m.0 - truth.2).powi(2))
        .sqrt();
        let uncorrected_error_m = ((uncorrected.ecef_x_m.0 - truth.0).powi(2)
            + (uncorrected.ecef_y_m.0 - truth.1).powi(2)
            + (uncorrected.ecef_z_m.0 - truth.2).powi(2))
        .sqrt();

        assert!(corrected.valid);
        assert!(corrected_error_m < 5.0);
        assert!(uncorrected_error_m > corrected_error_m + 3.0);
    }

    #[test]
    fn synthetic_solution_surfaces_raim_fault_detection() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_075.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
            make_eph(6, 4.0, 4.5, t_rx_s),
        ];
        let mut obs = make_obs_epoch_for_solution(21, t_rx_s, truth, &ephs);
        let bad_sat = SatId { constellation: Constellation::Gps, prn: 6 };
        let pseudorange_bias_m = 1_000.0;
        let signal_travel_time_bias_s = pseudorange_bias_m / 299_792_458.0;
        let bad_observation = obs
            .sats
            .iter_mut()
            .find(|sat| sat.signal_id.sat == bad_sat)
            .expect("bad-satellite synthetic observation");
        bad_observation.pseudorange_m.0 += pseudorange_bias_m;
        if let Some(timing) = &mut bad_observation.timing {
            timing.signal_travel_time_s.0 += signal_travel_time_bias_s;
            timing.transmit_gps_time =
                timing.transmit_gps_time.offset_seconds(-signal_travel_time_bias_s);
        }

        let solution = nav.solve_epoch(&obs, &ephs).expect("raim-detected solution");

        assert_ne!(solution.status, SolutionStatus::Invalid);
        assert!(solution.valid);
        assert!(solution.explain_reasons.iter().any(|reason| reason == "raim_fault_detected"));
        assert!(solution.explain_reasons.iter().any(|reason| reason == "raim_fault_excluded"));
        assert!(solution.explain_reasons.iter().any(|reason| reason == "raim_suspect_prn=6"));
        assert!(solution.explain_reasons.iter().any(|reason| reason == "raim_excluded_prn=6"));
        assert!(solution.residuals.iter().any(|residual| {
            residual.sat == bad_sat
                && residual.rejected
                && residual.reject_reason == Some(MeasurementRejectReason::Outlier)
        }));
        let pre_fit_residual_rms_m = solution
            .pre_fit_residual_rms_m
            .expect("raim-tested solution should report pre-fit residual RMS");
        let post_fit_residual_rms_m = solution
            .post_fit_residual_rms_m
            .expect("raim-tested solution should report post-fit residual RMS");
        assert!(
            pre_fit_residual_rms_m.0 > post_fit_residual_rms_m.0 + 100.0,
            "RAIM exclusion should materially reduce residual RMS: pre={pre_fit_residual_rms_m:?} post={post_fit_residual_rms_m:?}",
        );
        assert!((solution.rms_m.0 - post_fit_residual_rms_m.0).abs() < 1.0e-12);
    }

    #[test]
    fn synthetic_solution_refuses_underdetermined_raim_exclusion() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_085.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
            make_eph(5, 3.2, 3.6, t_rx_s),
        ];
        let mut obs = make_obs_epoch_for_solution(22, t_rx_s, truth, &ephs);
        let bad_sat = SatId { constellation: Constellation::Gps, prn: 5 };
        let pseudorange_bias_m = 1_000.0;
        let signal_travel_time_bias_s = pseudorange_bias_m / 299_792_458.0;
        let bad_observation = obs
            .sats
            .iter_mut()
            .find(|sat| sat.signal_id.sat == bad_sat)
            .expect("bad-satellite synthetic observation");
        bad_observation.pseudorange_m.0 += pseudorange_bias_m;
        if let Some(timing) = &mut bad_observation.timing {
            timing.signal_travel_time_s.0 += signal_travel_time_bias_s;
            timing.transmit_gps_time =
                timing.transmit_gps_time.offset_seconds(-signal_travel_time_bias_s);
        }

        let solution = nav.solve_epoch(&obs, &ephs).expect("underdetermined refusal");

        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InsufficientGeometry));
        assert!(!solution.valid);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "raim_exclusion_underdetermined"));
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("raim_suspect_prn=")));
        assert_eq!(solution.pre_fit_residual_rms_m, None);
        assert_eq!(solution.post_fit_residual_rms_m, None);
    }

    #[test]
    fn navigation_position_solution_smoothing_reduces_static_jitter() {
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t0_rx_s = 100_090.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t0_rx_s),
            make_eph(2, 0.8, 0.9, t0_rx_s),
            make_eph(3, 1.6, 1.8, t0_rx_s),
            make_eph(4, 2.4, 2.7, t0_rx_s),
            make_eph(5, 3.2, 3.6, t0_rx_s),
        ];
        let mut baseline_config = ReceiverPipelineConfig::default();
        baseline_config.position_solution_smoothing = false;
        let mut baseline_nav =
            Navigation::new(baseline_config, crate::engine::runtime::ReceiverRuntime::default());
        let mut smoothed_config = ReceiverPipelineConfig::default();
        smoothed_config.position_solution_motion_class = NavigationMotionClass::Static;
        let mut smoothed_nav =
            Navigation::new(smoothed_config, crate::engine::runtime::ReceiverRuntime::default());
        let mut raw_errors_m = Vec::new();
        let mut smoothed_errors_m = Vec::new();
        let mut raw_solutions = Vec::new();
        let mut smoothed_solutions = Vec::new();

        for epoch_idx in 0..60u64 {
            let t_rx_s = t0_rx_s + epoch_idx as f64;
            let mut obs = make_obs_epoch_for_solution(epoch_idx, t_rx_s, truth, &ephs);
            apply_deterministic_pseudorange_offsets(&mut obs);
            let baseline_solution =
                baseline_nav.solve_epoch(&obs, &ephs).expect("baseline static solution");
            let smoothed_solution =
                smoothed_nav.solve_epoch(&obs, &ephs).expect("smoothed static solution");
            raw_errors_m.push(solution_error_3d_m(&baseline_solution, truth));
            smoothed_errors_m.push(solution_error_3d_m(&smoothed_solution, truth));
            raw_solutions.push(baseline_solution);
            smoothed_solutions.push(smoothed_solution);
        }

        assert!(root_mean_square(&smoothed_errors_m) < root_mean_square(&raw_errors_m) * 0.75);
        assert!(
            solution_trace_path_length_m(&smoothed_solutions)
                < solution_trace_path_length_m(&raw_solutions) * 0.4
        );
    }

    #[test]
    fn navigation_position_solution_smoothing_preserves_linear_motion() {
        let truth_origin = geodetic_to_ecef(37.0, -122.0, 25.0);
        let truth_velocity_mps = (8.0, -3.0, 1.5);
        let t0_rx_s = 100_190.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t0_rx_s),
            make_eph(2, 0.8, 0.9, t0_rx_s),
            make_eph(3, 1.6, 1.8, t0_rx_s),
            make_eph(4, 2.4, 2.7, t0_rx_s),
            make_eph(5, 3.2, 3.6, t0_rx_s),
        ];
        let mut baseline_config = ReceiverPipelineConfig::default();
        baseline_config.position_solution_smoothing = false;
        let mut baseline_nav =
            Navigation::new(baseline_config, crate::engine::runtime::ReceiverRuntime::default());
        let mut smoothed_nav = Navigation::new(
            ReceiverPipelineConfig::default(),
            crate::engine::runtime::ReceiverRuntime::default(),
        );
        let mut truth_solutions = Vec::new();
        let mut raw_errors_m = Vec::new();
        let mut smoothed_errors_m = Vec::new();
        let mut smoothed_solutions = Vec::new();

        for epoch_idx in 0..60u64 {
            let truth = translate_truth_ecef_m(truth_origin, truth_velocity_mps, epoch_idx as f64);
            let t_rx_s = t0_rx_s + epoch_idx as f64;
            let mut obs = make_obs_epoch_for_solution(epoch_idx, t_rx_s, truth, &ephs);
            apply_deterministic_pseudorange_offsets(&mut obs);
            let baseline_solution =
                baseline_nav.solve_epoch(&obs, &ephs).expect("baseline moving solution");
            let smoothed_solution =
                smoothed_nav.solve_epoch(&obs, &ephs).expect("smoothed moving solution");
            truth_solutions.push(NavSolutionEpoch {
                ecef_x_m: Meters(truth.0),
                ecef_y_m: Meters(truth.1),
                ecef_z_m: Meters(truth.2),
                ..sample_last_solution()
            });
            raw_errors_m.push(solution_error_3d_m(&baseline_solution, truth));
            smoothed_errors_m.push(solution_error_3d_m(&smoothed_solution, truth));
            smoothed_solutions.push(smoothed_solution);
        }

        let truth_path_length_m = solution_trace_path_length_m(&truth_solutions);
        let smoothed_path_length_m = solution_trace_path_length_m(&smoothed_solutions);
        let smoothed_final = smoothed_solutions.last().expect("smoothed final solution");
        let truth_final = translate_truth_ecef_m(truth_origin, truth_velocity_mps, 59.0);

        assert!(root_mean_square(&smoothed_errors_m) <= root_mean_square(&raw_errors_m));
        assert!(smoothed_path_length_m > truth_path_length_m * 0.9);
        assert!(smoothed_path_length_m < truth_path_length_m * 1.15);
        assert!(solution_error_3d_m(smoothed_final, truth_final) < 6.0);
    }

    #[test]
    fn deterministic_solution_transition_handles_refusal_and_regression() {
        assert_eq!(
            deterministic_solution_transition(
                Some(SolutionStatus::Converged),
                SolutionStatus::Coarse,
                None,
                false,
            ),
            SolutionStatus::Degraded
        );
        assert_eq!(
            deterministic_solution_transition(
                Some(SolutionStatus::Converged),
                SolutionStatus::Degraded,
                Some(NavRefusalClass::InsufficientGeometry),
                true,
            ),
            SolutionStatus::Degraded
        );
        assert_eq!(
            deterministic_solution_transition(
                Some(SolutionStatus::Converged),
                SolutionStatus::Coarse,
                Some(NavRefusalClass::SolverFailure),
                false,
            ),
            SolutionStatus::Refused
        );
    }

    #[test]
    fn solver_refusal_status_distinguishes_divergence_from_missing_inputs() {
        assert_eq!(
            solver_refusal_status(
                PositionSolveRefusalKind::UnderdeterminedRaimExclusion,
                NavRefusalClass::InsufficientGeometry,
            ),
            SolutionStatus::IntegrityFailed
        );
        assert_eq!(
            solver_refusal_status(
                PositionSolveRefusalKind::FilterDivergence(
                    PositionFilterDivergenceReason::CovarianceDivergence,
                ),
                NavRefusalClass::SolverFailure,
            ),
            SolutionStatus::Diverged
        );
        assert_eq!(
            solver_refusal_status(
                PositionSolveRefusalKind::SolverFailure,
                NavRefusalClass::SolverFailure,
            ),
            SolutionStatus::Diverged
        );
        assert_eq!(
            solver_refusal_status(
                PositionSolveRefusalKind::SolverFailure,
                NavRefusalClass::InvalidEphemeris,
            ),
            SolutionStatus::Unavailable
        );
    }

    #[test]
    fn scientific_policy_can_refuse_optimistic_solution_claims() {
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_100.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(17, t_rx_s, truth, &ephs);
        let baseline_solution = Navigation::new(
            ReceiverPipelineConfig::default(),
            crate::engine::runtime::ReceiverRuntime::default(),
        )
        .solve_epoch(&obs, &ephs)
        .expect("baseline solution");
        let mut config = ReceiverPipelineConfig::default();
        config.science_thresholds.max_pdop = baseline_solution.pdop - 0.1;
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());
        let solution = nav.solve_epoch(&obs, &ephs).expect("solution");
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InsufficientGeometry));
        assert_eq!(solution.explain_decision, "refused");
        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert_eq!(solution.position_covariance_ecef_m2, None);
        assert_eq!(solution.sigma_e_m, None);
        assert_eq!(solution.sigma_n_m, None);
        assert_eq!(solution.sigma_u_m, None);
        assert_eq!(solution.horizontal_error_ellipse_major_axis_m, None);
        assert_eq!(solution.horizontal_error_ellipse_minor_axis_m, None);
        assert_eq!(solution.horizontal_error_ellipse_azimuth_deg, None);
        assert_eq!(solution.sigma_h_m, None);
        assert_eq!(solution.sigma_v_m, None);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("pdop_above_threshold:")));
    }

    #[test]
    fn scientific_policy_can_refuse_high_gdop_geometry() {
        let truth = geodetic_to_ecef(37.0, -122.0, 25.0);
        let t_rx_s = 100_200.0;
        let ephs = vec![
            make_eph(1, 0.0, 0.0, t_rx_s),
            make_eph(2, 0.8, 0.9, t_rx_s),
            make_eph(3, 1.6, 1.8, t_rx_s),
            make_eph(4, 2.4, 2.7, t_rx_s),
        ];
        let obs = make_obs_epoch_for_solution(18, t_rx_s, truth, &ephs);
        let baseline_solution = Navigation::new(
            ReceiverPipelineConfig::default(),
            crate::engine::runtime::ReceiverRuntime::default(),
        )
        .solve_epoch(&obs, &ephs)
        .expect("baseline solution");
        let mut config = ReceiverPipelineConfig::default();
        config.science_thresholds.max_pdop = 100.0;
        config.science_thresholds.max_gdop = baseline_solution.gdop.expect("baseline gdop") - 0.1;
        let mut nav = Navigation::new(config, crate::engine::runtime::ReceiverRuntime::default());

        let solution = nav.solve_epoch(&obs, &ephs).expect("gdop refusal");

        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InsufficientGeometry));
        assert_eq!(solution.status, SolutionStatus::Refused);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("gdop_above_threshold:")));
    }
}
