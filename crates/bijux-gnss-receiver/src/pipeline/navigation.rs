#![allow(missing_docs)]

use bijux_gnss_core::api::{
    check_nav_solution_sanity, is_solution_valid, obs_epoch_stability_key, Constellation,
    MeasurementRejectReason, Meters, NavAssumptions, NavLifecycleState, NavProvenance,
    NavRefusalClass, NavResidual, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch, ObsSatellite,
    Seconds, SolutionStatus, SolutionValidity,
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, position_broadcast_navigation_from_gps_ephemerides,
    position_measurement_weight, position_observation_has_valid_satellite_time,
    sat_state_beidou_b1i_from_observation, sat_state_galileo_e1_from_observation,
    sat_state_glonass_l1_from_observation, sat_state_gps_l1ca_from_observation, GpsEphemeris,
    KlobucharCoefficients, PositionBroadcastNavigation, PositionObservation,
    PositionRobustWeighting, PositionSolveRefusalKind, PositionSolver, PositionWeightingModel,
    RaimFaultDetectionStatus, WeightingConfig,
};

use crate::engine::receiver_config::{NavigationWeightingMode, ReceiverPipelineConfig};
use crate::engine::runtime::ReceiverRuntime;

/// Navigation solution derived from observation epochs.
pub struct Navigation {
    #[allow(dead_code)]
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntime,
    solver: PositionSolver,
    clock: ClockModel,
    last_ecef: Option<(f64, f64, f64)>,
    last_solution: Option<NavSolutionEpoch>,
}

#[derive(Debug, Clone)]
struct NavDecision {
    status: SolutionStatus,
    refusal_class: Option<NavRefusalClass>,
    explain_decision: String,
    explain_reasons: Vec<String>,
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
        Self {
            config,
            runtime,
            solver,
            clock: ClockModel::new(),
            last_ecef: None,
            last_solution: None,
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
            .filter_map(|s| match prepare_position_observation(
                &self.config,
                obs,
                s,
                navigation,
                self.last_ecef,
            ) {
                PositionObservationPreparation::Included(observation) => Some(observation),
                PositionObservationPreparation::InvalidSatelliteTime(sat) => {
                    invalid_satellite_time_sats.push(sat);
                    None
                }
                PositionObservationPreparation::ExcludedByElevationMask => None,
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
                };
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
                return Some(apply_atmosphere_explainability(
                    if is_sparse_refusal {
                        self.current_epoch_refusal(
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
                        )
                    } else {
                        self.degraded_from_last(
                            obs,
                            source_observation_epoch_id,
                            nav_artifact_id,
                            NavDecision {
                                status: SolutionStatus::Degraded,
                                refusal_class: Some(refusal_class),
                                explain_decision: "refused".to_string(),
                                explain_reasons,
                            },
                            assumptions,
                        )
                    },
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
        self.last_ecef = Some((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        let inter_system_biases = solution.inter_system_biases.clone();
        let mut nav_epoch = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
            t_rx_s: obs.t_rx_s,
            source_time: obs.source_time,
            ecef_x_m: Meters(solution.ecef_x_m),
            ecef_y_m: Meters(solution.ecef_y_m),
            ecef_z_m: Meters(solution.ecef_z_m),
            latitude_deg: solution.latitude_deg,
            longitude_deg: solution.longitude_deg,
            altitude_m: Meters(solution.altitude_m),
            clock_bias_s: Seconds(solution.clock_bias_s),
            clock_bias_m: Meters(solution.clock_bias_s * 299_792_458.0),
            clock_drift_s_per_s,
            pdop: solution.pdop,
            pre_fit_residual_rms_m: Some(Meters(solution.pre_fit_residual_rms_m)),
            post_fit_residual_rms_m: Some(Meters(solution.post_fit_residual_rms_m)),
            rms_m: Meters(solution.rms_m),
            status: SolutionStatus::Coarse,
            quality: SolutionStatus::Coarse.quality_flag(),
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
            lifecycle_state: NavLifecycleState::Coarse,
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
            }
        }

        let sat_count = observations.len();
        let provisional_status = if sat_count < 4 {
            SolutionStatus::Degraded
        } else if nav_epoch.rms_m.0 < 10.0 {
            SolutionStatus::Converged
        } else {
            SolutionStatus::Coarse
        };
        nav_epoch.status = deterministic_solution_transition(
            self.last_solution.as_ref().map(|row| row.status),
            provisional_status,
            None,
            false,
        );

        let mut raim_explain_reasons = Vec::new();
        let mut resolved_by_raim_exclusion = false;
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
        } else if let Some(sep) = solution.separation_max_m {
            if sep > self.solver.separation_gate_m {
                nav_epoch.status = SolutionStatus::Degraded;
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "NAV_RAIM_SEPARATION",
                    format!("solution separation exceeded: {:.2} m", sep),
                ));
            }
        }

        let sanity_events = check_nav_solution_sanity(self.last_solution.as_ref(), &nav_epoch);
        if sanity_events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch.valid = false;
            nav_epoch.refusal_class = Some(NavRefusalClass::InconsistentObservations);
            for event in sanity_events {
                self.runtime.logger.event(&event);
            }
        } else {
            nav_epoch.valid = is_solution_valid(nav_epoch.status);
        }
        nav_epoch.quality = nav_epoch.status.quality_flag();

        let (innovation_rms, norm_rms, norm_max, predicted_var, observed_var) =
            innovation_stats(&nav_epoch.residuals);
        nav_epoch.innovation_rms_m = innovation_rms;
        nav_epoch.normalized_innovation_rms = norm_rms;
        nav_epoch.normalized_innovation_max = norm_max;
        nav_epoch.ekf_predicted_variance = predicted_var;
        nav_epoch.ekf_observed_variance = observed_var;
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
            klobuchar,
            self.config.tropo_enable,
        );
        for reason in raim_explain_reasons {
            if !nav_epoch.explain_reasons.iter().any(|existing| existing == &reason) {
                nav_epoch.explain_reasons.push(reason);
            }
        }
        nav_epoch.stability_signature = nav_output_stability_signature(&nav_epoch);
        if let Some(sigma_h) = nav_epoch.sigma_h_m {
            nav_epoch.integrity_hpl_m = Some(sigma_h.0 * 6.0);
        }
        if let Some(sigma_v) = nav_epoch.sigma_v_m {
            nav_epoch.integrity_vpl_m = Some(sigma_v.0 * 6.0);
        }

        if let Some(rms) = nav_epoch.innovation_rms_m {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Info,
                "NAV_INNOVATION_RMS",
                format!("innovation rms {:.3} m", rms),
            ));
        }

        self.last_solution = Some(nav_epoch.clone());
        Some(nav_epoch)
    }

    fn degraded_from_last(
        &self,
        obs: &ObsEpoch,
        source_observation_epoch_id: String,
        nav_artifact_id: String,
        decision: NavDecision,
        assumptions: NavAssumptions,
    ) -> NavSolutionEpoch {
        let mut degraded = self.last_solution.clone().unwrap_or_else(|| {
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
        degraded.epoch = bijux_gnss_core::api::Epoch { index: obs.epoch_idx };
        degraded.t_rx_s = obs.t_rx_s;
        degraded.source_time = obs.source_time;
        degraded.status = deterministic_solution_transition(
            self.last_solution.as_ref().map(|row| row.status),
            decision.status,
            decision.refusal_class,
            true,
        );
        degraded.lifecycle_state = lifecycle_state_from_status(degraded.status);
        degraded.quality = degraded.status.quality_flag();
        degraded.valid = is_solution_valid(degraded.status);
        degraded.validity = SolutionValidity::Coarse;
        degraded.processing_ms = None;
        degraded.residuals.clear();
        degraded.assumptions = Some(assumptions);
        degraded.source_observation_epoch_id = source_observation_epoch_id;
        degraded.artifact_id = nav_artifact_id;
        degraded.explain_decision = decision.explain_decision;
        degraded.explain_reasons = decision.explain_reasons;
        degraded.refusal_class = decision.refusal_class;
        degraded.stability_signature = nav_output_stability_signature(&degraded);
        degraded
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
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) -> NavSolutionEpoch {
    apply_atmosphere_explainability_in_place(&mut solution, klobuchar, tropo_enabled);
    solution
}

fn apply_atmosphere_explainability_in_place(
    solution: &mut NavSolutionEpoch,
    klobuchar: Option<&KlobucharCoefficients>,
    tropo_enabled: bool,
) {
    let ionosphere_reason = if klobuchar.is_some() {
        "ionosphere_correction=klobuchar_broadcast"
    } else {
        "ionosphere_uncorrected"
    };
    if !solution.explain_reasons.iter().any(|existing| existing == ionosphere_reason) {
        solution.explain_reasons.push(ionosphere_reason.to_string());
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
    let mut solution = NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        source_time: obs.source_time,
        ecef_x_m: Meters(0.0),
        ecef_y_m: Meters(0.0),
        ecef_z_m: Meters(0.0),
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
        status: SolutionStatus::Invalid,
        quality: SolutionStatus::Invalid.quality_flag(),
        validity: SolutionValidity::Invalid,
        valid: false,
        processing_ms: None,
        residuals: Vec::new(),
        constellation_residual_rms: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
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
        lifecycle_state: NavLifecycleState::Invalid,
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
    solution.clock_bias_s = Seconds(0.0);
    solution.clock_bias_m = Meters(0.0);
    solution.clock_drift_s_per_s = 0.0;
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
        return SolutionStatus::Invalid;
    }
    if refusal_class.is_some() && reused_previous {
        return SolutionStatus::Held;
    }
    match (previous, proposed) {
        (_, SolutionStatus::Invalid) => SolutionStatus::Invalid,
        (Some(SolutionStatus::Converged), SolutionStatus::Coarse) => SolutionStatus::Degraded,
        (Some(SolutionStatus::Fixed), SolutionStatus::Coarse) => SolutionStatus::Degraded,
        (_, other) => other,
    }
}

fn lifecycle_state_from_status(status: SolutionStatus) -> NavLifecycleState {
    match status {
        SolutionStatus::Invalid => NavLifecycleState::Invalid,
        SolutionStatus::Held => NavLifecycleState::Held,
        SolutionStatus::Degraded => NavLifecycleState::Degraded,
        SolutionStatus::Coarse => NavLifecycleState::Coarse,
        SolutionStatus::Converged => NavLifecycleState::Converged,
        SolutionStatus::Float => NavLifecycleState::Float,
        SolutionStatus::Fixed => NavLifecycleState::Fixed,
    }
}

fn uncertainty_class_from_solution(solution: &NavSolutionEpoch) -> NavUncertaintyClass {
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
    let pseudorange_sigma_m =
        (sat.pseudorange_var_m2.is_finite() && sat.pseudorange_var_m2 > 0.0)
            .then_some(sat.pseudorange_var_m2.sqrt());
    let weight = position_measurement_weight(
        Some(sat.cn0_dbhz),
        elevation,
        pseudorange_sigma_m,
        WeightingConfig {
            model: match config.weighting.mode {
                NavigationWeightingMode::Elevation => PositionWeightingModel::Elevation,
                NavigationWeightingMode::Cn0 => PositionWeightingModel::Cn0,
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

fn decision_for_solution(solution: &NavSolutionEpoch) -> NavDecision {
    if solution.refusal_class.is_some() || !solution.valid {
        return NavDecision {
            status: solution.status,
            refusal_class: solution.refusal_class,
            explain_decision: "refused".to_string(),
            explain_reasons: vec!["solution_marked_invalid".to_string()],
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
        let high_elevation = sample_pipeline_position_satellite(sat, Some(75.0), "scalar");
        let low_elevation = sample_pipeline_position_satellite(sat, Some(10.0), "scalar");

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
        let low_elevation = sample_pipeline_position_satellite(sat, Some(10.0), "scalar");

        let preparation =
            prepare_position_observation(&config, &obs_epoch, &low_elevation, &[], None);

        assert!(matches!(
            preparation,
            PositionObservationPreparation::ExcludedByElevationMask
        ));
    }

    #[test]
    fn navigation_refuses_epochs_without_ephemeris_even_with_last_solution() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation {
            config,
            runtime: crate::engine::runtime::ReceiverRuntime::default(),
            solver: PositionSolver::new(),
            clock: ClockModel::new(),
            last_ecef: Some((1.0, 2.0, 3.0)),
            last_solution: Some(sample_last_solution()),
        };
        let obs = fake_obs_epoch_for_nav_tests(10);
        let solution = nav.solve_epoch(&obs, &[]).expect("degraded solution");
        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
        assert_eq!(solution.refusal_class, Some(NavRefusalClass::InconsistentObservations));
        assert_eq!(solution.explain_decision, "invalid_observation_epoch");
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason == "input_observation_marked_invalid"));
        assert_eq!(solution.lifecycle_state, NavLifecycleState::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
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

        assert!(solution.valid);
        assert!(matches!(solution.status, SolutionStatus::Coarse | SolutionStatus::Converged));
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
            clock: ClockModel::new(),
            last_ecef: Some((10.0, 20.0, 30.0)),
            last_solution: Some(sample_last_solution()),
        };
        let mut obs = fake_obs_epoch_for_nav_tests(18);
        obs.sats.truncate(3);

        let solution = nav.solve_epoch(&obs, &[]).expect("sparse refusal");

        assert_eq!(solution.status, SolutionStatus::Invalid);
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
        assert!(solution.valid);
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

        assert_eq!(solution.status, SolutionStatus::Invalid);
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
            SolutionStatus::Held
        );
        assert_eq!(
            deterministic_solution_transition(
                Some(SolutionStatus::Converged),
                SolutionStatus::Coarse,
                Some(NavRefusalClass::SolverFailure),
                false,
            ),
            SolutionStatus::Invalid
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
        assert_eq!(solution.ecef_x_m.0, 0.0);
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
        assert_eq!(solution.status, SolutionStatus::Invalid);
        assert_eq!(solution.ecef_x_m.0, 0.0);
        assert!(solution
            .explain_reasons
            .iter()
            .any(|reason| reason.starts_with("gdop_above_threshold:")));
    }
}
