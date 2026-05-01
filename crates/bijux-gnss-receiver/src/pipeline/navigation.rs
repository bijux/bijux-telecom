#![allow(missing_docs)]

use bijux_gnss_core::api::{
    check_nav_solution_sanity, is_solution_valid, obs_epoch_stability_key, Meters, NavAssumptions,
    NavLifecycleState, NavProvenance, NavRefusalClass, NavResidual, NavSolutionEpoch,
    NavUncertaintyClass, ObsEpoch, Seconds, SolutionStatus, SolutionValidity,
    NAV_SOLUTION_MODEL_VERSION,
};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, GpsEphemeris,
    PositionObservation, PositionSolver, WeightingConfig,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
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
        solver.robust = config.robust_solver;
        solver.huber_k = config.huber_k;
        solver.raim = config.raim;
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
        let source_observation_epoch_id = source_observation_epoch_id(obs);
        let nav_artifact_id = nav_artifact_id(obs.epoch_idx, &source_observation_epoch_id);
        let assumptions = nav_assumptions(eph.len());
        if !obs.valid {
            return Some(invalid_solution_epoch(
                obs,
                source_observation_epoch_id,
                nav_artifact_id,
                Some(NavRefusalClass::InconsistentObservations),
                "invalid_observation_epoch".to_string(),
                vec!["input_observation_marked_invalid".to_string()],
                assumptions,
            ));
        }
        let observations: Vec<PositionObservation> = obs
            .sats
            .iter()
            .filter_map(|s| {
                let mut elevation = s.elevation_deg;
                if elevation.is_none() {
                    if let Some((rx_x, rx_y, rx_z)) = self.last_ecef {
                        if let Some(eph) = eph.iter().find(|e| e.sat == s.signal_id.sat) {
                            let sat = sat_state_gps_l1ca(eph, obs.t_rx_s.0, 0.0);
                            let (_az, el) =
                                elevation_azimuth_deg(rx_x, rx_y, rx_z, sat.x_m, sat.y_m, sat.z_m);
                            elevation = Some(el);
                        }
                    }
                }
                let weight_config = WeightingConfig {
                    enabled: self.config.weighting.enabled,
                    min_elev_deg: self.config.weighting.min_elev_deg,
                    elev_exponent: self.config.weighting.elev_exponent,
                    cn0_ref_dbhz: self.config.weighting.cn0_ref_dbhz,
                    min_weight: self.config.weighting.min_weight,
                };
                if let Some(el) = elevation {
                    if el < self.config.weighting.elev_mask_deg {
                        return None;
                    }
                }
                let tracking_mode_weight = if s.metadata.tracking_mode.contains("vector") {
                    self.config.weighting.tracking_mode_vector_weight
                } else {
                    self.config.weighting.tracking_mode_scalar_weight
                };
                let weight = elevation
                    .map(|el| weight_from_cn0_elev(s.cn0_dbhz, el, weight_config))
                    .unwrap_or(1.0);
                Some(PositionObservation {
                    sat: s.signal_id.sat,
                    pseudorange_m: s.pseudorange_m.0,
                    cn0_dbhz: s.cn0_dbhz,
                    elevation_deg: elevation,
                    weight: weight * tracking_mode_weight,
                })
            })
            .collect();
        if observations.len() < 4 {
            self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_INSUFFICIENT_SATS",
                "insufficient satellites for navigation solution",
            ));
            return Some(self.degraded_from_last(
                obs,
                source_observation_epoch_id,
                nav_artifact_id,
                NavDecision {
                    status: SolutionStatus::Degraded,
                    refusal_class: Some(NavRefusalClass::InsufficientGeometry),
                    explain_decision: "refused".to_string(),
                    explain_reasons: vec!["insufficient_geometry".to_string()],
                },
                assumptions,
            ));
        }
        let eph_covered_count =
            observations.iter().filter(|row| eph.iter().any(|entry| entry.sat == row.sat)).count();
        let solution = match self.solver.solve_wls(&observations, eph, obs.t_rx_s.0) {
            Some(solution) => solution,
            None => {
                self.runtime.logger.event(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "NAV_SOLVER_FAILED",
                    "nav solver failed to converge",
                ));
                let refusal_class = if eph_covered_count < 4 {
                    NavRefusalClass::InvalidEphemeris
                } else {
                    NavRefusalClass::SolverFailure
                };
                return Some(self.degraded_from_last(
                    obs,
                    source_observation_epoch_id,
                    nav_artifact_id,
                    NavDecision {
                        status: SolutionStatus::Degraded,
                        refusal_class: Some(refusal_class),
                        explain_decision: "refused".to_string(),
                        explain_reasons: vec![
                            "position_solver_failed".to_string(),
                            format!("ephemeris_covered_count={eph_covered_count}"),
                        ],
                    },
                    assumptions,
                ));
            }
        };
        let (clock_bias_s, clock_drift_s_per_s) = self.clock.update(solution.clock_bias_s, 0.001);
        self.last_ecef = Some((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        let mut nav_epoch = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
            t_rx_s: obs.t_rx_s,
            ecef_x_m: Meters(solution.ecef_x_m),
            ecef_y_m: Meters(solution.ecef_y_m),
            ecef_z_m: Meters(solution.ecef_z_m),
            latitude_deg: solution.latitude_deg,
            longitude_deg: solution.longitude_deg,
            altitude_m: Meters(solution.altitude_m),
            clock_bias_s: Seconds(clock_bias_s),
            clock_drift_s_per_s,
            pdop: solution.pdop,
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
            isb: Vec::new(),
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
        };

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

        if let Some(sep) = solution.separation_max_m {
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
        let decision = decision_for_solution(&nav_epoch);
        nav_epoch.explain_decision = decision.explain_decision;
        nav_epoch.explain_reasons = decision.explain_reasons;
        nav_epoch.refusal_class = decision.refusal_class;
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
        degraded
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
    NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch { index: obs.epoch_idx },
        t_rx_s: obs.t_rx_s,
        ecef_x_m: Meters(0.0),
        ecef_y_m: Meters(0.0),
        ecef_z_m: Meters(0.0),
        latitude_deg: 0.0,
        longitude_deg: 0.0,
        altitude_m: Meters(0.0),
        clock_bias_s: Seconds(0.0),
        clock_drift_s_per_s: 0.0,
        pdop: 0.0,
        rms_m: Meters(0.0),
        status: SolutionStatus::Invalid,
        quality: SolutionStatus::Invalid.quality_flag(),
        validity: SolutionValidity::Invalid,
        valid: false,
        processing_ms: None,
        residuals: Vec::new(),
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
    }
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
    NavAssumptions {
        time_system: "gps".to_string(),
        ephemeris_source: "broadcast_lnav".to_string(),
        frame_decode_mode: "lnav".to_string(),
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
        .or_else(|| solution.innovation_rms_m)
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
    let weighting_mode =
        if config.weighting.enabled { "cn0_elevation_weighted" } else { "uniform" };
    let solver_family =
        if observations.iter().any(|row| row.weight < 0.999) { "wls_weighted" } else { "wls" };
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
        Constellation, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ObservationSupportClass,
        ObservationUncertaintyClass, ReceiverRole, SatId, Seconds, SigId, SignalBand, SignalCode,
        SignalSpec,
    };
    use bijux_gnss_nav::api::{geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris};

    fn sample_last_solution() -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: 0 },
            t_rx_s: Seconds(0.0),
            ecef_x_m: Meters(1.0),
            ecef_y_m: Meters(2.0),
            ecef_z_m: Meters(3.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: Meters(0.0),
            clock_bias_s: Seconds(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            rms_m: Meters(1.0),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
            validity: bijux_gnss_core::api::SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            residuals: Vec::new(),
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
        }
    }

    fn make_eph(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 0,
            iode: 0,
            week: 0,
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

    fn make_obs_epoch_for_solution(
        epoch_idx: u64,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
        ephs: &[GpsEphemeris],
    ) -> ObsEpoch {
        let sats = ephs
            .iter()
            .map(|eph| {
                let pseudorange_m = synthetic_pseudorange_m(eph, t_rx_s, position_ecef);
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
        ObsEpoch {
            t_rx_s: Seconds(t_rx_s),
            gps_week: None,
            tow_s: None,
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats,
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: Some(bijux_gnss_core::api::ObsEpochManifest {
                version: bijux_gnss_core::api::OBSERVATION_MODEL_VERSION,
                artifact_id: format!("obs-epoch-{epoch_idx:010}"),
                epoch_id: format!("obs-epoch-{epoch_idx:010}-synthetic"),
                source_epoch_idx: epoch_idx,
                source_sample_index: epoch_idx,
                decision: ObservationEpochDecision::Accepted,
                downstream_profile_version:
                    bijux_gnss_core::api::OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
            }),
        }
    }

    fn synthetic_pseudorange_m(
        eph: &GpsEphemeris,
        t_rx_s: f64,
        position_ecef: (f64, f64, f64),
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
            pseudorange_m = range - sat.clock_bias_s * c;
            let next_tau = pseudorange_m / c;
            if (next_tau - tau).abs() < 1e-12 {
                break;
            }
            tau = next_tau;
        }
        pseudorange_m
    }

    #[test]
    fn navigation_degrades_on_solver_failure() {
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
        assert_eq!(solution.status, SolutionStatus::Held);
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
        assert!(solution.provenance.is_some());
        assert_eq!(solution.explain_decision, "accepted");
        assert_eq!(solution.refusal_class, None);
        assert!(solution.valid);
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
}
