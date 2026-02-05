#![allow(missing_docs)]

use bijux_gnss_core::api::{
    check_nav_solution_sanity, is_solution_valid, Meters, NavResidual, NavSolutionEpoch, ObsEpoch,
    Seconds, SolutionStatus,
};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, GpsEphemeris,
    PositionObservation, PositionSolver, WeightingConfig,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::engine::runtime_context::ReceiverRuntimeConfig;

/// Navigation solution derived from observation epochs.
pub struct Navigation {
    #[allow(dead_code)]
    config: ReceiverPipelineConfig,
    runtime: ReceiverRuntimeConfig,
    solver: PositionSolver,
    clock: ClockModel,
    last_ecef: Option<(f64, f64, f64)>,
    last_solution: Option<NavSolutionEpoch>,
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
        Self {
            state: NavigationState::ColdStart,
        }
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
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntimeConfig) -> Self {
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
        if !obs.valid {
            return None;
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
            crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_INSUFFICIENT_SATS",
                "insufficient satellites for navigation solution",
            ));
            crate::engine::diagnostics::dump_on_error(
                &self.runtime,
                "nav insufficient satellites",
                Some(obs),
                self.last_solution.as_ref(),
            );
            return self.degraded_from_last(obs);
        }
        let solution = match self.solver.solve_wls(&observations, eph, obs.t_rx_s.0) {
            Some(solution) => solution,
            None => {
                crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "NAV_SOLVER_FAILED",
                    "nav solver failed to converge",
                ));
                crate::engine::diagnostics::dump_on_error(
                    &self.runtime,
                    "nav solver failed to converge",
                    Some(obs),
                    self.last_solution.as_ref(),
                );
                return self.degraded_from_last(obs);
            }
        };
        let (clock_bias_s, clock_drift_s_per_s) = self.clock.update(solution.clock_bias_s, 0.001);
        self.last_ecef = Some((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        let mut nav_epoch = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch {
                index: obs.epoch_idx,
            },
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
            validity: bijux_gnss_core::api::SolutionValidity::Invalid,
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
        };

        if solution.covariance_symmetrized {
            nav_epoch
                .health
                .push(bijux_gnss_core::api::NavHealthEvent::CovarianceSymmetrized);
            crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_COV_SYMM",
                "nav covariance symmetrized",
            ));
        }
        if solution.covariance_clamped {
            nav_epoch.health.push(
                bijux_gnss_core::api::NavHealthEvent::CovarianceClamped {
                    min_eigenvalue: 0.0,
                },
            );
            crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Warning,
                "NAV_COV_CLAMP",
                "nav covariance clamped",
            ));
        }
        if let Some(max_var) = solution.covariance_max_variance {
            if max_var > 1e6 {
                nav_epoch.health.push(
                    bijux_gnss_core::api::NavHealthEvent::CovarianceDiverged {
                        max_variance: max_var,
                    },
                );
            }
        }

        let sat_count = observations.len();
        nav_epoch.status = if sat_count < 4 {
            SolutionStatus::Degraded
        } else if nav_epoch.rms_m.0 < 10.0 {
            SolutionStatus::Converged
        } else {
            SolutionStatus::Coarse
        };

        if let Some(sep) = solution.separation_max_m {
            if sep > self.solver.separation_gate_m {
                nav_epoch.status = SolutionStatus::Degraded;
                crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
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
            for event in sanity_events {
                crate::engine::logging::diagnostic(&self.runtime, &event);
            }
            crate::engine::diagnostics::dump_on_error(
                &self.runtime,
                "nav solution sanity failed",
                Some(obs),
                Some(&nav_epoch),
            );
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
        if let Some(sigma_h) = nav_epoch.sigma_h_m {
            nav_epoch.integrity_hpl_m = Some(sigma_h.0 * 6.0);
        }
        if let Some(sigma_v) = nav_epoch.sigma_v_m {
            nav_epoch.integrity_vpl_m = Some(sigma_v.0 * 6.0);
        }

        if let Some(rms) = nav_epoch.innovation_rms_m {
            crate::engine::logging::diagnostic(&self.runtime, &bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Info,
                "NAV_INNOVATION_RMS",
                format!("innovation rms {:.3} m", rms),
            ));
        }

        self.last_solution = Some(nav_epoch.clone());
        Some(nav_epoch)
    }

    fn degraded_from_last(&self, obs: &ObsEpoch) -> Option<NavSolutionEpoch> {
        let mut degraded = self.last_solution.clone()?;
        degraded.epoch = bijux_gnss_core::api::Epoch {
            index: obs.epoch_idx,
        };
        degraded.t_rx_s = obs.t_rx_s;
        degraded.status = SolutionStatus::Degraded;
        degraded.quality = degraded.status.quality_flag();
        degraded.valid = is_solution_valid(degraded.status);
        degraded.validity = bijux_gnss_core::api::SolutionValidity::Coarse;
        degraded.processing_ms = None;
        degraded.residuals.clear();
        Some(degraded)
    }
}

impl bijux_gnss_nav::api::NavEngine for Navigation {
    fn update(
        &mut self,
        obs: &bijux_gnss_core::api::ObsEpochV1,
    ) -> bijux_gnss_core::api::NavSolutionEpochV1 {
        let solution = self
            .solve_epoch(&obs.payload, &[])
            .unwrap_or_else(|| invalid_solution_epoch(&obs.payload));
        bijux_gnss_core::api::NavSolutionEpochV1 {
            header: obs.header.clone(),
            payload: solution,
        }
    }
}

fn invalid_solution_epoch(obs: &ObsEpoch) -> NavSolutionEpoch {
    NavSolutionEpoch {
        epoch: bijux_gnss_core::api::Epoch {
            index: obs.epoch_idx,
        },
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
        validity: bijux_gnss_core::api::SolutionValidity::Invalid,
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
    }
}

fn innovation_stats(
    residuals: &[NavResidual],
) -> (Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>) {
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
    let norm_rms = if sum_norm > 0.0 {
        Some((sum_norm / count).sqrt())
    } else {
        None
    };
    let max_norm_opt = if max_norm > 0.0 { Some(max_norm) } else { None };
    let pred_var = if sum_pred > 0.0 {
        Some(sum_pred / count)
    } else {
        None
    };
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
        Self {
            bias_s: 0.0,
            drift_s: 0.0,
        }
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
    use bijux_gnss_core::api::{Meters, Seconds};

    #[test]
    fn navigation_degrades_on_solver_failure() {
        let config = ReceiverPipelineConfig::default();
        let mut nav = Navigation {
            config,
            solver: PositionSolver::new(),
            clock: ClockModel::new(),
            last_ecef: Some((1.0, 2.0, 3.0)),
            last_solution: Some(NavSolutionEpoch {
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
            }),
        };
        let obs = fake_obs_epoch_for_nav_tests(10);
        let solution = nav.solve_epoch(&obs, &[]).expect("degraded solution");
        assert_eq!(solution.status, SolutionStatus::Degraded);
        assert_eq!(solution.epoch.index, 10);
    }
}
