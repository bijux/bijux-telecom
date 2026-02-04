#![allow(missing_docs)]

use bijux_gnss_core::{
    check_nav_solution_sanity, is_solution_valid, Meters, NavResidual, NavSolutionEpoch, ObsEpoch,
    Seconds, SolutionStatus,
};
use bijux_gnss_nav::{
    elevation_azimuth_deg, sat_state_gps_l1ca, weight_from_cn0_elev, GpsEphemeris,
    PositionObservation, PositionSolver, WeightingConfig,
};

use crate::ReceiverConfig;

/// Navigation solution derived from observation epochs.
pub struct Navigation {
    #[allow(dead_code)]
    config: ReceiverConfig,
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
    pub fn new(config: ReceiverConfig) -> Self {
        let mut solver = PositionSolver::new();
        solver.robust = config.robust_solver;
        solver.huber_k = config.huber_k;
        solver.raim = config.raim;
        Self {
            config,
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
        let start_time = std::time::Instant::now();
        if observations.len() < 4 {
            crate::logging::diagnostic(&bijux_gnss_core::DiagnosticEvent::new(
                bijux_gnss_core::DiagnosticSeverity::Warning,
                "NAV_INSUFFICIENT_SATS",
                "insufficient satellites for navigation solution",
            ));
            return self.degraded_from_last(obs, start_time);
        }
        let solution = match self.solver.solve_wls(&observations, eph, obs.t_rx_s.0) {
            Some(solution) => solution,
            None => {
                crate::logging::diagnostic(&bijux_gnss_core::DiagnosticEvent::new(
                    bijux_gnss_core::DiagnosticSeverity::Warning,
                    "NAV_SOLVER_FAILED",
                    "nav solver failed to converge",
                ));
                return self.degraded_from_last(obs, start_time);
            }
        };
        let clock_bias_s = self.clock.update(solution.clock_bias_s, 0.001);
        self.last_ecef = Some((solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m));
        let mut nav_epoch = NavSolutionEpoch {
            epoch: bijux_gnss_core::Epoch {
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
            pdop: solution.pdop,
            rms_m: Meters(solution.rms_m),
            status: SolutionStatus::Coarse,
            valid: true,
            processing_ms: Some(start_time.elapsed().as_secs_f64() * 1000.0),
            sigma_h_m: solution.sigma_h_m.map(Meters),
            sigma_v_m: solution.sigma_v_m.map(Meters),
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            residuals: solution
                .residuals
                .into_iter()
                .map(|(sat, residual_m, weight)| NavResidual {
                    sat,
                    residual_m: Meters(residual_m),
                    rejected: false,
                    weight: Some(weight),
                })
                .chain(solution.rejected.into_iter().map(|sat| NavResidual {
                    sat,
                    residual_m: Meters(0.0),
                    rejected: true,
                    weight: None,
                }))
                .collect(),
            isb: Vec::new(),
        };

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
                crate::logging::diagnostic(&bijux_gnss_core::DiagnosticEvent::new(
                    bijux_gnss_core::DiagnosticSeverity::Warning,
                    "NAV_RAIM_SEPARATION",
                    format!("solution separation exceeded: {:.2} m", sep),
                ));
            }
        }

        let sanity_events = check_nav_solution_sanity(self.last_solution.as_ref(), &nav_epoch);
        if sanity_events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::DiagnosticSeverity::Error))
        {
            nav_epoch.status = SolutionStatus::Degraded;
            nav_epoch.valid = false;
            for event in sanity_events {
                crate::logging::diagnostic(&event);
            }
        } else {
            nav_epoch.valid = is_solution_valid(nav_epoch.status);
        }

        self.last_solution = Some(nav_epoch.clone());
        Some(nav_epoch)
    }

    fn degraded_from_last(
        &self,
        obs: &ObsEpoch,
        start_time: std::time::Instant,
    ) -> Option<NavSolutionEpoch> {
        let mut degraded = self.last_solution.clone()?;
        degraded.epoch = bijux_gnss_core::Epoch {
            index: obs.epoch_idx,
        };
        degraded.t_rx_s = obs.t_rx_s;
        degraded.status = SolutionStatus::Degraded;
        degraded.valid = is_solution_valid(degraded.status);
        degraded.processing_ms = Some(start_time.elapsed().as_secs_f64() * 1000.0);
        degraded.residuals.clear();
        Some(degraded)
    }
}

impl bijux_gnss_nav::NavEngine for Navigation {
    fn update(
        &mut self,
        obs: &bijux_gnss_core::ObsEpochV1,
    ) -> Option<bijux_gnss_core::NavSolutionEpochV1> {
        let solution = self.solve_epoch(&obs.payload, &[])?;
        Some(bijux_gnss_core::NavSolutionEpochV1 {
            header: obs.header.clone(),
            payload: solution,
        })
    }
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

    fn update(&mut self, measurement_bias_s: f64, dt_s: f64) -> f64 {
        let alpha = 0.1;
        let beta = 0.01;
        let mut bias = self.bias_s + self.drift_s * dt_s;
        let residual = measurement_bias_s - bias;
        bias += alpha * residual;
        let drift = self.drift_s + (beta * residual / dt_s.max(1e-6));
        self.bias_s = bias;
        self.drift_s = drift;
        self.bias_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsMetadata, ObsSatellite, ReceiverRole,
        SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    fn fake_obs_epoch(epoch_idx: u64) -> ObsEpoch {
        let sats = (1..=4)
            .map(|prn| ObsSatellite {
                signal_id: SigId {
                    sat: SatId {
                        constellation: Constellation::Gps,
                        prn,
                    },
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                },
                pseudorange_m: Meters(20_200_000.0 + prn as f64),
                pseudorange_var_m2: 100.0,
                carrier_phase_cycles: Cycles(0.0),
                carrier_phase_var_cycles2: 1.0,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                elevation_deg: Some(45.0),
                azimuth_deg: Some(0.0),
                weight: Some(1.0),
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "scalar".to_string(),
                    integration_ms: 1,
                    lock_quality: 1.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: bijux_gnss_core::signal_spec_gps_l1_ca(),
                },
            })
            .collect();
        ObsEpoch {
            t_rx_s: Seconds(epoch_idx as f64 * 0.001),
            gps_week: None,
            tow_s: None,
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats,
        }
    }

    #[test]
    fn navigation_degrades_on_solver_failure() {
        let config = ReceiverConfig::default();
        let mut nav = Navigation {
            config,
            solver: PositionSolver::new(),
            clock: ClockModel::new(),
            last_ecef: Some((1.0, 2.0, 3.0)),
            last_solution: Some(NavSolutionEpoch {
                epoch: bijux_gnss_core::Epoch { index: 0 },
                t_rx_s: Seconds(0.0),
                ecef_x_m: Meters(1.0),
                ecef_y_m: Meters(2.0),
                ecef_z_m: Meters(3.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: Meters(0.0),
                clock_bias_s: Seconds(0.0),
                pdop: 1.0,
                rms_m: Meters(1.0),
                status: SolutionStatus::Converged,
                valid: true,
                processing_ms: None,
                residuals: Vec::new(),
                isb: Vec::new(),
                sigma_h_m: None,
                sigma_v_m: None,
                ekf_innovation_rms: None,
                ekf_condition_number: None,
                ekf_whiteness_ratio: None,
                ekf_predicted_variance: None,
                ekf_observed_variance: None,
            }),
        };
        let obs = fake_obs_epoch(10);
        let solution = nav.solve_epoch(&obs, &[]).expect("degraded solution");
        assert_eq!(solution.status, SolutionStatus::Degraded);
        assert_eq!(solution.epoch.index, 10);
    }
}
