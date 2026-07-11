#![allow(missing_docs)]

use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
use crate::orbits::gps::{
    is_ephemeris_valid, sat_state_gps_l1ca, sat_state_gps_l1ca_from_observation, GpsEphemeris,
    GpsSatState,
};
use crate::{RaimFaultDetection, RaimFaultExclusion};
use bijux_gnss_core::api::{
    GpsTime, Llh, MeasurementRejectReason, ObsSignalTiming, SatId, Seconds,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SIGNAL_TIMING_CONSISTENCY_TOLERANCE_S: f64 = 1.0e-6;

#[derive(Debug, Clone)]
pub struct PositionSolution {
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub clock_bias_s: f64,
    pub pdop: f64,
    pub hdop: Option<f64>,
    pub vdop: Option<f64>,
    pub gdop: Option<f64>,
    pub tdop: Option<f64>,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub residuals: Vec<(SatId, f64, f64)>,
    pub rejected: Vec<(SatId, bijux_gnss_core::api::MeasurementRejectReason)>,
    pub raim_fault_detection: Option<RaimFaultDetection>,
    pub raim_fault_exclusion: Option<RaimFaultExclusion>,
    pub separation_max_m: Option<f64>,
    pub separation_suspect: Option<SatId>,
    pub covariance_symmetrized: bool,
    pub covariance_clamped: bool,
    pub covariance_max_variance: Option<f64>,
    pub sat_count: usize,
    pub used_sat_count: usize,
    pub rejected_sat_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionDops {
    pub pdop: f64,
    pub hdop: f64,
    pub vdop: f64,
    pub gdop: f64,
    pub tdop: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionSolveRefusalKind {
    InsufficientObservations,
    InvalidSatelliteTime,
    InvalidEphemeris,
    InsufficientUsableSatellites,
    UnderdeterminedRaimExclusion,
    SolverFailure,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PositionSolveRefusal {
    pub kind: PositionSolveRefusalKind,
    pub sat_count: usize,
    pub used_sat_count: usize,
    pub rejected: Vec<(SatId, MeasurementRejectReason)>,
}

impl PositionSolveRefusal {
    pub fn rejected_sat_count(&self) -> usize {
        self.rejected.len()
    }
}

#[derive(Debug, Clone)]
pub struct PositionObservation {
    pub sat: SatId,
    pub pseudorange_m: f64,
    pub cn0_dbhz: f64,
    pub elevation_deg: Option<f64>,
    pub weight: f64,
    pub gps_receive_time: Option<GpsTime>,
    pub signal_timing: Option<ObsSignalTiming>,
}

#[derive(Debug, Clone)]
struct PositionSolveInput {
    observation: PositionObservation,
    ephemeris: GpsEphemeris,
    receive_tow_s: f64,
}

#[derive(Debug, Clone, Copy)]
struct PositionEstimate {
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    clock_bias_s: f64,
}

#[derive(Debug, Clone)]
struct WorkingSetResidual {
    sat: SatId,
    residual_m: f64,
    base_weight: f64,
    effective_weight: f64,
}

#[derive(Debug, Clone)]
struct SatelliteGeometry {
    observation: PositionObservation,
    state: GpsSatState,
    iono_delay_m: f64,
    tropo_delay_m: f64,
}

#[derive(Debug, Clone)]
struct WorkingSetSolution {
    estimate: PositionEstimate,
    geometry: Vec<SatelliteGeometry>,
    residuals: Vec<WorkingSetResidual>,
    covariance: Option<[[f64; 4]; 4]>,
    covariance_symmetrized: bool,
    covariance_clamped: bool,
    covariance_max_variance: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
struct RaimSolutionSeparation {
    suspect_sat: SatId,
    separation_m: f64,
}

#[derive(Debug, Clone, Copy)]
struct RaimExclusionCandidate {
    excluded_index: usize,
    excluded_sat: SatId,
    candidate_estimate: PositionEstimate,
    pre_exclusion_rms_m: f64,
    post_exclusion_rms_m: f64,
    solution_shift_m: f64,
}

#[derive(Debug, Clone)]
pub struct PositionSolver {
    pub max_iterations: usize,
    pub convergence_m: f64,
    pub residual_gate_m: f64,
    pub chi_square_gate: f64,
    pub robust: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub separation_gate_m: f64,
    pub apply_troposphere: bool,
}

impl Default for PositionSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl PositionSolver {
    pub fn new() -> Self {
        Self {
            max_iterations: 10,
            convergence_m: 1e-3,
            residual_gate_m: 150.0,
            chi_square_gate: 9.0,
            robust: true,
            huber_k: 30.0,
            raim: true,
            separation_gate_m: 50.0,
            apply_troposphere: false,
        }
    }

    pub fn solve_wls(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
    ) -> Option<PositionSolution> {
        self.try_solve_wls(observations, ephemerides, t_rx_s).ok()
    }

    pub fn try_solve_wls(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
    ) -> Result<PositionSolution, PositionSolveRefusal> {
        self.try_solve_wls_with_broadcast_ionosphere(observations, ephemerides, t_rx_s, None)
    }

    pub fn solve_wls_with_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<PositionSolution> {
        self.try_solve_wls_with_broadcast_ionosphere(observations, ephemerides, t_rx_s, klobuchar)
            .ok()
    }

    pub fn try_solve_wls_with_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Result<PositionSolution, PositionSolveRefusal> {
        let sat_count = observations.len();
        if sat_count < 4 {
            return Err(position_solve_refusal(
                PositionSolveRefusalKind::InsufficientObservations,
                sat_count,
                sat_count,
                Vec::new(),
            ));
        }
        let mut observations = observations.to_vec();
        observations.sort_by_key(|obs| (obs.sat.constellation as u8, obs.sat.prn));
        let mut timing_rejected = Vec::new();
        observations.retain(|obs| {
            if position_observation_has_valid_satellite_time(obs, t_rx_s) {
                true
            } else {
                timing_rejected.push((obs.sat, MeasurementRejectReason::TimeInconsistency));
                false
            }
        });
        if observations.len() < 4 {
            let kind = if timing_rejected.is_empty() {
                PositionSolveRefusalKind::InsufficientObservations
            } else {
                PositionSolveRefusalKind::InvalidSatelliteTime
            };
            return Err(position_solve_refusal(
                kind,
                sat_count,
                observations.len(),
                timing_rejected,
            ));
        }
        let mut x = 0.0_f64;
        let mut y = 0.0_f64;
        let mut z = 0.0_f64;
        let mut cb = 0.0_f64;

        let mut rejected = timing_rejected;
        let inputs = resolve_position_inputs(&observations, ephemerides, t_rx_s, &mut rejected);
        if inputs.len() < 4 {
            return Err(position_solve_refusal(
                PositionSolveRefusalKind::InvalidEphemeris,
                sat_count,
                inputs.len(),
                rejected,
            ));
        }
        let initial_estimate =
            PositionEstimate { ecef_x_m: x, ecef_y_m: y, ecef_z_m: z, clock_bias_s: cb };
        let mut working_inputs = inputs;
        let mut estimate = initial_estimate;
        let mut raim_fault_detection = None;
        let mut raim_fault_exclusion = None;
        let working_set = loop {
            let solved = self.solve_working_set(&working_inputs, estimate, klobuchar).ok_or_else(
                || {
                    position_solve_refusal(
                        PositionSolveRefusalKind::SolverFailure,
                        sat_count,
                        working_inputs.len(),
                        rejected.clone(),
                    )
                },
            )?;
            let outlier_indices = self.outlier_indices(&solved.residuals);
            if outlier_indices.is_empty() {
                break solved;
            }

            if let Some(exclusion_candidate) =
                self.best_single_outlier_candidate(&working_inputs, &solved, klobuchar)
            {
                if self.raim && !supports_reliable_raim_exclusion(working_inputs.len()) {
                    push_unique_rejection(
                        &mut rejected,
                        exclusion_candidate.excluded_sat,
                        MeasurementRejectReason::Outlier,
                    );
                    return Err(position_solve_refusal(
                        PositionSolveRefusalKind::UnderdeterminedRaimExclusion,
                        sat_count,
                        working_inputs.len().saturating_sub(1),
                        rejected,
                    ));
                }
                let separation_m = exclusion_candidate.solution_shift_m;
                if raim_fault_detection.is_none() && separation_m > self.separation_gate_m {
                    raim_fault_detection = Some(RaimFaultDetection::fault_detected(
                        exclusion_candidate.excluded_sat,
                        separation_m,
                        self.separation_gate_m,
                    ));
                }
                if raim_fault_exclusion.is_none() {
                    raim_fault_exclusion = Some(RaimFaultExclusion {
                        excluded_sat: exclusion_candidate.excluded_sat,
                        pre_exclusion_rms_m: exclusion_candidate.pre_exclusion_rms_m,
                        post_exclusion_rms_m: exclusion_candidate.post_exclusion_rms_m,
                        solution_shift_m: exclusion_candidate.solution_shift_m,
                    });
                }
                push_unique_rejection(
                    &mut rejected,
                    exclusion_candidate.excluded_sat,
                    MeasurementRejectReason::Outlier,
                );
                working_inputs = working_inputs
                    .into_iter()
                    .enumerate()
                    .filter_map(|(index, input)| {
                        (index != exclusion_candidate.excluded_index).then_some(input)
                    })
                    .collect();
                estimate = exclusion_candidate.candidate_estimate;
                continue;
            }

            for &outlier_index in &outlier_indices {
                let residual = solved
                    .residuals
                    .get(outlier_index)
                    .expect("outlier index must reference a residual");
                push_unique_rejection(
                    &mut rejected,
                    residual.sat,
                    MeasurementRejectReason::Outlier,
                );
            }

            let retained_len = working_inputs.len().saturating_sub(outlier_indices.len());
            if retained_len < 4 {
                return Err(position_solve_refusal(
                    PositionSolveRefusalKind::InsufficientUsableSatellites,
                    sat_count,
                    retained_len,
                    rejected,
                ));
            }

            let retained_indices = outlier_indices.into_iter().collect::<std::collections::BTreeSet<_>>();
            working_inputs = working_inputs
                .into_iter()
                .enumerate()
                .filter_map(|(index, input)| (!retained_indices.contains(&index)).then_some(input))
                .collect();
            estimate = solved.estimate;
        };
        x = working_set.estimate.ecef_x_m;
        y = working_set.estimate.ecef_y_m;
        z = working_set.estimate.ecef_z_m;
        cb = working_set.estimate.clock_bias_s;

        let filtered = working_set
            .geometry
            .iter()
            .zip(&working_set.residuals)
            .map(|(geometry, residual)| {
                (
                    geometry.observation.clone(),
                    geometry.state.clone(),
                    residual.residual_m,
                    residual.effective_weight,
                )
            })
            .collect::<Vec<_>>();
        if filtered.len() < 4 {
            return Err(position_solve_refusal(
                PositionSolveRefusalKind::InsufficientUsableSatellites,
                sat_count,
                filtered.len(),
                rejected,
            ));
        }

        let final_estimate = working_set.estimate;
        let separation = self
            .raim
            .then(|| max_solution_separation(&filtered, final_estimate))
            .flatten();
        if raim_fault_detection.is_none() {
            if let Some(separation) = separation {
                raim_fault_detection =
                    Some(raim_fault_detection_from_separation(separation, self.separation_gate_m));
            }
        }

        let mut h = Vec::new();
        let mut v = Vec::new();
        for (_obs, state, residual_m, _effective_weight) in &filtered {
            let dx = final_estimate.ecef_x_m - state.x_m;
            let dy = final_estimate.ecef_y_m - state.y_m;
            let dz = final_estimate.ecef_z_m - state.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            let hx = dx / range;
            let hy = dy / range;
            let hz = dz / range;
            h.push([hx, hy, hz, 1.0]);
            v.push(*residual_m);
        }

        let dops = compute_dops(&h);
        let rms = if !v.is_empty() {
            let sum = v.iter().map(|r| r * r).sum::<f64>();
            (sum / v.len() as f64).sqrt()
        } else {
            0.0
        };

        let (lat, lon, alt) = ecef_to_geodetic(x, y, z);

        let (sigma_h_m, sigma_v_m) = working_set
            .covariance
            .map(|cov| {
                let sigma2 = if !v.is_empty() {
                    let sum = v.iter().map(|r| r * r).sum::<f64>();
                    let dof = (v.len() as i32 - 4).max(1) as f64;
                    sum / dof
                } else {
                    0.0
                };
                let var_x = cov[0][0] * sigma2;
                let var_y = cov[1][1] * sigma2;
                let var_z = cov[2][2] * sigma2;
                ((var_x + var_y).max(0.0).sqrt(), var_z.max(0.0).sqrt())
            })
            .unwrap_or((0.0, 0.0));

        let rejected_sat_count = rejected.len();
        Ok(PositionSolution {
            ecef_x_m: x,
            ecef_y_m: y,
            ecef_z_m: z,
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            clock_bias_s: cb,
            pdop: dops.map(|dops| dops.pdop).unwrap_or(0.0),
            hdop: dops.map(|dops| dops.hdop),
            vdop: dops.map(|dops| dops.vdop),
            gdop: dops.map(|dops| dops.gdop),
            tdop: dops.map(|dops| dops.tdop),
            rms_m: rms,
            sigma_h_m: Some(sigma_h_m),
            sigma_v_m: Some(sigma_v_m),
            residuals: filtered
                .iter()
                .map(|(observation, _state, residual_m, effective_weight)| {
                    (observation.sat, *residual_m, *effective_weight)
                })
                .collect(),
            rejected,
            raim_fault_detection,
            raim_fault_exclusion,
            separation_max_m: separation.map(|separation| separation.separation_m),
            separation_suspect: separation.map(|separation| separation.suspect_sat),
            covariance_symmetrized: working_set.covariance_symmetrized,
            covariance_clamped: working_set.covariance_clamped,
            covariance_max_variance: working_set.covariance_max_variance,
            sat_count: observations.len(),
            used_sat_count: filtered.len(),
            rejected_sat_count,
        })
    }
}

fn position_solve_refusal(
    kind: PositionSolveRefusalKind,
    sat_count: usize,
    used_sat_count: usize,
    rejected: Vec<(SatId, MeasurementRejectReason)>,
) -> PositionSolveRefusal {
    PositionSolveRefusal { kind, sat_count, used_sat_count, rejected }
}

fn resolve_position_inputs(
    observations: &[PositionObservation],
    ephemerides: &[GpsEphemeris],
    t_rx_s: f64,
    rejected: &mut Vec<(SatId, MeasurementRejectReason)>,
) -> Vec<PositionSolveInput> {
    observations
        .iter()
        .filter_map(|obs| {
            let Some(ephemeris) = ephemerides.iter().find(|eph| eph.sat == obs.sat) else {
                rejected.push((obs.sat, MeasurementRejectReason::InvalidEphemeris));
                return None;
            };
            let receive_tow_s =
                obs.gps_receive_time.map(|gps_time| gps_time.tow_s).unwrap_or(t_rx_s);
            if !is_ephemeris_valid(ephemeris, receive_tow_s) {
                rejected.push((obs.sat, MeasurementRejectReason::InvalidEphemeris));
                return None;
            }
            Some(PositionSolveInput {
                observation: obs.clone(),
                ephemeris: ephemeris.clone(),
                receive_tow_s,
            })
        })
        .collect()
}

fn resolve_satellite_geometry(
    inputs: &[PositionSolveInput],
    estimate: PositionEstimate,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_troposphere: bool,
) -> Option<Vec<SatelliteGeometry>> {
    let mut geometry = Vec::with_capacity(inputs.len());
    for input in inputs {
        let obs = &input.observation;
        let mut tau = obs
            .signal_timing
            .map(|timing| timing.signal_travel_time_s.0)
            .unwrap_or(obs.pseudorange_m / 299_792_458.0);
        let mut state = sat_state_gps_l1ca_from_observation(
            &input.ephemeris,
            input.receive_tow_s,
            obs.pseudorange_m,
            obs.signal_timing,
        );
        let mut converged = false;
        for _ in 0..5 {
            let range_m = geometric_range_m(estimate, &state);
            let next_tau = predicted_signal_travel_time_s(
                range_m,
                estimate.clock_bias_s,
                state.clock_correction.bias_s,
            );
            if (next_tau - tau).abs() < 1.0e-9 {
                converged = true;
            }
            tau = next_tau;
            state = sat_state_gps_l1ca(&input.ephemeris, input.receive_tow_s - tau, tau);
            if converged {
                break;
            }
        }
        if !converged {
            return None;
        }
        let iono_delay_m = estimate_klobuchar_delay_m(estimate, input, &state, klobuchar);
        let tropo_delay_m = estimate_saastamoinen_delay_m(estimate, &state, apply_troposphere);
        geometry.push(SatelliteGeometry {
            observation: obs.clone(),
            state,
            iono_delay_m,
            tropo_delay_m,
        });
    }
    Some(geometry)
}

fn linearized_pseudorange_row(
    estimate: PositionEstimate,
    geometry: &SatelliteGeometry,
) -> (f64, [f64; 4]) {
    let dx = estimate.ecef_x_m - geometry.state.x_m;
    let dy = estimate.ecef_y_m - geometry.state.y_m;
    let dz = estimate.ecef_z_m - geometry.state.z_m;
    let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
    let predicted_pseudorange_m = predicted_pseudorange_m(
        range_m,
        estimate.clock_bias_s,
        geometry.state.clock_correction.bias_s,
    );
    let residual_m = geometry.observation.pseudorange_m
        - geometry.iono_delay_m
        - geometry.tropo_delay_m
        - predicted_pseudorange_m;
    let design_row = [dx / range_m, dy / range_m, dz / range_m, 1.0];
    (residual_m, design_row)
}

fn estimate_klobuchar_delay_m(
    estimate: PositionEstimate,
    input: &PositionSolveInput,
    state: &GpsSatState,
    klobuchar: Option<&KlobucharCoefficients>,
) -> f64 {
    let Some(coefficients) = klobuchar else {
        return 0.0;
    };
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    if !receiver_radius_m.is_finite() || receiver_radius_m < 1.0 {
        return 0.0;
    }
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    let receiver = Llh { lat_deg: latitude_deg, lon_deg: longitude_deg, alt_m: altitude_m };
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        estimate.ecef_x_m,
        estimate.ecef_y_m,
        estimate.ecef_z_m,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !azimuth_deg.is_finite() || !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    KlobucharModel::new(*coefficients).delay_m(
        receiver,
        azimuth_deg,
        elevation_deg,
        Seconds(input.receive_tow_s),
    )
}

fn estimate_saastamoinen_delay_m(
    estimate: PositionEstimate,
    state: &GpsSatState,
    apply_troposphere: bool,
) -> f64 {
    if !apply_troposphere {
        return 0.0;
    }
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    if !receiver_radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&receiver_radius_m) {
        return 0.0;
    }
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    if !latitude_deg.is_finite()
        || !longitude_deg.is_finite()
        || !altitude_m.is_finite()
        || !(-1_000.0..=20_000.0).contains(&altitude_m)
    {
        return 0.0;
    }
    let receiver = Llh { lat_deg: latitude_deg, lon_deg: longitude_deg, alt_m: altitude_m };
    let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        estimate.ecef_x_m,
        estimate.ecef_y_m,
        estimate.ecef_z_m,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = SaastamoinenModel;
    model.delay_m(receiver, elevation_deg, Seconds(0.0))
}

fn geometric_range_m(estimate: PositionEstimate, state: &GpsSatState) -> f64 {
    let dx = estimate.ecef_x_m - state.x_m;
    let dy = estimate.ecef_y_m - state.y_m;
    let dz = estimate.ecef_z_m - state.z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn predicted_signal_travel_time_s(
    range_m: f64,
    receiver_clock_bias_s: f64,
    satellite_clock_bias_s: f64,
) -> f64 {
    predicted_pseudorange_m(range_m, receiver_clock_bias_s, satellite_clock_bias_s)
        / SPEED_OF_LIGHT_MPS
}

fn predicted_pseudorange_m(
    range_m: f64,
    receiver_clock_bias_s: f64,
    satellite_clock_bias_s: f64,
) -> f64 {
    range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
        - satellite_clock_bias_s * SPEED_OF_LIGHT_MPS
}

/// Returns whether a position observation carries a finite and internally consistent
/// transmit-time description for navigation use.
pub fn position_observation_has_valid_satellite_time(
    obs: &PositionObservation,
    t_rx_s: f64,
) -> bool {
    let Some(signal_timing) = obs.signal_timing else {
        return false;
    };
    let signal_travel_time_s = signal_timing.signal_travel_time_s.0;
    if !signal_travel_time_s.is_finite() || signal_travel_time_s <= 0.0 {
        return false;
    }
    if !obs.pseudorange_m.is_finite() {
        return false;
    }
    let pseudorange_travel_time_s = obs.pseudorange_m / SPEED_OF_LIGHT_MPS;
    if (pseudorange_travel_time_s - signal_travel_time_s).abs()
        > SIGNAL_TIMING_CONSISTENCY_TOLERANCE_S
    {
        return false;
    }
    if !signal_timing.transmit_gps_time.tow_s.is_finite() {
        return false;
    }
    let receive_delta_s = if let Some(receive_gps_time) = obs.gps_receive_time {
        ((receive_gps_time.week as i64 - signal_timing.transmit_gps_time.week as i64) as f64
            * 604_800.0)
            + receive_gps_time.tow_s
            - signal_timing.transmit_gps_time.tow_s
    } else {
        t_rx_s - signal_timing.transmit_gps_time.tow_s
    };
    receive_delta_s.is_finite()
        && (receive_delta_s - signal_travel_time_s).abs() <= SIGNAL_TIMING_CONSISTENCY_TOLERANCE_S
}

fn sanitize_covariance(mut cov: [[f64; 4]; 4]) -> ([[f64; 4]; 4], bool, bool, Option<f64>) {
    let mut sym = false;
    let mut clamp = false;
    let mut max_var = None;
    let mut i = 0;
    while i < 4 {
        let mut j = 0;
        while j < 4 {
            if (cov[i][j] - cov[j][i]).abs() > 1e-9 {
                sym = true;
                let avg = 0.5 * (cov[i][j] + cov[j][i]);
                cov[i][j] = avg;
                cov[j][i] = avg;
            }
            j += 1;
        }
        if cov[i][i] < 0.0 {
            clamp = true;
            cov[i][i] = 0.0;
        }
        max_var = Some(max_var.map(|v: f64| v.max(cov[i][i])).unwrap_or(cov[i][i]));
        i += 1;
    }
    (cov, sym, clamp, max_var)
}

impl PositionSolver {
    fn solve_working_set(
        &self,
        inputs: &[PositionSolveInput],
        initial_estimate: PositionEstimate,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<WorkingSetSolution> {
        let mut estimate = initial_estimate;
        let mut geometry =
            resolve_satellite_geometry(inputs, estimate, klobuchar, self.apply_troposphere)?;
        let mut covariance = None;
        let mut covariance_symmetrized = false;
        let mut covariance_clamped = false;
        let mut covariance_max_variance = None;

        for _ in 0..self.max_iterations {
            if geometry.len() < 4 {
                return None;
            }

            let mut h = Vec::with_capacity(geometry.len());
            let mut residual_values = Vec::with_capacity(geometry.len());
            for satellite_geometry in &geometry {
                let (residual_m, design_row) =
                    linearized_pseudorange_row(estimate, satellite_geometry);
                residual_values.push(residual_m);
                h.push(design_row);
            }

            let weights = self.measurement_weights(&geometry, &residual_values);
            let (dx, dy, dz, dcb, covariance_out) =
                solve_weighted_normal_eq(&h, &residual_values, &weights)?;
            let (covariance_out, symmetrized, clamped, max_variance) =
                sanitize_covariance(covariance_out);
            covariance_symmetrized |= symmetrized;
            covariance_clamped |= clamped;
            if let Some(max_variance) = max_variance {
                covariance_max_variance = Some(
                    covariance_max_variance
                        .map(|running_max: f64| running_max.max(max_variance))
                        .unwrap_or(max_variance),
                );
            }
            covariance = Some(covariance_out);

            estimate.ecef_x_m += dx;
            estimate.ecef_y_m += dy;
            estimate.ecef_z_m += dz;
            estimate.clock_bias_s += dcb / SPEED_OF_LIGHT_MPS;

            if (dx * dx + dy * dy + dz * dz).sqrt() < self.convergence_m {
                break;
            }

            geometry =
                resolve_satellite_geometry(inputs, estimate, klobuchar, self.apply_troposphere)?;
        }

        geometry =
            resolve_satellite_geometry(inputs, estimate, klobuchar, self.apply_troposphere)?;
        if geometry.len() < 4 {
            return None;
        }
        let residuals = self.finalize_working_set_residuals(estimate, &geometry);

        Some(WorkingSetSolution {
            estimate,
            geometry,
            residuals,
            covariance,
            covariance_symmetrized,
            covariance_clamped,
            covariance_max_variance,
        })
    }

    fn finalize_working_set_residuals(
        &self,
        estimate: PositionEstimate,
        geometry: &[SatelliteGeometry],
    ) -> Vec<WorkingSetResidual> {
        let residual_values = geometry
            .iter()
            .map(|satellite_geometry| linearized_pseudorange_row(estimate, satellite_geometry).0)
            .collect::<Vec<_>>();
        let weights = self.measurement_weights(geometry, &residual_values);

        geometry
            .iter()
            .zip(residual_values)
            .zip(weights)
            .map(|((satellite_geometry, residual_m), effective_weight)| WorkingSetResidual {
                sat: satellite_geometry.observation.sat,
                residual_m,
                base_weight: satellite_geometry.observation.weight,
                effective_weight,
            })
            .collect()
    }

    fn measurement_weights(
        &self,
        geometry: &[SatelliteGeometry],
        residuals: &[f64],
    ) -> Vec<f64> {
        let mut weights = if self.robust {
            huber_weights(residuals, self.huber_k)
        } else {
            vec![1.0; residuals.len()]
        };
        for (weight, satellite_geometry) in weights.iter_mut().zip(geometry) {
            *weight *= satellite_geometry.observation.weight;
        }
        weights
    }

    fn outlier_indices(&self, residuals: &[WorkingSetResidual]) -> Vec<usize> {
        residuals
            .iter()
            .enumerate()
            .filter_map(|(index, residual)| {
                let sigma_m = (1.0 / residual.base_weight.max(1e-6)).sqrt();
                let normalized_residual = residual.residual_m / sigma_m;
                ((residual.residual_m.abs() > self.residual_gate_m)
                    || (normalized_residual * normalized_residual) > self.chi_square_gate)
                    .then_some((index, normalized_residual.abs(), residual.residual_m.abs()))
            })
            .max_by(|left, right| {
                left.1
                    .partial_cmp(&right.1)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| {
                        left.2
                            .partial_cmp(&right.2)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
            })
            .map(|(index, _normalized_residual, _residual_m)| vec![index])
            .unwrap_or_default()
    }

    fn best_single_outlier_candidate(
        &self,
        inputs: &[PositionSolveInput],
        solved: &WorkingSetSolution,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<RaimExclusionCandidate> {
        if inputs.len() < 5 {
            return None;
        }

        let pre_exclusion_rms_m = working_set_rms_m(&solved.residuals);
        let mut best_candidate = None;
        for excluded_index in 0..inputs.len() {
            let candidate_inputs = inputs
                .iter()
                .enumerate()
                .filter_map(|(index, input)| (index != excluded_index).then_some(input.clone()))
                .collect::<Vec<_>>();
            let Some(candidate_solution) =
                self.solve_working_set(&candidate_inputs, solved.estimate, klobuchar)
            else {
                continue;
            };
            if !self.outlier_indices(&candidate_solution.residuals).is_empty() {
                continue;
            }

            let candidate_rms_m = working_set_rms_m(&candidate_solution.residuals);
            let excluded_sat = inputs
                .get(excluded_index)
                .expect("candidate exclusion must reference an input")
                .observation
                .sat;
            let candidate = RaimExclusionCandidate {
                excluded_index,
                excluded_sat,
                candidate_estimate: candidate_solution.estimate,
                pre_exclusion_rms_m,
                post_exclusion_rms_m: candidate_rms_m,
                solution_shift_m: solution_separation_m(solved.estimate, candidate_solution.estimate),
            };
            let better_candidate = best_candidate
                .as_ref()
                .map(|best_candidate: &RaimExclusionCandidate| {
                    candidate.post_exclusion_rms_m < best_candidate.post_exclusion_rms_m
                })
                .unwrap_or(true);
            if better_candidate {
                best_candidate = Some(candidate);
            }
        }

        best_candidate
    }
}

fn push_unique_rejection(
    rejected: &mut Vec<(SatId, MeasurementRejectReason)>,
    sat: SatId,
    reason: MeasurementRejectReason,
) {
    if rejected.iter().any(|(rejected_sat, rejected_reason)| {
        *rejected_sat == sat && *rejected_reason == reason
    }) {
        return;
    }
    rejected.push((sat, reason));
}

fn working_set_rms_m(residuals: &[WorkingSetResidual]) -> f64 {
    if residuals.is_empty() {
        return 0.0;
    }
    let squared_sum = residuals.iter().map(|residual| residual.residual_m.powi(2)).sum::<f64>();
    (squared_sum / residuals.len() as f64).sqrt()
}

fn solution_separation_m(left: PositionEstimate, right: PositionEstimate) -> f64 {
    let dx = left.ecef_x_m - right.ecef_x_m;
    let dy = left.ecef_y_m - right.ecef_y_m;
    let dz = left.ecef_z_m - right.ecef_z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn supports_reliable_raim_exclusion(usable_sat_count: usize) -> bool {
    usable_sat_count >= 6
}

fn raim_fault_detection_from_separation(
    separation: RaimSolutionSeparation,
    threshold_m: f64,
) -> RaimFaultDetection {
    if separation.separation_m > threshold_m {
        RaimFaultDetection::fault_detected(
            separation.suspect_sat,
            separation.separation_m,
            threshold_m,
        )
    } else {
        RaimFaultDetection::consistent(separation.separation_m, threshold_m)
    }
}

fn max_solution_separation(
    filtered: &[(PositionObservation, GpsSatState, f64, f64)],
    final_estimate: PositionEstimate,
) -> Option<RaimSolutionSeparation> {
    if filtered.len() < 5 {
        return None;
    }

    let mut max_separation = None;
    for idx in 0..filtered.len() {
        let mut subset = filtered.to_vec();
        let removed = subset.remove(idx);
        if subset.len() < 4 {
            continue;
        }
        let mut h_sep = Vec::with_capacity(subset.len());
        let mut v_sep = Vec::with_capacity(subset.len());
        for (_obs, state, residual_m, _effective_weight) in &subset {
            let dx = final_estimate.ecef_x_m - state.x_m;
            let dy = final_estimate.ecef_y_m - state.y_m;
            let dz = final_estimate.ecef_z_m - state.z_m;
            let range = (dx * dx + dy * dy + dz * dz).sqrt();
            let hx = dx / range;
            let hy = dy / range;
            let hz = dz / range;
            h_sep.push([hx, hy, hz, 1.0]);
            v_sep.push(*residual_m);
        }
        if let Some((dx, dy, dz, _dcb, _)) =
            solve_weighted_normal_eq(&h_sep, &v_sep, &vec![1.0; v_sep.len()])
        {
            let separation_m = (dx * dx + dy * dy + dz * dz).sqrt();
            let candidate = RaimSolutionSeparation {
                suspect_sat: removed.0.sat,
                separation_m,
            };
            if max_separation
                .map(|current: RaimSolutionSeparation| candidate.separation_m > current.separation_m)
                .unwrap_or(true)
            {
                max_separation = Some(candidate);
            }
        }
    }

    max_separation
}

type NormalEqSolution = (f64, f64, f64, f64, [[f64; 4]; 4]);

fn solve_weighted_normal_eq(h: &[[f64; 4]], v: &[f64], w: &[f64]) -> Option<NormalEqSolution> {
    let mut n = [[0.0_f64; 4]; 4];
    let mut u = [0.0_f64; 4];
    for (i, row) in h.iter().enumerate() {
        let wi = w.get(i).copied().unwrap_or(1.0);
        for r in 0..4 {
            u[r] += row[r] * v[i] * wi;
            for c in 0..4 {
                n[r][c] += row[r] * row[c] * wi;
            }
        }
    }
    let inv = invert_4x4(n)?;
    let dx = inv[0][0] * u[0] + inv[0][1] * u[1] + inv[0][2] * u[2] + inv[0][3] * u[3];
    let dy = inv[1][0] * u[0] + inv[1][1] * u[1] + inv[1][2] * u[2] + inv[1][3] * u[3];
    let dz = inv[2][0] * u[0] + inv[2][1] * u[1] + inv[2][2] * u[2] + inv[2][3] * u[3];
    let dcb = inv[3][0] * u[0] + inv[3][1] * u[1] + inv[3][2] * u[2] + inv[3][3] * u[3];
    Some((dx, dy, dz, dcb, inv))
}

fn huber_weights(residuals: &[f64], k: f64) -> Vec<f64> {
    residuals
        .iter()
        .map(|r| {
            let a = r.abs();
            if a <= k {
                1.0
            } else {
                k / a
            }
        })
        .collect()
}

pub fn invert_4x4(a: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut m = [[0.0_f64; 8]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][i + 4] = 1.0;
    }
    for i in 0..4 {
        let mut pivot = i;
        let mut max = m[i][i].abs();
        for (r, row) in m.iter().enumerate().skip(i + 1) {
            if row[i].abs() > max {
                max = row[i].abs();
                pivot = r;
            }
        }
        if max < 1e-12 {
            return None;
        }
        if pivot != i {
            m.swap(i, pivot);
        }
        let inv_pivot = 1.0 / m[i][i];
        let mut j = i;
        while j < 8 {
            m[i][j] *= inv_pivot;
            j += 1;
        }
        for r in 0..4 {
            if r == i {
                continue;
            }
            let factor = m[r][i];
            let mut j = i;
            while j < 8 {
                m[r][j] -= factor * m[i][j];
                j += 1;
            }
        }
    }
    let mut inv = [[0.0_f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            inv[i][j] = m[i][j + 4];
        }
    }
    Some(inv)
}

fn compute_pdop(h: &[[f64; 4]]) -> Option<f64> {
    let mut n = [[0.0_f64; 4]; 4];
    for row in h {
        for r in 0..4 {
            for c in 0..4 {
                n[r][c] += row[r] * row[c];
            }
        }
    }
    let inv = invert_4x4(n)?;
    let pdop = (inv[0][0] + inv[1][1] + inv[2][2]).sqrt();
    Some(pdop)
}

pub fn position_dops_from_satellite_positions(
    receiver_ecef_m: [f64; 3],
    satellite_positions_ecef_m: &[[f64; 3]],
) -> Option<PositionDops> {
    if satellite_positions_ecef_m.len() < 4 {
        return None;
    }
    let mut h = Vec::with_capacity(satellite_positions_ecef_m.len());
    for satellite_ecef_m in satellite_positions_ecef_m {
        let dx = receiver_ecef_m[0] - satellite_ecef_m[0];
        let dy = receiver_ecef_m[1] - satellite_ecef_m[1];
        let dz = receiver_ecef_m[2] - satellite_ecef_m[2];
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        if !range_m.is_finite() || range_m <= 0.0 {
            return None;
        }
        h.push([dx / range_m, dy / range_m, dz / range_m, 1.0]);
    }
    compute_dops(&h)
}

fn compute_dops(h: &[[f64; 4]]) -> Option<PositionDops> {
    let mut n = [[0.0_f64; 4]; 4];
    for row in h {
        for r in 0..4 {
            for c in 0..4 {
                n[r][c] += row[r] * row[c];
            }
        }
    }
    let inv = invert_4x4(n)?;
    let hdop = (inv[0][0] + inv[1][1]).max(0.0).sqrt();
    let vdop = inv[2][2].max(0.0).sqrt();
    let tdop = inv[3][3].max(0.0).sqrt();
    let pdop = (hdop.powi(2) + vdop.powi(2)).sqrt();
    let gdop = (pdop.powi(2) + tdop.powi(2)).sqrt();
    Some(PositionDops { pdop, hdop, vdop, gdop, tdop })
}

pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);

    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - e2));
    let mut alt = 0.0;
    for _ in 0..5 {
        let sin_lat = lat.sin();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        alt = p / lat.cos() - n;
        lat = z.atan2(p * (1.0 - e2 * n / (n + alt)));
    }
    (lat.to_degrees(), lon.to_degrees(), alt)
}

pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_m) * cos_lat * lon.cos();
    let y = (n + alt_m) * cos_lat * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * sin_lat;
    (x, y, z)
}

pub fn ecef_to_enu(
    x: f64,
    y: f64,
    z: f64,
    ref_lat_deg: f64,
    ref_lon_deg: f64,
    ref_alt_m: f64,
) -> (f64, f64, f64) {
    let (xr, yr, zr) = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m);
    let dx = x - xr;
    let dy = y - yr;
    let dz = z - zr;
    let lat = ref_lat_deg.to_radians();
    let lon = ref_lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();
    let east = -sin_lon * dx + cos_lon * dy;
    let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    (east, north, up)
}

pub fn elevation_azimuth_deg(
    rx_x: f64,
    rx_y: f64,
    rx_z: f64,
    sat_x: f64,
    sat_y: f64,
    sat_z: f64,
) -> (f64, f64) {
    let (lat, lon, alt) = ecef_to_geodetic(rx_x, rx_y, rx_z);
    let (e, n, u) = ecef_to_enu(sat_x, sat_y, sat_z, lat, lon, alt);
    let az = e.atan2(n).to_degrees().rem_euclid(360.0);
    let el = (u / (e * e + n * n + u * u).sqrt()).asin().to_degrees();
    (az, el)
}

#[derive(Debug, Clone, Copy)]
pub struct WeightingConfig {
    pub min_elev_deg: f64,
    pub elev_exponent: f64,
    pub cn0_ref_dbhz: f64,
    pub min_weight: f64,
    pub enabled: bool,
}

impl Default for WeightingConfig {
    fn default() -> Self {
        Self {
            min_elev_deg: 5.0,
            elev_exponent: 2.0,
            cn0_ref_dbhz: 50.0,
            min_weight: 0.1,
            enabled: true,
        }
    }
}

pub fn weight_from_cn0_elev(cn0_dbhz: f64, elev_deg: f64, config: WeightingConfig) -> f64 {
    if !config.enabled {
        return 1.0;
    }
    let elev = elev_deg.clamp(0.0, 90.0).max(config.min_elev_deg);
    let w_elev = (elev / 90.0).powf(config.elev_exponent).max(config.min_weight);
    let w_cn0 = (cn0_dbhz / config.cn0_ref_dbhz).max(config.min_weight);
    (w_elev * w_cn0).max(config.min_weight)
}

/// Convert a pseudorange standard deviation in meters into a least-squares weight.
///
/// The returned value is the inverse measurement variance in m^-2. Invalid or
/// missing sigma values fall back to unit weighting.
pub fn weight_from_pseudorange_sigma(pseudorange_sigma_m: Option<f64>) -> f64 {
    let Some(sigma_m) = pseudorange_sigma_m else {
        return 1.0;
    };
    if !sigma_m.is_finite() || sigma_m <= 0.0 {
        return 1.0;
    }
    1.0 / sigma_m.powi(2)
}

/// Build a composite code-pseudorange weight from geometry and measurement sigma.
///
/// The geometry term is driven by C/N0 and elevation when available. The sigma
/// term is always driven by inverse pseudorange variance when available.
pub fn position_measurement_weight(
    cn0_dbhz: f64,
    elev_deg: Option<f64>,
    pseudorange_sigma_m: Option<f64>,
    config: WeightingConfig,
) -> f64 {
    let geometry_weight =
        elev_deg.map(|elev| weight_from_cn0_elev(cn0_dbhz, elev, config)).unwrap_or(1.0);
    geometry_weight * weight_from_pseudorange_sigma(pseudorange_sigma_m)
}
