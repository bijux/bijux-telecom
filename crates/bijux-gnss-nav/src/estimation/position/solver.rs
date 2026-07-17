#![allow(missing_docs)]

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, horizontal_error_ellipse,
};

use super::navigation::{
    broadcast_group_delay_correction_chain, observation_consistency_metrics,
    resolve_position_inputs, satellite_state_at_time, satellite_state_from_observation,
    unknown_inter_system_time_offset_sats, PositionObservationCorrectionChain,
    PositionObservationCorrectionKind, PositionSolveInput, SatelliteState,
    POSITION_OBSERVATION_CORRECTION_ORDER,
};
use super::raim::{
    formal_protection_levels, PositionProtectionLevels, RaimFaultDetection, RaimFaultExclusion,
    RaimFaultHypothesis, RaimSolutionSeparationCheck, RaimSolutionSeparationSubset,
};
use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::{
    gps_ephemeris_age, is_ephemeris_valid, GpsBroadcastNavigationData, GpsEphemeris,
};
use bijux_gnss_core::api::{
    Constellation, InterSystemBias, Llh, MeasurementRejectReason, NavConstellationResidualRms,
    ObsEpoch, SatId, Seconds, SigId,
};

pub mod dops;
pub mod geodesy;
mod least_squares;
mod matrix;
pub mod observation_inputs;
pub mod robust_weighting;
pub mod weighting;
use dops::{compute_dops, scaled_position_covariance_ecef_m2};
use geodesy::{ecef_to_geodetic, elevation_azimuth_deg};
use least_squares::solve_weighted_least_squares;
use observation_inputs::constellation_primary_band;
#[cfg(test)]
use robust_weighting::robust_weight;
use robust_weighting::robust_weights;
use robust_weighting::PositionRobustWeighting;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SIGNAL_TIMING_CONSISTENCY_TOLERANCE_S: f64 = 1.0e-6;
const EPHEMERIS_MISMATCH_CODE_RESIDUAL_GATE_M: f64 = 1_500.0;
const TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M: f64 = 6_000_000.0;
const TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M: f64 = 7_000_000.0;
const TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M: f64 = -1_000.0;
const TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M: f64 = 20_000.0;
const REPLAY_TIMING_ANOMALY_MIN_MATCHED_SATELLITES: usize = 4;
const REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M: f64 = 40.0;
const REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M: f64 = 60.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpossibleGeometryEvidence {
    pub receiver_radius_m: f64,
    pub altitude_m: f64,
    pub used_satellite_count: usize,
    pub min_receiver_radius_m: f64,
    pub max_receiver_radius_m: f64,
    pub min_altitude_m: f64,
    pub max_altitude_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReplayTimingAnomalyEvidence {
    pub matched_satellite_count: usize,
    pub median_excess_delay_m: f64,
    pub centered_delay_rms_m: f64,
    pub max_centered_delay_m: f64,
    pub centered_delay_rms_threshold_m: f64,
    pub max_centered_delay_threshold_m: f64,
}

#[derive(Debug, Clone)]
pub struct PositionSolution {
    pub ecef_x_m: f64,
    pub ecef_y_m: f64,
    pub ecef_z_m: f64,
    pub position_covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    pub horizontal_error_ellipse_major_axis_m: Option<f64>,
    pub horizontal_error_ellipse_minor_axis_m: Option<f64>,
    pub horizontal_error_ellipse_azimuth_deg: Option<f64>,
    pub sigma_e_m: Option<f64>,
    pub sigma_n_m: Option<f64>,
    pub sigma_u_m: Option<f64>,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
    pub broadcast_ionosphere_applied: bool,
    pub clock_reference_constellation: Constellation,
    pub clock_bias_s: f64,
    pub inter_system_biases: Vec<InterSystemBias>,
    pub pdop: f64,
    pub hdop: Option<f64>,
    pub vdop: Option<f64>,
    pub gdop: Option<f64>,
    pub tdop: Option<f64>,
    pub pre_fit_residual_rms_m: f64,
    pub post_fit_residual_rms_m: f64,
    pub rms_m: f64,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
    pub integrity_hpl_m: Option<f64>,
    pub integrity_vpl_m: Option<f64>,
    pub residuals: Vec<(SatId, f64, f64)>,
    pub corrected_observations: Vec<PositionCorrectedObservation>,
    pub constellation_residual_rms: Vec<NavConstellationResidualRms>,
    pub rejected: Vec<(SatId, bijux_gnss_core::api::MeasurementRejectReason)>,
    pub raim_fault_detection: Option<RaimFaultDetection>,
    pub raim_fault_exclusion: Option<RaimFaultExclusion>,
    pub raim_fault_exclusions: Vec<RaimFaultExclusion>,
    pub raim_solution_separation: Option<RaimSolutionSeparationCheck>,
    pub impossible_geometry: Option<ImpossibleGeometryEvidence>,
    pub replay_timing_anomaly: Option<ReplayTimingAnomalyEvidence>,
    pub covariance_symmetrized: bool,
    pub covariance_clamped: bool,
    pub covariance_max_variance: Option<f64>,
    pub solver_rank: usize,
    pub solver_condition_number: Option<f64>,
    pub sat_count: usize,
    pub used_sat_count: usize,
    pub rejected_sat_count: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionCorrectedObservation {
    pub sat: SatId,
    pub signal_id: Option<SigId>,
    pub correction_chain: PositionObservationCorrectionChain,
    pub geometric_range_m: f64,
    pub residual_m: f64,
}

impl PositionCorrectedObservation {
    pub fn reconstructed_corrected_pseudorange_m(&self) -> f64 {
        self.correction_chain.reconstructed_pseudorange_m()
    }

    pub fn reconstruction_error_m(&self) -> f64 {
        self.correction_chain.reconstruction_error_m()
    }

    pub fn reconstructed_residual_m(&self) -> f64 {
        self.reconstructed_corrected_pseudorange_m() - self.geometric_range_m
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionSolveRefusalKind {
    InsufficientObservations,
    InvalidSatelliteTime,
    InvalidEphemeris,
    UnknownInterSystemTimeOffset,
    InsufficientUsableSatellites,
    UnderdeterminedRaimExclusion,
    SolverFailure,
    FilterDivergence(PositionFilterDivergenceReason),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionFilterDivergenceReason {
    InnovationInconsistency,
    InnovationGrowth,
    CovarianceCollapse,
    CovarianceDivergence,
    ResidualExplosion,
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

pub type PositionObservation = observation_inputs::PositionObservation;
pub type PositionBroadcastNavigation = observation_inputs::PositionBroadcastNavigation;

pub fn position_broadcast_navigation_from_gps_ephemerides(
    ephemerides: &[GpsEphemeris],
) -> Vec<PositionBroadcastNavigation> {
    observation_inputs::position_broadcast_navigation_from_gps_ephemerides(ephemerides)
}

pub fn position_broadcast_navigation_from_galileo_navigations(
    navigations: &[GalileoBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    observation_inputs::position_broadcast_navigation_from_galileo_navigations(navigations)
}

pub fn position_broadcast_navigation_from_glonass_frames(
    navigation_frames: &[GlonassBroadcastNavigationFrame],
) -> Vec<PositionBroadcastNavigation> {
    observation_inputs::position_broadcast_navigation_from_glonass_frames(navigation_frames)
}

pub fn position_broadcast_navigation_from_beidou_navigations(
    navigations: &[BeidouBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    observation_inputs::position_broadcast_navigation_from_beidou_navigations(navigations)
}

pub fn position_observations_from_epoch(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    observation_inputs::position_observations_from_epoch(epoch)
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ClockStateModel {
    reference_constellation: Constellation,
    offset_constellations: Vec<Constellation>,
}

impl ClockStateModel {
    fn from_constellations<I>(constellations: I) -> Option<Self>
    where
        I: IntoIterator<Item = Constellation>,
    {
        let unique = constellations.into_iter().collect::<std::collections::BTreeSet<_>>();
        if unique.is_empty() {
            return None;
        }
        let reference_constellation = if unique.contains(&Constellation::Gps) {
            Constellation::Gps
        } else {
            *unique.iter().next().expect("non-empty constellation set")
        };
        let offset_constellations = unique
            .into_iter()
            .filter(|constellation| *constellation != reference_constellation)
            .collect();
        Some(Self { reference_constellation, offset_constellations })
    }

    fn from_inputs(inputs: &[PositionSolveInput]) -> Option<Self> {
        Self::from_constellations(inputs.iter().map(|input| input.observation.sat.constellation))
    }

    fn state_len(&self) -> usize {
        1 + self.offset_constellations.len()
    }

    fn parameter_len(&self) -> usize {
        3 + self.state_len()
    }

    fn offset_index(&self, constellation: Constellation) -> Option<usize> {
        self.offset_constellations
            .iter()
            .position(|candidate| *candidate == constellation)
            .map(|index| index + 1)
    }

    fn contains(&self, constellation: Constellation) -> bool {
        constellation == self.reference_constellation || self.offset_index(constellation).is_some()
    }

    fn reference_clock_bias_s(&self, state: &[f64]) -> f64 {
        state.first().copied().unwrap_or(0.0)
    }

    fn constellation_clock_bias_s(
        &self,
        state: &[f64],
        constellation: Constellation,
    ) -> Option<f64> {
        if constellation == self.reference_constellation {
            return Some(self.reference_clock_bias_s(state));
        }
        let offset_index = self.offset_index(constellation)?;
        Some(self.reference_clock_bias_s(state) + state.get(offset_index).copied().unwrap_or(0.0))
    }

    fn design_row(&self, constellation: Constellation) -> Option<Vec<f64>> {
        if !self.contains(constellation) {
            return None;
        }
        let mut row = vec![0.0; self.state_len()];
        row[0] = 1.0;
        if let Some(offset_index) = self.offset_index(constellation) {
            row[offset_index] = 1.0;
        }
        Some(row)
    }

    fn reproject_state(&self, previous: &ClockStateModel, previous_state: &[f64]) -> Vec<f64> {
        let mut projected_state = vec![0.0; self.state_len()];
        projected_state[0] = previous
            .constellation_clock_bias_s(previous_state, self.reference_constellation)
            .unwrap_or(previous.reference_clock_bias_s(previous_state));
        for (index, constellation) in self.offset_constellations.iter().copied().enumerate() {
            let constellation_bias_s = previous
                .constellation_clock_bias_s(previous_state, constellation)
                .unwrap_or(projected_state[0]);
            projected_state[index + 1] = constellation_bias_s - projected_state[0];
        }
        projected_state
    }

    fn inter_system_biases(&self, state: &[f64]) -> Vec<InterSystemBias> {
        self.offset_constellations
            .iter()
            .enumerate()
            .map(|(index, constellation)| InterSystemBias {
                constellation: *constellation,
                band: Some(constellation_primary_band(*constellation)),
                bias_s: Seconds(state.get(index + 1).copied().unwrap_or(0.0)),
            })
            .collect()
    }
}

#[derive(Debug, Clone)]
struct PositionEstimate {
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    clock_model: ClockStateModel,
    clock_state_s: Vec<f64>,
}

impl PositionEstimate {
    fn origin(clock_model: ClockStateModel) -> Self {
        Self {
            ecef_x_m: 0.0,
            ecef_y_m: 0.0,
            ecef_z_m: 0.0,
            clock_state_s: vec![0.0; clock_model.state_len()],
            clock_model,
        }
    }

    fn reproject(&self, clock_model: ClockStateModel) -> Self {
        if self.clock_model == clock_model {
            return self.clone();
        }
        Self {
            ecef_x_m: self.ecef_x_m,
            ecef_y_m: self.ecef_y_m,
            ecef_z_m: self.ecef_z_m,
            clock_state_s: clock_model.reproject_state(&self.clock_model, &self.clock_state_s),
            clock_model,
        }
    }

    fn reference_clock_bias_s(&self) -> f64 {
        self.clock_model.reference_clock_bias_s(&self.clock_state_s)
    }

    fn constellation_clock_bias_s(&self, constellation: Constellation) -> Option<f64> {
        self.clock_model.constellation_clock_bias_s(&self.clock_state_s, constellation)
    }
}

#[derive(Debug, Clone)]
struct WorkingSetResidual {
    sat: SatId,
    residual_m: f64,
    base_weight: f64,
    effective_weight: f64,
}

#[derive(Debug, Default)]
struct ConstellationResidualAccumulator {
    pre_fit_sum_sq_m2: f64,
    pre_fit_sat_count: usize,
    post_fit_sum_sq_m2: f64,
    post_fit_sat_count: usize,
}

#[derive(Debug, Clone)]
struct SatelliteGeometry {
    observation: PositionObservation,
    corrected_pseudorange_m: f64,
    broadcast_group_delay_correction_chain: PositionObservationCorrectionChain,
    state: SatelliteState,
    iono_delay_m: f64,
    tropo_delay_m: f64,
}

#[derive(Debug, Clone)]
struct WorkingSetSolution {
    estimate: PositionEstimate,
    geometry: Vec<SatelliteGeometry>,
    residuals: Vec<WorkingSetResidual>,
    covariance: Option<Vec<Vec<f64>>>,
    covariance_symmetrized: bool,
    covariance_clamped: bool,
    covariance_max_variance: Option<f64>,
    solver_rank: usize,
    solver_condition_number: Option<f64>,
}

#[derive(Debug, Clone)]
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
    pub robust_weighting: PositionRobustWeighting,
    pub raim: bool,
    pub separation_gate_m: f64,
    pub apply_broadcast_ionosphere: bool,
    pub apply_broadcast_group_delay: bool,
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
            robust_weighting: PositionRobustWeighting::huber(30.0),
            raim: true,
            separation_gate_m: 50.0,
            apply_broadcast_ionosphere: true,
            apply_broadcast_group_delay: true,
            apply_troposphere: false,
        }
    }

    pub fn with_broadcast_ionosphere(mut self, apply_broadcast_ionosphere: bool) -> Self {
        self.apply_broadcast_ionosphere = apply_broadcast_ionosphere;
        self
    }

    pub fn with_broadcast_group_delay(mut self, apply_broadcast_group_delay: bool) -> Self {
        self.apply_broadcast_group_delay = apply_broadcast_group_delay;
        self
    }

    pub fn with_robust_weighting(mut self, robust_weighting: PositionRobustWeighting) -> Self {
        self.robust_weighting = robust_weighting;
        self
    }

    pub fn without_robust_weighting(self) -> Self {
        self.with_robust_weighting(PositionRobustWeighting::disabled())
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
        let navigation = position_broadcast_navigation_from_gps_ephemerides(ephemerides);
        self.try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
            observations,
            &navigation,
            t_rx_s,
            None,
        )
    }

    pub fn solve_wls_with_gps_broadcast_navigation(
        &self,
        observations: &[PositionObservation],
        navigation: &GpsBroadcastNavigationData,
        t_rx_s: f64,
    ) -> Option<PositionSolution> {
        self.try_solve_wls_with_gps_broadcast_navigation(observations, navigation, t_rx_s).ok()
    }

    pub fn try_solve_wls_with_gps_broadcast_navigation(
        &self,
        observations: &[PositionObservation],
        navigation: &GpsBroadcastNavigationData,
        t_rx_s: f64,
    ) -> Result<PositionSolution, PositionSolveRefusal> {
        self.try_solve_wls_with_broadcast_ionosphere(
            observations,
            &navigation.ephemerides,
            t_rx_s,
            navigation.klobuchar.as_ref(),
        )
    }

    pub fn solve_wls_with_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<PositionSolution> {
        let navigation = position_broadcast_navigation_from_gps_ephemerides(ephemerides);
        self.try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
            observations,
            &navigation,
            t_rx_s,
            klobuchar,
        )
        .ok()
    }

    pub fn try_solve_wls_with_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        ephemerides: &[GpsEphemeris],
        t_rx_s: f64,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Result<PositionSolution, PositionSolveRefusal> {
        let navigation = position_broadcast_navigation_from_gps_ephemerides(ephemerides);
        self.try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
            observations,
            &navigation,
            t_rx_s,
            klobuchar,
        )
    }

    pub fn solve_wls_with_navigation_data(
        &self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
        t_rx_s: f64,
    ) -> Option<PositionSolution> {
        self.try_solve_wls_with_navigation_data(observations, navigation, t_rx_s).ok()
    }

    pub fn try_solve_wls_with_navigation_data(
        &self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
        t_rx_s: f64,
    ) -> Result<PositionSolution, PositionSolveRefusal> {
        self.try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
            observations,
            navigation,
            t_rx_s,
            None,
        )
    }

    pub fn solve_wls_with_navigation_data_and_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
        t_rx_s: f64,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<PositionSolution> {
        self.try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
            observations,
            navigation,
            t_rx_s,
            klobuchar,
        )
        .ok()
    }

    pub fn try_solve_wls_with_navigation_data_and_broadcast_ionosphere(
        &self,
        observations: &[PositionObservation],
        navigation: &[PositionBroadcastNavigation],
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
        let mut rejected = timing_rejected;
        let unknown_time_offset_sats =
            unknown_inter_system_time_offset_sats(&observations, navigation);
        if !unknown_time_offset_sats.is_empty() {
            rejected.extend(
                unknown_time_offset_sats
                    .iter()
                    .copied()
                    .map(|sat| (sat, MeasurementRejectReason::TimeInconsistency)),
            );
            return Err(position_solve_refusal(
                PositionSolveRefusalKind::UnknownInterSystemTimeOffset,
                sat_count,
                observations.len().saturating_sub(unknown_time_offset_sats.len()),
                rejected,
            ));
        }
        let inputs = resolve_position_inputs(&observations, navigation, t_rx_s, &mut rejected);
        if inputs.len() < 4 {
            return Err(position_solve_refusal(
                PositionSolveRefusalKind::InvalidEphemeris,
                sat_count,
                inputs.len(),
                rejected,
            ));
        }
        let clock_model = ClockStateModel::from_inputs(&inputs).ok_or_else(|| {
            position_solve_refusal(
                PositionSolveRefusalKind::InvalidEphemeris,
                sat_count,
                inputs.len(),
                rejected.clone(),
            )
        })?;
        let initial_estimate = PositionEstimate::origin(clock_model);
        let mut working_inputs = inputs;
        let mut estimate = initial_estimate;
        let mut raim_fault_detection = None;
        let mut raim_fault_exclusion = None;
        let mut raim_fault_exclusions = Vec::new();
        let mut pre_fit_residual_rms_m = None;
        let mut pre_fit_constellation_residuals = None;
        let working_set = loop {
            let solved =
                self.solve_working_set(&working_inputs, estimate, klobuchar).ok_or_else(|| {
                    position_solve_refusal(
                        PositionSolveRefusalKind::SolverFailure,
                        sat_count,
                        working_inputs.len(),
                        rejected.clone(),
                    )
                })?;
            if pre_fit_residual_rms_m.is_none() {
                pre_fit_residual_rms_m = Some(working_set_rms_m(&solved.residuals));
                pre_fit_constellation_residuals = Some(solved.residuals.clone());
            }
            let outlier_indices = self.outlier_indices(&solved.residuals);
            if outlier_indices.is_empty() {
                break solved;
            }

            if let Some(exclusion_candidate) =
                self.best_single_outlier_candidate(&working_inputs, &solved, klobuchar)
            {
                let rejection_reason = rejection_reason_for_excluded_input(
                    working_inputs
                        .get(exclusion_candidate.excluded_index)
                        .expect("candidate exclusion must reference a working input"),
                    &exclusion_candidate.candidate_estimate,
                    self.apply_broadcast_group_delay,
                );
                if self.raim
                    && !supports_reliable_raim_exclusion_after_prior_exclusions(
                        working_inputs.len(),
                        !raim_fault_exclusions.is_empty(),
                    )
                    && rejection_reason != MeasurementRejectReason::InvalidEphemeris
                {
                    push_unique_rejection(
                        &mut rejected,
                        exclusion_candidate.excluded_sat,
                        rejection_reason,
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
                let exclusion = RaimFaultExclusion {
                    excluded_sat: exclusion_candidate.excluded_sat,
                    pre_exclusion_rms_m: exclusion_candidate.pre_exclusion_rms_m,
                    post_exclusion_rms_m: exclusion_candidate.post_exclusion_rms_m,
                    solution_shift_m: exclusion_candidate.solution_shift_m,
                };
                if raim_fault_exclusion.is_none() {
                    raim_fault_exclusion = Some(exclusion);
                }
                if !raim_fault_exclusions.iter().any(|existing: &RaimFaultExclusion| {
                    existing.excluded_sat == exclusion.excluded_sat
                }) {
                    raim_fault_exclusions.push(exclusion);
                }
                push_unique_rejection(
                    &mut rejected,
                    exclusion_candidate.excluded_sat,
                    rejection_reason,
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
                let input = working_inputs
                    .get(outlier_index)
                    .expect("outlier index must reference an input");
                push_unique_rejection(
                    &mut rejected,
                    input.observation.sat,
                    rejection_reason_for_excluded_input(
                        input,
                        &solved.estimate,
                        self.apply_broadcast_group_delay,
                    ),
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

            let retained_indices =
                outlier_indices.into_iter().collect::<std::collections::BTreeSet<_>>();
            working_inputs = working_inputs
                .into_iter()
                .enumerate()
                .filter_map(|(index, input)| (!retained_indices.contains(&index)).then_some(input))
                .collect();
            estimate = solved.estimate;
        };
        let x = working_set.estimate.ecef_x_m;
        let y = working_set.estimate.ecef_y_m;
        let z = working_set.estimate.ecef_z_m;
        let cb = working_set.estimate.reference_clock_bias_s();

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
        let corrected_observations = corrected_observation_records(
            &final_estimate,
            &working_set.geometry,
            &working_set.residuals,
        )
        .ok_or_else(|| {
            position_solve_refusal(
                PositionSolveRefusalKind::SolverFailure,
                sat_count,
                filtered.len(),
                rejected.clone(),
            )
        })?;
        let raim_solution_separation = self
            .raim
            .then(|| self.solution_separation_check(&working_inputs, &final_estimate, klobuchar))
            .flatten();
        if raim_fault_detection.is_none() {
            if let Some(separation) = raim_solution_separation.as_ref() {
                raim_fault_detection =
                    Some(raim_fault_detection_from_separation(separation, self.separation_gate_m));
            }
        }

        let mut h = Vec::new();
        let mut v = Vec::new();
        for (observation, state, residual_m, _effective_weight) in &filtered {
            let (_range_m, design_row) =
                linearized_geometry_row(&final_estimate, observation.sat.constellation, state)
                    .ok_or_else(|| {
                        position_solve_refusal(
                            PositionSolveRefusalKind::SolverFailure,
                            sat_count,
                            filtered.len(),
                            rejected.clone(),
                        )
                    })?;
            h.push(design_row);
            v.push(*residual_m);
        }

        let dops = compute_dops([x, y, z], &h);
        let post_fit_residual_rms_m = if !v.is_empty() {
            let sum = v.iter().map(|r| r * r).sum::<f64>();
            (sum / v.len() as f64).sqrt()
        } else {
            0.0
        };
        let pre_fit_residual_rms_m = pre_fit_residual_rms_m.unwrap_or(post_fit_residual_rms_m);
        let constellation_residual_rms = constellation_residual_rms(
            pre_fit_constellation_residuals.as_deref().unwrap_or(&working_set.residuals),
            &filtered,
        );

        let (lat, lon, alt) = ecef_to_geodetic(x, y, z);

        let position_covariance_ecef_m2 = working_set.covariance.as_ref().and_then(|covariance| {
            let sigma2 = if !v.is_empty() {
                let sum = v.iter().map(|r| r * r).sum::<f64>();
                let dof = (v.len() as i32 - final_estimate.clock_model.parameter_len() as i32)
                    .max(1) as f64;
                sum / dof
            } else {
                0.0
            };
            scaled_position_covariance_ecef_m2(covariance, sigma2)
        });
        let (sigma_e_m, sigma_n_m, sigma_u_m) = position_covariance_ecef_m2
            .and_then(|covariance_xyz| {
                covariance_enu_standard_deviations_m([x, y, z], covariance_xyz)
            })
            .map(|(sigma_e_m, sigma_n_m, sigma_u_m)| {
                (Some(sigma_e_m), Some(sigma_n_m), Some(sigma_u_m))
            })
            .unwrap_or((None, None, None));
        let (
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
        ) = position_covariance_ecef_m2
            .and_then(|covariance_xyz| horizontal_error_ellipse([x, y, z], covariance_xyz))
            .map(|ellipse| {
                (Some(ellipse.major_axis_m), Some(ellipse.minor_axis_m), Some(ellipse.azimuth_deg))
            })
            .unwrap_or((None, None, None));
        let (sigma_h_m, sigma_v_m) = match (sigma_e_m, sigma_n_m, sigma_u_m) {
            (Some(sigma_e_m), Some(sigma_n_m), Some(sigma_u_m)) => {
                (Some((sigma_e_m * sigma_e_m + sigma_n_m * sigma_n_m).sqrt()), Some(sigma_u_m))
            }
            _ => (None, None),
        };
        let protection_levels = position_covariance_ecef_m2
            .and_then(|covariance_xyz| formal_protection_levels([x, y, z], covariance_xyz));
        let (integrity_hpl_m, integrity_vpl_m) = protection_levels
            .map(|levels: PositionProtectionLevels| {
                (Some(levels.horizontal_m), Some(levels.vertical_m))
            })
            .unwrap_or((None, None));
        let impossible_geometry = detect_impossible_geometry(&final_estimate, filtered.len());
        let replay_timing_anomaly = detect_replay_timing_anomaly(&filtered);

        let rejected_sat_count = rejected.len();
        let broadcast_ionosphere_applied =
            working_set.geometry.iter().any(|geometry| geometry.iono_delay_m.abs() > 0.0);
        Ok(PositionSolution {
            ecef_x_m: x,
            ecef_y_m: y,
            ecef_z_m: z,
            position_covariance_ecef_m2,
            horizontal_error_ellipse_major_axis_m,
            horizontal_error_ellipse_minor_axis_m,
            horizontal_error_ellipse_azimuth_deg,
            sigma_e_m,
            sigma_n_m,
            sigma_u_m,
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            broadcast_ionosphere_applied,
            clock_reference_constellation: final_estimate.clock_model.reference_constellation,
            clock_bias_s: cb,
            inter_system_biases: final_estimate
                .clock_model
                .inter_system_biases(&final_estimate.clock_state_s),
            pdop: dops.map(|dops| dops.pdop).unwrap_or(0.0),
            hdop: dops.map(|dops| dops.hdop),
            vdop: dops.map(|dops| dops.vdop),
            gdop: dops.map(|dops| dops.gdop),
            tdop: dops.map(|dops| dops.tdop),
            pre_fit_residual_rms_m,
            post_fit_residual_rms_m,
            rms_m: post_fit_residual_rms_m,
            sigma_h_m,
            sigma_v_m,
            integrity_hpl_m,
            integrity_vpl_m,
            residuals: filtered
                .iter()
                .map(|(observation, _state, residual_m, effective_weight)| {
                    (observation.sat, *residual_m, *effective_weight)
                })
                .collect(),
            corrected_observations,
            constellation_residual_rms,
            rejected,
            raim_fault_detection,
            raim_fault_exclusion,
            raim_fault_exclusions,
            raim_solution_separation,
            impossible_geometry,
            replay_timing_anomaly,
            covariance_symmetrized: working_set.covariance_symmetrized,
            covariance_clamped: working_set.covariance_clamped,
            covariance_max_variance: working_set.covariance_max_variance,
            solver_rank: working_set.solver_rank,
            solver_condition_number: working_set.solver_condition_number,
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

fn select_valid_ephemeris(
    ephemerides: &[GpsEphemeris],
    sat: SatId,
    receive_tow_s: f64,
) -> Option<&GpsEphemeris> {
    ephemerides
        .iter()
        .filter(|ephemeris| ephemeris.sat == sat)
        .filter(|ephemeris| is_ephemeris_valid(ephemeris, receive_tow_s))
        .min_by(|left, right| {
            let left_age = gps_ephemeris_age(left, receive_tow_s);
            let right_age = gps_ephemeris_age(right, receive_tow_s);
            let left_max_age_s = left_age.toe_age_s.max(left_age.toc_age_s);
            let right_max_age_s = right_age.toe_age_s.max(right_age.toc_age_s);
            left_max_age_s.total_cmp(&right_max_age_s).then_with(|| {
                (left_age.toe_age_s + left_age.toc_age_s)
                    .total_cmp(&(right_age.toe_age_s + right_age.toc_age_s))
            })
        })
}

#[cfg(test)]
#[path = "solver_tests.rs"]
mod tests;

fn resolve_satellite_geometry(
    inputs: &[PositionSolveInput],
    estimate: &PositionEstimate,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_broadcast_ionosphere: bool,
    apply_broadcast_group_delay: bool,
    apply_troposphere: bool,
) -> Option<Vec<SatelliteGeometry>> {
    let mut geometry = Vec::with_capacity(inputs.len());
    for input in inputs {
        let obs = &input.observation;
        let group_delay_chain = broadcast_group_delay_correction_chain(
            obs,
            &input.navigation,
            apply_broadcast_group_delay,
        );
        let corrected_pseudorange_m = group_delay_chain.corrected_pseudorange_m;
        let mut tau = obs
            .signal_timing
            .map(|timing| timing.signal_travel_time_s.0)
            .unwrap_or(corrected_pseudorange_m / 299_792_458.0);
        let mut state = satellite_state_from_observation(
            &input.navigation,
            input.receive_tow_s,
            corrected_pseudorange_m,
            obs.signal_timing,
        )?;
        let mut converged = false;
        for _ in 0..5 {
            let range_m = geometric_range_m(&estimate, &state);
            let receiver_clock_bias_s =
                estimate.constellation_clock_bias_s(obs.sat.constellation)?;
            let next_tau =
                predicted_signal_travel_time_s(range_m, receiver_clock_bias_s, state.clock_bias_s);
            if (next_tau - tau).abs() < 1.0e-9 {
                converged = true;
            }
            tau = next_tau;
            state = satellite_state_at_time(&input.navigation, input.receive_tow_s - tau, tau)?;
            if converged {
                break;
            }
        }
        if !converged {
            return None;
        }
        let iono_delay_m = estimate_broadcast_ionosphere_delay_m(
            &estimate,
            input,
            &state,
            klobuchar,
            apply_broadcast_ionosphere,
        );
        let tropo_delay_m = estimate_saastamoinen_delay_m(&estimate, &state, apply_troposphere);
        geometry.push(SatelliteGeometry {
            observation: obs.clone(),
            corrected_pseudorange_m,
            broadcast_group_delay_correction_chain: group_delay_chain,
            state,
            iono_delay_m,
            tropo_delay_m,
        });
    }
    Some(geometry)
}

fn linearized_pseudorange_row(
    estimate: &PositionEstimate,
    geometry: &SatelliteGeometry,
) -> Option<(f64, Vec<f64>)> {
    let (range_m, mut design_row) =
        linearized_geometry_row(estimate, geometry.observation.sat.constellation, &geometry.state)?;
    let receiver_clock_bias_s =
        estimate.constellation_clock_bias_s(geometry.observation.sat.constellation)?;
    let predicted_pseudorange_m =
        predicted_pseudorange_m(range_m, receiver_clock_bias_s, geometry.state.clock_bias_s);
    let residual_m = geometry.corrected_pseudorange_m
        - geometry.iono_delay_m
        - geometry.tropo_delay_m
        - predicted_pseudorange_m;
    design_row[0] /= range_m;
    design_row[1] /= range_m;
    design_row[2] /= range_m;
    Some((residual_m, design_row))
}

fn estimate_broadcast_ionosphere_delay_m(
    estimate: &PositionEstimate,
    input: &PositionSolveInput,
    state: &SatelliteState,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_broadcast_ionosphere: bool,
) -> f64 {
    if !apply_broadcast_ionosphere {
        return 0.0;
    }
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
    match &input.navigation {
        PositionBroadcastNavigation::Gps(_) => {
            let Some(coefficients) = klobuchar else {
                return 0.0;
            };
            if input.observation.sat.constellation != Constellation::Gps {
                return 0.0;
            }
            KlobucharModel::new(*coefficients).delay_m(
                receiver,
                azimuth_deg,
                elevation_deg,
                Seconds(input.receive_tow_s),
            )
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let Some(signal) = input.observation.signal_id else {
                return 0.0;
            };
            let Some(gps_time) = input.observation.gps_receive_time else {
                return 0.0;
            };
            let (sat_latitude_deg, sat_longitude_deg, sat_altitude_m) =
                ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
            let satellite = Llh {
                lat_deg: sat_latitude_deg,
                lon_deg: sat_longitude_deg,
                alt_m: sat_altitude_m,
            };
            navigation.nequick_delay_m(signal, receiver, satellite, gps_time).unwrap_or(0.0)
        }
        PositionBroadcastNavigation::Beidou(_) | PositionBroadcastNavigation::Glonass(_) => 0.0,
    }
}

fn estimate_saastamoinen_delay_m(
    estimate: &PositionEstimate,
    state: &SatelliteState,
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

fn geometric_range_m(estimate: &PositionEstimate, state: &SatelliteState) -> f64 {
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

fn corrected_observation_records(
    estimate: &PositionEstimate,
    geometry: &[SatelliteGeometry],
    residuals: &[WorkingSetResidual],
) -> Option<Vec<PositionCorrectedObservation>> {
    geometry
        .iter()
        .zip(residuals)
        .map(|(geometry, residual)| {
            let geometric_range_m = geometric_range_m(estimate, &geometry.state);
            let receiver_clock_bias_s =
                estimate.constellation_clock_bias_s(geometry.observation.sat.constellation)?;
            let correction_chain = corrected_observation_chain(geometry, receiver_clock_bias_s);
            Some(PositionCorrectedObservation {
                sat: geometry.observation.sat,
                signal_id: geometry.observation.signal_id,
                correction_chain,
                geometric_range_m,
                residual_m: residual.residual_m,
            })
        })
        .collect()
}

fn corrected_observation_chain(
    geometry: &SatelliteGeometry,
    receiver_clock_bias_s: f64,
) -> PositionObservationCorrectionChain {
    let mut chain = PositionObservationCorrectionChain::new(geometry.observation.pseudorange_m);
    for kind in POSITION_OBSERVATION_CORRECTION_ORDER {
        match kind {
            PositionObservationCorrectionKind::SatelliteClock => {
                chain.push_component(kind, geometry.state.clock_bias_s * SPEED_OF_LIGHT_MPS)
            }
            PositionObservationCorrectionKind::BroadcastGroupDelay => {
                let delta_m = geometry
                    .broadcast_group_delay_correction_chain
                    .component_delta_m(PositionObservationCorrectionKind::BroadcastGroupDelay);
                let applied =
                    geometry.broadcast_group_delay_correction_chain.components.iter().any(
                        |component| {
                            component.kind == PositionObservationCorrectionKind::BroadcastGroupDelay
                                && component.applied
                        },
                    );
                chain.push_component_with_application(kind, delta_m, applied);
            }
            PositionObservationCorrectionKind::Ionosphere => chain.push_component_with_application(
                kind,
                -geometry.iono_delay_m,
                geometry.iono_delay_m != 0.0,
            ),
            PositionObservationCorrectionKind::Troposphere => chain
                .push_component_with_application(
                    kind,
                    -geometry.tropo_delay_m,
                    geometry.tropo_delay_m != 0.0,
                ),
            PositionObservationCorrectionKind::ReceiverClock => {
                chain.push_component(kind, -receiver_clock_bias_s * SPEED_OF_LIGHT_MPS);
            }
            PositionObservationCorrectionKind::Relativity
            | PositionObservationCorrectionKind::EarthRotation
            | PositionObservationCorrectionKind::ReceiverAntenna
            | PositionObservationCorrectionKind::SatelliteAntenna
            | PositionObservationCorrectionKind::EarthTide
            | PositionObservationCorrectionKind::PhaseWindup => {
                chain.push_unapplied_component(kind);
            }
        }
    }
    chain
}

fn linearized_geometry_row(
    estimate: &PositionEstimate,
    constellation: Constellation,
    state: &SatelliteState,
) -> Option<(f64, Vec<f64>)> {
    let dx = estimate.ecef_x_m - state.x_m;
    let dy = estimate.ecef_y_m - state.y_m;
    let dz = estimate.ecef_z_m - state.z_m;
    let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
    if !range_m.is_finite() || range_m <= 0.0 {
        return None;
    }
    let mut design_row = vec![0.0; estimate.clock_model.parameter_len()];
    design_row[0] = dx;
    design_row[1] = dy;
    design_row[2] = dz;
    let clock_design = estimate.clock_model.design_row(constellation)?;
    for (index, coefficient) in clock_design.into_iter().enumerate() {
        design_row[index + 3] = coefficient;
    }
    Some((range_m, design_row))
}

#[cfg(test)]
#[path = "solver_tests/broadcast_group_delay.rs"]
mod broadcast_group_delay;

/// Returns whether a position observation carries a finite and internally consistent
/// transmit-time description for navigation use.
pub fn position_observation_has_valid_satellite_time(
    obs: &PositionObservation,
    t_rx_s: f64,
) -> bool {
    let Some(signal_timing) = obs.signal_timing else {
        return obs.pseudorange_m.is_finite() && obs.pseudorange_m > 0.0;
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

fn sanitize_covariance(mut cov: Vec<Vec<f64>>) -> (Vec<Vec<f64>>, bool, bool, Option<f64>) {
    let mut sym = false;
    let mut clamp = false;
    let mut max_var = None;
    let mut i = 0;
    while i < cov.len() {
        let mut j = 0;
        while j < cov.len() {
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
        let clock_model = ClockStateModel::from_inputs(inputs)?;
        let mut estimate = initial_estimate.reproject(clock_model);
        let mut geometry = resolve_satellite_geometry(
            inputs,
            &estimate,
            klobuchar,
            self.apply_broadcast_ionosphere,
            self.apply_broadcast_group_delay,
            self.apply_troposphere,
        )?;
        let mut covariance = None;
        let mut covariance_symmetrized = false;
        let mut covariance_clamped = false;
        let mut covariance_max_variance = None;
        let mut solver_rank = 0;
        let mut solver_condition_number = None;

        for iteration_index in 0..self.max_iterations {
            if geometry.len() < 4 {
                return None;
            }

            let mut h = Vec::with_capacity(geometry.len());
            let mut residual_values = Vec::with_capacity(geometry.len());
            for satellite_geometry in &geometry {
                let (residual_m, design_row) =
                    linearized_pseudorange_row(&estimate, satellite_geometry)?;
                residual_values.push(residual_m);
                h.push(design_row);
            }

            let weights = self.measurement_weights(iteration_index, &geometry, &residual_values);
            let least_squares = solve_weighted_least_squares(&h, &residual_values, &weights)?;
            let delta = least_squares.delta;
            let covariance_out = least_squares.covariance;
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
            solver_rank = least_squares.rank;
            solver_condition_number = least_squares.condition_number;

            estimate.ecef_x_m += delta.first().copied().unwrap_or(0.0);
            estimate.ecef_y_m += delta.get(1).copied().unwrap_or(0.0);
            estimate.ecef_z_m += delta.get(2).copied().unwrap_or(0.0);
            for (index, clock_delta_m) in delta.iter().copied().skip(3).enumerate() {
                if let Some(clock_state_s) = estimate.clock_state_s.get_mut(index) {
                    *clock_state_s += clock_delta_m / SPEED_OF_LIGHT_MPS;
                }
            }

            let dx = delta.first().copied().unwrap_or(0.0);
            let dy = delta.get(1).copied().unwrap_or(0.0);
            let dz = delta.get(2).copied().unwrap_or(0.0);
            if (dx * dx + dy * dy + dz * dz).sqrt() < self.convergence_m {
                break;
            }

            geometry = resolve_satellite_geometry(
                inputs,
                &estimate,
                klobuchar,
                self.apply_broadcast_ionosphere,
                self.apply_broadcast_group_delay,
                self.apply_troposphere,
            )?;
        }

        geometry = resolve_satellite_geometry(
            inputs,
            &estimate,
            klobuchar,
            self.apply_broadcast_ionosphere,
            self.apply_broadcast_group_delay,
            self.apply_troposphere,
        )?;
        if geometry.len() < 4 {
            return None;
        }
        let residuals = self.finalize_working_set_residuals(&estimate, &geometry);

        Some(WorkingSetSolution {
            estimate,
            geometry,
            residuals,
            covariance,
            covariance_symmetrized,
            covariance_clamped,
            covariance_max_variance,
            solver_rank,
            solver_condition_number,
        })
    }

    fn finalize_working_set_residuals(
        &self,
        estimate: &PositionEstimate,
        geometry: &[SatelliteGeometry],
    ) -> Vec<WorkingSetResidual> {
        let residual_values = geometry
            .iter()
            .map(|satellite_geometry| {
                linearized_pseudorange_row(&estimate, satellite_geometry)
                    .map(|row| row.0)
                    .expect("working-set geometry must linearize")
            })
            .collect::<Vec<_>>();
        let weights = self.measurement_weights(1, geometry, &residual_values);

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
        iteration_index: usize,
        geometry: &[SatelliteGeometry],
        residuals: &[f64],
    ) -> Vec<f64> {
        // Start IRLS from the observation model's base weights so robust kernels
        // do not collapse the first linearization before the state is near truth.
        let mut weights = if iteration_index == 0 {
            vec![1.0; residuals.len()]
        } else {
            robust_weights(residuals, self.robust_weighting)
        };
        if weights.iter().all(|weight| *weight <= 0.0) {
            weights.fill(1.0);
        }
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
                    .then_with(|| left.2.partial_cmp(&right.2).unwrap_or(std::cmp::Ordering::Equal))
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
                self.solve_working_set(&candidate_inputs, solved.estimate.clone(), klobuchar)
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
            let solution_shift_m =
                solution_separation_m(&solved.estimate, &candidate_solution.estimate);
            let candidate = RaimExclusionCandidate {
                excluded_index,
                excluded_sat,
                candidate_estimate: candidate_solution.estimate,
                pre_exclusion_rms_m,
                post_exclusion_rms_m: candidate_rms_m,
                solution_shift_m,
            };
            let better_candidate = best_candidate
                .as_ref()
                .map(|best_candidate: &RaimExclusionCandidate| {
                    let candidate_plausible =
                        estimate_has_plausible_terrestrial_geometry(&candidate.candidate_estimate);
                    let best_plausible = estimate_has_plausible_terrestrial_geometry(
                        &best_candidate.candidate_estimate,
                    );
                    candidate_plausible
                        .cmp(&best_plausible)
                        .then_with(|| {
                            candidate
                                .post_exclusion_rms_m
                                .total_cmp(&best_candidate.post_exclusion_rms_m)
                                .reverse()
                        })
                        .then_with(|| {
                            best_candidate.solution_shift_m.total_cmp(&candidate.solution_shift_m)
                        })
                        == std::cmp::Ordering::Greater
                })
                .unwrap_or(true);
            if better_candidate {
                best_candidate = Some(candidate);
            }
        }

        best_candidate
    }

    fn solution_separation_check(
        &self,
        inputs: &[PositionSolveInput],
        reference_estimate: &PositionEstimate,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Option<RaimSolutionSeparationCheck> {
        if inputs.len() < 5 {
            return None;
        }

        let mut compared_subsets = Vec::with_capacity(inputs.len());
        let reference_rms_m = working_set_rms_m(
            &self.solve_working_set(inputs, reference_estimate.clone(), klobuchar)?.residuals,
        );
        for excluded_index in 0..inputs.len() {
            let candidate_inputs = inputs
                .iter()
                .enumerate()
                .filter_map(|(index, input)| (index != excluded_index).then_some(input.clone()))
                .collect::<Vec<_>>();
            let Some(candidate_solution) =
                self.solve_working_set(&candidate_inputs, reference_estimate.clone(), klobuchar)
            else {
                continue;
            };
            let excluded_sat = inputs
                .get(excluded_index)
                .expect("subset exclusion must reference an input")
                .observation
                .sat;
            compared_subsets.push(RaimSolutionSeparationSubset {
                excluded_sat,
                separation_m: solution_separation_m(
                    reference_estimate,
                    &candidate_solution.estimate,
                ),
            });
        }
        let compared_multi_fault_hypotheses =
            self.multi_fault_solution_separation_hypotheses(inputs, reference_estimate, klobuchar);

        (!compared_subsets.is_empty() || !compared_multi_fault_hypotheses.is_empty()).then_some(
            RaimSolutionSeparationCheck {
                reference_sat_count: inputs.len(),
                compared_subsets,
                compared_multi_fault_hypotheses: compared_multi_fault_hypotheses
                    .into_iter()
                    .filter(|hypothesis| hypothesis.post_exclusion_rms_m <= reference_rms_m)
                    .collect(),
            },
        )
    }

    fn multi_fault_solution_separation_hypotheses(
        &self,
        inputs: &[PositionSolveInput],
        reference_estimate: &PositionEstimate,
        klobuchar: Option<&KlobucharCoefficients>,
    ) -> Vec<RaimFaultHypothesis> {
        if inputs.len() < 6 {
            return Vec::new();
        }

        let mut hypotheses = Vec::new();
        for first_index in 0..inputs.len() {
            for second_index in (first_index + 1)..inputs.len() {
                let candidate_inputs = inputs
                    .iter()
                    .enumerate()
                    .filter_map(|(index, input)| {
                        (index != first_index && index != second_index).then_some(input.clone())
                    })
                    .collect::<Vec<_>>();
                if candidate_inputs.len() < 4 {
                    continue;
                }
                let Some(candidate_solution) = self.solve_working_set(
                    &candidate_inputs,
                    reference_estimate.clone(),
                    klobuchar,
                ) else {
                    continue;
                };
                hypotheses.push(RaimFaultHypothesis {
                    excluded_sats: vec![
                        inputs[first_index].observation.sat,
                        inputs[second_index].observation.sat,
                    ],
                    separation_m: solution_separation_m(
                        reference_estimate,
                        &candidate_solution.estimate,
                    ),
                    post_exclusion_rms_m: working_set_rms_m(&candidate_solution.residuals),
                });
            }
        }
        hypotheses
    }
}

fn rejection_reason_for_excluded_input(
    input: &PositionSolveInput,
    estimate: &PositionEstimate,
    apply_broadcast_group_delay: bool,
) -> MeasurementRejectReason {
    let receiver_clock_bias_s = estimate
        .constellation_clock_bias_s(input.observation.sat.constellation)
        .unwrap_or_else(|| estimate.reference_clock_bias_s());
    let receiver_position_ecef_m = [estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m];
    let gross_ephemeris_mismatch = observation_consistency_metrics(
        &input.observation,
        &input.navigation,
        apply_broadcast_group_delay,
        receiver_position_ecef_m,
        receiver_clock_bias_s,
        None,
        None,
    )
    .map(|metrics| metrics.code_residual_m.abs() >= EPHEMERIS_MISMATCH_CODE_RESIDUAL_GATE_M)
    .unwrap_or(true);

    if gross_ephemeris_mismatch {
        MeasurementRejectReason::InvalidEphemeris
    } else {
        MeasurementRejectReason::Outlier
    }
}

fn detect_impossible_geometry(
    estimate: &PositionEstimate,
    used_satellite_count: usize,
) -> Option<ImpossibleGeometryEvidence> {
    let (receiver_radius_m, altitude_m) = terrestrial_geometry_metrics(estimate);
    if terrestrial_geometry_is_plausible(receiver_radius_m, altitude_m) {
        return None;
    }

    Some(ImpossibleGeometryEvidence {
        receiver_radius_m,
        altitude_m,
        used_satellite_count,
        min_receiver_radius_m: TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M,
        max_receiver_radius_m: TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M,
        min_altitude_m: TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M,
        max_altitude_m: TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M,
    })
}

fn estimate_has_plausible_terrestrial_geometry(estimate: &PositionEstimate) -> bool {
    let (receiver_radius_m, altitude_m) = terrestrial_geometry_metrics(estimate);
    terrestrial_geometry_is_plausible(receiver_radius_m, altitude_m)
}

fn terrestrial_geometry_metrics(estimate: &PositionEstimate) -> (f64, f64) {
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    let (_latitude_deg, _longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    (receiver_radius_m, altitude_m)
}

fn terrestrial_geometry_is_plausible(receiver_radius_m: f64, altitude_m: f64) -> bool {
    receiver_radius_m.is_finite()
        && altitude_m.is_finite()
        && (TERRESTRIAL_GEOMETRY_MIN_RECEIVER_RADIUS_M..=TERRESTRIAL_GEOMETRY_MAX_RECEIVER_RADIUS_M)
            .contains(&receiver_radius_m)
        && (TERRESTRIAL_GEOMETRY_MIN_ALTITUDE_M..=TERRESTRIAL_GEOMETRY_MAX_ALTITUDE_M)
            .contains(&altitude_m)
}

fn push_unique_rejection(
    rejected: &mut Vec<(SatId, MeasurementRejectReason)>,
    sat: SatId,
    reason: MeasurementRejectReason,
) {
    if rejected
        .iter()
        .any(|(rejected_sat, rejected_reason)| *rejected_sat == sat && *rejected_reason == reason)
    {
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

fn detect_replay_timing_anomaly(
    filtered: &[(PositionObservation, SatelliteState, f64, f64)],
) -> Option<ReplayTimingAnomalyEvidence> {
    let mut excess_delays_m = filtered
        .iter()
        .filter_map(|(observation, _state, residual_m, _effective_weight)| {
            observation.signal_timing.is_some().then_some(*residual_m)
        })
        .filter(|delay_m| delay_m.is_finite())
        .collect::<Vec<_>>();
    if excess_delays_m.len() < REPLAY_TIMING_ANOMALY_MIN_MATCHED_SATELLITES {
        return None;
    }

    let median_excess_delay_m = median(&mut excess_delays_m);
    let mut centered_delays_m = excess_delays_m
        .into_iter()
        .map(|delay_m| delay_m - median_excess_delay_m)
        .collect::<Vec<_>>();
    let centered_delay_rms_m =
        (centered_delays_m.iter().map(|delay_m| delay_m * delay_m).sum::<f64>()
            / centered_delays_m.len() as f64)
            .sqrt();
    let max_centered_delay_m =
        centered_delays_m.iter().map(|delay_m| delay_m.abs()).fold(0.0, f64::max);
    centered_delays_m.sort_by(|left, right| left.total_cmp(right));
    let strongest_negative_m = centered_delays_m.first().copied().unwrap_or(0.0).abs();
    let strongest_positive_m = centered_delays_m.last().copied().unwrap_or(0.0);

    if centered_delay_rms_m < REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M
        || max_centered_delay_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M
        || strongest_negative_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M * 0.5
        || strongest_positive_m < REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M * 0.5
    {
        return None;
    }

    Some(ReplayTimingAnomalyEvidence {
        matched_satellite_count: centered_delays_m.len(),
        median_excess_delay_m,
        centered_delay_rms_m,
        max_centered_delay_m,
        centered_delay_rms_threshold_m: REPLAY_TIMING_ANOMALY_CENTERED_DELAY_RMS_THRESHOLD_M,
        max_centered_delay_threshold_m: REPLAY_TIMING_ANOMALY_MAX_CENTERED_DELAY_THRESHOLD_M,
    })
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(|left, right| left.total_cmp(right));
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) * 0.5
    } else {
        values[middle]
    }
}

#[cfg(test)]
#[path = "solver_tests/replay_timing.rs"]
mod replay_timing;

fn constellation_residual_rms(
    pre_fit: &[WorkingSetResidual],
    post_fit: &[(PositionObservation, SatelliteState, f64, f64)],
) -> Vec<NavConstellationResidualRms> {
    let mut by_constellation = BTreeMap::<Constellation, ConstellationResidualAccumulator>::new();

    for residual in pre_fit {
        let entry = by_constellation.entry(residual.sat.constellation).or_default();
        entry.pre_fit_sum_sq_m2 += residual.residual_m.powi(2);
        entry.pre_fit_sat_count += 1;
    }

    for (observation, _state, residual_m, _effective_weight) in post_fit {
        let entry = by_constellation.entry(observation.sat.constellation).or_default();
        entry.post_fit_sum_sq_m2 += residual_m.powi(2);
        entry.post_fit_sat_count += 1;
    }

    by_constellation
        .into_iter()
        .map(|(constellation, summary)| NavConstellationResidualRms {
            constellation,
            pre_fit_rms_m: (summary.pre_fit_sat_count > 0).then(|| {
                bijux_gnss_core::api::Meters(
                    (summary.pre_fit_sum_sq_m2 / summary.pre_fit_sat_count as f64).sqrt(),
                )
            }),
            post_fit_rms_m: (summary.post_fit_sat_count > 0).then(|| {
                bijux_gnss_core::api::Meters(
                    (summary.post_fit_sum_sq_m2 / summary.post_fit_sat_count as f64).sqrt(),
                )
            }),
            pre_fit_sat_count: summary.pre_fit_sat_count,
            post_fit_sat_count: summary.post_fit_sat_count,
        })
        .collect()
}

fn solution_separation_m(left: &PositionEstimate, right: &PositionEstimate) -> f64 {
    let dx = left.ecef_x_m - right.ecef_x_m;
    let dy = left.ecef_y_m - right.ecef_y_m;
    let dz = left.ecef_z_m - right.ecef_z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn supports_reliable_raim_exclusion(usable_sat_count: usize) -> bool {
    usable_sat_count >= 6
}

fn supports_reliable_raim_exclusion_after_prior_exclusions(
    usable_sat_count: usize,
    has_prior_exclusion: bool,
) -> bool {
    supports_reliable_raim_exclusion(usable_sat_count)
        || (has_prior_exclusion && usable_sat_count >= 5)
}

fn raim_fault_detection_from_separation(
    separation: &RaimSolutionSeparationCheck,
    threshold_m: f64,
) -> RaimFaultDetection {
    if let Some(max_subset) = separation.max_separation() {
        if max_subset.separation_m > threshold_m {
            RaimFaultDetection::fault_detected(
                max_subset.excluded_sat,
                max_subset.separation_m,
                threshold_m,
            )
        } else {
            RaimFaultDetection::consistent(max_subset.separation_m, threshold_m)
        }
    } else {
        RaimFaultDetection::consistent(0.0, threshold_m)
    }
}

pub fn invert_4x4(a: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    matrix::invert_4x4(a)
}
