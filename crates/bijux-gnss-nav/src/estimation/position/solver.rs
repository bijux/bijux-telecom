#![allow(missing_docs)]

use std::collections::BTreeMap;

use super::navigation::{
    corrected_pseudorange_m, resolve_position_inputs, satellite_state_at_time,
    satellite_state_from_observation, unknown_inter_system_time_offset_sats, PositionSolveInput,
    SatelliteState,
};
use super::raim::{RaimFaultDetection, RaimFaultExclusion};
use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::{gps_ephemeris_age, is_ephemeris_valid, GpsEphemeris};
use bijux_gnss_core::api::{
    Constellation, GpsTime, InterSystemBias, Llh, MeasurementRejectReason,
    NavConstellationResidualRms, ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus, SatId,
    Seconds, SigId, SignalBand,
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
    pub residuals: Vec<(SatId, f64, f64)>,
    pub constellation_residual_rms: Vec<NavConstellationResidualRms>,
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
    UnknownInterSystemTimeOffset,
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
    pub doppler_hz: Option<f64>,
    pub doppler_var_hz2: Option<f64>,
    pub cn0_dbhz: f64,
    pub elevation_deg: Option<f64>,
    pub weight: f64,
    pub gps_receive_time: Option<GpsTime>,
    pub signal_timing: Option<ObsSignalTiming>,
    pub signal_id: Option<SigId>,
}

#[derive(Debug, Clone)]
pub enum PositionBroadcastNavigation {
    Gps(GpsEphemeris),
    Galileo(GalileoBroadcastNavigationData),
    Beidou(BeidouBroadcastNavigationData),
    Glonass(GlonassBroadcastNavigationFrame),
}

impl PositionBroadcastNavigation {
    pub fn sat(&self) -> SatId {
        match self {
            Self::Gps(ephemeris) => ephemeris.sat,
            Self::Galileo(navigation) => navigation.sat,
            Self::Beidou(navigation) => navigation.sat,
            Self::Glonass(navigation) => navigation.sat,
        }
    }

    pub fn constellation(&self) -> Constellation {
        self.sat().constellation
    }
}

pub fn position_broadcast_navigation_from_gps_ephemerides(
    ephemerides: &[GpsEphemeris],
) -> Vec<PositionBroadcastNavigation> {
    ephemerides.iter().cloned().map(PositionBroadcastNavigation::Gps).collect()
}

pub fn position_broadcast_navigation_from_glonass_frames(
    navigation_frames: &[GlonassBroadcastNavigationFrame],
) -> Vec<PositionBroadcastNavigation> {
    navigation_frames.iter().cloned().map(PositionBroadcastNavigation::Glonass).collect()
}

pub fn position_broadcast_navigation_from_beidou_navigations(
    navigations: &[BeidouBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    navigations.iter().cloned().map(PositionBroadcastNavigation::Beidou).collect()
}

pub fn position_observations_from_epoch(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    let gps_receive_time = epoch.gps_time();
    let mut preferred_by_sat: BTreeMap<SatId, &ObsSatellite> = BTreeMap::new();
    for observation in &epoch.sats {
        preferred_by_sat
            .entry(observation.signal_id.sat)
            .and_modify(|current| {
                if prefer_position_observation(observation, current) {
                    *current = observation;
                }
            })
            .or_insert(observation);
    }
    preferred_by_sat
        .into_values()
        .map(|observation| PositionObservation {
            sat: observation.signal_id.sat,
            pseudorange_m: observation.pseudorange_m.0,
            doppler_hz: Some(observation.doppler_hz.0),
            doppler_var_hz2: Some(observation.doppler_var_hz2),
            cn0_dbhz: observation.cn0_dbhz,
            elevation_deg: observation.elevation_deg,
            weight: observation.weight.unwrap_or(1.0),
            gps_receive_time,
            signal_timing: observation.timing,
            signal_id: Some(observation.signal_id),
        })
        .collect()
}

fn prefer_position_observation(candidate: &ObsSatellite, current: &ObsSatellite) -> bool {
    let candidate_rank = position_observation_preference(candidate);
    let current_rank = position_observation_preference(current);
    candidate_rank < current_rank
}

fn position_observation_preference(observation: &ObsSatellite) -> (u8, u8, u8, u8) {
    (
        observation_status_rank(observation.observation_status),
        signal_band_rank(observation.signal_id.band),
        if observation.timing.is_some() { 0 } else { 1 },
        if observation.lock_flags.carrier_lock { 0 } else { 1 },
    )
}

fn observation_status_rank(status: ObservationStatus) -> u8 {
    match status {
        ObservationStatus::Accepted => 0,
        ObservationStatus::Weak => 1,
        ObservationStatus::Missing => 2,
        ObservationStatus::Rejected => 3,
        ObservationStatus::Inconsistent => 4,
    }
}

fn signal_band_rank(band: SignalBand) -> u8 {
    match band {
        SignalBand::L1 => 0,
        SignalBand::L2 => 1,
        SignalBand::L5 => 2,
        SignalBand::E1 => 3,
        SignalBand::E5 => 4,
        SignalBand::B1 => 5,
        SignalBand::B2 => 6,
        SignalBand::Unknown => 7,
    }
}

fn constellation_primary_band(constellation: Constellation) -> SignalBand {
    match constellation {
        Constellation::Gps => SignalBand::L1,
        Constellation::Galileo => SignalBand::E1,
        Constellation::Glonass => SignalBand::L1,
        Constellation::Beidou => SignalBand::B1,
        Constellation::Unknown => SignalBand::Unknown,
    }
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
}

#[derive(Debug, Clone, Copy)]
struct RaimSolutionSeparation {
    suspect_sat: SatId,
    separation_m: f64,
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
    pub robust: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub separation_gate_m: f64,
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
            robust: true,
            huber_k: 30.0,
            raim: true,
            separation_gate_m: 50.0,
            apply_broadcast_group_delay: true,
            apply_troposphere: false,
        }
    }

    pub fn with_broadcast_group_delay(mut self, apply_broadcast_group_delay: bool) -> Self {
        self.apply_broadcast_group_delay = apply_broadcast_group_delay;
        self
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
        let separation =
            self.raim.then(|| max_solution_separation(&filtered, &final_estimate)).flatten();
        if raim_fault_detection.is_none() {
            if let Some(separation) = separation {
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

        let (sigma_h_m, sigma_v_m) = working_set
            .covariance
            .map(|cov| {
                let sigma2 = if !v.is_empty() {
                    let sum = v.iter().map(|r| r * r).sum::<f64>();
                    let dof = (v.len() as i32 - final_estimate.clock_model.parameter_len() as i32)
                        .max(1) as f64;
                    sum / dof
                } else {
                    0.0
                };
                let covariance_xyz = [
                    [cov[0][0] * sigma2, cov[0][1] * sigma2, cov[0][2] * sigma2],
                    [cov[1][0] * sigma2, cov[1][1] * sigma2, cov[1][2] * sigma2],
                    [cov[2][0] * sigma2, cov[2][1] * sigma2, cov[2][2] * sigma2],
                ];
                covariance_horizontal_vertical([x, y, z], covariance_xyz).unwrap_or((0.0, 0.0))
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
            sigma_h_m: Some(sigma_h_m),
            sigma_v_m: Some(sigma_v_m),
            residuals: filtered
                .iter()
                .map(|(observation, _state, residual_m, effective_weight)| {
                    (observation.sat, *residual_m, *effective_weight)
                })
                .collect(),
            constellation_residual_rms,
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
mod tests {
    use super::{
        constellation_residual_rms, position_broadcast_navigation_from_beidou_navigations,
        position_broadcast_navigation_from_glonass_frames,
        position_broadcast_navigation_from_gps_ephemerides, position_observations_from_epoch,
        resolve_position_inputs, unknown_inter_system_time_offset_sats,
        PositionBroadcastNavigation, PositionObservation, SatelliteState, WorkingSetResidual,
    };
    use crate::estimation::position::navigation::navigation_time_relationship_is_known;
    use crate::orbits::beidou::{
        BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
        BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
    };
    use crate::orbits::galileo::{
        GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
        GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
        GalileoSystemTime,
    };
    use crate::orbits::glonass::{
        GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame, GlonassFrameTime,
        GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassSatelliteType,
        GlonassStateVector, GlonassSystemTime,
    };
    use crate::orbits::gps::GpsEphemeris;
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        Seconds, SigId, SignalBand, SignalCode,
    };

    fn sample_ephemeris(sat: SatId, toe_s: f64, toc_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat,
            iodc: 0,
            iode: 0,
            week: 1573,
            sv_health: 0,
            toe_s,
            toc_s,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
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

    fn sample_galileo_navigation(
        sat: SatId,
        toe_s: f64,
        t0c_s: f64,
    ) -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat,
            iodnav: 0x01,
            gst: GalileoSystemTime { week: 2222, tow_s: toe_s as u32 },
            sisa_e1_e5b: 0,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                bgd_e1_e5a_s: 0.0,
                bgd_e1_e5b_s: 0.0,
            },
            ephemeris: GalileoEphemeris {
                sat,
                iodnav: 0x01,
                toe_s,
                sqrt_a: 5_440.612_319,
                e: 0.001_23,
                i0: 0.953,
                idot: -2.1e-10,
                omega0: 1.17,
                omegadot: -5.8e-9,
                w: -0.37,
                m0: 0.84,
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

    fn sample_glonass_navigation(
        sat: SatId,
        ephemeris_reference_time_s: u32,
        gps_minus_glonass_s: f64,
    ) -> GlonassBroadcastNavigationFrame {
        GlonassBroadcastNavigationFrame {
            sat,
            immediate: GlonassImmediateNavigationData {
                sat,
                frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
                ephemeris_reference_time_s,
                tb_update_interval_min: 30,
                tb_is_odd: Some(true),
                state_vector: GlonassStateVector {
                    x_m: -7_557_760.253_906_25,
                    y_m: -23_962_225.585_937_5,
                    z_m: -4_337_567.871_093_75,
                    vx_mps: 101.318_359_375,
                    vy_mps: 602.112_770_080_566_4,
                    vz_mps: -3_495.733_261_108_398_4,
                    ax_mps2: -3.725_290_298_461_914e-6,
                    ay_mps2: 0.0,
                    az_mps2: 1.862_645_149_230_957e-6,
                },
                relative_frequency_bias: 0.0,
                clock_bias_s: -2.572_406_083_345_413_2e-5,
                l2_l1_delay_s: Some(5.587_935_448e-9),
                health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
                immediate_data_age_days: 28,
                satellite_type: GlonassSatelliteType::GlonassM,
                reported_slot: None,
                system_time: Some(GlonassSystemTime {
                    day_number: 864,
                    four_year_interval: Some(8),
                }),
                accuracy_code: Some(2),
            },
            system_time: Some(GlonassAlmanacTimeData {
                system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
                utc_offset_s: 0.0,
                gps_minus_glonass_s,
            }),
            almanac_entries: Vec::new(),
        }
    }

    fn sample_beidou_navigation(
        sat: SatId,
        toe_s: f64,
        toc_s: f64,
    ) -> BeidouBroadcastNavigationData {
        BeidouBroadcastNavigationData {
            sat,
            bdt: BeidouSystemTime { week: 888, sow_s: toe_s as u32 },
            urai: 0,
            signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
            clock: BeidouClockCorrection {
                toc_s,
                aodc: 0,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                tgd1_s: 0.0,
                tgd2_s: 0.0,
            },
            ephemeris: BeidouEphemeris {
                sat,
                aode: 0,
                toe_s,
                sqrt_a: 5_282.61,
                e: 0.0021,
                i0: 0.962,
                idot: -1.4e-10,
                omega0: 0.77,
                omegadot: -7.9e-9,
                w: -0.41,
                m0: 0.53,
                delta_n: 3.4e-9,
                cuc: -1.2e-6,
                cus: 3.1e-6,
                crc: 146.0,
                crs: -84.0,
                cic: 1.1e-7,
                cis: -1.7e-7,
            },
            ionosphere: BeidouIonosphericCorrection {
                alpha0: 0.0,
                alpha1: 0.0,
                alpha2: 0.0,
                alpha3: 0.0,
                beta0: 0.0,
                beta1: 0.0,
                beta2: 0.0,
                beta3: 0.0,
            },
        }
    }

    #[test]
    fn resolve_position_inputs_uses_nearest_valid_ephemeris() {
        let sat = SatId { constellation: Constellation::Gps, prn: 13 };
        let observations = vec![PositionObservation {
            sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        }];
        let ephemerides = vec![
            sample_ephemeris(sat, 100_000.0, 100_000.0),
            sample_ephemeris(sat, 200_000.0, 200_000.0),
        ];
        let navigation = position_broadcast_navigation_from_gps_ephemerides(&ephemerides);
        let mut rejected = Vec::new();

        let inputs = resolve_position_inputs(&observations, &navigation, 200_030.0, &mut rejected);

        assert!(rejected.is_empty());
        assert_eq!(inputs.len(), 1);
        match &inputs[0].navigation {
            PositionBroadcastNavigation::Gps(ephemeris) => {
                assert_eq!(ephemeris.toe_s, 200_000.0);
                assert_eq!(ephemeris.toc_s, 200_000.0);
            }
            PositionBroadcastNavigation::Galileo(_)
            | PositionBroadcastNavigation::Beidou(_)
            | PositionBroadcastNavigation::Glonass(_) => panic!("expected gps navigation"),
        }
    }

    #[test]
    fn position_broadcast_navigation_preserves_satellite_identity() {
        let gps_sat = SatId { constellation: Constellation::Gps, prn: 13 };
        let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
        let beidou_sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        let gps_navigation =
            PositionBroadcastNavigation::Gps(sample_ephemeris(gps_sat, 200_000.0, 200_000.0));
        let galileo_navigation = PositionBroadcastNavigation::Galileo(sample_galileo_navigation(
            galileo_sat,
            64_800.0,
            66_000.0,
        ));
        let beidou_navigation = PositionBroadcastNavigation::Beidou(sample_beidou_navigation(
            beidou_sat, 345_600.0, 345_600.0,
        ));
        let glonass_navigation = PositionBroadcastNavigation::Glonass(sample_glonass_navigation(
            glonass_sat,
            83_700,
            -10_782.0,
        ));

        assert_eq!(gps_navigation.sat(), gps_sat);
        assert_eq!(gps_navigation.constellation(), Constellation::Gps);
        assert_eq!(galileo_navigation.sat(), galileo_sat);
        assert_eq!(galileo_navigation.constellation(), Constellation::Galileo);
        assert_eq!(beidou_navigation.sat(), beidou_sat);
        assert_eq!(beidou_navigation.constellation(), Constellation::Beidou);
        assert_eq!(glonass_navigation.sat(), glonass_sat);
        assert_eq!(glonass_navigation.constellation(), Constellation::Glonass);
    }

    #[test]
    fn gps_ephemerides_convert_into_position_navigation_entries() {
        let ephemerides = vec![
            sample_ephemeris(
                SatId { constellation: Constellation::Gps, prn: 13 },
                100_000.0,
                100_000.0,
            ),
            sample_ephemeris(
                SatId { constellation: Constellation::Gps, prn: 14 },
                200_000.0,
                200_000.0,
            ),
        ];

        let navigation = position_broadcast_navigation_from_gps_ephemerides(&ephemerides);

        assert_eq!(navigation.len(), 2);
        assert_eq!(navigation[0].sat(), ephemerides[0].sat);
        assert_eq!(navigation[1].sat(), ephemerides[1].sat);
    }

    #[test]
    fn glonass_frames_convert_into_position_navigation_entries() {
        let navigation_frames = vec![
            sample_glonass_navigation(
                SatId { constellation: Constellation::Glonass, prn: 14 },
                83_700,
                -10_782.0,
            ),
            sample_glonass_navigation(
                SatId { constellation: Constellation::Glonass, prn: 21 },
                84_600,
                -10_782.0,
            ),
        ];

        let navigation = position_broadcast_navigation_from_glonass_frames(&navigation_frames);

        assert_eq!(navigation.len(), 2);
        assert_eq!(navigation[0].sat(), navigation_frames[0].sat);
        assert_eq!(navigation[1].sat(), navigation_frames[1].sat);
    }

    #[test]
    fn beidou_navigation_convert_into_position_navigation_entries() {
        let navigations = vec![
            sample_beidou_navigation(
                SatId { constellation: Constellation::Beidou, prn: 11 },
                345_600.0,
                345_600.0,
            ),
            sample_beidou_navigation(
                SatId { constellation: Constellation::Beidou, prn: 12 },
                346_200.0,
                346_200.0,
            ),
        ];

        let navigation = position_broadcast_navigation_from_beidou_navigations(&navigations);

        assert_eq!(navigation.len(), 2);
        assert_eq!(navigation[0].sat(), navigations[0].sat);
        assert_eq!(navigation[1].sat(), navigations[1].sat);
    }

    #[test]
    fn resolve_position_inputs_accepts_valid_glonass_navigation() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        let observations = vec![PositionObservation {
            sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        }];
        let navigation_frames = vec![sample_glonass_navigation(sat, 83_700, -10_782.0)];
        let navigation = position_broadcast_navigation_from_glonass_frames(&navigation_frames);
        let mut rejected = Vec::new();

        let inputs = resolve_position_inputs(&observations, &navigation, 504_918.0, &mut rejected);

        assert!(rejected.is_empty());
        assert_eq!(inputs.len(), 1);
        match &inputs[0].navigation {
            PositionBroadcastNavigation::Glonass(navigation) => {
                assert_eq!(navigation.sat, sat);
                assert_eq!(navigation.immediate.ephemeris_reference_time_s, 83_700);
            }
            PositionBroadcastNavigation::Gps(_)
            | PositionBroadcastNavigation::Galileo(_)
            | PositionBroadcastNavigation::Beidou(_) => panic!("expected glonass navigation"),
        }
    }

    #[test]
    fn resolve_position_inputs_accepts_valid_beidou_navigation() {
        let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
        let observations = vec![PositionObservation {
            sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        }];
        let navigations = vec![sample_beidou_navigation(sat, 345_600.0, 345_600.0)];
        let navigation = position_broadcast_navigation_from_beidou_navigations(&navigations);
        let mut rejected = Vec::new();

        let inputs = resolve_position_inputs(&observations, &navigation, 345_630.0, &mut rejected);

        assert!(rejected.is_empty());
        assert_eq!(inputs.len(), 1);
        match &inputs[0].navigation {
            PositionBroadcastNavigation::Beidou(navigation) => {
                assert_eq!(navigation.sat, sat);
                assert_eq!(navigation.ephemeris.toe_s, 345_600.0);
                assert_eq!(navigation.clock.toc_s, 345_600.0);
            }
            PositionBroadcastNavigation::Gps(_)
            | PositionBroadcastNavigation::Galileo(_)
            | PositionBroadcastNavigation::Glonass(_) => panic!("expected beidou navigation"),
        }
    }

    #[test]
    fn glonass_navigation_without_gps_time_offset_is_not_time_resolved() {
        let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        let navigation = sample_glonass_navigation(sat, 83_700, -10_782.0);

        assert!(navigation_time_relationship_is_known(&PositionBroadcastNavigation::Glonass(
            navigation.clone(),
        )));

        let mut navigation_without_system_time = navigation;
        navigation_without_system_time.system_time = None;

        assert!(!navigation_time_relationship_is_known(&PositionBroadcastNavigation::Glonass(
            navigation_without_system_time
        )));
    }

    #[test]
    fn mixed_observations_detect_unknown_glonass_time_offset() {
        let gps_sat = SatId { constellation: Constellation::Gps, prn: 13 };
        let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        let observations = vec![
            PositionObservation {
                sat: gps_sat,
                pseudorange_m: 24_000_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: None,
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            PositionObservation {
                sat: glonass_sat,
                pseudorange_m: 24_100_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: None,
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
        ];
        let mut glonass_navigation = sample_glonass_navigation(glonass_sat, 83_700, -10_782.0);
        glonass_navigation.system_time = None;
        let navigation = vec![
            PositionBroadcastNavigation::Gps(sample_ephemeris(gps_sat, 200_000.0, 200_000.0)),
            PositionBroadcastNavigation::Glonass(glonass_navigation),
        ];

        let unknown = unknown_inter_system_time_offset_sats(&observations, &navigation);

        assert_eq!(unknown, vec![glonass_sat]);
    }

    #[test]
    fn constellation_residual_rms_tracks_pre_fit_and_post_fit_groups() {
        let gps_sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
        let pre_fit = vec![
            WorkingSetResidual {
                sat: gps_sat,
                residual_m: 3.0,
                base_weight: 1.0,
                effective_weight: 1.0,
            },
            WorkingSetResidual {
                sat: gps_sat,
                residual_m: 4.0,
                base_weight: 1.0,
                effective_weight: 1.0,
            },
            WorkingSetResidual {
                sat: galileo_sat,
                residual_m: 12.0,
                base_weight: 1.0,
                effective_weight: 1.0,
            },
        ];
        let post_fit = vec![
            (
                PositionObservation {
                    sat: gps_sat,
                    pseudorange_m: 24_000_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(45.0),
                    weight: 1.0,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                SatelliteState {
                    x_m: 0.0,
                    y_m: 0.0,
                    z_m: 0.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                },
                1.5,
                1.0,
            ),
            (
                PositionObservation {
                    sat: galileo_sat,
                    pseudorange_m: 24_100_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(50.0),
                    weight: 1.0,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                SatelliteState {
                    x_m: 0.0,
                    y_m: 0.0,
                    z_m: 0.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                },
                2.0,
                1.0,
            ),
        ];

        let summaries = constellation_residual_rms(&pre_fit, &post_fit);

        assert_eq!(summaries.len(), 2);
        let gps = summaries
            .iter()
            .find(|summary| summary.constellation == Constellation::Gps)
            .expect("gps summary");
        let galileo = summaries
            .iter()
            .find(|summary| summary.constellation == Constellation::Galileo)
            .expect("galileo summary");
        assert_eq!(gps.pre_fit_sat_count, 2);
        assert_eq!(gps.post_fit_sat_count, 1);
        assert!((gps.pre_fit_rms_m.expect("gps pre-fit").0 - 3.535_533_905_9).abs() < 1.0e-9);
        assert!((gps.post_fit_rms_m.expect("gps post-fit").0 - 1.5).abs() < 1.0e-12);
        assert_eq!(galileo.pre_fit_sat_count, 1);
        assert_eq!(galileo.post_fit_sat_count, 1);
        assert!((galileo.pre_fit_rms_m.expect("galileo pre-fit").0 - 12.0).abs() < 1.0e-12);
        assert!((galileo.post_fit_rms_m.expect("galileo post-fit").0 - 2.0).abs() < 1.0e-12);
    }

    #[test]
    fn position_observation_without_signal_timing_is_valid_when_pseudorange_is_finite() {
        let observation = PositionObservation {
            sat: SatId { constellation: Constellation::Gps, prn: 13 },
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        };

        assert!(super::position_observation_has_valid_satellite_time(&observation, 0.0));
    }

    #[test]
    fn position_observations_from_epoch_prefers_l1_per_satellite() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let epoch = ObsEpoch {
            t_rx_s: Seconds(1000.0),
            source_time: ReceiverSampleTrace::from_sample_index(0, 1000.0),
            gps_week: Some(2000),
            tow_s: Some(Seconds(1000.0)),
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                ObsSatellite {
                    signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
                    pseudorange_m: Meters(22_000_010.0),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: Cycles(10.0),
                    carrier_phase_var_cycles2: 1.0,
                    doppler_hz: Hertz(0.0),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 35.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: None,
                    azimuth_deg: None,
                    weight: None,
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata::default(),
                },
                ObsSatellite {
                    signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: Meters(22_000_000.0),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: Cycles(20.0),
                    carrier_phase_var_cycles2: 1.0,
                    doppler_hz: Hertz(0.0),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: None,
                    azimuth_deg: None,
                    weight: None,
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata::default(),
                },
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };

        let observations = position_observations_from_epoch(&epoch);

        assert_eq!(observations.len(), 1);
        assert_eq!(observations[0].sat, sat);
        assert_eq!(observations[0].pseudorange_m, 22_000_000.0);
        assert_eq!(observations[0].doppler_hz, Some(0.0));
        assert_eq!(observations[0].doppler_var_hz2, Some(1.0));
        assert_eq!(observations[0].cn0_dbhz, 45.0);
        assert_eq!(
            observations[0].signal_id,
            Some(SigId { sat, band: SignalBand::L1, code: SignalCode::Ca })
        );
    }
}

fn resolve_satellite_geometry(
    inputs: &[PositionSolveInput],
    estimate: &PositionEstimate,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_broadcast_group_delay: bool,
    apply_troposphere: bool,
) -> Option<Vec<SatelliteGeometry>> {
    let mut geometry = Vec::with_capacity(inputs.len());
    for input in inputs {
        let obs = &input.observation;
        let corrected_pseudorange_m =
            corrected_pseudorange_m(obs, &input.navigation, apply_broadcast_group_delay);
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
        let iono_delay_m = estimate_klobuchar_delay_m(&estimate, input, &state, klobuchar);
        let tropo_delay_m = estimate_saastamoinen_delay_m(&estimate, &state, apply_troposphere);
        geometry.push(SatelliteGeometry {
            observation: obs.clone(),
            corrected_pseudorange_m,
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

fn estimate_klobuchar_delay_m(
    estimate: &PositionEstimate,
    input: &PositionSolveInput,
    state: &SatelliteState,
    klobuchar: Option<&KlobucharCoefficients>,
) -> f64 {
    let Some(coefficients) = klobuchar else {
        return 0.0;
    };
    if input.observation.sat.constellation != Constellation::Gps {
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
    KlobucharModel::new(*coefficients).delay_m(
        receiver,
        azimuth_deg,
        elevation_deg,
        Seconds(input.receive_tow_s),
    )
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
mod broadcast_group_delay_tests {
    use super::{
        corrected_pseudorange_m, PositionBroadcastNavigation, PositionObservation, PositionSolver,
    };
    use crate::corrections::broadcast_group_delay::gps_broadcast_group_delay_code_bias_m;
    use crate::orbits::gps::GpsEphemeris;
    use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

    fn sample_gps_ephemeris_with_tgd(prn: u8, tgd_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 1,
            iode: 1,
            week: 2209,
            sv_health: 0,
            toe_s: 504_000.0,
            toc_s: 504_018.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
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
            tgd: tgd_s,
        }
    }

    #[test]
    fn position_solver_applies_broadcast_group_delay_by_default() {
        assert!(PositionSolver::new().apply_broadcast_group_delay);
    }

    #[test]
    fn position_solver_can_disable_broadcast_group_delay() {
        assert!(
            !PositionSolver::new().with_broadcast_group_delay(false).apply_broadcast_group_delay
        );
    }

    #[test]
    fn corrected_pseudorange_respects_broadcast_group_delay_toggle() {
        let ephemeris = sample_gps_ephemeris_with_tgd(7, -8.0e-9);
        let navigation = PositionBroadcastNavigation::Gps(ephemeris.clone());
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let signal_id = SigId { sat, band: SignalBand::L5, code: SignalCode::Unknown };
        let observation = PositionObservation {
            sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: Some(signal_id),
        };
        let expected_bias_m = gps_broadcast_group_delay_code_bias_m(signal_id, &ephemeris)
            .expect("GPS L5 broadcast group delay bias");

        let corrected = corrected_pseudorange_m(&observation, &navigation, true);
        let uncorrected = corrected_pseudorange_m(&observation, &navigation, false);

        assert_eq!(uncorrected, observation.pseudorange_m);
        assert!((corrected - (observation.pseudorange_m - expected_bias_m)).abs() < 1.0e-12);
    }
}

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
            self.apply_broadcast_group_delay,
            self.apply_troposphere,
        )?;
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
                    linearized_pseudorange_row(&estimate, satellite_geometry)?;
                residual_values.push(residual_m);
                h.push(design_row);
            }

            let weights = self.measurement_weights(&geometry, &residual_values);
            let (delta, covariance_out) = solve_weighted_normal_eq(&h, &residual_values, &weights)?;
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
                self.apply_broadcast_group_delay,
                self.apply_troposphere,
            )?;
        }

        geometry = resolve_satellite_geometry(
            inputs,
            &estimate,
            klobuchar,
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

    fn measurement_weights(&self, geometry: &[SatelliteGeometry], residuals: &[f64]) -> Vec<f64> {
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
    filtered: &[(PositionObservation, SatelliteState, f64, f64)],
    final_estimate: &PositionEstimate,
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
        for (observation, state, residual_m, _effective_weight) in &subset {
            let (_range_m, design_row) =
                linearized_geometry_row(final_estimate, observation.sat.constellation, state)?;
            h_sep.push(design_row);
            v_sep.push(*residual_m);
        }
        if let Some((delta, _)) = solve_weighted_normal_eq(&h_sep, &v_sep, &vec![1.0; v_sep.len()])
        {
            let dx = delta.first().copied().unwrap_or(0.0);
            let dy = delta.get(1).copied().unwrap_or(0.0);
            let dz = delta.get(2).copied().unwrap_or(0.0);
            let separation_m = (dx * dx + dy * dy + dz * dz).sqrt();
            let candidate = RaimSolutionSeparation { suspect_sat: removed.0.sat, separation_m };
            if max_separation
                .map(|current: RaimSolutionSeparation| {
                    candidate.separation_m > current.separation_m
                })
                .unwrap_or(true)
            {
                max_separation = Some(candidate);
            }
        }
    }

    max_separation
}

type NormalEqSolution = (Vec<f64>, Vec<Vec<f64>>);

fn solve_weighted_normal_eq(h: &[Vec<f64>], v: &[f64], w: &[f64]) -> Option<NormalEqSolution> {
    let dimension = h.first()?.len();
    let mut n = vec![vec![0.0_f64; dimension]; dimension];
    let mut u = vec![0.0_f64; dimension];
    for (i, row) in h.iter().enumerate() {
        let wi = w.get(i).copied().unwrap_or(1.0);
        if row.len() != dimension {
            return None;
        }
        for r in 0..dimension {
            u[r] += row[r] * v[i] * wi;
            for c in 0..dimension {
                n[r][c] += row[r] * row[c] * wi;
            }
        }
    }
    let inv = invert_matrix(n)?;
    let mut delta = vec![0.0_f64; dimension];
    for row in 0..dimension {
        for col in 0..dimension {
            delta[row] += inv[row][col] * u[col];
        }
    }
    Some((delta, inv))
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

fn invert_matrix(mut matrix: Vec<Vec<f64>>) -> Option<Vec<Vec<f64>>> {
    let dimension = matrix.len();
    if dimension == 0 || matrix.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut augmented = vec![vec![0.0_f64; dimension * 2]; dimension];
    for row in 0..dimension {
        for col in 0..dimension {
            augmented[row][col] = matrix[row][col];
        }
        augmented[row][row + dimension] = 1.0;
    }
    for pivot_col in 0..dimension {
        let mut pivot_row = pivot_col;
        let mut pivot_abs = augmented[pivot_col][pivot_col].abs();
        for (row_index, row) in augmented.iter().enumerate().skip(pivot_col + 1) {
            if row[pivot_col].abs() > pivot_abs {
                pivot_abs = row[pivot_col].abs();
                pivot_row = row_index;
            }
        }
        if pivot_abs < 1.0e-12 {
            return None;
        }
        if pivot_row != pivot_col {
            augmented.swap(pivot_col, pivot_row);
        }
        let pivot_inverse = 1.0 / augmented[pivot_col][pivot_col];
        for col in pivot_col..(dimension * 2) {
            augmented[pivot_col][col] *= pivot_inverse;
        }
        for row in 0..dimension {
            if row == pivot_col {
                continue;
            }
            let factor = augmented[row][pivot_col];
            for col in pivot_col..(dimension * 2) {
                augmented[row][col] -= factor * augmented[pivot_col][col];
            }
        }
    }
    let mut inverse = vec![vec![0.0_f64; dimension]; dimension];
    for row in 0..dimension {
        for col in 0..dimension {
            inverse[row][col] = augmented[row][col + dimension];
        }
    }
    matrix.clear();
    Some(inverse)
}

fn compute_pdop(h: &[Vec<f64>]) -> Option<f64> {
    let inv = normal_matrix_inverse(h)?;
    Some(position_covariance_trace(&inv).sqrt())
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
        h.push(vec![dx / range_m, dy / range_m, dz / range_m, 1.0]);
    }
    compute_dops(receiver_ecef_m, &h)
}

fn compute_dops(receiver_ecef_m: [f64; 3], h: &[Vec<f64>]) -> Option<PositionDops> {
    let inv = normal_matrix_inverse(h)?;
    let pdop = position_covariance_trace(&inv).sqrt();
    let tdop = inv[3][3].max(0.0).sqrt();
    let gdop = (pdop.powi(2) + tdop.powi(2)).sqrt();
    let (hdop, vdop) = local_horizontal_vertical_dops(receiver_ecef_m, &inv)?;
    Some(PositionDops { pdop, hdop, vdop, gdop, tdop })
}

fn normal_matrix_inverse(h: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let dimension = h.first()?.len();
    let mut n = vec![vec![0.0_f64; dimension]; dimension];
    for row in h {
        if row.len() != dimension {
            return None;
        }
        for r in 0..dimension {
            for c in 0..dimension {
                n[r][c] += row[r] * row[c];
            }
        }
    }
    invert_matrix(n)
}

fn position_covariance_trace(inv: &[Vec<f64>]) -> f64 {
    (inv[0][0] + inv[1][1] + inv[2][2]).max(0.0)
}

fn local_horizontal_vertical_dops(
    receiver_ecef_m: [f64; 3],
    inv: &[Vec<f64>],
) -> Option<(f64, f64)> {
    let covariance_xyz = [
        [inv[0][0], inv[0][1], inv[0][2]],
        [inv[1][0], inv[1][1], inv[1][2]],
        [inv[2][0], inv[2][1], inv[2][2]],
    ];
    covariance_horizontal_vertical(receiver_ecef_m, covariance_xyz)
}

fn covariance_horizontal_vertical(
    receiver_ecef_m: [f64; 3],
    covariance_xyz: [[f64; 3]; 3],
) -> Option<(f64, f64)> {
    let covariance_enu = ecef_covariance_to_enu(receiver_ecef_m, covariance_xyz)?;
    let horizontal = (covariance_enu[0][0] + covariance_enu[1][1]).max(0.0).sqrt();
    let vertical = covariance_enu[2][2].max(0.0).sqrt();
    Some((horizontal, vertical))
}

fn ecef_covariance_to_enu(
    receiver_ecef_m: [f64; 3],
    covariance_xyz: [[f64; 3]; 3],
) -> Option<[[f64; 3]; 3]> {
    let receiver_radius_m =
        receiver_ecef_m.iter().map(|component| component * component).sum::<f64>().sqrt();
    if !receiver_radius_m.is_finite() || receiver_radius_m <= 0.0 {
        return None;
    }

    let (lat_deg, lon_deg, _alt_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return None;
    }
    let rotation = ecef_to_enu_rotation(lat_deg.to_radians(), lon_deg.to_radians());

    let mut covariance_enu = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            for inner_row in 0..3 {
                for inner_col in 0..3 {
                    covariance_enu[row][col] += rotation[row][inner_row]
                        * covariance_xyz[inner_row][inner_col]
                        * rotation[col][inner_col];
                }
            }
        }
    }
    Some(covariance_enu)
}

fn ecef_to_enu_rotation(lat_rad: f64, lon_rad: f64) -> [[f64; 3]; 3] {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    [
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ]
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
