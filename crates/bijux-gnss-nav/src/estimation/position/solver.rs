#![allow(missing_docs)]

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, covariance_horizontal_vertical, horizontal_error_ellipse,
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
    Constellation, GpsTime, InterSystemBias, Llh, MeasurementRejectReason,
    NavConstellationResidualRms, ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus, SatId,
    Seconds, SigId, SignalBand,
};

mod geodesy;
mod robust_weighting;
mod weighting;
pub use geodesy::{ecef_to_enu, ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef};
#[cfg(test)]
use robust_weighting::robust_weight;
use robust_weighting::robust_weights;
pub use robust_weighting::PositionRobustWeighting;
pub use weighting::{
    position_measurement_weight, weight_from_cn0, weight_from_elevation,
    weight_from_pseudorange_sigma, PositionWeightingModel, WeightingConfig,
};

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

pub fn position_broadcast_navigation_from_galileo_navigations(
    navigations: &[GalileoBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    navigations.iter().cloned().map(PositionBroadcastNavigation::Galileo).collect()
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
        .map(|observation| {
            let covariance_weight =
                weight_from_pseudorange_sigma(observation.covariance_pseudorange_sigma_m());
            PositionObservation {
                sat: observation.signal_id.sat,
                pseudorange_m: observation.pseudorange_m.0,
                doppler_hz: Some(observation.doppler_hz.0),
                doppler_var_hz2: Some(observation.doppler_var_hz2),
                cn0_dbhz: observation.cn0_dbhz,
                elevation_deg: observation.elevation_deg,
                weight: observation.weight.unwrap_or(1.0) * covariance_weight,
                gps_receive_time,
                signal_timing: observation.timing,
                signal_id: Some(observation.signal_id),
            }
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
mod tests {
    use super::{
        constellation_residual_rms, corrected_observation_records,
        position_broadcast_navigation_from_beidou_navigations,
        position_broadcast_navigation_from_galileo_navigations,
        position_broadcast_navigation_from_glonass_frames,
        position_broadcast_navigation_from_gps_ephemerides, position_observations_from_epoch,
        resolve_position_inputs, robust_weight, robust_weights, solve_weighted_least_squares,
        unknown_inter_system_time_offset_sats, ClockStateModel, PositionBroadcastNavigation,
        PositionEstimate, PositionObservation, PositionRobustWeighting, SatelliteGeometry,
        SatelliteState, WorkingSetResidual, SPEED_OF_LIGHT_MPS,
    };
    use crate::estimation::position::navigation::{
        navigation_time_relationship_is_known, PositionObservationCorrectionChain,
        PositionObservationCorrectionKind,
    };
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
    use crate::orbits::satellite_uncertainty::SatelliteStateUncertainty;
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, MeasurementErrorModel, Meters, ObsEpoch,
        ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, TrackingUncertainty,
    };
    use bijux_gnss_signal::api::signal_spec_gps_l1_ca;

    #[test]
    fn weighted_least_squares_recovers_full_rank_solution() {
        let h = vec![
            vec![1.0, 0.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0, 1.0],
            vec![1.0, -1.0, 0.5, 2.0],
        ];
        let expected_delta = [2.0, -1.0, 0.5, 3.0];
        let v = h
            .iter()
            .map(|row| row.iter().zip(expected_delta).map(|(left, right)| left * right).sum())
            .collect::<Vec<_>>();

        let solution = solve_weighted_least_squares(&h, &v, &[1.0; 5])
            .expect("full-rank geometry should solve");

        assert_eq!(solution.rank, 4);
        assert!(solution.condition_number.is_some_and(|condition| condition.is_finite()));
        for (actual, expected) in solution.delta.iter().zip(expected_delta) {
            assert!((actual - expected).abs() < 1.0e-10);
        }
    }

    #[test]
    fn weighted_least_squares_rejects_rank_deficient_geometry() {
        let h = vec![
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.0, 0.0, 1.0],
        ];

        assert!(solve_weighted_least_squares(&h, &[1.0; 4], &[1.0; 4]).is_none());
    }

    #[test]
    fn weighted_least_squares_reports_larger_condition_for_weak_geometry() {
        let healthy = vec![
            vec![1.0, 0.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0, 0.0],
            vec![0.0, 0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0, 1.0],
        ];
        let weak = vec![
            vec![1.0, 0.0, 0.0, 1.0],
            vec![0.0, 1.0, 0.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 3.000_001],
            vec![1.0, -1.0, 0.0, 0.0],
        ];

        let healthy_condition = solve_weighted_least_squares(&healthy, &[0.0; 4], &[1.0; 4])
            .and_then(|solution| solution.condition_number)
            .expect("healthy geometry condition");
        let weak_condition = solve_weighted_least_squares(&weak, &[0.0; 5], &[1.0; 5])
            .and_then(|solution| solution.condition_number)
            .expect("weak geometry condition");

        assert!(weak_condition > healthy_condition);
    }

    fn sample_ephemeris(sat: SatId, toe_s: f64, toc_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat,
            iodc: 0,
            iode: 0,
            week: 1573,
            sv_health: 0,
            sv_accuracy: Some(2),
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
                resolved_day_index: None,
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
    fn galileo_navigation_convert_into_position_navigation_entries() {
        let navigations = vec![
            sample_galileo_navigation(
                SatId { constellation: Constellation::Galileo, prn: 19 },
                504_000.0,
                504_018.0,
            ),
            sample_galileo_navigation(
                SatId { constellation: Constellation::Galileo, prn: 24 },
                504_000.0,
                504_018.0,
            ),
        ];

        let navigation = position_broadcast_navigation_from_galileo_navigations(&navigations);

        assert_eq!(navigation.len(), 2);
        assert_eq!(navigation[0].sat(), navigations[0].sat);
        assert_eq!(navigation[1].sat(), navigations[1].sat);
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
                    uncertainty: SatelliteStateUncertainty::unavailable(),
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
                    uncertainty: SatelliteStateUncertainty::unavailable(),
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
    fn disabled_robust_weighting_preserves_unit_weights() {
        let weights = robust_weights(&[0.0, 12.0, 80.0], PositionRobustWeighting::disabled());

        assert_eq!(weights, vec![1.0, 1.0, 1.0]);
    }

    #[test]
    fn huber_robust_weighting_caps_large_residuals_linearly() {
        let inlier_weight = robust_weight(15.0, PositionRobustWeighting::huber(30.0));
        let outlier_weight = robust_weight(120.0, PositionRobustWeighting::huber(30.0));

        assert_eq!(inlier_weight, 1.0);
        assert!((outlier_weight - 0.25).abs() < 1.0e-12);
    }

    #[test]
    fn tukey_robust_weighting_zeroes_residuals_beyond_cutoff() {
        let inlier_weight = robust_weight(15.0, PositionRobustWeighting::tukey_biweight(30.0));
        let boundary_weight = robust_weight(30.0, PositionRobustWeighting::tukey_biweight(30.0));
        let far_outlier_weight =
            robust_weight(120.0, PositionRobustWeighting::tukey_biweight(30.0));

        assert!(inlier_weight > 0.0 && inlier_weight < 1.0);
        assert_eq!(boundary_weight, 0.0);
        assert_eq!(far_outlier_weight, 0.0);
    }

    #[test]
    fn first_iteration_measurement_weights_start_from_base_weights() {
        let solver = super::PositionSolver::new()
            .with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
        let geometry = vec![
            SatelliteGeometry {
                observation: PositionObservation {
                    sat: SatId { constellation: Constellation::Gps, prn: 1 },
                    pseudorange_m: 24_000_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(45.0),
                    weight: 0.5,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                corrected_pseudorange_m: 24_000_000.0,
                broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                    24_000_000.0,
                ),
                state: SatelliteState {
                    x_m: 20_200_000.0,
                    y_m: -1_500_000.0,
                    z_m: 21_300_000.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                    uncertainty: SatelliteStateUncertainty::unavailable(),
                },
                iono_delay_m: 0.0,
                tropo_delay_m: 0.0,
            },
            SatelliteGeometry {
                observation: PositionObservation {
                    sat: SatId { constellation: Constellation::Gps, prn: 2 },
                    pseudorange_m: 24_100_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(50.0),
                    weight: 2.0,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                corrected_pseudorange_m: 24_100_000.0,
                broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                    24_100_000.0,
                ),
                state: SatelliteState {
                    x_m: 20_300_000.0,
                    y_m: -1_600_000.0,
                    z_m: 21_200_000.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                    uncertainty: SatelliteStateUncertainty::unavailable(),
                },
                iono_delay_m: 0.0,
                tropo_delay_m: 0.0,
            },
        ];

        let weights = solver.measurement_weights(0, &geometry, &[1.0e6, 1.0e6]);

        assert_eq!(weights, vec![0.5, 2.0]);
    }

    #[test]
    fn later_iteration_measurement_weights_fall_back_when_tukey_zeroes_everything() {
        let solver = super::PositionSolver::new()
            .with_robust_weighting(PositionRobustWeighting::tukey_biweight(30.0));
        let geometry = vec![
            SatelliteGeometry {
                observation: PositionObservation {
                    sat: SatId { constellation: Constellation::Gps, prn: 1 },
                    pseudorange_m: 24_000_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(45.0),
                    weight: 0.5,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                corrected_pseudorange_m: 24_000_000.0,
                broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                    24_000_000.0,
                ),
                state: SatelliteState {
                    x_m: 20_200_000.0,
                    y_m: -1_500_000.0,
                    z_m: 21_300_000.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                    uncertainty: SatelliteStateUncertainty::unavailable(),
                },
                iono_delay_m: 0.0,
                tropo_delay_m: 0.0,
            },
            SatelliteGeometry {
                observation: PositionObservation {
                    sat: SatId { constellation: Constellation::Gps, prn: 2 },
                    pseudorange_m: 24_100_000.0,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(50.0),
                    weight: 2.0,
                    gps_receive_time: None,
                    signal_timing: None,
                    signal_id: None,
                },
                corrected_pseudorange_m: 24_100_000.0,
                broadcast_group_delay_correction_chain: PositionObservationCorrectionChain::new(
                    24_100_000.0,
                ),
                state: SatelliteState {
                    x_m: 20_300_000.0,
                    y_m: -1_600_000.0,
                    z_m: 21_200_000.0,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                    uncertainty: SatelliteStateUncertainty::unavailable(),
                },
                iono_delay_m: 0.0,
                tropo_delay_m: 0.0,
            },
        ];

        let weights = solver.measurement_weights(1, &geometry, &[1.0e6, 1.0e6]);

        assert_eq!(weights, vec![0.5, 2.0]);
    }

    #[test]
    fn corrected_observation_records_reconstruct_solver_residual() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };
        let mut group_delay_chain = PositionObservationCorrectionChain::new(20_000_010.0);
        group_delay_chain
            .push_component(PositionObservationCorrectionKind::BroadcastGroupDelay, -3.0);
        let geometry = vec![SatelliteGeometry {
            observation: PositionObservation {
                sat,
                pseudorange_m: 20_000_010.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            corrected_pseudorange_m: 20_000_007.0,
            broadcast_group_delay_correction_chain: group_delay_chain,
            state: SatelliteState {
                x_m: 20_000_000.0,
                y_m: 0.0,
                z_m: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 2.0e-6,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            iono_delay_m: 5.0,
            tropo_delay_m: 2.0,
        }];
        let estimate = PositionEstimate {
            ecef_x_m: 0.0,
            ecef_y_m: 0.0,
            ecef_z_m: 0.0,
            clock_model: ClockStateModel::from_constellations([Constellation::Gps])
                .expect("clock model"),
            clock_state_s: vec![1.0e-6],
        };
        let expected_residual_m = 20_000_010.0 + 2.0e-6 * SPEED_OF_LIGHT_MPS
            - 3.0
            - 5.0
            - 2.0
            - 1.0e-6 * SPEED_OF_LIGHT_MPS
            - 20_000_000.0;
        let residuals = vec![WorkingSetResidual {
            sat,
            residual_m: expected_residual_m,
            base_weight: 1.0,
            effective_weight: 1.0,
        }];

        let records = corrected_observation_records(&estimate, &geometry, &residuals)
            .expect("corrected observation records");
        let record = records.first().expect("recorded observation");

        assert_eq!(record.correction_chain.components.len(), 11);
        assert_eq!(record.reconstruction_error_m(), 0.0);
        assert!((record.reconstructed_residual_m() - expected_residual_m).abs() < 1.0e-9);
        assert_eq!(record.residual_m, expected_residual_m);
        assert!(
            !record
                .correction_chain
                .components
                .iter()
                .find(|component| component.kind == PositionObservationCorrectionKind::Relativity)
                .expect("relativity component")
                .applied
        );
        assert!(
            record
                .correction_chain
                .components
                .iter()
                .find(|component| {
                    component.kind == PositionObservationCorrectionKind::BroadcastGroupDelay
                })
                .expect("group delay component")
                .applied
        );
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

    #[test]
    fn position_observations_from_epoch_weight_with_observation_covariance() {
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
            sats: vec![ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: Meters(22_000_000.0),
                pseudorange_var_m2: 16.0,
                carrier_phase_cycles: Cycles(20.0),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 4.0,
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
                weight: Some(2.0),
                timing: None,
                error_model: Some(MeasurementErrorModel {
                    thermal_noise_m: Meters(0.0),
                    tracking_jitter_m: Meters(4.0),
                    multipath_proxy_m: Meters(0.0),
                    clock_error_m: Meters(0.0),
                }),
                metadata: ObsMetadata {
                    signal: signal_spec_gps_l1_ca(),
                    tracking_uncertainty: Some(TrackingUncertainty {
                        code_phase_samples: 0.01,
                        carrier_phase_cycles: 0.1,
                        doppler_hz: 2.0,
                        cn0_dbhz: 0.5,
                    }),
                    ..ObsMetadata::default()
                },
            }],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: None,
            manifest: None,
        };

        let observations = position_observations_from_epoch(&epoch);

        assert_eq!(observations.len(), 1);
        assert!((observations[0].weight - 0.125).abs() < 1.0e-12);
    }
}

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
mod broadcast_group_delay_tests {
    use super::{PositionBroadcastNavigation, PositionObservation, PositionSolver};
    use crate::corrections::broadcast_group_delay::gps_broadcast_group_delay_code_bias_m;
    use crate::estimation::position::navigation::corrected_pseudorange_m;
    use crate::orbits::gps::GpsEphemeris;
    use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};

    fn sample_gps_ephemeris_with_tgd(prn: u8, tgd_s: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn },
            iodc: 1,
            iode: 1,
            week: 2209,
            sv_health: 0,
            sv_accuracy: Some(2),
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
    fn position_solver_applies_broadcast_ionosphere_by_default() {
        assert!(PositionSolver::new().apply_broadcast_ionosphere);
    }

    #[test]
    fn position_solver_can_disable_broadcast_ionosphere() {
        assert!(!PositionSolver::new().with_broadcast_ionosphere(false).apply_broadcast_ionosphere);
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
mod replay_timing_tests {
    use super::{
        detect_replay_timing_anomaly, PositionObservation, SatelliteState, SPEED_OF_LIGHT_MPS,
    };
    use crate::orbits::satellite_uncertainty::SatelliteStateUncertainty;
    use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};

    #[test]
    fn detects_centered_excess_delay_pattern() {
        let filtered = filtered_timing_residuals(&[
            ("G03", 105.0, true),
            ("G07", 48.0, true),
            ("G11", -44.0, true),
            ("G19", -109.0, true),
            ("G23", 92.0, true),
            ("G29", -95.0, true),
        ]);

        let anomaly = detect_replay_timing_anomaly(&filtered)
            .expect("replay-like excess delays must surface");

        assert_eq!(anomaly.matched_satellite_count, 6);
        assert!(anomaly.centered_delay_rms_m >= anomaly.centered_delay_rms_threshold_m);
        assert!(anomaly.max_centered_delay_m >= anomaly.max_centered_delay_threshold_m);
        assert!(anomaly.median_excess_delay_m.abs() < 10.0);
    }

    #[test]
    fn ignores_uniform_delay_shift() {
        let filtered = filtered_timing_residuals(&[
            ("G03", 85.0, true),
            ("G07", 84.0, true),
            ("G11", 86.0, true),
            ("G19", 83.0, true),
            ("G23", 87.0, true),
            ("G29", 84.5, true),
        ]);

        assert!(detect_replay_timing_anomaly(&filtered).is_none());
    }

    #[test]
    fn requires_timing_tagged_satellites() {
        let filtered = filtered_timing_residuals(&[
            ("G03", 102.0, true),
            ("G07", 44.0, true),
            ("G11", -46.0, true),
            ("G19", -100.0, false),
            ("G23", 96.0, false),
            ("G29", -96.0, false),
        ]);

        assert!(detect_replay_timing_anomaly(&filtered).is_none());
    }

    fn filtered_timing_residuals(
        entries: &[(&str, f64, bool)],
    ) -> Vec<(PositionObservation, SatelliteState, f64, f64)> {
        entries
            .iter()
            .enumerate()
            .map(|(index, (satellite, residual_m, has_timing))| {
                let sat = parse_satellite_id(satellite);
                let observation = PositionObservation {
                    sat,
                    pseudorange_m: 24_000_000.0 + index as f64,
                    doppler_hz: None,
                    doppler_var_hz2: None,
                    cn0_dbhz: 45.0,
                    elevation_deg: Some(45.0),
                    weight: 1.0,
                    gps_receive_time: None,
                    signal_timing: has_timing.then_some(ObsSignalTiming {
                        signal_travel_time_s: Seconds(24_000_000.0 / SPEED_OF_LIGHT_MPS),
                        transmit_gps_time: GpsTime { week: 0, tow_s: 100_000.0 },
                    }),
                    signal_id: None,
                };
                let state = SatelliteState {
                    x_m: 20_000_000.0 + index as f64,
                    y_m: 21_000_000.0 + index as f64,
                    z_m: 22_000_000.0 + index as f64,
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    vz_mps: 0.0,
                    clock_bias_s: 0.0,
                    clock_drift_s_per_s: 0.0,
                    uncertainty: SatelliteStateUncertainty::unavailable(),
                };
                (observation, state, *residual_m, 1.0)
            })
            .collect()
    }

    fn parse_satellite_id(spec: &str) -> SatId {
        let prn = spec[1..].parse::<u8>().expect("two-digit PRN");
        let constellation = match &spec[..1] {
            "G" => Constellation::Gps,
            "E" => Constellation::Galileo,
            "C" => Constellation::Beidou,
            "R" => Constellation::Glonass,
            other => panic!("unexpected constellation designator: {other}"),
        };
        SatId { constellation, prn }
    }
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

#[derive(Debug, Clone)]
struct WeightedLeastSquaresSolution {
    delta: Vec<f64>,
    covariance: Vec<Vec<f64>>,
    rank: usize,
    condition_number: Option<f64>,
}

#[derive(Debug, Clone)]
struct PivotedQr {
    q_columns: Vec<Vec<f64>>,
    r: Vec<Vec<f64>>,
    pivots: Vec<usize>,
    rank: usize,
    condition_number: Option<f64>,
}

fn solve_weighted_least_squares(
    h: &[Vec<f64>],
    v: &[f64],
    w: &[f64],
) -> Option<WeightedLeastSquaresSolution> {
    let dimension = h.first()?.len();
    if h.len() != v.len() || h.len() < dimension {
        return None;
    }
    let mut weighted_residuals = vec![0.0_f64; h.len()];
    for row_index in 0..h.len() {
        let weight = w.get(row_index).copied().unwrap_or(1.0);
        if !weight.is_finite() || weight < 0.0 || !v[row_index].is_finite() {
            return None;
        }
        weighted_residuals[row_index] = v[row_index] * weight.sqrt();
    }

    let decomposition = decompose_weighted_design(h, w)?;
    if decomposition.rank < dimension {
        return None;
    }
    let pivoted_delta = solve_qr_coordinates(&decomposition, &weighted_residuals)?;
    let delta = unpivot_vector(&pivoted_delta, &decomposition.pivots);
    let covariance = covariance_from_upper_triangular(&decomposition.r, &decomposition.pivots)?;

    Some(WeightedLeastSquaresSolution {
        delta,
        covariance,
        rank: decomposition.rank,
        condition_number: decomposition.condition_number,
    })
}

fn decompose_weighted_design(h: &[Vec<f64>], w: &[f64]) -> Option<PivotedQr> {
    let dimension = h.first()?.len();
    if h.len() < dimension {
        return None;
    }
    let mut weighted_design = vec![vec![0.0_f64; dimension]; h.len()];
    for (row_index, row) in h.iter().enumerate() {
        if row.len() != dimension {
            return None;
        }
        let weight = w.get(row_index).copied().unwrap_or(1.0);
        if !weight.is_finite() || weight < 0.0 {
            return None;
        }
        let scale = weight.sqrt();
        for col in 0..dimension {
            if !row[col].is_finite() {
                return None;
            }
            weighted_design[row_index][col] = row[col] * scale;
        }
    }
    pivoted_qr(&weighted_design)
}

fn pivoted_qr(a: &[Vec<f64>]) -> Option<PivotedQr> {
    let row_count = a.len();
    let dimension = a.first()?.len();
    if dimension == 0 || a.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut columns = vec![vec![0.0_f64; row_count]; dimension];
    for (row_index, row) in a.iter().enumerate() {
        for col in 0..dimension {
            columns[col][row_index] = row[col];
        }
    }
    let mut pivots = (0..dimension).collect::<Vec<_>>();
    let mut q_columns = vec![vec![0.0_f64; row_count]; dimension];
    let mut r = vec![vec![0.0_f64; dimension]; dimension];
    let mut column_norms = columns.iter().map(|column| dot(column, column)).collect::<Vec<_>>();
    let max_initial_norm = column_norms.iter().copied().fold(0.0_f64, f64::max).sqrt();
    if !max_initial_norm.is_finite() || max_initial_norm <= 0.0 {
        return None;
    }
    let rank_tolerance = (row_count.max(dimension) as f64) * max_initial_norm * 1.0e-12;
    let mut rank = 0;
    let mut max_diagonal = 0.0_f64;
    let mut min_diagonal = f64::INFINITY;

    for col in 0..dimension {
        let pivot_col = (col..dimension).max_by(|left, right| {
            column_norms[*left]
                .partial_cmp(&column_norms[*right])
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if pivot_col != col {
            columns.swap(col, pivot_col);
            column_norms.swap(col, pivot_col);
            pivots.swap(col, pivot_col);
            for row in r.iter_mut().take(col) {
                row.swap(col, pivot_col);
            }
        }

        let mut vector = columns[col].clone();
        for previous_col in 0..col {
            let projection = dot(&q_columns[previous_col], &vector);
            r[previous_col][col] += projection;
            axpy(&mut vector, &q_columns[previous_col], -projection);
        }
        for previous_col in 0..col {
            let projection = dot(&q_columns[previous_col], &vector);
            r[previous_col][col] += projection;
            axpy(&mut vector, &q_columns[previous_col], -projection);
        }
        let diagonal = dot(&vector, &vector).sqrt();
        r[col][col] = diagonal;
        if diagonal <= rank_tolerance || !diagonal.is_finite() {
            break;
        }
        max_diagonal = max_diagonal.max(diagonal);
        min_diagonal = min_diagonal.min(diagonal);
        for row in 0..row_count {
            q_columns[col][row] = vector[row] / diagonal;
        }
        rank += 1;
        for trailing_col in (col + 1)..dimension {
            column_norms[trailing_col] = dot(&columns[trailing_col], &columns[trailing_col]);
        }
    }

    let condition_number = (rank == dimension && min_diagonal.is_finite() && min_diagonal > 0.0)
        .then_some(max_diagonal / min_diagonal);
    Some(PivotedQr { q_columns, r, pivots, rank, condition_number })
}

fn solve_qr_coordinates(qr: &PivotedQr, b: &[f64]) -> Option<Vec<f64>> {
    let dimension = qr.r.len();
    if qr.rank < dimension || qr.q_columns.iter().any(|column| column.len() != b.len()) {
        return None;
    }
    let mut qtb = vec![0.0_f64; dimension];
    for col in 0..dimension {
        qtb[col] = dot(&qr.q_columns[col], b);
    }
    solve_upper_triangular(&qr.r, &qtb)
}

fn solve_upper_triangular(r: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let dimension = r.len();
    if dimension == 0 || b.len() != dimension || r.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut x = vec![0.0_f64; dimension];
    for row in (0..dimension).rev() {
        let diagonal = r[row][row];
        if !diagonal.is_finite() || diagonal.abs() <= 1.0e-12 {
            return None;
        }
        let mut sum = b[row];
        for col in (row + 1)..dimension {
            sum -= r[row][col] * x[col];
        }
        x[row] = sum / diagonal;
    }
    Some(x)
}

fn covariance_from_upper_triangular(r: &[Vec<f64>], pivots: &[usize]) -> Option<Vec<Vec<f64>>> {
    let dimension = r.len();
    if pivots.len() != dimension || r.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut inverse_r = vec![vec![0.0_f64; dimension]; dimension];
    for basis_col in 0..dimension {
        let mut basis = vec![0.0_f64; dimension];
        basis[basis_col] = 1.0;
        let solution = solve_upper_triangular(r, &basis)?;
        for row in 0..dimension {
            inverse_r[row][basis_col] = solution[row];
        }
    }
    let mut covariance_pivoted = vec![vec![0.0_f64; dimension]; dimension];
    for row in 0..dimension {
        for col in 0..dimension {
            covariance_pivoted[row][col] =
                (0..dimension).map(|k| inverse_r[row][k] * inverse_r[col][k]).sum();
        }
    }
    let mut covariance = vec![vec![0.0_f64; dimension]; dimension];
    for pivoted_row in 0..dimension {
        for pivoted_col in 0..dimension {
            covariance[pivots[pivoted_row]][pivots[pivoted_col]] =
                covariance_pivoted[pivoted_row][pivoted_col];
        }
    }
    Some(covariance)
}

fn unpivot_vector(vector: &[f64], pivots: &[usize]) -> Vec<f64> {
    let mut out = vec![0.0_f64; vector.len()];
    for (pivoted_index, original_index) in pivots.iter().copied().enumerate() {
        out[original_index] = vector[pivoted_index];
    }
    out
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter().zip(right).map(|(left, right)| left * right).sum()
}

fn axpy(out: &mut [f64], x: &[f64], alpha: f64) {
    for (out, x) in out.iter_mut().zip(x) {
        *out += alpha * x;
    }
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
    let unit_weights = vec![1.0; h.len()];
    let decomposition = decompose_weighted_design(h, &unit_weights)?;
    if decomposition.rank < h.first()?.len() {
        return None;
    }
    let covariance = covariance_from_upper_triangular(&decomposition.r, &decomposition.pivots)?;
    let pdop = position_covariance_trace(&covariance).sqrt();
    let tdop = covariance[3][3].max(0.0).sqrt();
    let gdop = (pdop.powi(2) + tdop.powi(2)).sqrt();
    let (hdop, vdop) = local_horizontal_vertical_dops(receiver_ecef_m, &covariance)?;
    Some(PositionDops { pdop, hdop, vdop, gdop, tdop })
}

fn position_covariance_trace(inv: &[Vec<f64>]) -> f64 {
    (inv[0][0] + inv[1][1] + inv[2][2]).max(0.0)
}

fn scaled_position_covariance_ecef_m2(
    covariance: &[Vec<f64>],
    sigma2: f64,
) -> Option<[[f64; 3]; 3]> {
    if covariance.len() < 3 || covariance.iter().take(3).any(|row| row.len() < 3) {
        return None;
    }
    let mut position_covariance = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            let value = covariance[row][col] * sigma2;
            if !value.is_finite() {
                return None;
            }
            position_covariance[row][col] = value;
        }
    }
    Some(position_covariance)
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
