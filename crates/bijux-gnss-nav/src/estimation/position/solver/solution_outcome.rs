use serde::{Deserialize, Serialize};

use crate::estimation::position::navigation::PositionObservationCorrectionChain;
use crate::estimation::position::raim::{
    RaimFaultDetection, RaimFaultExclusion, RaimSolutionSeparationCheck,
};
use bijux_gnss_core::api::{
    Constellation, InterSystemBias, MeasurementRejectReason, NavConstellationResidualRms, SatId,
    SigId,
};

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
    pub rejected: Vec<(SatId, MeasurementRejectReason)>,
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
