//! Validation report builders for GNSS runs.

use crate::api::ReceiverConfig;
use crate::api::TrackingResult;
use crate::pipeline::observation_validation::{
    summarize_carrier_smoothed_code, validate_carrier_smoothed_code_from_artifacts,
    CarrierSmoothedCodeValidationReport,
};
use crate::pipeline::observations::ObservationPipelineArtifacts;
use crate::reference_validation::{
    check_solution_consistency, reference_ecef, SolutionConsistencyReport, ValidationReferenceEpoch,
};
use crate::validation_helpers::{check_budgets, to_validation_stats};
use bijux_gnss_core::api::{stats, NavSolutionEpoch, ObsEpoch, SatId, SignalBand, SolutionStatus};
use bijux_gnss_nav::api::{
    apply_downgrade_policy, evaluate_prerequisites, support_status_matrix, AdvancedMaturity,
    AdvancedMode, AdvancedPrerequisites, AdvancedRefusalClass, AdvancedSolutionClaim,
    AdvancedSupportRow,
};
use bijux_gnss_nav::api::{
    ecef_to_enu, geometry_free_diagnostics_from_obs_epochs, iono_free_code_from_obs_epochs,
    melbourne_wubbena_diagnostics_from_obs_epochs, GeometryFreeEvent, GeometryFreeThresholds,
    MelbourneWubbenaEvent, MelbourneWubbenaThresholds, PositionWeightingModel, PppConfig,
    PppConvergenceConfig, PppProcessNoise, WeightingConfig,
};
use bijux_gnss_signal::api::{
    check_dual_frequency_observations, check_inter_frequency_alignment,
    supported_dual_frequency_band_pairs, supported_dual_frequency_band_pairs_for_constellation,
    DualFrequencyObservationReport, DualFrequencyPairStatus, InterFrequencyAlignmentReport,
};
use serde::{Deserialize, Serialize};

/// Validation error summary statistics.
#[derive(Debug, Serialize)]
pub struct ValidationErrorStats {
    /// Sample count.
    pub count: usize,
    /// Mean error value.
    pub mean: f64,
    /// Median error value.
    pub median: f64,
    /// RMS error value.
    pub rms: f64,
    /// 95th percentile error value.
    pub p95: f64,
    /// Max error value.
    pub max: f64,
}

/// Convergence report for validation timelines.
#[derive(Debug, Serialize)]
pub struct ConvergenceReport {
    /// Time to reach 1m horizontal error.
    pub time_to_1m_s: Option<f64>,
    /// Time to reach 0.3m horizontal error.
    pub time_to_0_3m_s: Option<f64>,
    /// Time to reach 0.1m horizontal error.
    pub time_to_0_1m_s: Option<f64>,
    /// Worst horizontal error.
    pub worst_horiz_m: f64,
    /// Worst vertical error.
    pub worst_vert_m: f64,
}

/// Fix timeline entry for validation report.
#[derive(Debug, Serialize)]
pub struct FixTimelineEntry {
    /// Epoch index.
    pub epoch_idx: u64,
    /// Whether the solution is fixed.
    pub fixed: bool,
}

/// PPP readiness summary.
#[derive(Debug, Serialize)]
pub struct PppReadinessReport {
    /// Whether multiple frequencies are present.
    pub multi_freq_present: bool,
    /// Whether linear combinations are valid.
    pub combinations_valid: bool,
    /// Whether precise products were available.
    pub products_ok: bool,
    /// Fallback descriptions when precise products are missing.
    pub product_fallbacks: Vec<String>,
    /// Declared maturity state for PPP behavior.
    pub maturity: AdvancedMaturity,
    /// Whether explicit execution prerequisites were met.
    pub prerequisites_met: bool,
    /// Refusal class when prerequisites were not met.
    pub refusal_class: Option<AdvancedRefusalClass>,
    /// Runtime status for PPP execution in this surface.
    pub status: String,
    /// Runtime status reason when present.
    pub status_reason: Option<String>,
    /// Reported claim class for PPP output.
    pub claim: AdvancedSolutionClaim,
    /// Support matrix row for PPP.
    pub support: AdvancedSupportRow,
}

/// Geometry-free behavior summary derived from dual-frequency observation pairs.
#[derive(Debug, Serialize)]
pub struct GeometryFreeReport {
    /// Total geometry-free observations evaluated across supported band pairs.
    pub observations: usize,
    /// Observations with complete, valid dual-frequency combinations.
    pub complete_pairs: usize,
    /// Observations unavailable because the dual-frequency pair was incomplete or invalid.
    pub unavailable: usize,
    /// Valid observations that started a new per-satellite geometry-free arc.
    pub insufficient_history: usize,
    /// Valid observations whose geometry-free delta stayed below event thresholds.
    pub nominal: usize,
    /// Valid observations whose geometry-free delta reflected dispersive ionosphere drift.
    pub ionosphere_drift: usize,
    /// Valid observations whose geometry-free delta reflected a likely cycle slip.
    pub cycle_slip_suspects: usize,
    /// Largest absolute geometry-free delta observed on a continuous arc.
    pub max_abs_delta_m: Option<f64>,
}

/// Melbourne-Wubbena behavior summary derived from dual-frequency observation pairs.
#[derive(Debug, Serialize)]
pub struct MelbourneWubbenaReport {
    /// Total Melbourne-Wubbena observations evaluated across supported band pairs.
    pub observations: usize,
    /// Observations with complete, valid dual-frequency combinations.
    pub complete_pairs: usize,
    /// Observations unavailable because the dual-frequency pair was incomplete or invalid.
    pub unavailable: usize,
    /// Valid observations that started a new per-satellite Melbourne-Wubbena arc.
    pub insufficient_history: usize,
    /// Valid observations whose wide-lane delta stayed below the slip threshold.
    pub nominal: usize,
    /// Valid observations whose wide-lane delta reflected a likely slip.
    pub wide_lane_slip_suspects: usize,
    /// Largest absolute Melbourne-Wubbena delta observed in wide-lane cycles.
    pub max_abs_delta_wide_lane_cycles: Option<f64>,
}

/// Time consistency report for tracking epochs.
#[derive(Debug, Serialize)]
pub struct TimeConsistencyReport {
    /// Number of channels inspected.
    pub channels: usize,
    /// Number of epochs checked.
    pub epochs_checked: usize,
    /// Epochs that went backwards.
    pub epoch_backward: usize,
    /// Epoch gaps detected.
    pub epoch_gaps: usize,
    /// Sample index went backwards.
    pub sample_backward: usize,
    /// Sample cadence mismatch count.
    pub sample_step_mismatch: usize,
    /// Expected sample step based on configuration.
    pub expected_step: Option<u64>,
    /// Observed sample step mean (if inferred).
    pub observed_step_mean: Option<f64>,
    /// Warning messages.
    pub warnings: Vec<String>,
}

/// Validation budgets for various metrics.
#[derive(Debug, Serialize, Clone)]
pub struct ValidationBudgets {
    /// Acquisition Doppler tolerance (Hz).
    pub acq_doppler_hz: f64,
    /// Acquisition code phase tolerance (samples).
    pub acq_code_phase_samples: f64,
    /// Tracking carrier jitter tolerance (Hz).
    pub tracking_carrier_jitter_hz: f64,
    /// Ephemeris parity rate minimum.
    pub ephemeris_parity_rate_min: f64,
    /// Max PVT iterations.
    pub pvt_max_iterations: usize,
    /// Max navigation residual RMS (m).
    pub nav_residual_rms_m_max: f64,
    /// Max rejected measurement ratio.
    pub nav_rejected_ratio_max: f64,
    /// Max allowed NaN count in navigation outputs.
    pub nav_nan_max: usize,
    /// Minimum lock epochs before nav update.
    pub nav_min_lock_epochs: u64,
    /// Maximum allowed 3D reference-position error (m), when a reference is available.
    pub reference_position_error_3d_m_max: Option<f64>,
}

/// Scientific policy used for validation and integrity classification.
#[derive(Debug, Serialize, Clone)]
pub struct ValidationSciencePolicy {
    /// Minimum mean C/N0 expected for stable acceptance.
    pub min_mean_cn0_dbhz: f64,
    /// Maximum PDOP for stable geometry.
    pub max_pdop: f64,
    /// Maximum GDOP for stable geometry.
    pub max_gdop: f64,
    /// Maximum residual RMS (meters) for stable quality.
    pub max_residual_rms_m: f64,
    /// Minimum used satellites expected for stable geometry.
    pub min_used_satellites: usize,
    /// Minimum lock ratio for stable tracking support.
    pub min_lock_ratio: f64,
}

impl Default for ValidationSciencePolicy {
    fn default() -> Self {
        Self {
            min_mean_cn0_dbhz: 28.0,
            max_pdop: 8.0,
            max_gdop: 12.0,
            max_residual_rms_m: 25.0,
            min_used_satellites: 4,
            min_lock_ratio: 0.7,
        }
    }
}

/// Integrity class for a navigation epoch.
#[derive(Debug, Serialize, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum NavIntegrityClass {
    /// No integrity red flags were detected.
    Nominal,
    /// The solution did not provide the protection-level evidence required for integrity claims.
    IntegrityEvidenceMissing,
    /// Geometry-related checks were weak (PDOP or used satellites).
    WeakGeometry,
    /// Residual-related checks were suspicious.
    SuspiciousResiduals,
    /// Tracking lock support was unstable.
    UnstableLock,
    /// Multiple integrity concerns were present simultaneously.
    Compound,
}

/// Integrity report entry for one navigation epoch.
#[derive(Debug, Serialize)]
pub struct NavIntegrityReport {
    /// Epoch index.
    pub epoch_idx: u64,
    /// Classified integrity state.
    pub class: NavIntegrityClass,
    /// Reasons behind classification.
    pub reasons: Vec<String>,
}

/// Summary of emitted protection levels against measured reference-position error.
#[derive(Debug, Serialize)]
pub struct ProtectionLevelValidationReport {
    /// Reference-matched epochs considered in validation.
    pub matched_epoch_count: usize,
    /// Matched epochs that reported horizontal protection levels.
    pub horizontal_reported_epoch_count: usize,
    /// Matched epochs that reported vertical protection levels.
    pub vertical_reported_epoch_count: usize,
    /// Matched epochs whose horizontal error stayed within the reported HPL.
    pub horizontal_contained_epoch_count: usize,
    /// Matched epochs whose vertical error stayed within the reported VPL.
    pub vertical_contained_epoch_count: usize,
    /// Epochs whose measured horizontal error exceeded the reported HPL.
    pub horizontal_breach_epochs: Vec<u64>,
    /// Epochs whose measured vertical error exceeded the reported VPL.
    pub vertical_breach_epochs: Vec<u64>,
    /// Smallest horizontal margin `HPL - horizontal_error`, when reported.
    pub min_horizontal_margin_m: Option<f64>,
    /// Smallest vertical margin `VPL - vertical_error`, when reported.
    pub min_vertical_margin_m: Option<f64>,
}

/// Partition of advisory diagnostics versus enforced refusal outcomes.
#[derive(Debug, Serialize)]
pub struct DiagnosticPartitionReport {
    /// Advisory diagnostics that do not directly enforce refusal.
    pub advisory_diagnostics: Vec<String>,
    /// Enforced refusal outcomes recorded in solutions.
    pub enforced_refusals: Vec<String>,
}

/// Assumption summary across validation outputs.
#[derive(Debug, Serialize)]
pub struct ValidationAssumptionReport {
    /// Time systems used in solutions.
    pub time_systems: Vec<String>,
    /// Reference frames used in solutions.
    pub reference_frames: Vec<String>,
    /// Clock models used in solutions.
    pub clock_models: Vec<String>,
}

/// Navigation residual report per epoch.
#[derive(Debug, Serialize)]
pub struct NavConstellationResidualReport {
    /// Constellation represented by the summary.
    pub constellation: String,
    /// Pre-fit residual RMS for that constellation (m).
    pub pre_fit_rms_m: Option<f64>,
    /// Post-fit residual RMS for that constellation (m).
    pub post_fit_rms_m: Option<f64>,
    /// Number of satellites contributing to the pre-fit RMS.
    pub pre_fit_sat_count: usize,
    /// Number of satellites contributing to the post-fit RMS.
    pub post_fit_sat_count: usize,
}

/// Navigation residual report per epoch.
#[derive(Debug, Serialize)]
pub struct NavResidualReport {
    /// Epoch index.
    pub epoch_idx: u64,
    /// Post-fit RMS residual compatibility field (m).
    pub rms_m: f64,
    /// Pre-fit RMS residual (m), when the solution reports it.
    pub pre_fit_rms_m: Option<f64>,
    /// Post-fit RMS residual (m), when the solution reports it.
    pub post_fit_rms_m: Option<f64>,
    /// PDOP value.
    pub pdop: f64,
    /// Per-satellite residuals.
    pub residuals: Vec<(SatId, f64)>,
    /// Per-constellation residual RMS summaries.
    pub constellation_residual_rms: Vec<NavConstellationResidualReport>,
    /// Rejected satellites.
    pub rejected: Vec<SatId>,
}

/// Reference-position error components for one navigation epoch.
#[derive(Debug, Serialize)]
pub struct ReferencePositionErrorEpoch {
    /// Epoch index.
    pub epoch_idx: u64,
    /// East error (m).
    pub east_m: f64,
    /// North error (m).
    pub north_m: f64,
    /// Up error (m).
    pub up_m: f64,
    /// Horizontal error magnitude (m).
    pub horiz_m: f64,
    /// Vertical error magnitude (m).
    pub vert_m: f64,
    /// 3D position error magnitude (m).
    pub error_3d_m: f64,
    /// Horizontal protection level reported for the epoch (m), when available.
    pub hpl_m: Option<f64>,
    /// Vertical protection level reported for the epoch (m), when available.
    pub vpl_m: Option<f64>,
    /// Whether the measured horizontal error stayed within the reported HPL.
    pub horizontal_within_hpl: Option<bool>,
    /// Whether the measured vertical error stayed within the reported VPL.
    pub vertical_within_vpl: Option<bool>,
    /// Horizontal protection margin `HPL - horizontal_error` (m), when HPL is available.
    pub horizontal_margin_m: Option<f64>,
    /// Vertical protection margin `VPL - vertical_error` (m), when VPL is available.
    pub vertical_margin_m: Option<f64>,
}

/// Coverage summary for trusted reference coordinates used in position validation.
#[derive(Debug, Serialize)]
pub struct ReferenceCoordinateCoverageReport {
    /// Whether the current validation budgets require trusted reference coordinates.
    pub required: bool,
    /// Whether every solved navigation epoch had a matching trusted reference coordinate.
    pub ready: bool,
    /// Number of solved navigation epochs that matched trusted reference coordinates.
    pub matched_solution_epoch_count: usize,
    /// Solved navigation epochs that lacked trusted reference coordinates.
    pub unmatched_solution_epochs: Vec<u64>,
    /// Trusted reference epochs that were provided but not consumed by any solved navigation epoch.
    pub unused_reference_epochs: Vec<u64>,
}

/// Validation report output structure.
#[derive(Debug, Serialize)]
pub struct ValidationReport {
    /// Total samples counted.
    pub samples: usize,
    /// Number of epochs.
    pub epochs: usize,
    /// East error statistics.
    pub east_error_m: ValidationErrorStats,
    /// North error statistics.
    pub north_error_m: ValidationErrorStats,
    /// Up error statistics.
    pub up_error_m: ValidationErrorStats,
    /// Horizontal error statistics.
    pub horiz_error_m: ValidationErrorStats,
    /// Vertical error statistics.
    pub vert_error_m: ValidationErrorStats,
    /// 3D position error statistics.
    pub error_3d_m: ValidationErrorStats,
    /// Convergence report.
    pub convergence: ConvergenceReport,
    /// Fix timeline.
    pub fix_timeline: Vec<FixTimelineEntry>,
    /// Reference-position error components for matched epochs.
    pub reference_position_errors: Vec<ReferencePositionErrorEpoch>,
    /// Protection-level validation summary against measured reference-position error.
    pub protection_levels: ProtectionLevelValidationReport,
    /// Coverage status for trusted reference coordinates used by position validation.
    pub reference_coordinate_coverage: ReferenceCoordinateCoverageReport,
    /// Residuals per epoch.
    pub residuals: Vec<NavResidualReport>,
    /// Time consistency report.
    pub time_consistency: TimeConsistencyReport,
    /// Solution consistency report.
    pub consistency: SolutionConsistencyReport,
    /// Validation budgets.
    pub budgets: ValidationBudgets,
    /// Budget violations.
    pub budget_violations: Vec<String>,
    /// NIS mean value.
    pub nis_mean: Option<f64>,
    /// NEES mean value.
    pub nees_mean: Option<f64>,
    /// Empirical covariance realism summary against trusted reference coordinates.
    pub covariance_realism: crate::covariance_realism::CovarianceRealismReport,
    /// Consistency warnings.
    pub consistency_warnings: Vec<String>,
    /// Inter-frequency alignment report.
    pub inter_frequency_alignment: InterFrequencyAlignmentReport,
    /// Dual-frequency observation support report.
    pub dual_frequency_observations: DualFrequencyObservationReport,
    /// Geometry-free behavior summary for supported dual-frequency pairs.
    pub geometry_free: GeometryFreeReport,
    /// Melbourne-Wubbena behavior summary for supported dual-frequency pairs.
    pub melbourne_wubbena: MelbourneWubbenaReport,
    /// Carrier-smoothed code behavior summary for Hatch-smoothed pseudorange arcs.
    pub carrier_smoothed_code: CarrierSmoothedCodeValidationReport,
    /// PPP readiness report.
    pub ppp_readiness: PppReadinessReport,
    /// Scientific policy used when classifying outputs.
    pub science_policy: ValidationSciencePolicy,
    /// Integrity classifications per navigation epoch.
    pub integrity: Vec<NavIntegrityReport>,
    /// Partition between advisory diagnostics and enforced refusal outcomes.
    pub diagnostic_partition: DiagnosticPartitionReport,
    /// Assumption summary for time, frame, and clock model usage.
    pub assumptions: ValidationAssumptionReport,
}

#[derive(Debug, Serialize, Deserialize)]
#[allow(dead_code)]
struct PppEvaluationReport {
    epochs: usize,
    horiz_rms_m: Option<f64>,
    vert_rms_m: Option<f64>,
    time_to_first_meter_s: Option<f64>,
    time_to_decimeter_s: Option<f64>,
    time_to_centimeter_s: Option<f64>,
    residual_rms_m: f64,
    checkpoint_path: Option<String>,
}

/// Build the validation report for a run.
#[allow(clippy::too_many_arguments)]
pub fn build_validation_report(
    tracks: &[TrackingResult],
    obs: &[ObsEpoch],
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
    science_policy: ValidationSciencePolicy,
) -> Result<ValidationReport, bijux_gnss_core::api::InputError> {
    build_validation_report_with_budgets(
        tracks,
        obs,
        solutions,
        reference,
        sample_rate_hz,
        products_ok,
        product_fallbacks,
        science_policy,
        ValidationBudgets::default(),
    )
}

/// Build the validation report for a run from observation artifacts that preserve raw residuals.
#[allow(clippy::too_many_arguments)]
pub fn build_validation_report_from_observation_artifacts(
    tracks: &[TrackingResult],
    observation_artifacts: &ObservationPipelineArtifacts,
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
    science_policy: ValidationSciencePolicy,
) -> Result<ValidationReport, bijux_gnss_core::api::InputError> {
    build_validation_report_from_observation_artifacts_with_budgets(
        tracks,
        observation_artifacts,
        solutions,
        reference,
        sample_rate_hz,
        products_ok,
        product_fallbacks,
        science_policy,
        ValidationBudgets::default(),
    )
}

/// Build the validation report for a run with explicit validation budgets.
#[allow(clippy::too_many_arguments)]
pub fn build_validation_report_with_budgets(
    tracks: &[TrackingResult],
    obs: &[ObsEpoch],
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
    science_policy: ValidationSciencePolicy,
    budgets: ValidationBudgets,
) -> Result<ValidationReport, bijux_gnss_core::api::InputError> {
    build_validation_report_with_observation_context(
        tracks,
        obs,
        None,
        solutions,
        reference,
        sample_rate_hz,
        products_ok,
        product_fallbacks,
        science_policy,
        budgets,
    )
}

/// Build the validation report for a run from observation artifacts with explicit validation budgets.
#[allow(clippy::too_many_arguments)]
pub fn build_validation_report_from_observation_artifacts_with_budgets(
    tracks: &[TrackingResult],
    observation_artifacts: &ObservationPipelineArtifacts,
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
    science_policy: ValidationSciencePolicy,
    budgets: ValidationBudgets,
) -> Result<ValidationReport, bijux_gnss_core::api::InputError> {
    build_validation_report_with_observation_context(
        tracks,
        &observation_artifacts.epochs,
        Some(observation_artifacts),
        solutions,
        reference,
        sample_rate_hz,
        products_ok,
        product_fallbacks,
        science_policy,
        budgets,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_validation_report_with_observation_context(
    tracks: &[TrackingResult],
    obs: &[ObsEpoch],
    observation_artifacts: Option<&ObservationPipelineArtifacts>,
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
    science_policy: ValidationSciencePolicy,
    budgets: ValidationBudgets,
) -> Result<ValidationReport, bijux_gnss_core::api::InputError> {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }

    let mut horiz_errors = Vec::new();
    let mut vert_errors = Vec::new();
    let mut east_errors = Vec::new();
    let mut north_errors = Vec::new();
    let mut up_errors = Vec::new();
    let mut error_3d = Vec::new();
    let mut horiz_by_time = Vec::new();
    let mut vert_by_time = Vec::new();
    let mut fix_timeline = Vec::new();
    let mut reference_position_errors = Vec::new();
    let mut unmatched_solution_epochs = Vec::new();
    let mut matched_reference_epochs = std::collections::BTreeSet::new();
    let mut residuals = Vec::new();
    let mut covariance_realism_samples = Vec::new();
    let mut nees_values = Vec::new();

    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            matched_reference_epochs.insert(sol.epoch.index);
            let (east, north, up) = ecef_to_enu(
                sol.ecef_x_m.0,
                sol.ecef_y_m.0,
                sol.ecef_z_m.0,
                r.latitude_deg,
                r.longitude_deg,
                r.altitude_m,
            );
            let horiz = (east * east + north * north).sqrt();
            let vert = up.abs();
            let position_3d = (horiz * horiz + up * up).sqrt();
            let horizontal_margin_m = sol.integrity_hpl_m.map(|hpl_m| hpl_m - horiz);
            let vertical_margin_m = sol.integrity_vpl_m.map(|vpl_m| vpl_m - vert);
            east_errors.push(east);
            north_errors.push(north);
            up_errors.push(up);
            horiz_errors.push(horiz);
            vert_errors.push(vert);
            error_3d.push(position_3d);
            horiz_by_time.push((sol.t_rx_s.0, horiz));
            vert_by_time.push((sol.t_rx_s.0, vert));
            reference_position_errors.push(ReferencePositionErrorEpoch {
                epoch_idx: sol.epoch.index,
                east_m: east,
                north_m: north,
                up_m: up,
                horiz_m: horiz,
                vert_m: vert,
                error_3d_m: position_3d,
                hpl_m: sol.integrity_hpl_m,
                vpl_m: sol.integrity_vpl_m,
                horizontal_within_hpl: horizontal_margin_m.map(|margin_m| margin_m >= 0.0),
                vertical_within_vpl: vertical_margin_m.map(|margin_m| margin_m >= 0.0),
                horizontal_margin_m,
                vertical_margin_m,
            });
            covariance_realism_samples.push(
                crate::covariance_realism::CovarianceRealismEpochSample {
                    receiver_ecef_m: [sol.ecef_x_m.0, sol.ecef_y_m.0, sol.ecef_z_m.0],
                    east_m: east,
                    north_m: north,
                    up_m: up,
                    position_covariance_ecef_m2: sol.position_covariance_ecef_m2,
                },
            );
            if let (Some(sig_h), Some(sig_v)) = (sol.sigma_h_m, sol.sigma_v_m) {
                if sig_h.0 > 0.0 && sig_v.0 > 0.0 {
                    let nees = (east * east + north * north) / (sig_h.0 * sig_h.0)
                        + (up * up) / (sig_v.0 * sig_v.0);
                    nees_values.push(nees);
                }
            }
        } else {
            unmatched_solution_epochs.push(sol.epoch.index);
        }
        fix_timeline.push(FixTimelineEntry {
            epoch_idx: sol.epoch.index,
            fixed: matches!(sol.status, SolutionStatus::Fixed),
        });
        let mut per_sat = Vec::new();
        let mut rejected = Vec::new();
        for r in &sol.residuals {
            if r.rejected {
                rejected.push(r.sat);
            } else {
                per_sat.push((r.sat, r.residual_m.0));
            }
        }
        residuals.push(NavResidualReport {
            epoch_idx: sol.epoch.index,
            rms_m: sol.rms_m.0,
            pre_fit_rms_m: sol.pre_fit_residual_rms_m.map(|value| value.0),
            post_fit_rms_m: sol.post_fit_residual_rms_m.map(|value| value.0),
            pdop: sol.pdop,
            residuals: per_sat,
            constellation_residual_rms: sol
                .constellation_residual_rms
                .iter()
                .map(|summary| NavConstellationResidualReport {
                    constellation: format!("{:?}", summary.constellation),
                    pre_fit_rms_m: summary.pre_fit_rms_m.map(|value| value.0),
                    post_fit_rms_m: summary.post_fit_rms_m.map(|value| value.0),
                    pre_fit_sat_count: summary.pre_fit_sat_count,
                    post_fit_sat_count: summary.post_fit_sat_count,
                })
                .collect(),
            rejected,
        });
    }

    let unused_reference_epochs = reference
        .iter()
        .map(|epoch| epoch.epoch_idx)
        .filter(|epoch_idx| !matched_reference_epochs.contains(epoch_idx))
        .collect::<Vec<_>>();
    let reference_coordinate_required =
        budgets.reference_position_error_3d_m_max.is_some() && !solutions.is_empty();
    let protection_levels = protection_level_validation_report(&reference_position_errors);
    let reference_coordinate_coverage = ReferenceCoordinateCoverageReport {
        required: reference_coordinate_required,
        ready: !reference_coordinate_required
            || (unmatched_solution_epochs.is_empty() && !reference_position_errors.is_empty()),
        matched_solution_epoch_count: reference_position_errors.len(),
        unmatched_solution_epochs,
        unused_reference_epochs,
    };

    let east_stats = to_validation_stats(stats(&east_errors));
    let north_stats = to_validation_stats(stats(&north_errors));
    let up_stats = to_validation_stats(stats(&up_errors));
    let horiz_stats = to_validation_stats(stats(&horiz_errors));
    let vert_stats = to_validation_stats(stats(&vert_errors));
    let error_3d_stats = to_validation_stats(stats(&error_3d));
    let convergence = convergence_report(&horiz_by_time, &vert_by_time);
    let violations = check_budgets(
        tracks,
        solutions,
        &reference_position_errors,
        &reference_coordinate_coverage,
        &budgets,
    );
    let time_consistency = check_time_consistency(tracks, sample_rate_hz);
    let consistency = check_solution_consistency(solutions);
    let inter_frequency_alignment = check_inter_frequency_alignment(obs);
    let dual_frequency_observations = check_dual_frequency_observations(obs);
    let geometry_free = geometry_free_report(obs, &dual_frequency_observations);
    let melbourne_wubbena = melbourne_wubbena_report(obs, &dual_frequency_observations);
    let carrier_smoothed_code = observation_artifacts
        .map(validate_carrier_smoothed_code_from_artifacts)
        .unwrap_or_else(|| summarize_carrier_smoothed_code(obs));
    let multi_freq_present = dual_frequency_observations.observed_pairs > 0;
    let combinations_valid = dual_frequency_combinations_valid(obs);
    let support_matrix = support_status_matrix();
    let ppp_support =
        support_matrix.rows.iter().find(|row| row.mode == AdvancedMode::Ppp).cloned().unwrap_or(
            AdvancedSupportRow {
                mode: AdvancedMode::Ppp,
                maturity: AdvancedMaturity::NotReady,
                real_solver: false,
                required_inputs: Vec::new(),
                notes: "ppp support row missing".to_string(),
            },
        );
    let ppp_prereq = AdvancedPrerequisites {
        has_base_observations: true,
        has_rover_observations: true,
        has_ephemeris: !solutions.is_empty(),
        has_reference_frame: !reference.is_empty(),
        has_corrections: products_ok,
        has_min_satellites: solutions.iter().any(|sol| sol.used_sat_count >= 4),
        has_ambiguity_state: combinations_valid,
    };
    let ppp_prereq_decision = evaluate_prerequisites(AdvancedMode::Ppp, &ppp_prereq);
    let ppp_claim_decision = apply_downgrade_policy(
        AdvancedMode::Ppp,
        &ppp_prereq_decision,
        AdvancedSolutionClaim::NotReady,
        None,
    );
    let nis_values: Vec<f64> = solutions
        .iter()
        .filter_map(|s| {
            let pred = s.ekf_predicted_variance?;
            if pred > 0.0 {
                Some((s.ekf_innovation_rms.unwrap_or(0.0).powi(2)) / pred)
            } else {
                None
            }
        })
        .collect();
    let nis_mean = if nis_values.is_empty() {
        None
    } else {
        Some(nis_values.iter().sum::<f64>() / nis_values.len() as f64)
    };
    let covariance_realism =
        crate::covariance_realism::evaluate_covariance_realism(&covariance_realism_samples);
    let nees_mean = if let Some(position_nees_mean) = covariance_realism.position_nees_mean {
        Some(position_nees_mean)
    } else if nees_values.is_empty() {
        None
    } else {
        Some(nees_values.iter().sum::<f64>() / nees_values.len() as f64)
    };
    let mut consistency_warnings = Vec::new();
    if let Some(nis) = nis_mean {
        if !(0.1..=10.0).contains(&nis) {
            consistency_warnings.push(format!("NIS out of expected range: {nis:.3}"));
        }
    }
    if let Some(nees) = nees_mean {
        if !(0.1..=10.0).contains(&nees) {
            consistency_warnings.push(format!("NEES out of expected range: {nees:.3}"));
        }
    }
    if let Some(summary) = innovation_consistency_anomaly_summary(solutions) {
        consistency_warnings.push(summary);
    }
    consistency_warnings.extend(covariance_realism.warnings.iter().cloned());
    let lock_ratio_by_epoch = lock_ratio_by_epoch(tracks);
    let integrity = classify_integrity(solutions, &lock_ratio_by_epoch, &science_policy);
    let assumptions = summarize_assumptions(solutions);
    let diagnostic_partition =
        partition_diagnostics(&time_consistency.warnings, &consistency_warnings, solutions);

    Ok(ValidationReport {
        samples: tracks.iter().map(|t| t.epochs.len()).sum(),
        epochs: solutions.len(),
        east_error_m: east_stats,
        north_error_m: north_stats,
        up_error_m: up_stats,
        horiz_error_m: horiz_stats,
        vert_error_m: vert_stats,
        error_3d_m: error_3d_stats,
        convergence,
        fix_timeline,
        reference_position_errors,
        protection_levels,
        reference_coordinate_coverage,
        residuals,
        time_consistency,
        consistency,
        budgets,
        budget_violations: violations,
        nis_mean,
        nees_mean,
        covariance_realism,
        consistency_warnings,
        inter_frequency_alignment,
        dual_frequency_observations,
        geometry_free,
        melbourne_wubbena,
        carrier_smoothed_code,
        ppp_readiness: PppReadinessReport {
            multi_freq_present,
            combinations_valid,
            products_ok,
            product_fallbacks,
            maturity: ppp_support.maturity,
            prerequisites_met: ppp_prereq_decision.ready,
            refusal_class: ppp_prereq_decision.refusal_class,
            status: ppp_claim_decision.status,
            status_reason: ppp_claim_decision.downgrade_reason,
            claim: ppp_claim_decision.claim,
            support: ppp_support,
        },
        science_policy,
        integrity,
        diagnostic_partition,
        assumptions,
    })
}

fn innovation_consistency_anomaly_summary(solutions: &[NavSolutionEpoch]) -> Option<String> {
    let mut anomaly_count = 0usize;
    let mut peak_anomaly: Option<(f64, f64, f64, usize)> = None;

    for event in solutions.iter().flat_map(|solution| solution.health.iter()) {
        let bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly {
            normalized_innovation_squared,
            lower_bound,
            upper_bound,
            measurement_dimension,
        } = event
        else {
            continue;
        };
        anomaly_count += 1;
        let candidate =
            (*normalized_innovation_squared, *lower_bound, *upper_bound, *measurement_dimension);
        peak_anomaly = Some(match peak_anomaly {
            Some(current) if current.0 >= candidate.0 => current,
            _ => candidate,
        });
    }

    peak_anomaly.map(
        |(normalized_innovation_squared, lower_bound, upper_bound, measurement_dimension)| {
            format!(
                "innovation consistency anomalies: {anomaly_count} event(s), peak NIS {:.3} outside [{:.3}, {:.3}] for measurement dimension {}",
                normalized_innovation_squared,
                lower_bound,
                upper_bound,
                measurement_dimension
            )
        },
    )
}

fn protection_level_validation_report(
    reference_position_errors: &[ReferencePositionErrorEpoch],
) -> ProtectionLevelValidationReport {
    let mut horizontal_reported_epoch_count = 0usize;
    let mut vertical_reported_epoch_count = 0usize;
    let mut horizontal_contained_epoch_count = 0usize;
    let mut vertical_contained_epoch_count = 0usize;
    let mut horizontal_breach_epochs = Vec::new();
    let mut vertical_breach_epochs = Vec::new();
    let mut min_horizontal_margin_m: Option<f64> = None;
    let mut min_vertical_margin_m: Option<f64> = None;

    for error in reference_position_errors {
        if let Some(margin_m) = error.horizontal_margin_m {
            horizontal_reported_epoch_count += 1;
            min_horizontal_margin_m = Some(match min_horizontal_margin_m {
                Some(current_min_m) => current_min_m.min(margin_m),
                None => margin_m,
            });
            if margin_m >= 0.0 {
                horizontal_contained_epoch_count += 1;
            } else {
                horizontal_breach_epochs.push(error.epoch_idx);
            }
        }
        if let Some(margin_m) = error.vertical_margin_m {
            vertical_reported_epoch_count += 1;
            min_vertical_margin_m = Some(match min_vertical_margin_m {
                Some(current_min_m) => current_min_m.min(margin_m),
                None => margin_m,
            });
            if margin_m >= 0.0 {
                vertical_contained_epoch_count += 1;
            } else {
                vertical_breach_epochs.push(error.epoch_idx);
            }
        }
    }

    ProtectionLevelValidationReport {
        matched_epoch_count: reference_position_errors.len(),
        horizontal_reported_epoch_count,
        vertical_reported_epoch_count,
        horizontal_contained_epoch_count,
        vertical_contained_epoch_count,
        horizontal_breach_epochs,
        vertical_breach_epochs,
        min_horizontal_margin_m,
        min_vertical_margin_m,
    }
}

fn dual_frequency_combinations_valid(obs: &[ObsEpoch]) -> bool {
    let mut saw_complete_pair = false;

    let mut constellations = std::collections::BTreeSet::new();
    for epoch in obs {
        for satellite in &epoch.sats {
            constellations.insert(satellite.signal_id.sat.constellation);
        }
    }

    for constellation in constellations {
        for &(band_1, band_2) in
            supported_dual_frequency_band_pairs_for_constellation(constellation)
        {
            let combinations = iono_free_code_from_obs_epochs(obs, band_1, band_2);
            let observed_pairs = combinations
                .iter()
                .filter(|combination| combination.sat.constellation == constellation)
                .filter(|combination| combination.reason != "missing_frequency")
                .collect::<Vec<_>>();
            if observed_pairs.is_empty() {
                continue;
            }

            if observed_pairs.iter().any(|combination| combination.status != "ok") {
                return false;
            }

            saw_complete_pair = true;
        }
    }

    saw_complete_pair
}

fn dual_frequency_pair_epochs(
    obs: &[ObsEpoch],
    dual_frequency_observations: &DualFrequencyObservationReport,
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<ObsEpoch> {
    let mut satellites_by_epoch: std::collections::BTreeMap<
        u64,
        std::collections::BTreeSet<SatId>,
    > = std::collections::BTreeMap::new();
    for pair in &dual_frequency_observations.pairs {
        if pair.band_1 != band_1 || pair.band_2 != band_2 {
            continue;
        }
        if pair.status != DualFrequencyPairStatus::Complete {
            continue;
        }
        satellites_by_epoch.entry(pair.epoch_idx).or_default().insert(pair.sat);
    }

    let mut filtered_epochs = Vec::new();
    for epoch in obs {
        let Some(satellites) = satellites_by_epoch.get(&epoch.epoch_idx) else {
            continue;
        };

        let mut filtered_epoch = epoch.clone();
        filtered_epoch.sats.retain(|satellite| {
            satellites.contains(&satellite.signal_id.sat)
                && (satellite.signal_id.band == band_1 || satellite.signal_id.band == band_2)
        });
        if !filtered_epoch.sats.is_empty() {
            filtered_epochs.push(filtered_epoch);
        }
    }

    filtered_epochs
}

fn geometry_free_report(
    obs: &[ObsEpoch],
    dual_frequency_observations: &DualFrequencyObservationReport,
) -> GeometryFreeReport {
    let mut observations = 0usize;
    let mut complete_pairs = 0usize;
    let mut unavailable = 0usize;
    let mut insufficient_history = 0usize;
    let mut nominal = 0usize;
    let mut ionosphere_drift = 0usize;
    let mut cycle_slip_suspects = 0usize;
    let mut max_abs_delta_m: Option<f64> = None;

    for &(band_1, band_2) in supported_dual_frequency_band_pairs() {
        let filtered_epochs =
            dual_frequency_pair_epochs(obs, dual_frequency_observations, band_1, band_2);
        if filtered_epochs.is_empty() {
            continue;
        }

        for diagnostic in geometry_free_diagnostics_from_obs_epochs(
            &filtered_epochs,
            band_1,
            band_2,
            GeometryFreeThresholds::default(),
        ) {
            observations += 1;
            if diagnostic.status == "ok" {
                complete_pairs += 1;
            } else {
                unavailable += 1;
            }

            if let Some(delta_m) = diagnostic.delta_from_previous_m {
                let abs_delta_m = delta_m.abs();
                max_abs_delta_m =
                    Some(max_abs_delta_m.map_or(abs_delta_m, |current| current.max(abs_delta_m)));
            }

            match diagnostic.event {
                GeometryFreeEvent::Unavailable => {}
                GeometryFreeEvent::InsufficientHistory => insufficient_history += 1,
                GeometryFreeEvent::Nominal => nominal += 1,
                GeometryFreeEvent::IonosphereDrift => ionosphere_drift += 1,
                GeometryFreeEvent::CycleSlipSuspect => cycle_slip_suspects += 1,
            }
        }
    }

    GeometryFreeReport {
        observations,
        complete_pairs,
        unavailable,
        insufficient_history,
        nominal,
        ionosphere_drift,
        cycle_slip_suspects,
        max_abs_delta_m,
    }
}

fn melbourne_wubbena_report(
    obs: &[ObsEpoch],
    dual_frequency_observations: &DualFrequencyObservationReport,
) -> MelbourneWubbenaReport {
    let mut observations = 0usize;
    let mut complete_pairs = 0usize;
    let mut unavailable = 0usize;
    let mut insufficient_history = 0usize;
    let mut nominal = 0usize;
    let mut wide_lane_slip_suspects = 0usize;
    let mut max_abs_delta_wide_lane_cycles: Option<f64> = None;

    for &(band_1, band_2) in supported_dual_frequency_band_pairs() {
        let filtered_epochs =
            dual_frequency_pair_epochs(obs, dual_frequency_observations, band_1, band_2);
        if filtered_epochs.is_empty() {
            continue;
        }

        for diagnostic in melbourne_wubbena_diagnostics_from_obs_epochs(
            &filtered_epochs,
            band_1,
            band_2,
            MelbourneWubbenaThresholds::default(),
        ) {
            observations += 1;
            if diagnostic.status == "ok" {
                complete_pairs += 1;
            } else {
                unavailable += 1;
            }

            if let Some(delta_cycles) = diagnostic.delta_from_previous_wide_lane_cycles {
                let abs_delta_cycles = delta_cycles.abs();
                max_abs_delta_wide_lane_cycles = Some(
                    max_abs_delta_wide_lane_cycles
                        .map_or(abs_delta_cycles, |current| current.max(abs_delta_cycles)),
                );
            }

            match diagnostic.event {
                MelbourneWubbenaEvent::Unavailable => {}
                MelbourneWubbenaEvent::InsufficientHistory => insufficient_history += 1,
                MelbourneWubbenaEvent::Nominal => nominal += 1,
                MelbourneWubbenaEvent::WideLaneSlipSuspect => wide_lane_slip_suspects += 1,
            }
        }
    }

    MelbourneWubbenaReport {
        observations,
        complete_pairs,
        unavailable,
        insufficient_history,
        nominal,
        wide_lane_slip_suspects,
        max_abs_delta_wide_lane_cycles,
    }
}

fn lock_ratio_by_epoch(tracks: &[TrackingResult]) -> std::collections::BTreeMap<u64, f64> {
    let mut counts: std::collections::BTreeMap<u64, (u64, u64)> = std::collections::BTreeMap::new();
    for track in tracks {
        for epoch in &track.epochs {
            let entry = counts.entry(epoch.epoch.index).or_insert((0, 0));
            entry.0 = entry.0.saturating_add(1);
            if epoch.lock {
                entry.1 = entry.1.saturating_add(1);
            }
        }
    }
    counts
        .into_iter()
        .map(|(epoch, (total, locked))| {
            let ratio = if total == 0 { 1.0 } else { locked as f64 / total as f64 };
            (epoch, ratio)
        })
        .collect()
}

fn classify_integrity(
    solutions: &[NavSolutionEpoch],
    lock_ratio_by_epoch: &std::collections::BTreeMap<u64, f64>,
    science_policy: &ValidationSciencePolicy,
) -> Vec<NavIntegrityReport> {
    let mut out = Vec::with_capacity(solutions.len());
    for solution in solutions {
        let mut reasons = Vec::new();
        if solution.integrity_hpl_m.is_none() || solution.integrity_vpl_m.is_none() {
            reasons.push("missing_integrity_evidence".to_string());
        }
        if solution.used_sat_count < science_policy.min_used_satellites
            || solution.pdop > science_policy.max_pdop
            || solution.gdop.unwrap_or(f64::INFINITY) > science_policy.max_gdop
        {
            reasons.push("weak_geometry".to_string());
        }
        if solution.rms_m.0 > science_policy.max_residual_rms_m {
            reasons.push("suspicious_residuals".to_string());
        }
        if let Some(ratio) = lock_ratio_by_epoch.get(&solution.epoch.index) {
            if *ratio < science_policy.min_lock_ratio {
                reasons.push("unstable_lock".to_string());
            }
        }
        let class = match reasons.as_slice() {
            [] => NavIntegrityClass::Nominal,
            [single] if single == "missing_integrity_evidence" => {
                NavIntegrityClass::IntegrityEvidenceMissing
            }
            [single] if single == "weak_geometry" => NavIntegrityClass::WeakGeometry,
            [single] if single == "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
            [single] if single == "unstable_lock" => NavIntegrityClass::UnstableLock,
            _ => NavIntegrityClass::Compound,
        };
        out.push(NavIntegrityReport { epoch_idx: solution.epoch.index, class, reasons });
    }
    out
}

fn summarize_assumptions(solutions: &[NavSolutionEpoch]) -> ValidationAssumptionReport {
    let mut time_systems = std::collections::BTreeSet::new();
    let mut reference_frames = std::collections::BTreeSet::new();
    let mut clock_models = std::collections::BTreeSet::new();
    for solution in solutions {
        if let Some(assumptions) = &solution.assumptions {
            time_systems.insert(assumptions.time_system.clone());
            reference_frames.insert(assumptions.reference_frame.clone());
            clock_models.insert(assumptions.clock_model.clone());
        }
    }
    ValidationAssumptionReport {
        time_systems: time_systems.into_iter().collect(),
        reference_frames: reference_frames.into_iter().collect(),
        clock_models: clock_models.into_iter().collect(),
    }
}

fn partition_diagnostics(
    time_warnings: &[String],
    consistency_warnings: &[String],
    solutions: &[NavSolutionEpoch],
) -> DiagnosticPartitionReport {
    let mut advisory = std::collections::BTreeSet::new();
    for warning in time_warnings {
        advisory.insert(format!("time_consistency:{warning}"));
    }
    for warning in consistency_warnings {
        advisory.insert(format!("consistency:{warning}"));
    }
    let mut refusals = std::collections::BTreeSet::new();
    for solution in solutions {
        if let Some(refusal) = solution.refusal_class {
            refusals.insert(format!(
                "epoch={} refusal={:?} decision={}",
                solution.epoch.index, refusal, solution.explain_decision
            ));
        }
    }
    DiagnosticPartitionReport {
        advisory_diagnostics: advisory.into_iter().collect(),
        enforced_refusals: refusals.into_iter().collect(),
    }
}

fn convergence_report(horiz: &[(f64, f64)], vert: &[(f64, f64)]) -> ConvergenceReport {
    let mut t1 = None;
    let mut t03 = None;
    let mut t01 = None;
    let mut worst_h: f64 = 0.0;
    let mut worst_v: f64 = 0.0;
    for (t, h) in horiz {
        worst_h = worst_h.max(*h);
        if t1.is_none() && *h <= 1.0 {
            t1 = Some(*t);
        }
        if t03.is_none() && *h <= 0.3 {
            t03 = Some(*t);
        }
        if t01.is_none() && *h <= 0.1 {
            t01 = Some(*t);
        }
    }
    for (_t, v) in vert {
        worst_v = worst_v.max(*v);
    }
    ConvergenceReport {
        time_to_1m_s: t1,
        time_to_0_3m_s: t03,
        time_to_0_1m_s: t01,
        worst_horiz_m: worst_h,
        worst_vert_m: worst_v,
    }
}

/// Check time consistency in tracking epochs.
pub fn check_time_consistency(
    tracks: &[TrackingResult],
    sample_rate_hz: f64,
) -> TimeConsistencyReport {
    let mut epoch_backward = 0;
    let mut epoch_gaps = 0;
    let mut sample_backward = 0;
    let mut warnings = Vec::new();
    let mut epochs_checked = 0;
    let mut sample_step_mismatch = 0;
    let configured_step = if sample_rate_hz > 0.0 {
        Some((sample_rate_hz * 0.001).round().max(1.0) as u64)
    } else {
        None
    };
    let mut expected_step: Option<u64> = configured_step;
    let mut sample_step_total = 0u64;
    let mut sample_step_count = 0u64;

    for track in tracks {
        let mut prev_epoch: Option<u64> = None;
        let mut prev_sample: Option<u64> = None;
        for epoch in &track.epochs {
            epochs_checked += 1;
            if let Some(prev) = prev_epoch {
                if epoch.epoch.index < prev {
                    epoch_backward += 1;
                } else if epoch.epoch.index > prev + 1 {
                    epoch_gaps += 1;
                }
            }
            if let Some(prev) = prev_sample {
                if epoch.sample_index < prev {
                    sample_backward += 1;
                } else {
                    let step = epoch.sample_index - prev;
                    if let Some(expected) = expected_step {
                        if step != expected {
                            sample_step_mismatch += 1;
                        }
                    } else {
                        expected_step = Some(step);
                    }
                    sample_step_total = sample_step_total.saturating_add(step);
                    sample_step_count = sample_step_count.saturating_add(1);
                }
            }
            prev_epoch = Some(epoch.epoch.index);
            prev_sample = Some(epoch.sample_index);
        }
    }

    if epoch_backward > 0 {
        warnings.push("epoch index went backwards".to_string());
    }
    if epoch_gaps > 0 {
        warnings.push("epoch index gaps detected".to_string());
    }
    if sample_backward > 0 {
        warnings.push("sample index went backwards".to_string());
    }
    if sample_step_mismatch > 0 {
        warnings.push("sample cadence mismatch detected".to_string());
    }
    let observed_step_mean = if sample_step_count > 0 {
        Some(sample_step_total as f64 / sample_step_count as f64)
    } else {
        None
    };

    TimeConsistencyReport {
        channels: tracks.len(),
        epochs_checked,
        epoch_backward,
        epoch_gaps,
        sample_backward,
        sample_step_mismatch,
        expected_step,
        observed_step_mean,
        warnings,
    }
}

#[allow(dead_code)]
fn build_ppp_config(profile: &ReceiverConfig) -> PppConfig {
    let p = &profile.navigation.ppp;
    let ar_mode = match p.ar_mode.as_str() {
        "ppp_ar_wide_lane" => bijux_gnss_nav::api::PppArMode::PppArWideLane,
        "ppp_ar_narrow_lane" => bijux_gnss_nav::api::PppArMode::PppArNarrowLane,
        _ => bijux_gnss_nav::api::PppArMode::FloatPpp,
    };
    PppConfig {
        enable_iono_state: p.enable_iono_state,
        use_iono_free: p.use_iono_free,
        use_doppler: p.use_doppler,
        ar_mode,
        ar_ratio_threshold: p.ar_ratio_threshold,
        ar_stability_epochs: p.ar_stability_epochs,
        ar_max_sats: p.ar_max_sats,
        ar_use_elevation: p.ar_use_elevation,
        prune_after_epochs: p.prune_after_epochs,
        reset_gap_s: p.reset_gap_s,
        residual_gate_m: p.residual_gate_m,
        drift_window_epochs: p.drift_window_epochs as usize,
        drift_threshold_m: p.drift_threshold_m,
        checkpoint_interval_epochs: p.checkpoint_interval_epochs,
        tropo_pressure_hpa: None,
        tropo_temperature_k: None,
        tropo_relative_humidity: None,
        solid_earth_tide_model: None,
        ocean_tide_loading_model: None,
        receiver_antenna_type: p.receiver_antenna_type.clone(),
        receiver_antenna_calibrations: None,
        satellite_antenna_calibrations: None,
        process_noise: PppProcessNoise {
            clock_drift_s: p.noise_clock_drift,
            ztd_m: p.noise_ztd,
            iono_m: p.noise_iono,
            ambiguity_cycles: p.noise_ambiguity,
            ..PppProcessNoise::default()
        },
        measurement_noise: Default::default(),
        weighting: WeightingConfig {
            model: match profile.navigation.weighting.mode {
                crate::engine::receiver_config::NavigationWeightingMode::Elevation => {
                    PositionWeightingModel::Elevation
                }
                crate::engine::receiver_config::NavigationWeightingMode::Cn0 => {
                    PositionWeightingModel::Cn0
                }
                crate::engine::receiver_config::NavigationWeightingMode::ElevationCn0 => {
                    PositionWeightingModel::ElevationCn0
                }
            },
            enabled: profile.navigation.weighting.enabled,
            min_elev_deg: profile.navigation.weighting.min_elev_deg,
            elev_exponent: profile.navigation.weighting.elev_exponent,
            cn0_ref_dbhz: profile.navigation.weighting.cn0_ref_dbhz,
            min_weight: profile.navigation.weighting.min_weight,
        },
        convergence: PppConvergenceConfig {
            min_time_s: p.convergence_min_time_s,
            pos_rate_mps: p.convergence_pos_rate_mps,
            sigma_h_m: p.convergence_sigma_h_m,
            sigma_v_m: p.convergence_sigma_v_m,
        },
    }
}

#[allow(dead_code)]
fn ppp_evaluation_report(
    solutions: &[bijux_gnss_nav::api::PppSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> PppEvaluationReport {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    let mut residual_rms = Vec::new();
    let mut last_conv = None;
    for sol in solutions {
        residual_rms.push(sol.rms_m);
        if let Some(r) = ref_map.get(&sol.epoch_idx) {
            let (x, y, z) = reference_ecef(r);
            let dx = sol.ecef_x_m - x;
            let dy = sol.ecef_y_m - y;
            let dz = sol.ecef_z_m - z;
            horiz.push((dx * dx + dy * dy).sqrt());
            vert.push(dz.abs());
        }
        last_conv = Some(sol.convergence.clone());
    }
    let horiz_rms = if horiz.is_empty() {
        None
    } else {
        Some((horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt())
    };
    let vert_rms = if vert.is_empty() {
        None
    } else {
        Some((vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt())
    };
    let residual_rms_m = if residual_rms.is_empty() {
        0.0
    } else {
        (residual_rms.iter().map(|v| v * v).sum::<f64>() / residual_rms.len() as f64).sqrt()
    };
    let (t1, t10, t1cm) = last_conv
        .map(|c| (c.time_to_first_meter_s, c.time_to_decimeter_s, c.time_to_centimeter_s))
        .unwrap_or((None, None, None));
    PppEvaluationReport {
        epochs: solutions.len(),
        horiz_rms_m: horiz_rms,
        vert_rms_m: vert_rms,
        time_to_first_meter_s: t1,
        time_to_decimeter_s: t10,
        time_to_centimeter_s: t1cm,
        residual_rms_m,
        checkpoint_path: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::TrackingResult;
    use bijux_gnss_core::api::{
        Chips, Constellation, Cycles, Epoch, Hertz, LockFlags, Meters, NavAssumptions, ObsMetadata,
        ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, SigId, SignalCode, TrackEpoch,
    };
    use bijux_gnss_signal::api::{carrier_wavelength_m, signal_registry};
    use serde::Deserialize;

    #[test]
    fn golden_reference_validation() {
        let (ecef_x_m, ecef_y_m, ecef_z_m) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let sol = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: 0 },
            t_rx_s: bijux_gnss_core::api::Seconds(0.0),
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(0, 1.0),
            ecef_x_m: bijux_gnss_core::api::Meters(ecef_x_m),
            ecef_y_m: bijux_gnss_core::api::Meters(ecef_y_m),
            ecef_z_m: bijux_gnss_core::api::Meters(ecef_z_m),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: bijux_gnss_core::api::Meters(0.0),
            clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
            clock_bias_m: bijux_gnss_core::api::Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            pre_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(0.0)),
            post_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(0.0)),
            rms_m: bijux_gnss_core::api::Meters(0.0),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
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
            wls_solver_rank: None,
            wls_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: bijux_gnss_core::api::NavLifecycleState::CodeOnly,
            uncertainty_class: bijux_gnss_core::api::NavUncertaintyClass::Low,
            assumptions: None,
            refusal_class: None,
            artifact_id: "nav-epoch-0000000000-golden".to_string(),
            source_observation_epoch_id: "obs-epoch-0000000000-golden".to_string(),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["navigation_solution_usable".to_string()],
            provenance: None,
            sat_count: 4,
            used_sat_count: 4,
            rejected_sat_count: 0,
            hdop: Some(1.0),
            vdop: Some(1.0),
            gdop: Some(1.0),
            tdop: Some(0.5),
            stability_signature: "navsig:v2:golden".to_string(),
            stability_signature_version:
                bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        let reference = ValidationReferenceEpoch {
            epoch_idx: 0,
            t_rx_s: Some(0.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(ecef_x_m),
            ecef_y_m: Some(ecef_y_m),
            ecef_z_m: Some(ecef_z_m),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };
        let report = build_validation_report(
            &[],
            &[],
            &[sol],
            &[reference],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");
        assert!(report.horiz_error_m.rms <= 1e-6);
        assert!(report.vert_error_m.rms <= 1e-6);
        assert!(report.error_3d_m.rms <= 1e-6);
        assert_eq!(report.reference_position_errors.len(), 1);
        assert_eq!(report.integrity.len(), 1);
    }

    #[test]
    fn validation_report_emits_enu_reference_components() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let solution = NavSolutionEpoch {
            ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 3.0),
            ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 1.0),
            ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 2.0),
            position_covariance_ecef_m2: Some([[9.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 4.0]]),
            sigma_e_m: Some(bijux_gnss_core::api::Meters(1.0)),
            sigma_n_m: Some(bijux_gnss_core::api::Meters(1.5)),
            sigma_u_m: Some(bijux_gnss_core::api::Meters(3.0)),
            horizontal_error_ellipse_major_axis_m: Some(bijux_gnss_core::api::Meters(2.0)),
            horizontal_error_ellipse_minor_axis_m: Some(bijux_gnss_core::api::Meters(1.0)),
            horizontal_error_ellipse_azimuth_deg: Some(42.0),
            sigma_h_m: Some(bijux_gnss_core::api::Meters(2.0)),
            sigma_v_m: Some(bijux_gnss_core::api::Meters(3.0)),
            integrity_hpl_m: Some(3.0),
            integrity_vpl_m: Some(4.0),
            ..fixture_solution(7, 1.0, 0.5, 4)
        };
        let reference = ValidationReferenceEpoch {
            epoch_idx: 7,
            t_rx_s: Some(7.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };

        let report = build_validation_report(
            &[],
            &[],
            &[solution],
            &[reference],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        let error = report.reference_position_errors.first().expect("reference position error");
        assert_eq!(error.epoch_idx, 7);
        assert!((error.east_m - 1.0).abs() < 1.0e-12);
        assert!((error.north_m - 2.0).abs() < 1.0e-12);
        assert!((error.up_m - 3.0).abs() < 1.0e-12);
        assert!((error.horiz_m - 5.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((error.vert_m - 3.0).abs() < 1.0e-12);
        assert!((error.error_3d_m - 14.0_f64.sqrt()).abs() < 1.0e-12);
        assert_eq!(error.hpl_m, Some(3.0));
        assert_eq!(error.vpl_m, Some(4.0));
        assert_eq!(error.horizontal_within_hpl, Some(true));
        assert_eq!(error.vertical_within_vpl, Some(true));
        assert!(
            (error.horizontal_margin_m.expect("horizontal margin") - (3.0 - 5.0_f64.sqrt())).abs()
                < 1.0e-12
        );
        assert!((error.vertical_margin_m.expect("vertical margin") - 1.0).abs() < 1.0e-12);
        assert!((report.east_error_m.rms - 1.0).abs() < 1.0e-12);
        assert!((report.north_error_m.rms - 2.0).abs() < 1.0e-12);
        assert!((report.up_error_m.rms - 3.0).abs() < 1.0e-12);
        assert!((report.horiz_error_m.rms - 5.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((report.vert_error_m.rms - 3.0).abs() < 1.0e-12);
        assert!((report.error_3d_m.rms - 14.0_f64.sqrt()).abs() < 1.0e-12);
        assert!(report.nees_mean.is_some());
        assert_eq!(report.covariance_realism.total_epoch_count, 1);
        assert_eq!(report.covariance_realism.covariance_epoch_count, 1);
        assert_eq!(report.covariance_realism.horizontal_95.sample_count, 1);
        assert_eq!(report.covariance_realism.vertical_95.sample_count, 1);
        assert_eq!(report.covariance_realism.position_3d_95.sample_count, 1);
        let horizontal_nees =
            report.covariance_realism.horizontal_nees_mean.expect("horizontal nees");
        let vertical_normalized_error = report
            .covariance_realism
            .vertical_normalized_error_squared_mean
            .expect("vertical normalized error");
        let position_nees = report.covariance_realism.position_nees_mean.expect("position nees");
        assert!((horizontal_nees - 2.0).abs() < 1.0e-5, "{horizontal_nees}");
        assert!((vertical_normalized_error - 1.0).abs() < 1.0e-9, "{vertical_normalized_error}");
        assert!((position_nees - 3.0).abs() < 1.0e-5, "{position_nees}");
        assert_eq!(report.protection_levels.matched_epoch_count, 1);
        assert_eq!(report.protection_levels.horizontal_reported_epoch_count, 1);
        assert_eq!(report.protection_levels.vertical_reported_epoch_count, 1);
        assert_eq!(report.protection_levels.horizontal_contained_epoch_count, 1);
        assert_eq!(report.protection_levels.vertical_contained_epoch_count, 1);
        assert!(report.protection_levels.horizontal_breach_epochs.is_empty());
        assert!(report.protection_levels.vertical_breach_epochs.is_empty());
    }

    #[test]
    fn validation_report_surfaces_innovation_consistency_anomalies() {
        let mut solution = fixture_solution(9, 1.0, 0.5, 4);
        solution.health.push(bijux_gnss_core::api::NavHealthEvent::InnovationConsistencyAnomaly {
            normalized_innovation_squared: 8.0,
            lower_bound: 0.1,
            upper_bound: 6.6,
            measurement_dimension: 1,
        });
        let reference = ValidationReferenceEpoch {
            epoch_idx: 9,
            t_rx_s: Some(9.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(0.0),
            ecef_y_m: Some(0.0),
            ecef_z_m: Some(0.0),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };

        let report = build_validation_report(
            &[],
            &[],
            &[solution],
            &[reference],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert!(report.consistency_warnings.iter().any(|warning| {
            warning.contains("innovation consistency anomalies")
                && warning.contains("peak NIS 8.000")
        }));
    }

    #[test]
    fn validation_report_summarizes_protection_level_breaches() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let solution = NavSolutionEpoch {
            ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 3.0),
            ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 1.0),
            ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 4.0),
            integrity_hpl_m: Some(1.0),
            integrity_vpl_m: Some(2.0),
            ..fixture_solution(8, 1.0, 0.5, 4)
        };
        let reference = ValidationReferenceEpoch {
            epoch_idx: 8,
            t_rx_s: Some(8.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(x_ref),
            ecef_y_m: Some(y_ref),
            ecef_z_m: Some(z_ref),
            vel_x_mps: None,
            vel_y_mps: None,
            vel_z_mps: None,
        };

        let report = build_validation_report(
            &[],
            &[],
            &[solution],
            &[reference],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        let error = report.reference_position_errors.first().expect("reference position error");
        assert_eq!(error.horizontal_within_hpl, Some(false));
        assert_eq!(error.vertical_within_vpl, Some(false));
        assert!(error.horizontal_margin_m.expect("horizontal margin") < 0.0);
        assert!(error.vertical_margin_m.expect("vertical margin") < 0.0);
        assert_eq!(report.protection_levels.horizontal_breach_epochs, vec![8]);
        assert_eq!(report.protection_levels.vertical_breach_epochs, vec![8]);
        assert_eq!(report.protection_levels.horizontal_contained_epoch_count, 0);
        assert_eq!(report.protection_levels.vertical_contained_epoch_count, 0);
        assert!(
            report.protection_levels.min_horizontal_margin_m.expect("horizontal min margin") < 0.0
        );
        assert!(report.protection_levels.min_vertical_margin_m.expect("vertical min margin") < 0.0);
    }

    #[test]
    fn validation_report_preserves_pre_and_post_fit_residual_rms() {
        let mut solution = fixture_solution(9, 1.0, 0.5, 4);
        solution.pre_fit_residual_rms_m = Some(bijux_gnss_core::api::Meters(12.0));
        solution.post_fit_residual_rms_m = Some(bijux_gnss_core::api::Meters(3.0));
        solution.rms_m = bijux_gnss_core::api::Meters(3.0);
        solution.constellation_residual_rms = vec![
            bijux_gnss_core::api::NavConstellationResidualRms {
                constellation: Constellation::Gps,
                pre_fit_rms_m: Some(bijux_gnss_core::api::Meters(10.0)),
                post_fit_rms_m: Some(bijux_gnss_core::api::Meters(2.0)),
                pre_fit_sat_count: 4,
                post_fit_sat_count: 4,
            },
            bijux_gnss_core::api::NavConstellationResidualRms {
                constellation: Constellation::Galileo,
                pre_fit_rms_m: Some(bijux_gnss_core::api::Meters(16.0)),
                post_fit_rms_m: Some(bijux_gnss_core::api::Meters(5.0)),
                pre_fit_sat_count: 2,
                post_fit_sat_count: 2,
            },
        ];

        let report = build_validation_report(
            &[],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        let residual = report.residuals.first().expect("residual report");
        assert_eq!(residual.epoch_idx, 9);
        assert_eq!(residual.rms_m, 3.0);
        assert_eq!(residual.pre_fit_rms_m, Some(12.0));
        assert_eq!(residual.post_fit_rms_m, Some(3.0));
        assert_eq!(residual.constellation_residual_rms.len(), 2);
        assert_eq!(residual.constellation_residual_rms[0].constellation, "Gps");
        assert_eq!(residual.constellation_residual_rms[0].pre_fit_rms_m, Some(10.0));
        assert_eq!(residual.constellation_residual_rms[0].post_fit_rms_m, Some(2.0));
        assert_eq!(residual.constellation_residual_rms[0].pre_fit_sat_count, 4);
        assert_eq!(residual.constellation_residual_rms[0].post_fit_sat_count, 4);
        assert_eq!(residual.constellation_residual_rms[1].constellation, "Galileo");
    }

    #[test]
    fn validation_report_accepts_reference_position_error_within_budget() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let budgets = ValidationBudgets {
            nav_min_lock_epochs: 0,
            reference_position_error_3d_m_max: Some(5.0),
            ..ValidationBudgets::default()
        };
        let report = build_validation_report_with_budgets(
            &[],
            &[],
            &[NavSolutionEpoch {
                ecef_x_m: bijux_gnss_core::api::Meters(x_ref),
                ecef_y_m: bijux_gnss_core::api::Meters(y_ref),
                ecef_z_m: bijux_gnss_core::api::Meters(z_ref),
                ..fixture_solution(11, 1.0, 0.5, 4)
            }],
            &[ValidationReferenceEpoch {
                epoch_idx: 11,
                t_rx_s: Some(11.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(x_ref),
                ecef_y_m: Some(y_ref),
                ecef_z_m: Some(z_ref),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            }],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
            budgets,
        )
        .expect("validation report");

        assert!(report.budget_violations.is_empty());
        assert!(report.reference_coordinate_coverage.required);
        assert!(report.reference_coordinate_coverage.ready);
        assert!(report.reference_coordinate_coverage.unmatched_solution_epochs.is_empty());
    }

    #[test]
    fn validation_report_reports_reference_position_budget_violation() {
        let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let budgets = ValidationBudgets {
            nav_min_lock_epochs: 0,
            reference_position_error_3d_m_max: Some(5.0),
            ..ValidationBudgets::default()
        };
        let report = build_validation_report_with_budgets(
            &[],
            &[],
            &[NavSolutionEpoch {
                ecef_x_m: bijux_gnss_core::api::Meters(x_ref + 2.0),
                ecef_y_m: bijux_gnss_core::api::Meters(y_ref + 3.0),
                ecef_z_m: bijux_gnss_core::api::Meters(z_ref + 4.0),
                ..fixture_solution(12, 1.0, 0.5, 4)
            }],
            &[ValidationReferenceEpoch {
                epoch_idx: 12,
                t_rx_s: Some(12.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(x_ref),
                ecef_y_m: Some(y_ref),
                ecef_z_m: Some(z_ref),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            }],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
            budgets,
        )
        .expect("validation report");

        assert_eq!(
            report.budget_violations,
            vec!["reference position 3d error too high at epoch 12: 5.39 m"]
        );
        assert!(report.reference_coordinate_coverage.required);
        assert!(report.reference_coordinate_coverage.ready);
    }

    #[test]
    fn validation_report_requires_reference_coordinates_when_budget_is_set() {
        let budgets = ValidationBudgets {
            nav_min_lock_epochs: 0,
            reference_position_error_3d_m_max: Some(5.0),
            ..ValidationBudgets::default()
        };
        let report = build_validation_report_with_budgets(
            &[],
            &[],
            &[fixture_solution(21, 1.0, 0.5, 4)],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
            budgets,
        )
        .expect("validation report");

        assert!(report.reference_coordinate_coverage.required);
        assert!(!report.reference_coordinate_coverage.ready);
        assert_eq!(report.reference_coordinate_coverage.matched_solution_epoch_count, 0);
        assert_eq!(report.reference_coordinate_coverage.unmatched_solution_epochs, vec![21]);
        assert_eq!(
            report.budget_violations,
            vec!["reference coordinates unavailable for solution epochs: 21"]
        );
    }

    #[test]
    fn validation_report_requires_reference_coordinates_for_all_solution_epochs() {
        let budgets = ValidationBudgets {
            nav_min_lock_epochs: 0,
            reference_position_error_3d_m_max: Some(5.0),
            ..ValidationBudgets::default()
        };
        let (x_ref, y_ref, z_ref) = bijux_gnss_core::api::lla_to_ecef(0.0, 0.0, 0.0);
        let mut matched_solution = fixture_solution(30, 1.0, 0.5, 4);
        matched_solution.ecef_x_m = bijux_gnss_core::api::Meters(x_ref);
        matched_solution.ecef_y_m = bijux_gnss_core::api::Meters(y_ref);
        matched_solution.ecef_z_m = bijux_gnss_core::api::Meters(z_ref);
        let report = build_validation_report_with_budgets(
            &[],
            &[],
            &[matched_solution, fixture_solution(31, 1.0, 0.5, 4)],
            &[ValidationReferenceEpoch {
                epoch_idx: 30,
                t_rx_s: Some(30.0),
                latitude_deg: 0.0,
                longitude_deg: 0.0,
                altitude_m: 0.0,
                ecef_x_m: Some(x_ref),
                ecef_y_m: Some(y_ref),
                ecef_z_m: Some(z_ref),
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            }],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
            budgets,
        )
        .expect("validation report");

        assert!(report.reference_coordinate_coverage.required);
        assert!(!report.reference_coordinate_coverage.ready);
        assert_eq!(report.reference_coordinate_coverage.matched_solution_epoch_count, 1);
        assert_eq!(report.reference_coordinate_coverage.unmatched_solution_epochs, vec![31]);
        assert_eq!(
            report.budget_violations,
            vec!["reference coordinates unavailable for solution epochs: 31"]
        );
    }

    #[test]
    fn validation_report_marks_l1_l2_dual_frequency_pairs_ready_for_combinations() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                40,
                vec![
                    dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                    dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true),
                ],
            )],
            &[fixture_solution(40, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
        assert_eq!(report.dual_frequency_observations.l1_l2_pairs, 1);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(report.ppp_readiness.combinations_valid);
        assert_eq!(report.ppp_readiness.maturity, AdvancedMaturity::NotReady);
        assert_eq!(report.ppp_readiness.status, "not_ready");
        assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
        assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
    }

    #[test]
    fn validation_report_marks_l1_l5_dual_frequency_pairs_ready_for_combinations() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                41,
                vec![
                    dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                    dual_frequency_satellite(SignalBand::L5, SignalCode::L5I, true, true),
                ],
            )],
            &[fixture_solution(41, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
        assert_eq!(report.dual_frequency_observations.l1_l5_pairs, 1);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(report.ppp_readiness.combinations_valid);
    }

    #[test]
    fn validation_report_marks_e1_e5_dual_frequency_pairs_ready_for_combinations() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                44,
                vec![
                    dual_frequency_satellite_for_constellation(
                        Constellation::Galileo,
                        SignalBand::E1,
                        SignalCode::E1B,
                        true,
                        true,
                    ),
                    dual_frequency_satellite_for_constellation(
                        Constellation::Galileo,
                        SignalBand::E5,
                        SignalCode::E5a,
                        true,
                        true,
                    ),
                ],
            )],
            &[fixture_solution(44, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
        assert_eq!(report.dual_frequency_observations.e1_e5_pairs, 1);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(report.ppp_readiness.combinations_valid);
    }

    #[test]
    fn validation_report_marks_b1_b2_dual_frequency_pairs_ready_for_combinations() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                45,
                vec![
                    dual_frequency_satellite_for_constellation(
                        Constellation::Beidou,
                        SignalBand::B1,
                        SignalCode::B1I,
                        true,
                        true,
                    ),
                    dual_frequency_satellite_for_constellation(
                        Constellation::Beidou,
                        SignalBand::B2,
                        SignalCode::B2I,
                        true,
                        true,
                    ),
                ],
            )],
            &[fixture_solution(45, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
        assert_eq!(report.dual_frequency_observations.b1_b2_pairs, 1);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(report.ppp_readiness.combinations_valid);
    }

    #[test]
    fn validation_report_marks_incomplete_dual_frequency_pairs_not_ready_for_combinations() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                42,
                vec![
                    dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                    dual_frequency_satellite(SignalBand::L2, SignalCode::Py, false, true),
                ],
            )],
            &[fixture_solution(42, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
        assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(!report.ppp_readiness.combinations_valid);
        assert!(!report.ppp_readiness.prerequisites_met);
        assert_eq!(report.ppp_readiness.status, "not_ready");
        assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
        assert_eq!(
            report.ppp_readiness.refusal_class,
            Some(AdvancedRefusalClass::UnsupportedModel)
        );
        assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
    }

    #[test]
    fn validation_report_accepts_code_ready_pairs_without_carrier_lock() {
        let report = build_validation_report(
            &[],
            &[dual_frequency_epoch(
                43,
                vec![
                    dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, false),
                    dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, false),
                ],
            )],
            &[fixture_solution(43, 1.0, 0.5, 4)],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
        assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
        assert!(report.ppp_readiness.multi_freq_present);
        assert!(report.ppp_readiness.combinations_valid);
        assert_eq!(report.ppp_readiness.status, "not_ready");
        assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
        assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
    }

    #[test]
    fn validation_report_surfaces_carrier_smoothed_code_summary_without_raw_residuals() {
        let report = build_validation_report(
            &[],
            &[
                dual_frequency_epoch(43, vec![carrier_smoothed_code_satellite(1, false)]),
                dual_frequency_epoch(44, vec![carrier_smoothed_code_satellite(6, false)]),
                dual_frequency_epoch(45, vec![carrier_smoothed_code_satellite(1, true)]),
            ],
            &[fixture_solution(43, 1.0, 0.5, 4)],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.carrier_smoothed_code.observations, 3);
        assert_eq!(report.carrier_smoothed_code.accepted_observations, 3);
        assert_eq!(report.carrier_smoothed_code.smoothed_observations, 3);
        assert_eq!(report.carrier_smoothed_code.cycle_slip_observations, 1);
        assert_eq!(report.carrier_smoothed_code.slip_reset_observations, 1);
        assert_eq!(report.carrier_smoothed_code.hidden_slip_observations, 0);
        assert_eq!(report.carrier_smoothed_code.improvement_verified, None);
        assert_eq!(report.carrier_smoothed_code.slip_visibility_verified, Some(true));
    }

    #[test]
    fn validation_report_summarizes_geometry_free_dynamics() {
        let report = build_validation_report(
            &[],
            &[
                dual_frequency_epoch(
                    50,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            22_000_000.0,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            21_999_999.50,
                        ),
                    ],
                ),
                dual_frequency_epoch(
                    51,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            22_000_000.0,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            21_999_999.46,
                        ),
                    ],
                ),
                dual_frequency_epoch(
                    52,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            22_000_000.0,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            21_999_999.20,
                        ),
                    ],
                ),
            ],
            &[
                fixture_solution(50, 1.0, 0.5, 4),
                fixture_solution(51, 1.0, 0.5, 4),
                fixture_solution(52, 1.0, 0.5, 4),
            ],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.geometry_free.observations, 3);
        assert_eq!(report.geometry_free.complete_pairs, 3);
        assert_eq!(report.geometry_free.unavailable, 0);
        assert_eq!(report.geometry_free.insufficient_history, 1);
        assert_eq!(report.geometry_free.ionosphere_drift, 1);
        assert_eq!(report.geometry_free.cycle_slip_suspects, 1);
        assert!(report.geometry_free.max_abs_delta_m.expect("max delta") > 0.2);
    }

    #[test]
    fn validation_report_summarizes_melbourne_wubbena_dynamics() {
        let report = build_validation_report(
            &[],
            &[
                dual_frequency_epoch(
                    60,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            21_999_999.0,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            22_000_000.5,
                        ),
                    ],
                ),
                dual_frequency_epoch(
                    61,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            21_999_999.01,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            22_000_000.49,
                        ),
                    ],
                ),
                dual_frequency_epoch(
                    62,
                    vec![
                        dual_frequency_satellite_with_phase(
                            SignalBand::L1,
                            SignalCode::Ca,
                            22_000_005.5,
                        ),
                        dual_frequency_satellite_with_phase(
                            SignalBand::L2,
                            SignalCode::Py,
                            22_000_000.5,
                        ),
                    ],
                ),
            ],
            &[
                fixture_solution(60, 1.0, 0.5, 4),
                fixture_solution(61, 1.0, 0.5, 4),
                fixture_solution(62, 1.0, 0.5, 4),
            ],
            &[],
            1.0,
            true,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("validation report");

        assert_eq!(report.melbourne_wubbena.observations, 3);
        assert_eq!(report.melbourne_wubbena.complete_pairs, 3);
        assert_eq!(report.melbourne_wubbena.unavailable, 0);
        assert_eq!(report.melbourne_wubbena.insufficient_history, 1);
        assert_eq!(report.melbourne_wubbena.nominal, 1);
        assert_eq!(report.melbourne_wubbena.wide_lane_slip_suspects, 1);
        assert!(
            report.melbourne_wubbena.max_abs_delta_wide_lane_cycles.expect("max wide-lane delta")
                >= 0.5
        );
    }

    #[derive(Debug, Deserialize)]
    struct ScienceFixtureCase {
        name: String,
        pdop: f64,
        rms_m: f64,
        used_sat_count: usize,
        lock_ratio: f64,
        expected_class: String,
    }

    fn fixture_solution(
        epoch_idx: u64,
        pdop: f64,
        rms_m: f64,
        used_sat_count: usize,
    ) -> NavSolutionEpoch {
        NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: epoch_idx },
            t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::from_sample_index(
                epoch_idx, 1.0,
            ),
            ecef_x_m: bijux_gnss_core::api::Meters(0.0),
            ecef_y_m: bijux_gnss_core::api::Meters(0.0),
            ecef_z_m: bijux_gnss_core::api::Meters(0.0),
            position_covariance_ecef_m2: None,
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: bijux_gnss_core::api::Meters(0.0),
            clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
            clock_bias_m: bijux_gnss_core::api::Meters(0.0),
            clock_drift_s_per_s: 0.0,
            pdop,
            pre_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
            post_fit_residual_rms_m: Some(bijux_gnss_core::api::Meters(rms_m)),
            rms_m: bijux_gnss_core::api::Meters(rms_m),
            status: SolutionStatus::CodeOnly,
            quality: SolutionStatus::CodeOnly.quality_flag(),
            validity: bijux_gnss_core::api::SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            sigma_e_m: Some(bijux_gnss_core::api::Meters(1.0)),
            sigma_n_m: Some(bijux_gnss_core::api::Meters(1.0)),
            sigma_u_m: Some(bijux_gnss_core::api::Meters(1.0)),
            horizontal_error_ellipse_major_axis_m: Some(bijux_gnss_core::api::Meters(1.0)),
            horizontal_error_ellipse_minor_axis_m: Some(bijux_gnss_core::api::Meters(0.5)),
            horizontal_error_ellipse_azimuth_deg: Some(0.0),
            sigma_h_m: Some(bijux_gnss_core::api::Meters(1.0)),
            sigma_v_m: Some(bijux_gnss_core::api::Meters(1.0)),
            residuals: Vec::new(),
            constellation_residual_rms: Vec::new(),
            isb: Vec::new(),
            health: Vec::new(),
            innovation_rms_m: None,
            normalized_innovation_rms: None,
            normalized_innovation_max: None,
            ekf_innovation_rms: None,
            ekf_condition_number: None,
            wls_solver_rank: None,
            wls_condition_number: None,
            ekf_whiteness_ratio: None,
            ekf_predicted_variance: None,
            ekf_observed_variance: None,
            integrity_hpl_m: None,
            integrity_vpl_m: None,
            model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: bijux_gnss_core::api::NavLifecycleState::CodeOnly,
            uncertainty_class: bijux_gnss_core::api::NavUncertaintyClass::Medium,
            assumptions: Some(NavAssumptions {
                time_system: "gps".to_string(),
                reference_frame: "ecef_wgs84".to_string(),
                clock_model: "receiver_clock_bias_drift_linear".to_string(),
                ephemeris_source: "broadcast_lnav".to_string(),
                frame_decode_mode: "lnav".to_string(),
                ephemeris_completeness: "sufficient".to_string(),
                ephemeris_count: used_sat_count,
            }),
            refusal_class: None,
            artifact_id: format!("nav-epoch-{epoch_idx:010}"),
            source_observation_epoch_id: format!("obs-epoch-{epoch_idx:010}"),
            explain_decision: "accepted".to_string(),
            explain_reasons: vec!["fixture".to_string()],
            provenance: None,
            sat_count: used_sat_count,
            used_sat_count,
            rejected_sat_count: 0,
            hdop: Some(pdop),
            vdop: Some(pdop),
            gdop: Some(pdop),
            tdop: Some(pdop / 2.0),
            stability_signature: format!("navsig:v2:fixture:{epoch_idx}"),
            stability_signature_version:
                bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
    }

    fn with_integrity_support(mut solution: NavSolutionEpoch) -> NavSolutionEpoch {
        solution.integrity_hpl_m = Some(10.0);
        solution.integrity_vpl_m = Some(12.0);
        solution
    }

    fn dual_frequency_epoch(epoch_idx: u64, sats: Vec<ObsSatellite>) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: bijux_gnss_core::api::Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
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
            manifest: None,
        }
    }

    fn dual_frequency_satellite(
        band: SignalBand,
        code: SignalCode,
        code_lock: bool,
        carrier_lock: bool,
    ) -> ObsSatellite {
        dual_frequency_satellite_for_constellation(
            Constellation::Gps,
            band,
            code,
            code_lock,
            carrier_lock,
        )
    }

    fn dual_frequency_satellite_for_constellation(
        constellation: Constellation,
        band: SignalBand,
        code: SignalCode,
        code_lock: bool,
        carrier_lock: bool,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat: SatId { constellation, prn: 12 }, band, code },
            pseudorange_m: Meters(22_000_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(200.0),
            carrier_phase_var_cycles2: 0.1,
            doppler_hz: Hertz(15.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags { code_lock, carrier_lock, bit_lock: true, cycle_slip: false },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: Some(45.0),
            azimuth_deg: Some(120.0),
            weight: Some(1.0),
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_registry(constellation, band, code)
                    .expect("dual-frequency signal must exist")
                    .spec,
                doppler_model: "tracked_carrier_hz_minus_intermediate_freq".to_string(),
                observation_lock_state: "locked".to_string(),
                ..ObsMetadata::default()
            },
        }
    }

    fn dual_frequency_satellite_with_phase(
        band: SignalBand,
        code: SignalCode,
        phase_m: f64,
    ) -> ObsSatellite {
        let mut satellite = dual_frequency_satellite(band, code, true, true);
        let wavelength_m = carrier_wavelength_m(satellite.metadata.signal.carrier_hz).0;
        satellite.carrier_phase_cycles = Cycles(phase_m / wavelength_m);
        satellite
    }

    fn carrier_smoothed_code_satellite(smoothing_age: u32, cycle_slip: bool) -> ObsSatellite {
        let mut satellite = dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true);
        satellite.lock_flags.cycle_slip = cycle_slip;
        satellite.metadata.smoothing_window = 8;
        satellite.metadata.smoothing_age = smoothing_age;
        satellite
    }

    fn fixture_track(epoch_idx: u64, lock_ratio: f64) -> TrackingResult {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let total = 8u64;
        let locked = ((total as f64) * lock_ratio).round() as u64;
        let mut epochs = Vec::new();
        for idx in 0..total {
            epochs.push(TrackEpoch {
                epoch: Epoch { index: epoch_idx },
                sample_index: idx,
                sat,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                carrier_hz: Hertz(0.0),
                code_rate_hz: Hertz(1_023_000.0),
                code_phase_samples: Chips(0.0),
                lock: idx < locked,
                cn0_dbhz: 35.0,
                pll_lock: idx < locked,
                dll_lock: idx < locked,
                fll_lock: idx < locked,
                cycle_slip: false,
                nav_bit_lock: false,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: None,
                lock_state: "tracking".to_string(),
                lock_state_reason: None,
                processing_ms: None,
                ..TrackEpoch::default()
            });
        }
        TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "accepted".to_string(),
            acquisition_score: 1.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "tracking".to_string(),
            epochs,
            transitions: Vec::new(),
        }
    }

    #[test]
    fn science_fixture_integrity_classes_are_deterministic() {
        let fixture_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../bijux-gnss-receiver/tests/data/validation/science_iteration07.json");
        let cases: Vec<ScienceFixtureCase> =
            serde_json::from_str(&std::fs::read_to_string(fixture_path).expect("fixture"))
                .expect("cases");
        for (idx, case) in cases.iter().enumerate() {
            let solution = with_integrity_support(fixture_solution(
                idx as u64,
                case.pdop,
                case.rms_m,
                case.used_sat_count,
            ));
            let track = fixture_track(idx as u64, case.lock_ratio);
            let report = build_validation_report(
                &[track],
                &[],
                &[solution],
                &[],
                1.0,
                false,
                Vec::new(),
                ValidationSciencePolicy::default(),
            )
            .expect("report");
            let class = report.integrity.first().expect("integrity row");
            let expected = match case.expected_class.as_str() {
                "missing_integrity_evidence" => NavIntegrityClass::IntegrityEvidenceMissing,
                "weak_geometry" => NavIntegrityClass::WeakGeometry,
                "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
                "unstable_lock" => NavIntegrityClass::UnstableLock,
                other => panic!("unsupported expected class: {other}"),
            };
            assert_eq!(class.class, expected, "case {}", case.name);
        }
    }

    #[test]
    fn validation_policy_marks_high_gdop_as_weak_geometry() {
        let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
        let report = build_validation_report(
            &[fixture_track(0, 1.0)],
            &[],
            &[NavSolutionEpoch { gdop: Some(20.0), ..solution }],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");

        assert_eq!(
            report.integrity.first().expect("integrity row").class,
            NavIntegrityClass::WeakGeometry
        );
    }

    #[test]
    fn validation_policy_requires_integrity_evidence_for_nominal_classification() {
        let solution = fixture_solution(0, 2.0, 1.0, 4);
        let report = build_validation_report(
            &[fixture_track(0, 1.0)],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");

        let integrity = report.integrity.first().expect("integrity row");
        assert_eq!(integrity.class, NavIntegrityClass::IntegrityEvidenceMissing);
        assert_eq!(integrity.reasons, vec!["missing_integrity_evidence".to_string()]);
    }

    #[test]
    fn validation_policy_allows_nominal_when_integrity_evidence_is_present() {
        let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
        let report = build_validation_report(
            &[fixture_track(0, 1.0)],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");

        let integrity = report.integrity.first().expect("integrity row");
        assert_eq!(integrity.class, NavIntegrityClass::Nominal);
        assert!(integrity.reasons.is_empty());
    }
}
