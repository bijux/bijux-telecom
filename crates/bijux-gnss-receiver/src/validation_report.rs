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
    PppConvergenceConfig, PppMeasurementNoise, PppPreciseProductAction, PppPreciseProductPolicy,
    PppProcessNoise, WeightingConfig,
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
                "innovation consistency anomalies: {anomaly_count} event(s), peak NIS {normalized_innovation_squared:.3} outside [{lower_bound:.3}, {upper_bound:.3}] for measurement dimension {measurement_dimension}"
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
            position_m: p.noise_position,
            velocity_mps: p.noise_velocity,
            clock_bias_s: p.noise_clock_bias,
            clock_drift_s: p.noise_clock_drift,
            inter_system_bias_s: p.noise_inter_system_bias,
            ztd_m: p.noise_ztd,
            iono_m: p.noise_iono,
            ambiguity_cycles: p.noise_ambiguity,
        },
        measurement_noise: PppMeasurementNoise {
            code_floor_m: p.measurement_code_floor_m,
            phase_floor_cycles: p.measurement_phase_floor_cycles,
            orbit_sigma_scale: p.measurement_orbit_sigma_scale,
            clock_sigma_scale: p.measurement_clock_sigma_scale,
            troposphere_residual_m: p.measurement_troposphere_residual_m,
            antenna_residual_m: p.measurement_antenna_residual_m,
        },
        precise_product_policy: PppPreciseProductPolicy {
            missing_satellite_action: ppp_precise_product_action(
                &p.precise_product_missing_satellite_action,
            ),
            out_of_coverage_action: ppp_precise_product_action(
                &p.precise_product_out_of_coverage_action,
            ),
            insufficient_support_action: ppp_precise_product_action(
                &p.precise_product_insufficient_support_action,
            ),
            orbit_gap_action: ppp_precise_product_action(&p.precise_product_orbit_gap_action),
            orbit_flag_action: ppp_precise_product_action(&p.precise_product_orbit_flag_action),
            clock_gap_action: ppp_precise_product_action(&p.precise_product_clock_gap_action),
            clock_jump_action: ppp_precise_product_action(&p.precise_product_clock_jump_action),
            satellite_state_inflation: p.precise_product_state_inflation,
        },
        weighting: WeightingConfig {
            model: match profile.navigation.weighting.mode {
                crate::engine::receiver_config::navigation::NavigationWeightingMode::Elevation => {
                    PositionWeightingModel::Elevation
                }
                crate::engine::receiver_config::navigation::NavigationWeightingMode::Cn0 => {
                    PositionWeightingModel::Cn0
                }
                crate::engine::receiver_config::navigation::NavigationWeightingMode::ElevationCn0 => {
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

fn ppp_precise_product_action(value: &str) -> PppPreciseProductAction {
    match value {
        "inflate_satellite_state" => PppPreciseProductAction::InflateSatelliteState,
        "reset_satellite_state" => PppPreciseProductAction::ResetSatelliteState,
        "refuse_satellite" => PppPreciseProductAction::RefuseSatellite,
        _ => PppPreciseProductAction::BridgeWithBroadcast,
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
#[path = "validation_report/tests.rs"]
mod tests;
