//! Validation report builders for GNSS runs.

use crate::api::ReceiverConfig;
use crate::api::TrackingResult;
use crate::rtk::status::{
    apply_downgrade_policy, evaluate_prerequisites, support_status_matrix, AdvancedMaturity,
    AdvancedMode, AdvancedPrerequisites, AdvancedRefusalClass, AdvancedSolutionClaim,
    AdvancedSupportRow,
};
use crate::validation_helpers::{check_budgets, to_validation_stats};
use bijux_gnss_core::api::{
    check_inter_frequency_alignment, check_solution_consistency, reference_ecef, stats,
    InterFrequencyAlignmentReport, NavSolutionEpoch, ObsEpoch, SatId, SignalBand,
    SolutionConsistencyReport, SolutionStatus, ValidationReferenceEpoch,
};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, ecef_to_enu, PppConfig, PppConvergenceConfig, PppProcessNoise,
    WeightingConfig,
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
    /// Whether the reported PPP path was downgraded.
    pub downgraded: bool,
    /// Downgrade reason when present.
    pub downgrade_reason: Option<String>,
    /// Reported claim class for PPP output.
    pub claim: AdvancedSolutionClaim,
    /// Support matrix row for PPP.
    pub support: AdvancedSupportRow,
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
}

/// Scientific policy used for validation and integrity classification.
#[derive(Debug, Serialize, Clone)]
pub struct ValidationSciencePolicy {
    /// Minimum mean C/N0 expected for stable acceptance.
    pub min_mean_cn0_dbhz: f64,
    /// Maximum PDOP for stable geometry.
    pub max_pdop: f64,
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
pub struct NavResidualReport {
    /// Epoch index.
    pub epoch_idx: u64,
    /// RMS residual (m).
    pub rms_m: f64,
    /// PDOP value.
    pub pdop: f64,
    /// Per-satellite residuals.
    pub residuals: Vec<(SatId, f64)>,
    /// Rejected satellites.
    pub rejected: Vec<SatId>,
}

/// Validation report output structure.
#[derive(Debug, Serialize)]
pub struct ValidationReport {
    /// Total samples counted.
    pub samples: usize,
    /// Number of epochs.
    pub epochs: usize,
    /// Horizontal error statistics.
    pub horiz_error_m: ValidationErrorStats,
    /// Vertical error statistics.
    pub vert_error_m: ValidationErrorStats,
    /// Convergence report.
    pub convergence: ConvergenceReport,
    /// Fix timeline.
    pub fix_timeline: Vec<FixTimelineEntry>,
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
    /// Consistency warnings.
    pub consistency_warnings: Vec<String>,
    /// Inter-frequency alignment report.
    pub inter_frequency_alignment: InterFrequencyAlignmentReport,
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
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }

    let mut horiz_errors = Vec::new();
    let mut vert_errors = Vec::new();
    let mut horiz_by_time = Vec::new();
    let mut vert_by_time = Vec::new();
    let mut fix_timeline = Vec::new();
    let mut residuals = Vec::new();
    let mut nees_values = Vec::new();

    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = reference_ecef(r);
            let dx = sol.ecef_x_m.0 - x;
            let dy = sol.ecef_y_m.0 - y;
            let dz = sol.ecef_z_m.0 - z;
            let horiz = (dx * dx + dy * dy).sqrt();
            let vert = dz.abs();
            horiz_errors.push(horiz);
            vert_errors.push(vert);
            horiz_by_time.push((sol.t_rx_s.0, horiz));
            vert_by_time.push((sol.t_rx_s.0, vert));
            if let (Some(sig_h), Some(sig_v)) = (sol.sigma_h_m, sol.sigma_v_m) {
                if sig_h.0 > 0.0 && sig_v.0 > 0.0 {
                    let (e, n, u) = ecef_to_enu(
                        sol.ecef_x_m.0,
                        sol.ecef_y_m.0,
                        sol.ecef_z_m.0,
                        r.latitude_deg,
                        r.longitude_deg,
                        r.altitude_m,
                    );
                    let nees =
                        (e * e + n * n) / (sig_h.0 * sig_h.0) + (u * u) / (sig_v.0 * sig_v.0);
                    nees_values.push(nees);
                }
            }
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
            pdop: sol.pdop,
            residuals: per_sat,
            rejected,
        });
    }

    let horiz_stats = to_validation_stats(stats(&horiz_errors));
    let vert_stats = to_validation_stats(stats(&vert_errors));
    let convergence = convergence_report(&horiz_by_time, &vert_by_time);
    let budgets = ValidationBudgets::default();
    let violations = check_budgets(tracks, solutions, &budgets);
    let time_consistency = check_time_consistency(tracks, sample_rate_hz);
    let consistency = check_solution_consistency(solutions);
    let inter_frequency_alignment = check_inter_frequency_alignment(obs);
    let multi_freq_present = obs.iter().any(|e| {
        let mut by_sat: std::collections::BTreeMap<SatId, std::collections::BTreeSet<_>> =
            std::collections::BTreeMap::new();
        for sat in &e.sats {
            by_sat.entry(sat.signal_id.sat).or_default().insert(sat.signal_id.band);
        }
        by_sat.values().any(|bands| bands.len() > 1)
    });
    let combos = combinations_from_obs_epochs(obs, SignalBand::L1, SignalBand::L2);
    let combinations_valid = combos.iter().all(|c| c.status == "ok") && !combos.is_empty();
    let support_matrix = support_status_matrix();
    let ppp_support =
        support_matrix.rows.iter().find(|row| row.mode == AdvancedMode::Ppp).cloned().unwrap_or(
            AdvancedSupportRow {
                mode: AdvancedMode::Ppp,
                maturity: AdvancedMaturity::Scaffolding,
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
    let (_ppp_status, downgraded, downgrade_reason, claim) = apply_downgrade_policy(
        AdvancedMode::Ppp,
        &ppp_prereq_decision,
        AdvancedSolutionClaim::Scaffolding,
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
    let nees_mean = if nees_values.is_empty() {
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
    let lock_ratio_by_epoch = lock_ratio_by_epoch(tracks);
    let integrity = classify_integrity(solutions, &lock_ratio_by_epoch, &science_policy);
    let assumptions = summarize_assumptions(solutions);
    let diagnostic_partition =
        partition_diagnostics(&time_consistency.warnings, &consistency_warnings, solutions);

    Ok(ValidationReport {
        samples: tracks.iter().map(|t| t.epochs.len()).sum(),
        epochs: solutions.len(),
        horiz_error_m: horiz_stats,
        vert_error_m: vert_stats,
        convergence,
        fix_timeline,
        residuals,
        time_consistency,
        consistency,
        budgets,
        budget_violations: violations,
        nis_mean,
        nees_mean,
        consistency_warnings,
        inter_frequency_alignment,
        ppp_readiness: PppReadinessReport {
            multi_freq_present,
            combinations_valid,
            products_ok,
            product_fallbacks,
            maturity: ppp_support.maturity,
            prerequisites_met: ppp_prereq_decision.ready,
            refusal_class: ppp_prereq_decision.refusal_class,
            downgraded,
            downgrade_reason,
            claim,
            support: ppp_support,
        },
        science_policy,
        integrity,
        diagnostic_partition,
        assumptions,
    })
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
        if solution.used_sat_count < science_policy.min_used_satellites
            || solution.pdop > science_policy.max_pdop
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
        process_noise: PppProcessNoise {
            clock_drift_s: p.noise_clock_drift,
            ztd_m: p.noise_ztd,
            iono_m: p.noise_iono,
            ambiguity_cycles: p.noise_ambiguity,
        },
        weighting: WeightingConfig {
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
        Chips, Constellation, Epoch, Hertz, NavAssumptions, SatId, TrackEpoch,
    };
    use serde::Deserialize;

    #[test]
    fn golden_reference_validation() {
        let sol = NavSolutionEpoch {
            epoch: bijux_gnss_core::api::Epoch { index: 0 },
            t_rx_s: bijux_gnss_core::api::Seconds(0.0),
            ecef_x_m: bijux_gnss_core::api::Meters(1.0),
            ecef_y_m: bijux_gnss_core::api::Meters(2.0),
            ecef_z_m: bijux_gnss_core::api::Meters(3.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: bijux_gnss_core::api::Meters(0.0),
            clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
            clock_drift_s_per_s: 0.0,
            pdop: 1.0,
            rms_m: bijux_gnss_core::api::Meters(0.0),
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
            model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: bijux_gnss_core::api::NavLifecycleState::Converged,
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
            stability_signature: "navsig:v1:golden".to_string(),
            stability_signature_version:
                bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        };
        let reference = ValidationReferenceEpoch {
            epoch_idx: 0,
            t_rx_s: Some(0.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: 0.0,
            ecef_x_m: Some(1.0),
            ecef_y_m: Some(2.0),
            ecef_z_m: Some(3.0),
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
        assert_eq!(report.integrity.len(), 1);
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
            ecef_x_m: bijux_gnss_core::api::Meters(0.0),
            ecef_y_m: bijux_gnss_core::api::Meters(0.0),
            ecef_z_m: bijux_gnss_core::api::Meters(0.0),
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            altitude_m: bijux_gnss_core::api::Meters(0.0),
            clock_bias_s: bijux_gnss_core::api::Seconds(0.0),
            clock_drift_s_per_s: 0.0,
            pdop,
            rms_m: bijux_gnss_core::api::Meters(rms_m),
            status: SolutionStatus::Converged,
            quality: SolutionStatus::Converged.quality_flag(),
            validity: bijux_gnss_core::api::SolutionValidity::Stable,
            valid: true,
            processing_ms: None,
            sigma_h_m: Some(bijux_gnss_core::api::Meters(1.0)),
            sigma_v_m: Some(bijux_gnss_core::api::Meters(1.0)),
            residuals: Vec::new(),
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
            model_version: bijux_gnss_core::api::NAV_SOLUTION_MODEL_VERSION,
            lifecycle_state: bijux_gnss_core::api::NavLifecycleState::Converged,
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
            stability_signature: format!("navsig:v1:fixture:{epoch_idx}"),
            stability_signature_version:
                bijux_gnss_core::api::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
        }
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
            let solution = fixture_solution(idx as u64, case.pdop, case.rms_m, case.used_sat_count);
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
                "weak_geometry" => NavIntegrityClass::WeakGeometry,
                "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
                "unstable_lock" => NavIntegrityClass::UnstableLock,
                other => panic!("unsupported expected class: {other}"),
            };
            assert_eq!(class.class, expected, "case {}", case.name);
        }
    }
}
