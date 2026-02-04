//! Validation report builders for GNSS runs.

use crate::api::TrackingResult;
use crate::api::ReceiverConfig;
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
pub fn build_validation_report(
    tracks: &[TrackingResult],
    obs: &[ObsEpoch],
    solutions: &[NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    sample_rate_hz: f64,
    products_ok: bool,
    product_fallbacks: Vec<String>,
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
            by_sat
                .entry(sat.signal_id.sat)
                .or_default()
                .insert(sat.signal_id.band);
        }
        by_sat.values().any(|bands| bands.len() > 1)
    });
    let combos = combinations_from_obs_epochs(obs, SignalBand::L1, SignalBand::L2);
    let combinations_valid = combos.iter().all(|c| c.status == "ok") && !combos.is_empty();
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
        },
    })
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
        .map(|c| {
            (
                c.time_to_first_meter_s,
                c.time_to_decimeter_s,
                c.time_to_centimeter_s,
            )
        })
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
        let report =
            build_validation_report(&[], &[], &[sol], &[reference], 1.0, false, Vec::new())
                .expect("validation report");
        assert!(report.horiz_error_m.rms <= 1e-6);
        assert!(report.vert_error_m.rms <= 1e-6);
    }
}
