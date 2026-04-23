//! Validation helper routines split out to keep report file size manageable.
#![allow(missing_docs)]

use crate::pipeline::tracking::TrackingResult;
use crate::validation_report::{ValidationBudgets, ValidationErrorStats};
use bijux_gnss_core::api::{NavSolutionEpoch, StatsSummary};

pub(crate) fn check_budgets(
    tracks: &[TrackingResult],
    solutions: &[NavSolutionEpoch],
    budgets: &ValidationBudgets,
) -> Vec<String> {
    let mut violations = Vec::new();
    for track in tracks {
        if track.epochs.len() > 3 {
            let carriers: Vec<f64> = track.epochs.iter().map(|e| e.carrier_hz.0).collect();
            let mean = carriers.iter().sum::<f64>() / carriers.len() as f64;
            let var = carriers.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>()
                / carriers.len() as f64;
            let std = var.sqrt();
            if std > budgets.tracking_carrier_jitter_hz {
                violations.push(format!(
                    "tracking jitter too high for PRN {}: {:.1} Hz",
                    track.sat.prn, std
                ));
            }
        }
    }
    for sol in solutions {
        if sol.residuals.len() >= 4 && sol.rms_m.0.is_nan() {
            violations.push(format!("PVT RMS is NaN at epoch {}", sol.epoch.index));
        }
        if sol.rms_m.0 > budgets.nav_residual_rms_m_max {
            violations.push(format!(
                "nav residual RMS too high at epoch {}: {:.2} m",
                sol.epoch.index, sol.rms_m.0
            ));
        }
        let rejected = sol.residuals.iter().filter(|r| r.rejected).count();
        if !sol.residuals.is_empty() {
            let ratio = rejected as f64 / sol.residuals.len() as f64;
            if ratio > budgets.nav_rejected_ratio_max {
                violations.push(format!(
                    "nav rejected ratio too high at epoch {}: {:.2}",
                    sol.epoch.index, ratio
                ));
            }
        }
    }
    let mut nan_count = 0usize;
    for sol in solutions {
        if !sol.ecef_x_m.0.is_finite() || !sol.ecef_y_m.0.is_finite() || !sol.ecef_z_m.0.is_finite()
        {
            nan_count += 1;
        }
    }
    if nan_count > budgets.nav_nan_max {
        violations.push(format!("nav NaN count too high: {nan_count} > {}", budgets.nav_nan_max));
    }
    if budgets.nav_min_lock_epochs > 0 && !solutions.is_empty() {
        let mut lock_ok = false;
        for track in tracks {
            let mut streak = 0u64;
            for epoch in &track.epochs {
                if epoch.lock {
                    streak += 1;
                } else {
                    streak = 0;
                }
                if streak >= budgets.nav_min_lock_epochs {
                    lock_ok = true;
                    break;
                }
            }
            if lock_ok {
                break;
            }
        }
        if !lock_ok {
            violations.push(format!(
                "no tracking lock streak >= {} epochs before nav",
                budgets.nav_min_lock_epochs
            ));
        }
    }
    violations
}

pub(crate) fn to_validation_stats(summary: StatsSummary) -> ValidationErrorStats {
    ValidationErrorStats {
        count: summary.count,
        mean: summary.mean,
        median: summary.median,
        rms: summary.rms,
        p95: summary.p95,
        max: summary.max,
    }
}

impl Default for ValidationBudgets {
    fn default() -> Self {
        Self {
            acq_doppler_hz: 500.0,
            acq_code_phase_samples: 5.0,
            tracking_carrier_jitter_hz: 50.0,
            ephemeris_parity_rate_min: 0.9,
            pvt_max_iterations: 10,
            nav_residual_rms_m_max: 50.0,
            nav_rejected_ratio_max: 0.5,
            nav_nan_max: 0,
            nav_min_lock_epochs: 3,
        }
    }
}
