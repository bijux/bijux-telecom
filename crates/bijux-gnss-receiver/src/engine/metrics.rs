//! Metrics summary output.
#![allow(missing_docs)]

use crate::api::RunArtifacts;
use crate::engine::runtime::ReceiverRuntime;
use serde::Serialize;
use std::fs;

#[derive(Debug, Serialize)]
struct MetricsSummary {
    acquisition_peak_mean_ratio_avg: Option<f32>,
    tracking_lock_ratio: Option<f64>,
    cn0_dbhz_mean: Option<f64>,
    cn0_dbhz_min: Option<f64>,
    cn0_dbhz_max: Option<f64>,
    nav_rms_m_mean: Option<f64>,
    nav_outlier_count: u64,
}

pub fn write_metrics_summary(runtime: &ReceiverRuntime, artifacts: &RunArtifacts) {
    let run_dir = match runtime.config.run_dir.as_ref() {
        Some(dir) => dir.clone(),
        None => return,
    };

    let acquisition_peak_mean_ratio_avg = if artifacts.acquisitions.is_empty() {
        None
    } else {
        let sum: f32 = artifacts
            .acquisitions
            .iter()
            .map(|a| a.peak_mean_ratio)
            .sum();
        Some(sum / artifacts.acquisitions.len() as f32)
    };

    let mut total_epochs = 0u64;
    let mut locked_epochs = 0u64;
    let mut cn0_sum = 0.0f64;
    let mut cn0_min = f64::INFINITY;
    let mut cn0_max = f64::NEG_INFINITY;
    for track in &artifacts.tracking {
        for epoch in &track.epochs {
            total_epochs += 1;
            if epoch.lock {
                locked_epochs += 1;
            }
            cn0_sum += epoch.cn0_dbhz;
            cn0_min = cn0_min.min(epoch.cn0_dbhz);
            cn0_max = cn0_max.max(epoch.cn0_dbhz);
        }
    }
    let tracking_lock_ratio = if total_epochs == 0 {
        None
    } else {
        Some(locked_epochs as f64 / total_epochs as f64)
    };
    let cn0_dbhz_mean = if total_epochs == 0 {
        None
    } else {
        Some(cn0_sum / total_epochs as f64)
    };
    let cn0_dbhz_min = if total_epochs == 0 {
        None
    } else {
        Some(cn0_min)
    };
    let cn0_dbhz_max = if total_epochs == 0 {
        None
    } else {
        Some(cn0_max)
    };

    let nav_rms_m_mean = if artifacts.navigation.is_empty() {
        None
    } else {
        let sum: f64 = artifacts.navigation.iter().map(|n| n.rms_m.0).sum();
        Some(sum / artifacts.navigation.len() as f64)
    };
    let nav_outlier_count = artifacts
        .navigation
        .iter()
        .map(|n| n.residuals.iter().filter(|r| r.rejected).count() as u64)
        .sum();

    let summary = MetricsSummary {
        acquisition_peak_mean_ratio_avg,
        tracking_lock_ratio,
        cn0_dbhz_mean,
        cn0_dbhz_min,
        cn0_dbhz_max,
        nav_rms_m_mean,
        nav_outlier_count,
    };

    let file = run_dir.join("metrics_summary.json");
    if let Ok(json) = serde_json::to_string_pretty(&summary) {
        let _ = fs::write(file, json);
    }
}
