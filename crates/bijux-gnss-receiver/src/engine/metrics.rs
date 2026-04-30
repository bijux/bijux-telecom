//! Metrics summary output.
#![allow(missing_docs)]

use crate::api::RunArtifacts;
use crate::engine::runtime::ReceiverRuntime;
use crate::pipeline::acquisition::AcquisitionStats;
use bijux_gnss_core::api::TrackEpoch;
use serde::Serialize;
use std::fs;

#[derive(Debug, Serialize)]
struct MetricsSummary {
    acquisition_peak_mean_ratio_avg: Option<f32>,
    acquisition_sat_count: u64,
    acquisition_doppler_bins: u64,
    acquisition_code_search_bins: u64,
    acquisition_cache_hits: u64,
    acquisition_cache_misses: u64,
    acquisition_accepted_count: u64,
    acquisition_ambiguous_count: u64,
    acquisition_rejected_count: u64,
    acquisition_deferred_count: u64,
    tracking_lock_ratio: Option<f64>,
    cn0_dbhz_mean: Option<f64>,
    cn0_dbhz_min: Option<f64>,
    cn0_dbhz_max: Option<f64>,
    tracking_cycle_slip_count: u64,
    tracking_anti_false_lock_count: u64,
    tracking_lock_state_unknown_count: u64,
    tracking_lock_quality_mean: Option<f64>,
    tracking_lock_quality_min: Option<f64>,
    tracking_lock_quality_max: Option<f64>,
    observation_count: u64,
    observation_satellite_count: u64,
    observation_lock_quality_mean: Option<f64>,
    observation_lock_quality_min: Option<f64>,
    observation_lock_quality_max: Option<f64>,
    observation_lock_quality_sample_count: u64,
    nav_rms_m_mean: Option<f64>,
    nav_outlier_count: u64,
}

pub fn write_metrics_summary(
    runtime: &ReceiverRuntime,
    artifacts: &RunArtifacts,
    acquisition_stats: AcquisitionStats,
) {
    let run_dir = match runtime.config.run_dir.as_ref() {
        Some(dir) => dir.clone(),
        None => return,
    };

    let acquisition_peak_mean_ratio_avg = if artifacts.acquisitions.is_empty() {
        None
    } else {
        let sum: f32 = artifacts.acquisitions.iter().map(|a| a.peak_mean_ratio).sum();
        Some(sum / artifacts.acquisitions.len() as f32)
    };

    let mut total_epochs = 0u64;
    let mut locked_epochs = 0u64;
    let mut tracking_cycle_slip_count = 0u64;
    let mut tracking_anti_false_lock_count = 0u64;
    let mut tracking_lock_state_unknown_count = 0u64;
    let mut tracking_lock_quality_sum = 0.0f64;
    let mut tracking_lock_quality_min = f64::INFINITY;
    let mut tracking_lock_quality_max = f64::NEG_INFINITY;
    let mut tracking_lock_quality_count = 0u64;
    let mut cn0_sum = 0.0f64;
    let mut cn0_min = f64::INFINITY;
    let mut cn0_max = f64::NEG_INFINITY;
    for track in &artifacts.tracking {
        for epoch in &track.epochs {
            let q = tracking_lock_quality(epoch);
            tracking_lock_quality_sum += q;
            tracking_lock_quality_count += 1;
            tracking_lock_quality_min = tracking_lock_quality_min.min(q);
            tracking_lock_quality_max = tracking_lock_quality_max.max(q);
            if !epoch.lock {
                tracking_lock_state_unknown_count =
                    tracking_lock_state_unknown_count.saturating_add(1);
            }
            if epoch.cycle_slip {
                tracking_cycle_slip_count = tracking_cycle_slip_count.saturating_add(1);
            }
            if epoch.anti_false_lock {
                tracking_anti_false_lock_count = tracking_anti_false_lock_count.saturating_add(1);
            }
        }
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
    let tracking_lock_ratio =
        if total_epochs == 0 { None } else { Some(locked_epochs as f64 / total_epochs as f64) };
    let cn0_dbhz_mean = if total_epochs == 0 { None } else { Some(cn0_sum / total_epochs as f64) };
    let cn0_dbhz_min = if total_epochs == 0 { None } else { Some(cn0_min) };
    let cn0_dbhz_max = if total_epochs == 0 { None } else { Some(cn0_max) };
    let tracking_lock_quality_mean = if tracking_lock_quality_count == 0 {
        None
    } else {
        Some(tracking_lock_quality_sum / tracking_lock_quality_count as f64)
    };
    let tracking_lock_quality_min =
        if tracking_lock_quality_count == 0 { None } else { Some(tracking_lock_quality_min) };
    let tracking_lock_quality_max =
        if tracking_lock_quality_count == 0 { None } else { Some(tracking_lock_quality_max) };

    let mut observation_count = 0u64;
    let mut observation_satellite_count = 0u64;
    let mut observation_lock_quality_sum = 0.0f64;
    let mut observation_lock_quality_min = f64::INFINITY;
    let mut observation_lock_quality_max = f64::NEG_INFINITY;
    let mut observation_lock_quality_count = 0u64;
    for epoch in &artifacts.observations {
        observation_count += 1;
        for sat in &epoch.sats {
            observation_satellite_count += 1;
            let quality = sat.metadata.tracking_lock_quality;
            observation_lock_quality_sum += quality;
            observation_lock_quality_count += 1;
            observation_lock_quality_min = observation_lock_quality_min.min(quality);
            observation_lock_quality_max = observation_lock_quality_max.max(quality);
        }
    }
    let observation_lock_quality_mean = if observation_lock_quality_count == 0 {
        None
    } else {
        Some(observation_lock_quality_sum / observation_lock_quality_count as f64)
    };
    let observation_lock_quality_min =
        if observation_lock_quality_count == 0 { None } else { Some(observation_lock_quality_min) };
    let observation_lock_quality_max =
        if observation_lock_quality_count == 0 { None } else { Some(observation_lock_quality_max) };

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
        acquisition_sat_count: acquisition_stats.sat_count,
        acquisition_doppler_bins: acquisition_stats.doppler_bins,
        acquisition_code_search_bins: acquisition_stats.code_search_bins,
        acquisition_cache_hits: acquisition_stats.cache_hits,
        acquisition_cache_misses: acquisition_stats.cache_misses,
        acquisition_accepted_count: acquisition_stats.accepted_count,
        acquisition_ambiguous_count: acquisition_stats.ambiguous_count,
        acquisition_rejected_count: acquisition_stats.rejected_count,
        acquisition_deferred_count: acquisition_stats.deferred_count,
        tracking_lock_ratio,
        cn0_dbhz_mean,
        cn0_dbhz_min,
        cn0_dbhz_max,
        tracking_cycle_slip_count,
        tracking_anti_false_lock_count,
        tracking_lock_state_unknown_count,
        tracking_lock_quality_mean,
        tracking_lock_quality_min,
        tracking_lock_quality_max,
        observation_count,
        observation_satellite_count,
        observation_lock_quality_mean,
        observation_lock_quality_min,
        observation_lock_quality_max,
        observation_lock_quality_sample_count: observation_lock_quality_count,
        nav_rms_m_mean,
        nav_outlier_count,
    };

    let file = run_dir.join("metrics_summary.json");
    if let Ok(json) = serde_json::to_string_pretty(&summary) {
        let _ = fs::write(file, json);
    }
}

fn tracking_lock_quality(epoch: &TrackEpoch) -> f64 {
    let mut quality = (epoch.cn0_dbhz / 60.0).clamp(0.0, 1.0);
    if epoch.anti_false_lock {
        quality *= 0.5;
    }
    if epoch.cycle_slip {
        quality *= 0.2;
    }
    if !epoch.lock {
        quality *= 0.2;
    }
    if !epoch.pll_lock {
        quality *= 0.7;
    }
    if !epoch.dll_lock {
        quality *= 0.8;
    }
    if !epoch.fll_lock {
        quality *= 0.9;
    }
    quality
}
