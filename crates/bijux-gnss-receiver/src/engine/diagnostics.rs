//! Diagnostics dump helpers.
#![allow(missing_docs)]

use crate::engine::runtime::ReceiverRuntime;
use bijux_gnss_core::api::{NavSolutionEpoch, ObsEpoch};
#[cfg(feature = "trace-heavy")]
use serde::Serialize;
#[cfg(feature = "trace-heavy")]
use std::fs;
#[cfg(feature = "trace-heavy")]
use std::path::PathBuf;

#[cfg(feature = "trace-heavy")]
#[derive(Debug, Serialize)]
struct DiagnosticsDump<'a> {
    run_id: String,
    obs_tail: Vec<&'a ObsEpoch>,
    nav_tail: Vec<&'a NavSolutionEpoch>,
    residual_stats: Option<ResidualStats>,
    config_snapshot: Option<serde_json::Value>,
    note: String,
}

#[cfg(feature = "trace-heavy")]
#[derive(Debug, Serialize)]
struct ResidualStats {
    count: usize,
    rejected: usize,
    mean_abs_m: f64,
    max_abs_m: f64,
}

#[cfg(feature = "trace-heavy")]
pub fn dump_on_error(
    runtime: &ReceiverRuntime,
    note: &str,
    obs: Option<&ObsEpoch>,
    nav: Option<&NavSolutionEpoch>,
) {
    if !runtime.config.diagnostics_dump {
        return;
    }
    let run_id = runtime
        .config
        .run_id
        .clone()
        .unwrap_or_else(|| "unknown".to_string());
    let obs_tail = obs.into_iter().collect::<Vec<_>>();
    let nav_tail = nav.into_iter().collect::<Vec<_>>();
    let residual_stats = nav.and_then(|nav| {
        if nav.residuals.is_empty() {
            return None;
        }
        let mut sum = 0.0f64;
        let mut max = 0.0f64;
        let mut rejected = 0usize;
        for residual in &nav.residuals {
            let value = residual.residual_m.0.abs();
            sum += value;
            max = max.max(value);
            if residual.rejected {
                rejected += 1;
            }
        }
        Some(ResidualStats {
            count: nav.residuals.len(),
            rejected,
            mean_abs_m: sum / nav.residuals.len() as f64,
            max_abs_m: max,
        })
    });

    let config_snapshot = runtime
        .config
        .run_dir
        .clone()
        .and_then(|dir| fs::read_to_string(dir.join("config.json")).ok())
        .and_then(|data| serde_json::from_str(&data).ok());

    let dump = DiagnosticsDump {
        run_id,
        obs_tail,
        nav_tail,
        residual_stats,
        config_snapshot,
        note: note.to_string(),
    };
    let path = runtime
        .config
        .run_dir
        .clone()
        .unwrap_or_else(|| PathBuf::from("."));
    let file = path.join("diagnostics_dump.json");
    if let Ok(data) = serde_json::to_string_pretty(&dump) {
        let _ = fs::write(file, data);
    }
}

#[cfg(not(feature = "trace-heavy"))]
pub fn dump_on_error(
    _runtime: &ReceiverRuntime,
    _note: &str,
    _obs: Option<&ObsEpoch>,
    _nav: Option<&NavSolutionEpoch>,
) {
}
