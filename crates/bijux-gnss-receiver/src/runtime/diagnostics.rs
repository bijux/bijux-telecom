//! Diagnostics dump helpers.
#![allow(missing_docs)]

use bijux_gnss_core::api::{NavSolutionEpoch, ObsEpoch};
use serde::Serialize;
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Serialize)]
struct DiagnosticsDump<'a> {
    run_id: String,
    obs_tail: Vec<&'a ObsEpoch>,
    nav_tail: Vec<&'a NavSolutionEpoch>,
    residual_stats: Option<ResidualStats>,
    config_snapshot: Option<serde_json::Value>,
    note: String,
}

#[derive(Debug, Serialize)]
struct ResidualStats {
    count: usize,
    rejected: usize,
    mean_abs_m: f64,
    max_abs_m: f64,
}

pub fn dump_on_error(note: &str, obs: Option<&ObsEpoch>, nav: Option<&NavSolutionEpoch>) {
    if std::env::var("BIJUX_DIAGNOSTICS_DUMP").ok().as_deref() != Some("1") {
        return;
    }
    let run_id = std::env::var("BIJUX_RUN_ID").unwrap_or_else(|_| "unknown".to_string());
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

    let config_snapshot = std::env::var("BIJUX_RUN_DIR")
        .ok()
        .map(PathBuf::from)
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
    let path = std::env::var("BIJUX_RUN_DIR")
        .ok()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    let file = path.join("diagnostics_dump.json");
    if let Ok(data) = serde_json::to_string_pretty(&dump) {
        let _ = fs::write(file, data);
    }
}
