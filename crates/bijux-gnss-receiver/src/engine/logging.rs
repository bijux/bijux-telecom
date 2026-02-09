#![allow(missing_docs)]

#[cfg(feature = "tracing")]
use tracing::{debug, info};

#[cfg(feature = "trace-dump")]
use std::fs;
#[cfg(feature = "trace-dump")]
use std::path::Path;

#[cfg(feature = "trace-dump")]
use serde::Serialize;

use crate::engine::runtime::ReceiverRuntimeConfig;
use crate::pipeline::tracking::ChannelState;
use bijux_gnss_core::api::{DiagnosticEvent, SatId};

#[cfg(feature = "tracing")]
fn run_id(runtime: &ReceiverRuntimeConfig) -> &str {
    runtime.run_id.as_deref().unwrap_or("unknown")
}

pub fn acquisition_hit(
    runtime: &ReceiverRuntimeConfig,
    sat: SatId,
    carrier_hz: f64,
    code_phase: usize,
    metric: f32,
    ratio: f32,
) {
    #[cfg(feature = "tracing")]
    info!(
        run_id = %run_id(runtime),
        prn = sat.prn,
        carrier_hz, code_phase, metric, ratio, "acquisition hit"
    );
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (runtime, sat, carrier_hz, code_phase, metric, ratio);
    }
}

pub fn channel_state_change(
    runtime: &ReceiverRuntimeConfig,
    channel: u8,
    from: ChannelState,
    to: ChannelState,
) {
    #[cfg(feature = "tracing")]
    info!(
        run_id = %run_id(runtime),
        channel,
        from = ?from,
        to = ?to,
        "channel state change"
    );
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (runtime, channel, from, to);
    }
}

pub fn lock_status(runtime: &ReceiverRuntimeConfig, channel: u8, locked: bool) {
    #[cfg(feature = "tracing")]
    debug!(run_id = %run_id(runtime), channel, locked, "lock status change");
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (runtime, channel, locked);
    }
}

pub fn diagnostic(runtime: &ReceiverRuntimeConfig, event: &DiagnosticEvent) {
    #[cfg(feature = "tracing")]
    {
        match event.severity {
            bijux_gnss_core::api::DiagnosticSeverity::Error => {
                tracing::event!(
                    tracing::Level::ERROR,
                    run_id = %run_id(runtime),
                    code = %event.code,
                    message = %event.message,
                    "diagnostic"
                )
            }
            bijux_gnss_core::api::DiagnosticSeverity::Warning => {
                tracing::event!(
                    tracing::Level::WARN,
                    run_id = %run_id(runtime),
                    code = %event.code,
                    message = %event.message,
                    "diagnostic"
                )
            }
            bijux_gnss_core::api::DiagnosticSeverity::Info => {
                tracing::event!(
                    tracing::Level::INFO,
                    run_id = %run_id(runtime),
                    code = %event.code,
                    message = %event.message,
                    "diagnostic"
                )
            }
        }
    }
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (runtime, event);
    }
}

#[cfg_attr(feature = "trace-dump", derive(Debug, Serialize))]
pub struct AcqTrace {
    pub sat: SatId,
    pub doppler_hz: f64,
    pub code_phase_samples: usize,
    pub peak: f32,
    pub mean: f32,
    pub second_peak: f32,
}

#[cfg(feature = "trace-dump")]
pub fn dump_acq_trace(dir: &Path, trace: &AcqTrace) -> std::io::Result<()> {
    fs::create_dir_all(dir)?;
    let filename = format!(
        "acq_prn{}_doppler{}.json",
        trace.sat.prn, trace.doppler_hz as i64
    );
    let path = dir.join(filename);
    let data = serde_json::to_string_pretty(trace).unwrap_or_else(|_| "{}".to_string());
    fs::write(path, data)
}
