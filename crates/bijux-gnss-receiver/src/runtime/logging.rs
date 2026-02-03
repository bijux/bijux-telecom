#![allow(missing_docs)]

#[cfg(feature = "tracing")]
use tracing::{debug, info};

#[cfg(feature = "trace-dump")]
use std::fs;
#[cfg(feature = "trace-dump")]
use std::path::Path;

#[cfg(feature = "trace-dump")]
use serde::Serialize;

use crate::tracking::ChannelState;
use bijux_gnss_core::SatId;

pub fn acquisition_hit(sat: SatId, carrier_hz: f64, code_phase: usize, metric: f32, ratio: f32) {
    #[cfg(feature = "tracing")]
    info!(
        prn = sat.prn,
        carrier_hz, code_phase, metric, ratio, "acquisition hit"
    );
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (sat, carrier_hz, code_phase, metric, ratio);
    }
}

pub fn channel_state_change(channel: u8, from: ChannelState, to: ChannelState) {
    #[cfg(feature = "tracing")]
    info!(channel, from = ?from, to = ?to, "channel state change");
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (channel, from, to);
    }
}

pub fn lock_status(channel: u8, locked: bool) {
    #[cfg(feature = "tracing")]
    debug!(channel, locked, "lock status change");
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (channel, locked);
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
