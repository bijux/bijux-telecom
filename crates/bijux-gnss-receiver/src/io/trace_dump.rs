#[cfg(feature = "trace-dump")]
use std::fs;
#[cfg(feature = "trace-dump")]
use std::path::Path;

#[cfg(feature = "trace-dump")]
use serde::Serialize;

use bijux_gnss_core::SatId;

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
