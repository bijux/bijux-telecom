//! Statistical helpers.

/// Summary statistics for error distributions.
#[derive(Debug, Clone)]
pub struct StatsSummary {
    /// Count of values.
    pub count: usize,
    /// Mean value.
    pub mean: f64,
    /// Median value.
    pub median: f64,
    /// Root-mean-square value.
    pub rms: f64,
    /// 95th percentile.
    pub p95: f64,
    /// Maximum value.
    pub max: f64,
}

/// Compute stats for a collection of values.
pub fn stats(values: &[f64]) -> StatsSummary {
    if values.is_empty() {
        return StatsSummary {
            count: 0,
            mean: 0.0,
            median: 0.0,
            rms: 0.0,
            p95: 0.0,
            max: 0.0,
        };
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let count = values.len();
    let mean = values.iter().sum::<f64>() / count as f64;
    let rms = (values.iter().map(|v| v * v).sum::<f64>() / count as f64).sqrt();
    let median = sorted[count / 2];
    let p95 = sorted[(count as f64 * 0.95).floor().min((count - 1) as f64) as usize];
    let max = *sorted.last().unwrap_or(&0.0);
    StatsSummary {
        count,
        mean,
        median,
        rms,
        p95,
        max,
    }
}

/// Convert LLA to ECEF (WGS-84).
pub fn lla_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_m) * cos_lat * lon.cos();
    let y = (n + alt_m) * cos_lat * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * sin_lat;
    (x, y, z)
}
