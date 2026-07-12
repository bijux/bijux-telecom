//! Public troposphere-validation reporting helpers.

use bijux_gnss_core::api::GpsTime;
use serde::{Deserialize, Serialize};

pub const LOW_ELEVATION_CEILING_DEG: f64 = 20.0;
pub const MID_ELEVATION_CEILING_DEG: f64 = 45.0;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TroposphereElevationBucketReport {
    pub bucket_name: String,
    pub min_elevation_deg: f64,
    pub max_elevation_deg: Option<f64>,
    pub sample_count: usize,
    pub corrected_mean_abs_residual_m: f64,
    pub uncorrected_mean_abs_residual_m: f64,
    pub corrected_rms_residual_m: f64,
    pub uncorrected_rms_residual_m: f64,
    pub mean_abs_improvement_m: f64,
    pub rms_improvement_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicTroposphereElevationEpoch {
    pub gps_time: GpsTime,
    pub compared_satellite_count: usize,
    pub low_elevation_satellite_count: usize,
    pub low_elevation_mean_abs_improvement_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PublicTroposphereElevationReport {
    pub marker_name: String,
    pub fixture_name: String,
    pub compared_epoch_count: usize,
    pub compared_sample_count: usize,
    pub low_elevation_sample_count: usize,
    pub overall_mean_abs_improvement_m: f64,
    pub overall_rms_improvement_m: f64,
    pub buckets: Vec<TroposphereElevationBucketReport>,
    pub epochs: Vec<PublicTroposphereElevationEpoch>,
}
