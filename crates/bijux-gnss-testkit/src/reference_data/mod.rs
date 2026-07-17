//! Reference catalogs and benchmark reports used by validation fixtures.

/// Stable CSV header used by station and surveyed reference-coordinate catalogs.
pub const REFERENCE_COORDINATE_CSV_HEADER: &str =
    "marker_name,fixture_name,lat_deg,lon_deg,alt_m,source";

pub mod ppp_convergence;
pub mod reference_coordinate;
pub mod station_truth;
pub mod troposphere_elevation;
