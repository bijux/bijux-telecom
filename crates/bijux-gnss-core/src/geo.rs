//! Geodetic and reference frame utilities (WGS-84).

/// Convert ECEF to geodetic (lat, lon, alt).
pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);

    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - e2));
    let mut alt = 0.0;
    for _ in 0..5 {
        let sin_lat = lat.sin();
        let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
        alt = p / lat.cos() - n;
        lat = z.atan2(p * (1.0 - e2 * n / (n + alt)));
    }
    (lat.to_degrees(), lon.to_degrees(), alt)
}

/// Convert geodetic (lat, lon, alt) to ECEF.
pub fn geodetic_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
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

/// Convert ECEF to local ENU.
pub fn ecef_to_enu(
    x: f64,
    y: f64,
    z: f64,
    ref_lat_deg: f64,
    ref_lon_deg: f64,
    ref_alt_m: f64,
) -> (f64, f64, f64) {
    let (xr, yr, zr) = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m);
    let dx = x - xr;
    let dy = y - yr;
    let dz = z - zr;
    let lat = ref_lat_deg.to_radians();
    let lon = ref_lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();
    let east = -sin_lon * dx + cos_lon * dy;
    let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    (east, north, up)
}

/// Compute elevation and azimuth (degrees).
pub fn elevation_azimuth_deg(
    rx_x: f64,
    rx_y: f64,
    rx_z: f64,
    sat_x: f64,
    sat_y: f64,
    sat_z: f64,
) -> (f64, f64) {
    let (lat, lon, alt) = ecef_to_geodetic(rx_x, rx_y, rx_z);
    let (e, n, u) = ecef_to_enu(sat_x, sat_y, sat_z, lat, lon, alt);
    let az = e.atan2(n).to_degrees().rem_euclid(360.0);
    let el = (u / (e * e + n * n + u * u).sqrt()).asin().to_degrees();
    (az, el)
}

