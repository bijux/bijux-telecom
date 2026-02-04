//! Geodetic and reference frame utilities (WGS-84).

/// GPS week number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct GpsWeek(pub u16);

/// GPS time-of-week in seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tow(pub f64);

/// GPS time (week + time-of-week).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsTime {
    pub week: GpsWeek,
    pub tow: Tow,
}

/// ECEF coordinate (meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ecef {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
}

/// Geodetic coordinate (degrees, meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Llh {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

/// Local ENU coordinate (meters).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Enu {
    pub e_m: f64,
    pub n_m: f64,
    pub u_m: f64,
}

/// Convert ECEF to geodetic.
pub fn ecef_to_llh(ecef: Ecef) -> Llh {
    let (lat, lon, alt) = ecef_to_geodetic(ecef.x_m, ecef.y_m, ecef.z_m);
    Llh {
        lat_deg: lat,
        lon_deg: lon,
        alt_m: alt,
    }
}

/// Convert geodetic to ECEF.
pub fn llh_to_ecef(llh: Llh) -> Ecef {
    let (x, y, z) = geodetic_to_ecef(llh.lat_deg, llh.lon_deg, llh.alt_m);
    Ecef {
        x_m: x,
        y_m: y,
        z_m: z,
    }
}

/// Convert ECEF to local ENU.
pub fn ecef_to_enu_ref(ecef: Ecef, reference: Llh) -> Enu {
    let (e, n, u) = ecef_to_enu(
        ecef.x_m,
        ecef.y_m,
        ecef.z_m,
        reference.lat_deg,
        reference.lon_deg,
        reference.alt_m,
    );
    Enu { e_m: e, n_m: n, u_m: u }
}

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

#[cfg(test)]
mod tests {
    use super::{ecef_to_llh, llh_to_ecef, Ecef, Llh};

    #[test]
    fn llh_ecef_roundtrip_equator() {
        let llh = Llh {
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_m: 0.0,
        };
        let ecef = llh_to_ecef(llh);
        assert!((ecef.x_m - 6_378_137.0).abs() < 1.0);
        assert!(ecef.y_m.abs() < 1.0);
        assert!(ecef.z_m.abs() < 1.0);
        let back = ecef_to_llh(Ecef {
            x_m: ecef.x_m,
            y_m: ecef.y_m,
            z_m: ecef.z_m,
        });
        assert!((back.lat_deg - llh.lat_deg).abs() < 1e-6);
        assert!((back.lon_deg - llh.lon_deg).abs() < 1e-6);
    }
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
