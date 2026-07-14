//! Independent broadcast atmosphere models for synthetic truth generation.

use bijux_gnss_core::api::{Llh, Seconds};
use bijux_gnss_nav::api::KlobucharCoefficients;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SECONDS_PER_DAY: f64 = 86_400.0;
const MIN_KLOBUCHAR_PERIOD_S: f64 = 72_000.0;
const NIGHTTIME_IONO_DELAY_S: f64 = 5.0e-9;
const MIN_TROPOSPHERE_ELEVATION_DEG: f64 = 0.1;

pub(crate) fn klobuchar_delay_m(
    receiver: Llh,
    azimuth_deg: f64,
    elevation_deg: f64,
    receive_time_s: f64,
    coefficients: KlobucharCoefficients,
) -> f64 {
    if !receiver.lat_deg.is_finite()
        || !receiver.lon_deg.is_finite()
        || !azimuth_deg.is_finite()
        || !elevation_deg.is_finite()
        || !receive_time_s.is_finite()
        || elevation_deg <= 0.0
    {
        return 0.0;
    }

    let elevation_semicircles = elevation_deg.clamp(0.0, 90.0) / 180.0;
    let azimuth_rad = azimuth_deg.to_radians();
    let latitude_semicircles = receiver.lat_deg / 180.0;
    let longitude_semicircles = receiver.lon_deg / 180.0;
    let earth_centered_angle = 0.0137 / (elevation_semicircles + 0.11) - 0.022;
    let subionospheric_latitude =
        (latitude_semicircles + earth_centered_angle * azimuth_rad.cos()).clamp(-0.416, 0.416);
    let subionospheric_longitude = longitude_semicircles
        + earth_centered_angle * azimuth_rad.sin()
            / (subionospheric_latitude * std::f64::consts::PI).cos();
    let geomagnetic_latitude = subionospheric_latitude
        + 0.064 * ((subionospheric_longitude - 1.617) * std::f64::consts::PI).cos();
    let local_time_s =
        (43_200.0 * subionospheric_longitude + receive_time_s).rem_euclid(SECONDS_PER_DAY);
    let slant_factor = 1.0 + 16.0 * (0.53 - elevation_semicircles).powi(3);
    let amplitude_s = cubic(coefficients.alpha, geomagnetic_latitude).max(0.0);
    let period_s = cubic(coefficients.beta, geomagnetic_latitude).max(MIN_KLOBUCHAR_PERIOD_S);
    let phase_rad = 2.0 * std::f64::consts::PI * (local_time_s - 50_400.0) / period_s;
    let vertical_delay_s = if phase_rad.abs() <= 1.57 {
        NIGHTTIME_IONO_DELAY_S
            + amplitude_s * (1.0 - phase_rad.powi(2) / 2.0 + phase_rad.powi(4) / 24.0)
    } else {
        NIGHTTIME_IONO_DELAY_S
    };

    slant_factor * SPEED_OF_LIGHT_MPS * vertical_delay_s
}

pub(crate) fn saastamoinen_delay_m(receiver: Llh, elevation_deg: f64, _time: Seconds) -> f64 {
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    zenith_delay_m(receiver) * saastamoinen_mapping_factor(elevation_deg)
}

fn zenith_delay_m(receiver: Llh) -> f64 {
    zenith_hydrostatic_delay_m(receiver) + zenith_wet_delay_m(receiver)
}

fn zenith_hydrostatic_delay_m(receiver: Llh) -> f64 {
    if !receiver.lat_deg.is_finite() || !receiver.alt_m.is_finite() {
        return 0.0;
    }
    let pressure_hpa = standard_pressure_hpa(receiver.alt_m.max(-500.0));
    let latitude_rad = receiver.lat_deg.to_radians();
    let height_km = receiver.alt_m.max(-500.0) / 1_000.0;
    0.002_276_8 * pressure_hpa
        / (1.0 - 0.002_66 * (2.0 * latitude_rad).cos() - 0.000_28 * height_km)
}

fn zenith_wet_delay_m(receiver: Llh) -> f64 {
    if !receiver.alt_m.is_finite() {
        return 0.0;
    }
    let altitude_m = receiver.alt_m.max(-500.0);
    let temperature_k = standard_temperature_k(altitude_m);
    let water_vapor_pressure_hpa = standard_water_vapor_pressure_hpa(altitude_m, 0.7);
    0.002_277 * (1_255.0 / temperature_k + 0.05) * water_vapor_pressure_hpa
}

fn saastamoinen_mapping_factor(elevation_deg: f64) -> f64 {
    let mapped_elevation_deg =
        (elevation_deg.max(MIN_TROPOSPHERE_ELEVATION_DEG).powi(2) + 6.25).sqrt();
    1.0 / mapped_elevation_deg.to_radians().sin()
}

fn cubic(coefficients: [f64; 4], argument: f64) -> f64 {
    coefficients[0]
        + coefficients[1] * argument
        + coefficients[2] * argument.powi(2)
        + coefficients[3] * argument.powi(3)
}

fn standard_pressure_hpa(altitude_m: f64) -> f64 {
    1_013.25 * (1.0 - 2.255_7e-5 * altitude_m).powf(5.256_8)
}

fn standard_temperature_k(altitude_m: f64) -> f64 {
    (288.15 - 0.0065 * altitude_m).max(180.0)
}

fn saturation_vapor_pressure_hpa(temperature_k: f64) -> f64 {
    let celsius = temperature_k - 273.15;
    6.1121 * ((17.502 * celsius) / (240.97 + celsius)).exp()
}

fn standard_water_vapor_pressure_hpa(altitude_m: f64, relative_humidity: f64) -> f64 {
    saturation_vapor_pressure_hpa(standard_temperature_k(altitude_m)) * relative_humidity
}

#[cfg(test)]
mod tests {
    use super::{klobuchar_delay_m, saastamoinen_delay_m};
    use bijux_gnss_core::api::{Llh, Seconds};
    use bijux_gnss_nav::api::KlobucharCoefficients;

    #[test]
    fn klobuchar_delay_is_positive_above_horizon() {
        let delay_m = klobuchar_delay_m(
            Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 },
            120.0,
            35.0,
            504_018.0,
            KlobucharCoefficients::new(
                [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
                [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
            ),
        );

        assert!(delay_m.is_finite());
        assert!(delay_m > 0.0);
    }

    #[test]
    fn saastamoinen_delay_grows_toward_horizon() {
        let receiver = Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 };
        let low = saastamoinen_delay_m(receiver, 10.0, Seconds(0.0));
        let high = saastamoinen_delay_m(receiver, 70.0, Seconds(0.0));

        assert!(low > high);
    }
}
