//! Atmospheric delay model scaffolding.
#![allow(missing_docs)]

use bijux_gnss_core::api::{Llh, Seconds};
use serde::{Deserialize, Serialize};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SECONDS_PER_DAY: f64 = 86_400.0;
const NIGHTTIME_IONO_DELAY_S: f64 = 5.0e-9;
const MIN_KLOBUCHAR_PERIOD_S: f64 = 72_000.0;
const MIN_TROPOSPHERE_ELEVATION_DEG: f64 = 0.1;
const STANDARD_RELATIVE_HUMIDITY: f64 = 0.7;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct KlobucharCoefficients {
    pub alpha: [f64; 4],
    pub beta: [f64; 4],
}

impl KlobucharCoefficients {
    pub fn new(alpha: [f64; 4], beta: [f64; 4]) -> Self {
        Self { alpha, beta }
    }
}

pub trait IonosphereModel {
    fn delay_m(&self, _receiver: Llh, _az_deg: f64, _el_deg: f64, _t: Seconds) -> f64;
}

pub trait TroposphereModel {
    fn delay_m(&self, _receiver: Llh, _el_deg: f64, _t: Seconds) -> f64;
}

#[derive(Debug, Clone)]
pub struct KlobucharModel {
    pub coefficients: KlobucharCoefficients,
}

impl KlobucharModel {
    pub fn new(coefficients: KlobucharCoefficients) -> Self {
        Self { coefficients }
    }
}

impl IonosphereModel for KlobucharModel {
    fn delay_m(&self, receiver: Llh, az_deg: f64, el_deg: f64, t: Seconds) -> f64 {
        if !receiver.lat_deg.is_finite()
            || !receiver.lon_deg.is_finite()
            || !az_deg.is_finite()
            || !el_deg.is_finite()
            || !t.0.is_finite()
            || el_deg <= 0.0
        {
            return 0.0;
        }

        let elevation_semicircles = el_deg.clamp(0.0, 90.0) / 180.0;
        let azimuth_rad = az_deg.to_radians();
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
        let local_time_s = (43_200.0 * subionospheric_longitude + t.0).rem_euclid(SECONDS_PER_DAY);
        let slant_factor = 1.0 + 16.0 * (0.53 - elevation_semicircles).powi(3);
        let amplitude_s =
            klobuchar_polynomial(self.coefficients.alpha, geomagnetic_latitude).max(0.0);
        let period_s = klobuchar_polynomial(self.coefficients.beta, geomagnetic_latitude)
            .max(MIN_KLOBUCHAR_PERIOD_S);
        let phase = 2.0 * std::f64::consts::PI * (local_time_s - 50_400.0) / period_s;
        let vertical_delay_s = if phase.abs() <= 1.57 {
            NIGHTTIME_IONO_DELAY_S
                + amplitude_s * (1.0 - phase.powi(2) / 2.0 + phase.powi(4) / 24.0)
        } else {
            NIGHTTIME_IONO_DELAY_S
        };
        slant_factor * SPEED_OF_LIGHT_MPS * vertical_delay_s
    }
}

#[derive(Debug, Clone)]
pub struct SaastamoinenModel;

impl SaastamoinenModel {
    pub fn zenith_hydrostatic_delay_m(receiver: Llh) -> f64 {
        if !receiver.lat_deg.is_finite() || !receiver.alt_m.is_finite() {
            return 0.0;
        }
        let altitude_m = receiver.alt_m.max(-500.0);
        let pressure_hpa = standard_pressure_hpa(altitude_m);
        let latitude_rad = receiver.lat_deg.to_radians();
        let height_km = altitude_m / 1_000.0;
        0.002_276_8 * pressure_hpa
            / (1.0 - 0.002_66 * (2.0 * latitude_rad).cos() - 0.000_28 * height_km)
    }

    pub fn zenith_wet_delay_m(receiver: Llh) -> f64 {
        if !receiver.alt_m.is_finite() {
            return 0.0;
        }
        let altitude_m = receiver.alt_m.max(-500.0);
        let temperature_k = standard_temperature_k(altitude_m);
        let water_vapor_pressure_hpa =
            standard_water_vapor_pressure_hpa(altitude_m, STANDARD_RELATIVE_HUMIDITY);
        0.002_277 * (1_255.0 / temperature_k + 0.05) * water_vapor_pressure_hpa
    }

    pub fn zenith_delay_m(receiver: Llh) -> f64 {
        Self::zenith_hydrostatic_delay_m(receiver) + Self::zenith_wet_delay_m(receiver)
    }

    pub fn mapping_factor(el_deg: f64) -> f64 {
        if !el_deg.is_finite() || el_deg <= 0.0 {
            return 0.0;
        }
        let mapped_elevation_deg =
            (el_deg.max(MIN_TROPOSPHERE_ELEVATION_DEG).powi(2) + 6.25).sqrt();
        1.0 / mapped_elevation_deg.to_radians().sin()
    }
}

impl TroposphereModel for SaastamoinenModel {
    fn delay_m(&self, receiver: Llh, el_deg: f64, _t: Seconds) -> f64 {
        Self::zenith_delay_m(receiver) * Self::mapping_factor(el_deg)
    }
}

fn klobuchar_polynomial(coefficients: [f64; 4], latitude_semicircles: f64) -> f64 {
    coefficients[0]
        + coefficients[1] * latitude_semicircles
        + coefficients[2] * latitude_semicircles.powi(2)
        + coefficients[3] * latitude_semicircles.powi(3)
}

fn standard_pressure_hpa(altitude_m: f64) -> f64 {
    1_013.25 * (1.0 - 2.255_7e-5 * altitude_m).powf(5.256_8)
}

fn standard_temperature_k(altitude_m: f64) -> f64 {
    (288.15 - 0.0065 * altitude_m).max(180.0)
}

fn standard_water_vapor_pressure_hpa(altitude_m: f64, relative_humidity: f64) -> f64 {
    let temperature_k = standard_temperature_k(altitude_m);
    let temperature_c = temperature_k - 273.15;
    let saturation_pressure_hpa = 6.108 * ((17.15 * temperature_c) / (234.7 + temperature_c)).exp();
    relative_humidity.clamp(0.0, 1.0) * saturation_pressure_hpa
}

#[cfg(test)]
mod tests {
    use super::{
        IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
    };
    use bijux_gnss_core::api::{Llh, Seconds};

    fn sample_receiver() -> Llh {
        Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }
    }

    fn equatorial_receiver() -> Llh {
        Llh { lat_deg: 0.0, lon_deg: 0.0, alt_m: 10.0 }
    }

    fn alpine_receiver() -> Llh {
        Llh { lat_deg: 46.0, lon_deg: 7.0, alt_m: 2_500.0 }
    }

    #[test]
    fn klobuchar_delay_is_zero_below_horizon() {
        let model = KlobucharModel::new(KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        ));

        assert_eq!(model.delay_m(sample_receiver(), 120.0, -2.0, Seconds(50_400.0)), 0.0);
    }

    #[test]
    fn klobuchar_delay_uses_nighttime_floor_when_amplitude_is_zero() {
        let model = KlobucharModel::new(KlobucharCoefficients::new([0.0; 4], [0.0; 4]));
        let elevation_deg = 30.0;
        let elevation_semicircles = elevation_deg / 180.0;
        let expected_delay_m = (1.0_f64 + 16.0_f64 * (0.53_f64 - elevation_semicircles).powi(3))
            * 299_792_458.0
            * 5.0e-9;

        let delay_m = model.delay_m(sample_receiver(), 120.0, elevation_deg, Seconds(50_400.0));

        assert!((delay_m - expected_delay_m).abs() < 1.0e-9);
    }

    #[test]
    fn klobuchar_delay_grows_for_low_elevation_daytime_paths() {
        let model = KlobucharModel::new(KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        ));
        let nighttime_floor = KlobucharModel::new(KlobucharCoefficients::new([0.0; 4], [0.0; 4]));
        let receiver = equatorial_receiver();

        let low_elevation_delay_m = model.delay_m(receiver, 120.0, 15.0, Seconds(50_400.0));
        let high_elevation_delay_m = model.delay_m(receiver, 120.0, 75.0, Seconds(50_400.0));
        let low_elevation_floor_m =
            nighttime_floor.delay_m(receiver, 120.0, 15.0, Seconds(50_400.0));

        assert!(low_elevation_delay_m.is_finite());
        assert!(high_elevation_delay_m.is_finite());
        assert!(low_elevation_delay_m > high_elevation_delay_m);
        assert!(low_elevation_delay_m > low_elevation_floor_m);
    }

    #[test]
    fn saastamoinen_zenith_delay_matches_standard_surface_reference() {
        let zenith_delay_m = SaastamoinenModel::zenith_delay_m(sample_receiver());

        assert!(zenith_delay_m > 2.3);
        assert!(zenith_delay_m < 2.7);
        assert!((zenith_delay_m - 2.46).abs() < 0.1);
    }

    #[test]
    fn saastamoinen_zenith_delay_drops_with_altitude() {
        let sea_level_delay_m = SaastamoinenModel::zenith_delay_m(sample_receiver());
        let alpine_delay_m = SaastamoinenModel::zenith_delay_m(alpine_receiver());

        assert!(alpine_delay_m.is_finite());
        assert!(alpine_delay_m < sea_level_delay_m - 0.4);
    }

    #[test]
    fn saastamoinen_mapping_factor_grows_at_low_elevation() {
        let low_elevation = SaastamoinenModel::mapping_factor(10.0);
        let high_elevation = SaastamoinenModel::mapping_factor(70.0);

        assert!(low_elevation.is_finite());
        assert!(high_elevation.is_finite());
        assert!(low_elevation > high_elevation);
        assert!(low_elevation > 5.0);
    }

    #[test]
    fn saastamoinen_delay_is_zero_below_horizon() {
        let model = SaastamoinenModel;

        assert_eq!(model.delay_m(sample_receiver(), -1.0, Seconds(50_400.0)), 0.0);
    }

    #[test]
    fn saastamoinen_delay_maps_zenith_delay_by_elevation() {
        let model = SaastamoinenModel;
        let receiver = sample_receiver();
        let zenith_delay_m = SaastamoinenModel::zenith_delay_m(receiver);
        let high_elevation_delay_m = model.delay_m(receiver, 80.0, Seconds(50_400.0));
        let low_elevation_delay_m = model.delay_m(receiver, 15.0, Seconds(50_400.0));

        assert!(high_elevation_delay_m > zenith_delay_m);
        assert!(high_elevation_delay_m < zenith_delay_m * 1.1);
        assert!(low_elevation_delay_m > high_elevation_delay_m * 2.0);
    }
}
