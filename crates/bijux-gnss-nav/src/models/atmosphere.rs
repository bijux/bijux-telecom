//! Atmospheric delay model scaffolding.
#![allow(missing_docs)]

use bijux_gnss_core::api::{Llh, Seconds};
use serde::{Deserialize, Serialize};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SECONDS_PER_DAY: f64 = 86_400.0;
const NIGHTTIME_IONO_DELAY_S: f64 = 5.0e-9;
const MIN_KLOBUCHAR_PERIOD_S: f64 = 72_000.0;

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
        let subionospheric_latitude = (latitude_semicircles + earth_centered_angle * azimuth_rad.cos())
            .clamp(-0.416, 0.416);
        let subionospheric_longitude = longitude_semicircles
            + earth_centered_angle * azimuth_rad.sin() / (subionospheric_latitude * std::f64::consts::PI).cos();
        let geomagnetic_latitude = subionospheric_latitude
            + 0.064 * ((subionospheric_longitude - 1.617) * std::f64::consts::PI).cos();
        let local_time_s =
            (43_200.0 * subionospheric_longitude + t.0).rem_euclid(SECONDS_PER_DAY);
        let slant_factor = 1.0 + 16.0 * (0.53 - elevation_semicircles).powi(3);
        let amplitude_s = klobuchar_polynomial(self.coefficients.alpha, geomagnetic_latitude).max(0.0);
        let period_s =
            klobuchar_polynomial(self.coefficients.beta, geomagnetic_latitude).max(MIN_KLOBUCHAR_PERIOD_S);
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

impl TroposphereModel for SaastamoinenModel {
    fn delay_m(&self, _receiver: Llh, _el_deg: f64, _t: Seconds) -> f64 {
        0.0
    }
}

fn klobuchar_polynomial(coefficients: [f64; 4], latitude_semicircles: f64) -> f64 {
    coefficients[0]
        + coefficients[1] * latitude_semicircles
        + coefficients[2] * latitude_semicircles.powi(2)
        + coefficients[3] * latitude_semicircles.powi(3)
}

#[cfg(test)]
mod tests {
    use super::{IonosphereModel, KlobucharCoefficients, KlobucharModel};
    use bijux_gnss_core::api::{Llh, Seconds};

    fn sample_receiver() -> Llh {
        Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 }
    }

    fn equatorial_receiver() -> Llh {
        Llh { lat_deg: 0.0, lon_deg: 0.0, alt_m: 10.0 }
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
        let low_elevation_floor_m = nighttime_floor.delay_m(receiver, 120.0, 15.0, Seconds(50_400.0));

        assert!(low_elevation_delay_m.is_finite());
        assert!(high_elevation_delay_m.is_finite());
        assert!(low_elevation_delay_m > high_elevation_delay_m);
        assert!(low_elevation_delay_m > low_elevation_floor_m);
    }
}
