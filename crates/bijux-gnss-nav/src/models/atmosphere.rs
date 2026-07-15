//! Atmospheric delay models and supporting meteorology utilities.
#![allow(missing_docs)]

use bijux_gnss_core::api::{Llh, Seconds};
use serde::{Deserialize, Serialize};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SECONDS_PER_DAY: f64 = 86_400.0;
const NIGHTTIME_IONO_DELAY_S: f64 = 5.0e-9;
const MIN_KLOBUCHAR_PERIOD_S: f64 = 72_000.0;
const MIN_TROPOSPHERE_ELEVATION_DEG: f64 = 0.1;
const STANDARD_RELATIVE_HUMIDITY: f64 = 0.7;
const MIN_PHYSICAL_PRESSURE_HPA: f64 = 100.0;
const MAX_PHYSICAL_PRESSURE_HPA: f64 = 1_100.0;
const MIN_PHYSICAL_TEMPERATURE_K: f64 = 180.0;
const MAX_PHYSICAL_TEMPERATURE_K: f64 = 330.0;
const SECONDS_PER_MEAN_YEAR: f64 = 365.25 * SECONDS_PER_DAY;
const NIELL_SEASONAL_PHASE_DAY_NORTH: f64 = 28.0;
const NIELL_LATITUDE_GRID_DEG: [f64; 5] = [15.0, 30.0, 45.0, 60.0, 75.0];
const NIELL_HYDROSTATIC_AVERAGE: [[f64; 3]; 5] = [
    [1.276_993_4e-3, 2.915_369_5e-3, 62.610_505e-3],
    [1.268_323_0e-3, 2.915_229_9e-3, 62.837_393e-3],
    [1.246_539_7e-3, 2.928_844_5e-3, 63.721_774e-3],
    [1.219_604_9e-3, 2.902_256_5e-3, 63.824_265e-3],
    [1.204_599_6e-3, 2.902_491_2e-3, 64.258_455e-3],
];
const NIELL_HYDROSTATIC_AMPLITUDE: [[f64; 3]; 5] = [
    [0.0, 0.0, 0.0],
    [1.270_962_6e-5, 2.141_497_9e-5, 9.012_840_0e-5],
    [2.652_366_2e-5, 3.016_077_9e-5, 4.349_703_7e-5],
    [3.400_045_2e-5, 7.256_272_2e-5, 84.795_348e-5],
    [4.120_219_1e-5, 11.723_375e-5, 170.372_06e-5],
];
const NIELL_WET: [[f64; 3]; 5] = [
    [5.802_189_7e-4, 1.427_526_8e-3, 4.347_296_1e-2],
    [5.679_484_7e-4, 1.513_862_5e-3, 4.672_951_0e-2],
    [5.811_801_9e-4, 1.457_275_2e-3, 4.390_893_1e-2],
    [5.972_754_2e-4, 1.500_742_8e-3, 4.462_698_2e-2],
    [6.164_169_3e-4, 1.759_908_2e-3, 5.473_603_8e-2],
];
const NIELL_HEIGHT: [f64; 3] = [2.53e-5, 5.49e-3, 1.14e-3];

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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TroposphereMeteorology {
    pub pressure_hpa: f64,
    pub temperature_k: f64,
    pub relative_humidity: f64,
}

impl TroposphereMeteorology {
    pub fn new(pressure_hpa: f64, temperature_k: f64, relative_humidity: f64) -> Self {
        Self { pressure_hpa, temperature_k, relative_humidity }
    }

    pub fn is_physical(self) -> bool {
        self.pressure_hpa.is_finite()
            && self.temperature_k.is_finite()
            && self.relative_humidity.is_finite()
            && (MIN_PHYSICAL_PRESSURE_HPA..=MAX_PHYSICAL_PRESSURE_HPA).contains(&self.pressure_hpa)
            && (MIN_PHYSICAL_TEMPERATURE_K..=MAX_PHYSICAL_TEMPERATURE_K)
                .contains(&self.temperature_k)
            && (0.0..=1.0).contains(&self.relative_humidity)
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NiellMappingFactors {
    pub hydrostatic: f64,
    pub wet: f64,
}

#[derive(Debug, Clone)]
pub struct NiellMappingFunction;

impl NiellMappingFunction {
    pub fn mapping_factors(receiver: Llh, el_deg: f64, t: Seconds) -> NiellMappingFactors {
        NiellMappingFactors {
            hydrostatic: Self::hydrostatic_mapping_factor(receiver, el_deg, t),
            wet: Self::wet_mapping_factor(receiver, el_deg),
        }
    }

    pub fn hydrostatic_mapping_factor(receiver: Llh, el_deg: f64, t: Seconds) -> f64 {
        if !receiver.lat_deg.is_finite()
            || !receiver.alt_m.is_finite()
            || !el_deg.is_finite()
            || !t.0.is_finite()
            || el_deg <= 0.0
        {
            return 0.0;
        }
        let elevation_rad = el_deg.max(MIN_TROPOSPHERE_ELEVATION_DEG).to_radians();
        let day_of_year = day_of_year_from_seconds(t);
        let seasonal_phase_days = if receiver.lat_deg < 0.0 {
            NIELL_SEASONAL_PHASE_DAY_NORTH + 365.25 / 2.0
        } else {
            NIELL_SEASONAL_PHASE_DAY_NORTH
        };
        let seasonal_cosine =
            (2.0 * std::f64::consts::PI * (day_of_year - seasonal_phase_days) / 365.25).cos();
        let average = interpolate_niell_coefficients(receiver.lat_deg, NIELL_HYDROSTATIC_AVERAGE);
        let amplitude =
            interpolate_niell_coefficients(receiver.lat_deg, NIELL_HYDROSTATIC_AMPLITUDE);
        let coefficients = [
            average[0] - amplitude[0] * seasonal_cosine,
            average[1] - amplitude[1] * seasonal_cosine,
            average[2] - amplitude[2] * seasonal_cosine,
        ];
        let sea_level_mapping = continued_fraction_mapping(elevation_rad, coefficients);
        let height_km = receiver.alt_m.max(0.0) / 1_000.0;
        let height_mapping = continued_fraction_mapping(elevation_rad, NIELL_HEIGHT);
        sea_level_mapping + (1.0 / elevation_rad.sin() - height_mapping) * height_km
    }

    pub fn wet_mapping_factor(receiver: Llh, el_deg: f64) -> f64 {
        if !receiver.lat_deg.is_finite() || !el_deg.is_finite() || el_deg <= 0.0 {
            return 0.0;
        }
        let coefficients = interpolate_niell_coefficients(receiver.lat_deg, NIELL_WET);
        continued_fraction_mapping(
            el_deg.max(MIN_TROPOSPHERE_ELEVATION_DEG).to_radians(),
            coefficients,
        )
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
        Self::zenith_hydrostatic_delay_from_pressure_m(receiver, pressure_hpa)
    }

    pub fn zenith_hydrostatic_delay_with_meteorology_m(
        receiver: Llh,
        meteorology: TroposphereMeteorology,
    ) -> f64 {
        if !meteorology.is_physical() {
            return 0.0;
        }
        Self::zenith_hydrostatic_delay_from_pressure_m(receiver, meteorology.pressure_hpa)
    }

    fn zenith_hydrostatic_delay_from_pressure_m(receiver: Llh, pressure_hpa: f64) -> f64 {
        if !receiver.lat_deg.is_finite() || !receiver.alt_m.is_finite() || !pressure_hpa.is_finite()
        {
            return 0.0;
        }
        let latitude_rad = receiver.lat_deg.to_radians();
        let height_km = receiver.alt_m.max(-500.0) / 1_000.0;
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
        Self::zenith_wet_delay_from_conditions_m(temperature_k, water_vapor_pressure_hpa)
    }

    pub fn zenith_wet_delay_with_meteorology_m(meteorology: TroposphereMeteorology) -> f64 {
        if !meteorology.is_physical() {
            return 0.0;
        }
        let water_vapor_pressure_hpa = saturation_vapor_pressure_hpa(meteorology.temperature_k)
            * meteorology.relative_humidity;
        Self::zenith_wet_delay_from_conditions_m(
            meteorology.temperature_k,
            water_vapor_pressure_hpa,
        )
    }

    fn zenith_wet_delay_from_conditions_m(
        temperature_k: f64,
        water_vapor_pressure_hpa: f64,
    ) -> f64 {
        if !temperature_k.is_finite() || !water_vapor_pressure_hpa.is_finite() {
            return 0.0;
        }
        0.002_277 * (1_255.0 / temperature_k + 0.05) * water_vapor_pressure_hpa
    }

    pub fn zenith_delay_m(receiver: Llh) -> f64 {
        Self::zenith_hydrostatic_delay_m(receiver) + Self::zenith_wet_delay_m(receiver)
    }

    pub fn zenith_delay_with_meteorology_m(
        receiver: Llh,
        meteorology: TroposphereMeteorology,
    ) -> f64 {
        Self::zenith_hydrostatic_delay_with_meteorology_m(receiver, meteorology)
            + Self::zenith_wet_delay_with_meteorology_m(meteorology)
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
    relative_humidity.clamp(0.0, 1.0) * saturation_vapor_pressure_hpa(temperature_k)
}

fn saturation_vapor_pressure_hpa(temperature_k: f64) -> f64 {
    let temperature_c = temperature_k - 273.15;
    6.108 * ((17.15 * temperature_c) / (234.7 + temperature_c)).exp()
}

fn day_of_year_from_seconds(t: Seconds) -> f64 {
    t.0.rem_euclid(SECONDS_PER_MEAN_YEAR) / SECONDS_PER_DAY + 1.0
}

fn interpolate_niell_coefficients(latitude_deg: f64, table: [[f64; 3]; 5]) -> [f64; 3] {
    let latitude_deg = latitude_deg.abs().clamp(
        *NIELL_LATITUDE_GRID_DEG.first().expect("latitude grid"),
        *NIELL_LATITUDE_GRID_DEG.last().expect("latitude grid"),
    );
    for index in 0..NIELL_LATITUDE_GRID_DEG.len() - 1 {
        let lower_latitude_deg = NIELL_LATITUDE_GRID_DEG[index];
        let upper_latitude_deg = NIELL_LATITUDE_GRID_DEG[index + 1];
        if latitude_deg <= upper_latitude_deg {
            let fraction =
                (latitude_deg - lower_latitude_deg) / (upper_latitude_deg - lower_latitude_deg);
            return [
                table[index][0] + fraction * (table[index + 1][0] - table[index][0]),
                table[index][1] + fraction * (table[index + 1][1] - table[index][1]),
                table[index][2] + fraction * (table[index + 1][2] - table[index][2]),
            ];
        }
    }
    table[NIELL_LATITUDE_GRID_DEG.len() - 1]
}

fn continued_fraction_mapping(elevation_rad: f64, coefficients: [f64; 3]) -> f64 {
    let sine = elevation_rad.sin();
    let [a, b, c] = coefficients;
    (1.0 + a / (1.0 + b / (1.0 + c))) / (sine + a / (sine + b / (sine + c)))
}

#[cfg(test)]
mod tests {
    use super::{
        IonosphereModel, KlobucharCoefficients, KlobucharModel, NiellMappingFunction,
        SaastamoinenModel, TroposphereMeteorology, TroposphereModel, SECONDS_PER_DAY,
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

    fn seconds_at_day_of_year(day_of_year: f64) -> Seconds {
        Seconds((day_of_year - 1.0) * SECONDS_PER_DAY)
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
    fn niell_mapping_matches_midlatitude_winter_reference_values() {
        let receiver = Llh { lat_deg: 45.0, lon_deg: 8.0, alt_m: 100.0 };
        let factors =
            NiellMappingFunction::mapping_factors(receiver, 10.0, seconds_at_day_of_year(28.0));

        assert!((factors.hydrostatic - 5.556_157_591).abs() < 1.0e-9);
        assert!((factors.wet - 5.657_127_345).abs() < 1.0e-9);
    }

    #[test]
    fn niell_hydrostatic_mapping_changes_with_season_and_hemisphere() {
        let northern = Llh { lat_deg: 60.0, lon_deg: 10.0, alt_m: 0.0 };
        let southern = Llh { lat_deg: -60.0, lon_deg: 10.0, alt_m: 0.0 };
        let january_north = NiellMappingFunction::hydrostatic_mapping_factor(
            northern,
            7.0,
            seconds_at_day_of_year(28.0),
        );
        let july_north = NiellMappingFunction::hydrostatic_mapping_factor(
            northern,
            7.0,
            seconds_at_day_of_year(210.625),
        );
        let january_south = NiellMappingFunction::hydrostatic_mapping_factor(
            southern,
            7.0,
            seconds_at_day_of_year(28.0),
        );

        assert!((january_north - 7.671_180_454).abs() < 1.0e-9);
        assert!((july_north - 7.645_142_217).abs() < 1.0e-9);
        assert!((january_south - july_north).abs() < 1.0e-9);
    }

    #[test]
    fn niell_hydrostatic_height_correction_increases_low_elevation_mapping() {
        let sea_level = Llh { lat_deg: 45.0, lon_deg: 8.0, alt_m: 0.0 };
        let mountain = Llh { lat_deg: 45.0, lon_deg: 8.0, alt_m: 2_000.0 };
        let sea_level_mapping = NiellMappingFunction::hydrostatic_mapping_factor(
            sea_level,
            5.0,
            seconds_at_day_of_year(100.0),
        );
        let mountain_mapping = NiellMappingFunction::hydrostatic_mapping_factor(
            mountain,
            5.0,
            seconds_at_day_of_year(100.0),
        );

        assert!((sea_level_mapping - 10.136_144_698).abs() < 1.0e-9);
        assert!((mountain_mapping - 10.180_088_797).abs() < 1.0e-9);
        assert!(mountain_mapping > sea_level_mapping);
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

    #[test]
    fn troposphere_meteorology_rejects_non_physical_ranges() {
        assert!(TroposphereMeteorology::new(1_013.25, 293.15, 0.5).is_physical());
        assert!(!TroposphereMeteorology::new(50.0, 293.15, 0.5).is_physical());
        assert!(!TroposphereMeteorology::new(1_013.25, 120.0, 0.5).is_physical());
        assert!(!TroposphereMeteorology::new(1_013.25, 293.15, 1.5).is_physical());
    }

    #[test]
    fn saastamoinen_hydrostatic_delay_tracks_pressure_changes() {
        let receiver = sample_receiver();
        let low_pressure = TroposphereMeteorology::new(850.0, 293.15, 0.4);
        let high_pressure = TroposphereMeteorology::new(1_030.0, 293.15, 0.4);

        let low_delay_m =
            SaastamoinenModel::zenith_hydrostatic_delay_with_meteorology_m(receiver, low_pressure);
        let high_delay_m =
            SaastamoinenModel::zenith_hydrostatic_delay_with_meteorology_m(receiver, high_pressure);

        assert!(high_delay_m > low_delay_m + 0.3);
    }

    #[test]
    fn saastamoinen_wet_delay_tracks_temperature_and_humidity_changes() {
        let hot_humid = TroposphereMeteorology::new(1_013.25, 303.15, 0.95);
        let cool_dry = TroposphereMeteorology::new(1_013.25, 278.15, 0.2);

        let hot_humid_delay_m = SaastamoinenModel::zenith_wet_delay_with_meteorology_m(hot_humid);
        let cool_dry_delay_m = SaastamoinenModel::zenith_wet_delay_with_meteorology_m(cool_dry);

        assert!(hot_humid_delay_m > cool_dry_delay_m + 0.15);
    }

    #[test]
    fn saastamoinen_total_delay_uses_local_meteorology() {
        let receiver = sample_receiver();
        let standard_delay_m = SaastamoinenModel::zenith_delay_m(receiver);
        let observed_meteorology = TroposphereMeteorology::new(980.0, 301.15, 0.9);
        let observed_delay_m =
            SaastamoinenModel::zenith_delay_with_meteorology_m(receiver, observed_meteorology);

        assert!(observed_delay_m.is_finite());
        assert!((observed_delay_m - standard_delay_m).abs() > 0.05);
    }
}
