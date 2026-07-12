use bijux_gnss_core::api::{gps_to_utc, GpsTime, LeapSeconds, Llh};
use time::OffsetDateTime;

use super::data::support_data;

const MEAN_EARTH_RADIUS_M: f64 = 6_371_200.0;
const DENSITY_FACTOR: f64 = 1.0e11;
const DELAY_FACTOR: f64 = 40.3e16;
const TOPSIDE_GRADIENT: f64 = 0.125;
const TOPSIDE_REFERENCE_HEIGHT_KM: f64 = 100.0;
const DAY_NIGHT_TRANSITION_DEG: f64 = 86.232_927_962_116_15;
const STEC_START_INTERVALS: usize = 8;
const STEC_MAX_REFINEMENTS: usize = 20;
const FOF2_SERIES_LENGTH: usize = 76;
const M3000_SERIES_LENGTH: usize = 49;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GalileoNequickModel {
    alpha: [f64; 3],
}

impl GalileoNequickModel {
    pub fn new(alpha: [f64; 3]) -> Self {
        Self { alpha }
    }

    pub fn stec_tecu(
        &self,
        month: u8,
        utc_hours: f64,
        receiver: Llh,
        satellite: Llh,
    ) -> Option<f64> {
        let time = NequickTime::new(month, utc_hours)?;
        let receiver = GeoPoint::from_llh(receiver)?;
        let satellite = GeoPoint::from_llh(satellite)?;
        let effective_ionization_level =
            self.effective_ionization_level(compute_modip(receiver.latitude_rad, receiver.longitude_rad));
        let tolerance = if receiver.altitude_m < 1_000_000.0 { 0.001 } else { 0.01 };

        let mut intervals = STEC_START_INTERVALS;
        let mut previous =
            self.integrate_stec(&time, receiver, satellite, effective_ionization_level, intervals)?;
        intervals *= 2;
        let mut current =
            self.integrate_stec(&time, receiver, satellite, effective_ionization_level, intervals)?;
        let mut refinements = 1usize;
        while (current - previous).abs() > tolerance * previous.abs().max(1.0e-12)
            && refinements < STEC_MAX_REFINEMENTS
        {
            previous = current;
            intervals *= 2;
            current =
                self.integrate_stec(&time, receiver, satellite, effective_ionization_level, intervals)?;
            refinements += 1;
        }
        if refinements == STEC_MAX_REFINEMENTS {
            return None;
        }

        Some((current + (current - previous) / 15.0) * 1.0e-16)
    }

    pub fn delay_m(
        &self,
        month: u8,
        utc_hours: f64,
        receiver: Llh,
        satellite: Llh,
        carrier_hz: f64,
    ) -> Option<f64> {
        if !carrier_hz.is_finite() || carrier_hz <= 0.0 {
            return None;
        }
        self.stec_tecu(month, utc_hours, receiver, satellite)
            .map(|stec_tecu| DELAY_FACTOR * stec_tecu / (carrier_hz * carrier_hz))
    }

    pub fn delay_m_at_gps_time(
        &self,
        gps_time: GpsTime,
        receiver: Llh,
        satellite: Llh,
        carrier_hz: f64,
    ) -> Option<f64> {
        let utc = gps_to_utc(gps_time, &LeapSeconds::default_table());
        let datetime =
            OffsetDateTime::from_unix_timestamp_nanos((utc.unix_s * 1_000_000_000.0).round() as i128)
                .ok()?;
        let month = u8::from(datetime.month());
        let utc_hours = datetime.hour() as f64
            + datetime.minute() as f64 / 60.0
            + datetime.second() as f64 / 3_600.0
            + datetime.nanosecond() as f64 / 3_600_000_000_000.0;
        self.delay_m(month, utc_hours, receiver, satellite, carrier_hz)
    }

    fn integrate_stec(
        &self,
        time: &NequickTime,
        receiver: GeoPoint,
        satellite: GeoPoint,
        effective_ionization_level: f64,
        intervals: usize,
    ) -> Option<f64> {
        let receiver_xyz = receiver.to_cartesian()?;
        let satellite_xyz = satellite.to_cartesian()?;
        let direction = [
            satellite_xyz[0] - receiver_xyz[0],
            satellite_xyz[1] - receiver_xyz[1],
            satellite_xyz[2] - receiver_xyz[2],
        ];
        let segment_length_m =
            (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]).sqrt();
        if !segment_length_m.is_finite() || segment_length_m <= 0.0 {
            return None;
        }

        let interval_fraction = 1.0 / intervals as f64;
        let gauss_offset = interval_fraction / 3.0_f64.sqrt();
        let mut density_sum = 0.0;
        for interval in 0..intervals {
            let base = interval as f64 * interval_fraction;
            let first_fraction = base + 0.5 * (interval_fraction - gauss_offset);
            let second_fraction = first_fraction + gauss_offset;
            for fraction in [first_fraction, second_fraction] {
                let point = GeoPoint::from_cartesian([
                    receiver_xyz[0] + direction[0] * fraction,
                    receiver_xyz[1] + direction[1] * fraction,
                    receiver_xyz[2] + direction[2] * fraction,
                ])?;
                density_sum += self.electron_density(time, point, effective_ionization_level);
            }
        }
        Some(0.5 * (segment_length_m / intervals as f64) * density_sum)
    }

    fn electron_density(
        &self,
        time: &NequickTime,
        point: GeoPoint,
        effective_ionization_level: f64,
    ) -> f64 {
        let modip_deg = compute_modip(point.latitude_rad, point.longitude_rad);
        let fourier = FourierSeries::new(time, effective_ionization_level);
        let parameters = NequickParameters::new(time, &fourier, point.latitude_rad, point.longitude_rad, modip_deg);
        let height_km = point.altitude_m * 0.001;
        if height_km <= parameters.hm_f2_km {
            self.bottomside_density(height_km, &parameters)
        } else {
            self.topside_density(height_km, &parameters)
        }
    }

    fn effective_ionization_level(&self, modip_deg: f64) -> f64 {
        if self.alpha == [0.0; 3] {
            63.7
        } else {
            (self.alpha[0] + modip_deg * (self.alpha[1] + modip_deg * self.alpha[2])).clamp(0.0, 400.0)
        }
    }

    fn bottomside_density(&self, height_km: f64, parameters: &NequickParameters) -> f64 {
        let be = if height_km > parameters.hm_e_km {
            parameters.be_top_km
        } else {
            parameters.be_bottom_km
        };
        let bf1 = if height_km > parameters.hm_f1_km {
            parameters.b1_top_km
        } else {
            parameters.b1_bottom_km
        };
        let inverse_thickness = [1.0 / parameters.b2_bottom_km, 1.0 / bf1, 1.0 / be];
        let arguments = self.exponential_arguments(height_km, parameters);

        let mut s = [0.0; 3];
        let mut ds = [0.0; 3];
        for index in 0..3 {
            if arguments[index].abs() <= 25.0 {
                let exp_argument = clip_exp(arguments[index]);
                let one_plus = 1.0 + exp_argument;
                s[index] = parameters.amplitudes[index] * (exp_argument / (one_plus * one_plus));
                ds[index] = inverse_thickness[index] * ((1.0 - exp_argument) / one_plus);
            }
        }

        let a_no = s.iter().sum::<f64>();
        if a_no <= 0.0 {
            return 0.0;
        }
        if height_km < 100.0 {
            let bc = 1.0
                - 10.0
                    * (s[0] * ds[0] + s[1] * ds[1] + s[2] * ds[2])
                    / a_no;
            let z = 0.1 * (height_km - 100.0);
            a_no * clip_exp(1.0 - bc * z - clip_exp(-z)) * DENSITY_FACTOR
        } else {
            a_no * DENSITY_FACTOR
        }
    }

    fn topside_density(&self, height_km: f64, parameters: &NequickParameters) -> f64 {
        let delta_h_km = height_km - parameters.hm_f2_km;
        let h0_km = self.h0_km(parameters);
        let z = delta_h_km
            / (h0_km
                * (1.0
                    + (TOPSIDE_REFERENCE_HEIGHT_KM * TOPSIDE_GRADIENT * delta_h_km)
                        / (TOPSIDE_REFERENCE_HEIGHT_KM * h0_km + TOPSIDE_GRADIENT * delta_h_km)));
        let exp_z = clip_exp(z);
        if exp_z > 1.0e11 {
            (4.0 * parameters.nm_f2 / exp_z) * DENSITY_FACTOR
        } else {
            let one_plus = 1.0 + exp_z;
            (4.0 * parameters.nm_f2 * exp_z / (one_plus * one_plus)) * DENSITY_FACTOR
        }
    }

    fn exponential_arguments(&self, height_km: f64, parameters: &NequickParameters) -> [f64; 3] {
        let clipped_height_km = height_km.max(100.0);
        let exp = clip_exp(10.0 / (1.0 + (clipped_height_km - parameters.hm_f2_km).abs()));
        [
            (clipped_height_km - parameters.hm_f2_km) / parameters.b2_bottom_km,
            ((clipped_height_km - parameters.hm_f1_km)
                / if height_km > parameters.hm_f1_km {
                    parameters.b1_top_km
                } else {
                    parameters.b1_bottom_km
                })
                * exp,
            ((clipped_height_km - parameters.hm_e_km)
                / if height_km > parameters.hm_e_km {
                    parameters.be_top_km
                } else {
                    parameters.be_bottom_km
                })
                * exp,
        ]
    }

    fn h0_km(&self, parameters: &NequickParameters) -> f64 {
        let ka = if (4..10).contains(&parameters.month) {
            6.705 - 0.014 * parameters.azr - 0.008 * parameters.hm_f2_km
        } else {
            let ratio = parameters.hm_f2_km / parameters.b2_bottom_km;
            -7.77 + 0.097 * ratio * ratio + 0.153 * parameters.nm_f2
        };
        let kb = join(8.0, join(ka, 2.0, 1.0, ka - 2.0), 1.0, join(ka, 2.0, 1.0, ka - 2.0) - 8.0);
        let h_a = kb * parameters.b2_bottom_km;
        let x = 0.01 * (h_a - 150.0);
        let v = (0.041_163 * x - 0.183_981) * x + 1.424_472;
        h_a / v
    }
}

#[derive(Debug, Clone, Copy)]
struct NequickTime {
    month: u8,
    utc_hours: f64,
}

impl NequickTime {
    fn new(month: u8, utc_hours: f64) -> Option<Self> {
        if !(1..=12).contains(&month) || !utc_hours.is_finite() {
            return None;
        }
        Some(Self { month, utc_hours })
    }
}

#[derive(Debug, Clone, Copy)]
struct GeoPoint {
    latitude_rad: f64,
    longitude_rad: f64,
    altitude_m: f64,
}

impl GeoPoint {
    fn from_llh(llh: Llh) -> Option<Self> {
        if !llh.lat_deg.is_finite() || !llh.lon_deg.is_finite() || !llh.alt_m.is_finite() {
            return None;
        }
        Some(Self {
            latitude_rad: llh.lat_deg.to_radians(),
            longitude_rad: llh.lon_deg.to_radians(),
            altitude_m: llh.alt_m,
        })
    }

    fn to_cartesian(self) -> Option<[f64; 3]> {
        let radius_m = MEAN_EARTH_RADIUS_M + self.altitude_m;
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return None;
        }
        let cos_latitude = self.latitude_rad.cos();
        Some([
            radius_m * cos_latitude * self.longitude_rad.cos(),
            radius_m * cos_latitude * self.longitude_rad.sin(),
            radius_m * self.latitude_rad.sin(),
        ])
    }

    fn from_cartesian(cartesian_m: [f64; 3]) -> Option<Self> {
        let radius_m = (cartesian_m[0] * cartesian_m[0]
            + cartesian_m[1] * cartesian_m[1]
            + cartesian_m[2] * cartesian_m[2])
            .sqrt();
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return None;
        }
        Some(Self {
            latitude_rad: cartesian_m[2].atan2((cartesian_m[0] * cartesian_m[0] + cartesian_m[1] * cartesian_m[1]).sqrt()),
            longitude_rad: cartesian_m[1].atan2(cartesian_m[0]),
            altitude_m: radius_m - MEAN_EARTH_RADIUS_M,
        })
    }
}

#[derive(Debug)]
struct FourierSeries {
    az: f64,
    azr: f64,
    cf2: [f64; FOF2_SERIES_LENGTH],
    cm3: [f64; M3000_SERIES_LENGTH],
}

impl FourierSeries {
    fn new(time: &NequickTime, az: f64) -> Self {
        let month_data = &support_data().ccir_months[(time.month - 1) as usize];
        let azr = (167_273.0 + (az - 63.7) * 1_123.6).sqrt() - 408.99;
        let time_argument = (15.0 * time.utc_hours).to_radians() - std::f64::consts::PI;
        let sc_t = sin_cos_series(time_argument, 6);
        let cf2 = compute_cf2(azr, &month_data.fof2_coefficients, &sc_t);
        let cm3 = compute_cm3(azr, &month_data.m3000_coefficients, &sc_t);
        Self { az, azr, cf2, cm3 }
    }
}

#[derive(Debug)]
struct NequickParameters {
    month: u8,
    azr: f64,
    nm_f2: f64,
    hm_f2_km: f64,
    hm_f1_km: f64,
    hm_e_km: f64,
    b2_bottom_km: f64,
    b1_top_km: f64,
    b1_bottom_km: f64,
    be_top_km: f64,
    be_bottom_km: f64,
    amplitudes: [f64; 3],
}

impl NequickParameters {
    fn new(
        time: &NequickTime,
        fourier: &FourierSeries,
        latitude_rad: f64,
        longitude_rad: f64,
        modip_deg: f64,
    ) -> Self {
        let xeff = effective_solar_angle_rad(time.month, time.utc_hours, latitude_rad, longitude_rad);
        let hm_e_km = 120.0;
        let fo_e = fo_e_mhz(time.month, fourier.az, xeff, latitude_rad);
        let nm_e = 0.124 * fo_e * fo_e;

        let sc_l = sin_cos_series(longitude_rad, 8);
        let fo_f2 = fo_f2_mhz(modip_deg, &fourier.cf2, latitude_rad, &sc_l);
        let m_f2 = m_f2_factor(modip_deg, &fourier.cm3, latitude_rad, &sc_l);
        let nm_f2 = 0.124 * fo_f2 * fo_f2;
        let hm_f2_km = hm_f2_km(fo_e, fo_f2, m_f2);

        let fo_f1 = fo_f1_mhz(fo_e, fo_f2);
        let nm_f1 = if fo_f1 <= 0.0 && fo_e > 2.0 {
            let fo_ep = fo_e + 0.5;
            0.124 * fo_ep * fo_ep
        } else {
            0.124 * fo_f1 * fo_f1
        };
        let hm_f1_km = 0.5 * (hm_f2_km + hm_e_km);

        let a = 0.01 * clip_exp(-3.467 + 0.857 * (fo_f2 * fo_f2).ln() + 2.02 * m_f2.ln());
        let b2_bottom_km = 0.385 * nm_f2 / a;
        let b1_top_km = 0.3 * (hm_f2_km - hm_f1_km);
        let b1_bottom_km = 0.5 * (hm_f1_km - hm_e_km);
        let be_top_km = b1_bottom_km.max(7.0);
        let be_bottom_km = 5.0;
        let amplitudes = layer_amplitudes(
            nm_e,
            nm_f1,
            fo_f1,
            nm_f2,
            hm_f2_km,
            hm_f1_km,
            hm_e_km,
            b2_bottom_km,
            b1_bottom_km,
            be_top_km,
        );

        Self {
            month: time.month,
            azr: fourier.azr,
            nm_f2,
            hm_f2_km,
            hm_f1_km,
            hm_e_km,
            b2_bottom_km,
            b1_top_km,
            b1_bottom_km,
            be_top_km,
            be_bottom_km,
            amplitudes,
        }
    }
}

fn compute_modip(latitude_rad: f64, longitude_rad: f64) -> f64 {
    let x = (latitude_rad + 0.5 * std::f64::consts::PI) / (std::f64::consts::PI / 36.0) + 1.0;
    if x < 1.0 {
        return -90.0;
    }
    if x >= 37.0 {
        return 90.0;
    }
    let i = x.floor() as usize;
    let delta_x = x - i as f64;

    let mut y = (longitude_rad + std::f64::consts::PI) / (std::f64::consts::PI / 18.0) + 1.0;
    while y < 1.0 {
        y += 36.0;
    }
    while y >= 37.0 {
        y -= 36.0;
    }
    let j = y.floor() as usize;
    let delta_y = y - j as f64;

    let grid = &support_data().modip_grid;
    let z1 = interpolate_modip(grid[i - 1][j - 1], grid[i][j - 1], grid[i + 1][j - 1], grid[i + 2][j - 1], delta_x);
    let z2 = interpolate_modip(grid[i - 1][j], grid[i][j], grid[i + 1][j], grid[i + 2][j], delta_x);
    let z3 =
        interpolate_modip(grid[i - 1][j + 1], grid[i][j + 1], grid[i + 1][j + 1], grid[i + 2][j + 1], delta_x);
    let z4 =
        interpolate_modip(grid[i - 1][j + 2], grid[i][j + 2], grid[i + 1][j + 2], grid[i + 2][j + 2], delta_x);
    interpolate_modip(z1, z2, z3, z4, delta_y)
}

fn interpolate_modip(z1: f64, z2: f64, z3: f64, z4: f64, position: f64) -> f64 {
    if position < 5.0e-11 {
        return z2;
    }
    let delta = 2.0 * position - 1.0;
    let g1 = z3 + z2;
    let g2 = z3 - z2;
    let g3 = z4 + z1;
    let g4 = (z4 - z1) / 3.0;
    let a0 = 9.0 * g1 - g3;
    let a1 = 9.0 * g2 - g4;
    let a2 = g3 - g1;
    let a3 = g4 - g2;
    0.0625 * (a0 + delta * (a1 + delta * (a2 + delta * a3)))
}

fn compute_cf2(azr: f64, coefficients: &[f64], sc_t: &[f64]) -> [f64; FOF2_SERIES_LENGTH] {
    let azr_scale = azr * 0.01;
    let inverse_scale = 1.0 - azr_scale;
    let mut array = [0.0; FOF2_SERIES_LENGTH];
    let mut index = 0usize;
    for value in &mut array {
        *value = inverse_scale * coefficients[index] + azr_scale * coefficients[index + 1]
            + (inverse_scale * coefficients[index + 2] + azr_scale * coefficients[index + 3]) * sc_t[0]
            + (inverse_scale * coefficients[index + 4] + azr_scale * coefficients[index + 5]) * sc_t[1]
            + (inverse_scale * coefficients[index + 6] + azr_scale * coefficients[index + 7]) * sc_t[2]
            + (inverse_scale * coefficients[index + 8] + azr_scale * coefficients[index + 9]) * sc_t[3]
            + (inverse_scale * coefficients[index + 10] + azr_scale * coefficients[index + 11]) * sc_t[4]
            + (inverse_scale * coefficients[index + 12] + azr_scale * coefficients[index + 13]) * sc_t[5]
            + (inverse_scale * coefficients[index + 14] + azr_scale * coefficients[index + 15]) * sc_t[6]
            + (inverse_scale * coefficients[index + 16] + azr_scale * coefficients[index + 17]) * sc_t[7]
            + (inverse_scale * coefficients[index + 18] + azr_scale * coefficients[index + 19]) * sc_t[8]
            + (inverse_scale * coefficients[index + 20] + azr_scale * coefficients[index + 21]) * sc_t[9]
            + (inverse_scale * coefficients[index + 22] + azr_scale * coefficients[index + 23]) * sc_t[10]
            + (inverse_scale * coefficients[index + 24] + azr_scale * coefficients[index + 25]) * sc_t[11];
        index += 26;
    }
    array
}

fn compute_cm3(azr: f64, coefficients: &[f64], sc_t: &[f64]) -> [f64; M3000_SERIES_LENGTH] {
    let azr_scale = azr * 0.01;
    let inverse_scale = 1.0 - azr_scale;
    let mut array = [0.0; M3000_SERIES_LENGTH];
    let mut index = 0usize;
    for value in &mut array {
        *value = inverse_scale * coefficients[index] + azr_scale * coefficients[index + 1]
            + (inverse_scale * coefficients[index + 2] + azr_scale * coefficients[index + 3]) * sc_t[0]
            + (inverse_scale * coefficients[index + 4] + azr_scale * coefficients[index + 5]) * sc_t[1]
            + (inverse_scale * coefficients[index + 6] + azr_scale * coefficients[index + 7]) * sc_t[2]
            + (inverse_scale * coefficients[index + 8] + azr_scale * coefficients[index + 9]) * sc_t[3]
            + (inverse_scale * coefficients[index + 10] + azr_scale * coefficients[index + 11]) * sc_t[4]
            + (inverse_scale * coefficients[index + 12] + azr_scale * coefficients[index + 13]) * sc_t[5]
            + (inverse_scale * coefficients[index + 14] + azr_scale * coefficients[index + 15]) * sc_t[6]
            + (inverse_scale * coefficients[index + 16] + azr_scale * coefficients[index + 17]) * sc_t[7];
        index += 18;
    }
    array
}

fn effective_solar_angle_rad(month: u8, utc_hours: f64, latitude_rad: f64, longitude_rad: f64) -> f64 {
    let local_time = utc_hours + longitude_rad / 15.0_f64.to_radians();
    let day_of_year = 30.5 * month as f64 - 15.0;
    let time = day_of_year + (18.0 - utc_hours) / 24.0;
    let am = (0.9856 * time - 3.289).to_radians();
    let al = am + (1.916 * am.sin() + 0.020 * (2.0 * am).sin() + 282.634).to_radians();
    let sin_declination = 0.39782 * al.sin();
    let cos_declination = (1.0 - sin_declination * sin_declination).sqrt();
    let c_zenith = latitude_rad.sin() * sin_declination
        + latitude_rad.cos() * cos_declination * ((std::f64::consts::PI / 12.0) * (12.0 - local_time)).cos();
    let angle_deg = ((1.0 - c_zenith * c_zenith).sqrt()).atan2(c_zenith).to_degrees();
    let effective_angle_deg = join(
        90.0 - 0.24 * clip_exp(20.0 - 0.2 * angle_deg),
        angle_deg,
        12.0,
        angle_deg - DAY_NIGHT_TRANSITION_DEG,
    );
    effective_angle_deg.to_radians()
}

fn fo_e_mhz(month: u8, az: f64, effective_solar_angle_rad: f64, latitude_rad: f64) -> f64 {
    let latitude_deg = latitude_rad.to_degrees();
    let sq_az = az.sqrt();
    let season = match month {
        1 | 2 | 11 | 12 => -1.0,
        3 | 4 | 9 | 10 => 0.0,
        _ => 1.0,
    };
    let ee = clip_exp(0.3 * latitude_deg);
    let seasonal_dependence = season * ((ee - 1.0) / (ee + 1.0));
    let coefficient = 1.112 - 0.019 * seasonal_dependence;
    (coefficient * coefficient * sq_az * effective_solar_angle_rad.cos().powf(0.6) + 0.49).sqrt()
}

fn fo_f2_mhz(modip_deg: f64, cf2: &[f64; FOF2_SERIES_LENGTH], latitude_rad: f64, sc_l: &[f64]) -> f64 {
    const LEGENDRE_GRADES: [usize; 9] = [12, 12, 9, 5, 2, 1, 1, 1, 1];

    let mut frequency = cf2[0];
    let sin_modip = modip_deg.to_radians().sin();
    let mut modip_powers = [0.0; 12];
    modip_powers[0] = 1.0;
    for index in 1..LEGENDRE_GRADES[0] {
        modip_powers[index] = sin_modip * modip_powers[index - 1];
        frequency += modip_powers[index] * cf2[index];
    }

    let mut coefficient_index = 12usize;
    let cos_latitude = latitude_rad.cos();
    let mut latitude_power = cos_latitude;
    for band in 1..LEGENDRE_GRADES.len() {
        let cosine_term = latitude_power * sc_l[2 * band - 1];
        let sine_term = latitude_power * sc_l[2 * band - 2];
        for order in 0..LEGENDRE_GRADES[band] {
            frequency += modip_powers[order] * cosine_term * cf2[coefficient_index];
            coefficient_index += 1;
            frequency += modip_powers[order] * sine_term * cf2[coefficient_index];
            coefficient_index += 1;
        }
        latitude_power *= cos_latitude;
    }

    frequency
}

fn m_f2_factor(modip_deg: f64, cm3: &[f64; M3000_SERIES_LENGTH], latitude_rad: f64, sc_l: &[f64]) -> f64 {
    const LEGENDRE_GRADES: [usize; 7] = [7, 8, 6, 3, 2, 1, 1];

    let mut factor = cm3[0];
    let sin_modip = modip_deg.to_radians().sin();
    let mut modip_powers = [0.0; 12];
    modip_powers[0] = 1.0;
    for index in 1..12 {
        modip_powers[index] = sin_modip * modip_powers[index - 1];
        if index < 7 {
            factor += modip_powers[index] * cm3[index];
        }
    }

    let mut coefficient_index = 7usize;
    let cos_latitude = latitude_rad.cos();
    let mut latitude_power = cos_latitude;
    for band in 1..LEGENDRE_GRADES.len() {
        let cosine_term = latitude_power * sc_l[2 * band - 1];
        let sine_term = latitude_power * sc_l[2 * band - 2];
        for order in 0..LEGENDRE_GRADES[band] {
            factor += modip_powers[order] * cosine_term * cm3[coefficient_index];
            coefficient_index += 1;
            factor += modip_powers[order] * sine_term * cm3[coefficient_index];
            coefficient_index += 1;
        }
        latitude_power *= cos_latitude;
    }

    factor
}

fn hm_f2_km(fo_e_mhz: f64, fo_f2_mhz: f64, m_f2: f64) -> f64 {
    let ratio = join(fo_f2_mhz / fo_e_mhz, 1.75, 20.0, fo_f2_mhz / fo_e_mhz - 1.75);
    let mut delta_m = -0.012;
    if fo_e_mhz >= 1.0e-30 {
        delta_m += 0.253 / (ratio - 1.215);
    }
    let m_f2_sq = m_f2 * m_f2;
    let temp = ((0.0196 * m_f2_sq + 1.0) / (1.2967 * m_f2_sq - 1.0)).sqrt();
    (1490.0 * m_f2 * temp) / (m_f2 + delta_m) - 176.0
}

fn fo_f1_mhz(fo_e_mhz: f64, fo_f2_mhz: f64) -> f64 {
    let temp = join(1.4 * fo_e_mhz, 0.0, 1000.0, fo_e_mhz - 2.0);
    let temp2 = join(0.0, temp, 1000.0, fo_e_mhz - temp);
    let value = join(temp2, 0.85 * temp2, 60.0, 0.85 * fo_f2_mhz - temp2);
    if value < 1.0e-6 { 0.0 } else { value }
}

fn layer_amplitudes(
    nm_e: f64,
    nm_f1: f64,
    fo_f1_mhz: f64,
    nm_f2: f64,
    hm_f2_km: f64,
    hm_f1_km: f64,
    hm_e_km: f64,
    b2_bottom_km: f64,
    b1_bottom_km: f64,
    be_top_km: f64,
) -> [f64; 3] {
    let a1 = 4.0 * nm_f2;
    if fo_f1_mhz < 0.5 {
        [a1, 0.0, 4.0 * (nm_e - epstein(a1, hm_f2_km, b2_bottom_km, hm_e_km))]
    } else {
        let mut a2 = 0.0;
        let mut a3 = 4.0 * nm_e;
        for _ in 0..5 {
            a2 = 4.0
                * (nm_f1 - epstein(a1, hm_f2_km, b2_bottom_km, hm_f1_km) - epstein(a3, hm_e_km, be_top_km, hm_f1_km));
            a2 = join(a2, 0.8 * nm_f1, 1.0, a2 - 0.8 * nm_f1);
            a3 = 4.0
                * (nm_e - epstein(a2, hm_f1_km, b1_bottom_km, hm_e_km) - epstein(a1, hm_f2_km, b2_bottom_km, hm_e_km));
        }
        [a1, a2, join(a3, 0.05, 60.0, a3 - 0.005)]
    }
}

fn epstein(amplitude: f64, layer_height_km: f64, thickness_km: f64, height_km: f64) -> f64 {
    let exp_term = clip_exp((height_km - layer_height_km) / thickness_km);
    amplitude * exp_term / ((1.0 + exp_term) * (1.0 + exp_term))
}

fn join(first: f64, second: f64, width: f64, x: f64) -> f64 {
    let exp_term = clip_exp(width * x);
    (first * exp_term + second) / (exp_term + 1.0)
}

fn clip_exp(power: f64) -> f64 {
    if power > 80.0 {
        5.5406e34
    } else if power < -80.0 {
        1.8049e-35
    } else {
        power.exp()
    }
}

fn sin_cos_series(argument: f64, terms: usize) -> Vec<f64> {
    let mut values = Vec::with_capacity(2 * terms);
    for multiple in 1..=terms {
        let scaled = multiple as f64 * argument;
        values.push(scaled.sin());
        values.push(scaled.cos());
    }
    values
}

#[cfg(test)]
mod tests {
    use super::GalileoNequickModel;
    use bijux_gnss_core::api::Llh;

    struct ValidationCase {
        month: u8,
        utc_hours: f64,
        receiver: Llh,
        satellite: Llh,
        expected_stec_tecu: f64,
    }

    fn validation_case(
        month: u8,
        utc_hours: f64,
        receiver_lon_deg: f64,
        receiver_lat_deg: f64,
        receiver_alt_m: f64,
        satellite_lon_deg: f64,
        satellite_lat_deg: f64,
        satellite_alt_m: f64,
        expected_stec_tecu: f64,
    ) -> ValidationCase {
        ValidationCase {
            month,
            utc_hours,
            receiver: Llh { lat_deg: receiver_lat_deg, lon_deg: receiver_lon_deg, alt_m: receiver_alt_m },
            satellite: Llh {
                lat_deg: satellite_lat_deg,
                lon_deg: satellite_lon_deg,
                alt_m: satellite_alt_m,
            },
            expected_stec_tecu,
        }
    }

    #[test]
    fn nequick_matches_official_annex_e_validation_vectors() {
        let cases = [
            (
                GalileoNequickModel::new([236.831_641, -0.393_628_78, 0.004_028_266_13]),
                [
                    validation_case(4, 0.0, 297.66, 82.49, 78.11, 8.23, 54.29, 20_281_546.18, 20.40),
                    validation_case(4, 0.0, 297.66, 82.49, 78.11, -158.03, 24.05, 20_275_295.43, 53.45),
                    validation_case(4, 4.0, 297.66, 82.49, 78.11, -85.72, 53.69, 20_544_786.65, 18.78),
                ],
            ),
            (
                GalileoNequickModel::new([121.129_893, 0.351_254_133, 0.013_463_534_8]),
                [
                    validation_case(4, 0.0, 40.19, -3.00, -23.32, 76.65, -41.43, 20_157_673.93, 18.26),
                    validation_case(4, 0.0, 40.19, -3.00, -23.32, -13.11, -4.67, 20_194_168.22, 35.84),
                    validation_case(4, 4.0, 40.19, -3.00, -23.32, 107.19, -10.65, 19_943_686.06, 76.77),
                ],
            ),
            (
                GalileoNequickModel::new([2.580_271, 0.127_628_236, 0.025_274_838_4]),
                [
                    validation_case(4, 0.0, 141.13, 39.14, 117.00, 165.14, -13.93, 20_181_976.50, 36.44),
                    validation_case(4, 0.0, 141.13, 39.14, 117.00, 85.59, 36.64, 20_015_444.79, 14.24),
                    validation_case(4, 8.0, 141.13, 39.14, 117.00, 84.26, 54.68, 20_305_726.98, 15.90),
                ],
            ),
        ];

        for (model, vectors) in cases {
            for case in vectors {
                let stec_tecu = model
                    .stec_tecu(case.month, case.utc_hours, case.receiver, case.satellite)
                    .expect("official validation case should converge");
                assert!(
                    (stec_tecu - case.expected_stec_tecu).abs() <= 0.05,
                    "expected {} TECU, got {} TECU for month={}, utc_hours={}, receiver={:?}, satellite={:?}",
                    case.expected_stec_tecu,
                    stec_tecu,
                    case.month,
                    case.utc_hours,
                    case.receiver,
                    case.satellite,
                );
            }
        }
    }
}
