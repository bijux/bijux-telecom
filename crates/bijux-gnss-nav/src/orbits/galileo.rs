#![allow(missing_docs)]

use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, ObsSignalTiming, SatId};

const OMEGA_E_DOT: f64 = 7.292_115_146_7e-5;
const MU: f64 = 3.986_004_418e14;
const RELATIVISTIC_F: f64 = -4.442_807_309e-10;
const MAX_NAVIGATION_AGE_S: f64 = 14_400.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoSystemTime {
    pub week: u16,
    pub tow_s: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoSignalHealth {
    pub e5b_signal_health: u8,
    pub e1b_signal_health: u8,
    pub e5b_data_valid: bool,
    pub e1b_data_valid: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoIonosphericDisturbanceFlags {
    pub region_1: bool,
    pub region_2: bool,
    pub region_3: bool,
    pub region_4: bool,
    pub region_5: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoIonosphericCorrection {
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub disturbance_flags: GalileoIonosphericDisturbanceFlags,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoClockCorrection {
    pub t0c_s: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub bgd_e1_e5a_s: f64,
    pub bgd_e1_e5b_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoSatelliteClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub drift_rate_s_per_s2: f64,
    pub base_bias_s: f64,
    pub relativistic_s: f64,
    pub bgd_e1_e5a_s: f64,
    pub bgd_e1_e5b_s: f64,
}

impl GalileoSatelliteClockCorrection {
    pub fn from_bias_s(bias_s: f64) -> Self {
        Self {
            bias_s,
            drift_s_per_s: 0.0,
            drift_rate_s_per_s2: 0.0,
            base_bias_s: bias_s,
            relativistic_s: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_correction: GalileoSatelliteClockCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoNavigationAge {
    pub reference_time_s: f64,
    pub toe_age_s: f64,
    pub toc_age_s: f64,
    pub max_age_s: f64,
}

impl GalileoNavigationAge {
    pub fn is_valid(&self) -> bool {
        self.toe_age_s <= self.max_age_s && self.toc_age_s <= self.max_age_s
    }

    pub fn is_stale(&self) -> bool {
        !self.is_valid()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoEphemeris {
    pub sat: SatId,
    pub iodnav: u16,
    pub toe_s: f64,
    pub sqrt_a: f64,
    pub e: f64,
    pub i0: f64,
    pub idot: f64,
    pub omega0: f64,
    pub omegadot: f64,
    pub w: f64,
    pub m0: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
}

impl GalileoEphemeris {
    pub fn with_prn(prn: u8, iodnav: u16) -> Self {
        Self {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav,
            toe_s: 0.0,
            sqrt_a: 0.0,
            e: 0.0,
            i0: 0.0,
            idot: 0.0,
            omega0: 0.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoBroadcastNavigationData {
    pub sat: SatId,
    pub iodnav: u16,
    pub gst: GalileoSystemTime,
    pub sisa_e1_e5b: u8,
    pub signal_health: GalileoSignalHealth,
    pub clock: GalileoClockCorrection,
    pub ephemeris: GalileoEphemeris,
    pub ionosphere: GalileoIonosphericCorrection,
}

pub fn galileo_satellite_clock_correction_e1(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
) -> GalileoSatelliteClockCorrection {
    let eph = &navigation.ephemeris;
    let a = eph.sqrt_a * eph.sqrt_a;
    let n0 = (MU / (a * a * a)).sqrt();
    let n = n0 + eph.delta_n;
    let tk = wrap_time(t_tx_s - eph.toe_s);
    let m = eph.m0 + n * tk;
    let (_e_anom, sin_e, _cos_e) = solve_kepler(m, eph.e);
    satellite_clock_correction_from_sin_e(navigation, t_tx_s, sin_e)
}

pub fn galileo_earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
) -> GalileoEarthRotationCorrection {
    let rotation_rad = OMEGA_E_DOT * signal_travel_time_s;
    let (x_rotated_m, y_rotated_m) = if rotation_rad.abs() > 0.0 {
        let cos_rot = rotation_rad.cos();
        let sin_rot = rotation_rad.sin();
        (cos_rot * x_m + sin_rot * y_m, -sin_rot * x_m + cos_rot * y_m)
    } else {
        (x_m, y_m)
    };
    GalileoEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad,
        delta_x_m: x_rotated_m - x_m,
        delta_y_m: y_rotated_m - y_m,
    }
}

pub fn sat_state_galileo_e1(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
    tau_s: f64,
) -> GalileoSatState {
    let eph = &navigation.ephemeris;
    let a = eph.sqrt_a * eph.sqrt_a;
    let n0 = (MU / (a * a * a)).sqrt();
    let n = n0 + eph.delta_n;
    let tk = wrap_time(t_tx_s - eph.toe_s);

    let m = eph.m0 + n * tk;
    let (_e_anom, sin_e, cos_e) = solve_kepler(m, eph.e);
    let v = (1.0 - eph.e * eph.e).sqrt() * sin_e;
    let v = v.atan2(cos_e - eph.e);
    let phi = v + eph.w;
    let sin2phi = (2.0 * phi).sin();
    let cos2phi = (2.0 * phi).cos();
    let u = phi + eph.cuc * cos2phi + eph.cus * sin2phi;
    let r = a * (1.0 - eph.e * cos_e) + eph.crc * cos2phi + eph.crs * sin2phi;
    let i = eph.i0 + eph.idot * tk + eph.cic * cos2phi + eph.cis * sin2phi;

    let x_orb = r * u.cos();
    let y_orb = r * u.sin();
    let omega = eph.omega0 + (eph.omegadot - OMEGA_E_DOT) * tk - OMEGA_E_DOT * eph.toe_s;

    let cos_omega = omega.cos();
    let sin_omega = omega.sin();
    let cos_i = i.cos();
    let sin_i = i.sin();

    let mut x_m = x_orb * cos_omega - y_orb * cos_i * sin_omega;
    let mut y_m = x_orb * sin_omega + y_orb * cos_i * cos_omega;
    let z_m = y_orb * sin_i;

    let earth_rotation = galileo_earth_rotation_correction(x_m, y_m, tau_s);
    x_m += earth_rotation.delta_x_m;
    y_m += earth_rotation.delta_y_m;

    GalileoSatState {
        x_m,
        y_m,
        z_m,
        clock_correction: satellite_clock_correction_from_sin_e(navigation, t_tx_s, sin_e),
    }
}

pub fn galileo_navigation_age(
    navigation: &GalileoBroadcastNavigationData,
    reference_time_s: f64,
) -> GalileoNavigationAge {
    GalileoNavigationAge {
        reference_time_s,
        toe_age_s: wrap_time(reference_time_s - navigation.ephemeris.toe_s).abs(),
        toc_age_s: wrap_time(reference_time_s - navigation.clock.t0c_s).abs(),
        max_age_s: MAX_NAVIGATION_AGE_S,
    }
}

pub fn is_galileo_navigation_valid(
    navigation: &GalileoBroadcastNavigationData,
    reference_time_s: f64,
) -> bool {
    galileo_navigation_age(navigation, reference_time_s).is_valid()
}

pub fn select_best_galileo_navigation<'a>(
    navigations: &'a [GalileoBroadcastNavigationData],
    sat: SatId,
    reference_time_s: f64,
) -> Option<&'a GalileoBroadcastNavigationData> {
    navigations.iter().filter(|navigation| navigation.sat == sat).min_by(|left, right| {
        let left_age = galileo_navigation_age(left, reference_time_s);
        let right_age = galileo_navigation_age(right, reference_time_s);
        let left_score = left_age.toe_age_s.max(left_age.toc_age_s);
        let right_score = right_age.toe_age_s.max(right_age.toc_age_s);
        left_score.partial_cmp(&right_score).unwrap_or(Ordering::Equal)
    })
}

pub fn sat_state_galileo_e1_at_receive_time(
    navigation: &GalileoBroadcastNavigationData,
    receive_tow_s: f64,
    signal_travel_time_s: f64,
) -> GalileoSatState {
    sat_state_galileo_e1(
        navigation,
        receive_tow_s - signal_travel_time_s,
        signal_travel_time_s,
    )
}

pub fn sat_state_galileo_e1_from_observation(
    navigation: &GalileoBroadcastNavigationData,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> GalileoSatState {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_tow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_tow_s - signal_travel_time_s);
    sat_state_galileo_e1(navigation, transmit_tow_s, signal_travel_time_s)
}

pub fn solve_kepler(m: f64, e: f64) -> (f64, f64, f64) {
    let mut e_anom = m;
    for _ in 0..12 {
        let f = e_anom - e * e_anom.sin() - m;
        let f_prime = 1.0 - e * e_anom.cos();
        let step = f / f_prime;
        e_anom -= step;
        if step.abs() < 1e-12 {
            break;
        }
    }
    if !e_anom.is_finite() {
        e_anom = m;
    }
    (e_anom, e_anom.sin(), e_anom.cos())
}

fn satellite_clock_correction_from_sin_e(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
    sin_e: f64,
) -> GalileoSatelliteClockCorrection {
    let dt = wrap_time(t_tx_s - navigation.clock.t0c_s);
    let base_bias_s = navigation.clock.af0
        + navigation.clock.af1 * dt
        + navigation.clock.af2 * dt * dt;
    let relativistic_s =
        RELATIVISTIC_F * navigation.ephemeris.e * navigation.ephemeris.sqrt_a * sin_e;

    GalileoSatelliteClockCorrection {
        bias_s: base_bias_s + relativistic_s - navigation.clock.bgd_e1_e5b_s,
        drift_s_per_s: navigation.clock.af1 + 2.0 * navigation.clock.af2 * dt,
        drift_rate_s_per_s2: 2.0 * navigation.clock.af2,
        base_bias_s,
        relativistic_s,
        bgd_e1_e5a_s: navigation.clock.bgd_e1_e5a_s,
        bgd_e1_e5b_s: navigation.clock.bgd_e1_e5b_s,
    }
}

fn wrap_time(mut t: f64) -> f64 {
    let half = 302_400.0;
    while t > half {
        t -= 604_800.0;
    }
    while t < -half {
        t += 604_800.0;
    }
    t
}
