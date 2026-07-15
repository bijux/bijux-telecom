#![allow(missing_docs)]

use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, ObsSignalTiming, SatId};

use crate::orbits::broadcast_orbit::{
    earth_rotation_correction, propagate_broadcast_orbit, solve_broadcast_orbit_anomaly,
    wrap_gnss_week_seconds, BroadcastKeplerianOrbit, BroadcastOrbitAnomaly,
    BroadcastOrbitConstants,
};

const OMEGA_E_DOT: f64 = 7.292_115_0e-5;
const MU: f64 = 3.986_004_418e14;
const RELATIVISTIC_F: f64 = -4.442_807_309e-10;
const MAX_NAVIGATION_AGE_S: f64 = 14_400.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouSystemTime {
    pub week: u16,
    pub sow_s: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BeidouSignalHealth {
    pub autonomous_satellite_good: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouIonosphericCorrection {
    pub alpha0: f64,
    pub alpha1: f64,
    pub alpha2: f64,
    pub alpha3: f64,
    pub beta0: f64,
    pub beta1: f64,
    pub beta2: f64,
    pub beta3: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouClockCorrection {
    pub toc_s: f64,
    pub aodc: u8,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd1_s: f64,
    pub tgd2_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouSatelliteClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub drift_rate_s_per_s2: f64,
    pub base_bias_s: f64,
    pub relativistic_s: f64,
    pub tgd1_s: f64,
    pub tgd2_s: f64,
}

impl BeidouSatelliteClockCorrection {
    pub fn from_bias_s(bias_s: f64) -> Self {
        Self {
            bias_s,
            drift_s_per_s: 0.0,
            drift_rate_s_per_s2: 0.0,
            base_bias_s: bias_s,
            relativistic_s: 0.0,
            tgd1_s: 0.0,
            tgd2_s: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub vz_mps: f64,
    pub clock_correction: BeidouSatelliteClockCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouNavigationAge {
    pub reference_time_s: f64,
    pub toe_age_s: f64,
    pub toc_age_s: f64,
    pub max_age_s: f64,
}

impl BeidouNavigationAge {
    pub fn is_valid(&self) -> bool {
        self.toe_age_s <= self.max_age_s && self.toc_age_s <= self.max_age_s
    }

    pub fn is_stale(&self) -> bool {
        !self.is_valid()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouEphemeris {
    pub sat: SatId,
    pub aode: u8,
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

impl BeidouEphemeris {
    pub fn with_prn(prn: u8, aode: u8) -> Self {
        Self {
            sat: SatId { constellation: Constellation::Beidou, prn },
            aode,
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
pub struct BeidouBroadcastNavigationData {
    pub sat: SatId,
    pub bdt: BeidouSystemTime,
    pub urai: u8,
    pub signal_health: BeidouSignalHealth,
    pub clock: BeidouClockCorrection,
    pub ephemeris: BeidouEphemeris,
    pub ionosphere: BeidouIonosphericCorrection,
}

pub fn beidou_satellite_clock_correction_b1i(
    navigation: &BeidouBroadcastNavigationData,
    t_tx_s: f64,
) -> BeidouSatelliteClockCorrection {
    let anomaly = solve_broadcast_orbit_anomaly(
        beidou_broadcast_orbit(&navigation.ephemeris),
        t_tx_s,
        beidou_orbit_constants(),
    );
    satellite_clock_correction_from_anomaly(navigation, t_tx_s, anomaly)
}

pub fn beidou_earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
) -> BeidouEarthRotationCorrection {
    let correction = earth_rotation_correction(x_m, y_m, signal_travel_time_s, OMEGA_E_DOT);
    BeidouEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad: correction.rotation_rad,
        delta_x_m: correction.delta_x_m,
        delta_y_m: correction.delta_y_m,
    }
}

pub fn sat_state_beidou_b1i(
    navigation: &BeidouBroadcastNavigationData,
    t_tx_s: f64,
    tau_s: f64,
) -> BeidouSatState {
    let orbit_state = propagate_broadcast_orbit(
        beidou_broadcast_orbit(&navigation.ephemeris),
        t_tx_s,
        tau_s,
        beidou_orbit_constants(),
    );

    BeidouSatState {
        x_m: orbit_state.x_m,
        y_m: orbit_state.y_m,
        z_m: orbit_state.z_m,
        vx_mps: orbit_state.vx_mps,
        vy_mps: orbit_state.vy_mps,
        vz_mps: orbit_state.vz_mps,
        clock_correction: satellite_clock_correction_from_anomaly(
            navigation,
            t_tx_s,
            solve_broadcast_orbit_anomaly(
                beidou_broadcast_orbit(&navigation.ephemeris),
                t_tx_s,
                beidou_orbit_constants(),
            ),
        ),
    }
}

pub fn beidou_navigation_age(
    navigation: &BeidouBroadcastNavigationData,
    reference_time_s: f64,
) -> BeidouNavigationAge {
    BeidouNavigationAge {
        reference_time_s,
        toe_age_s: wrap_time(reference_time_s - navigation.ephemeris.toe_s).abs(),
        toc_age_s: wrap_time(reference_time_s - navigation.clock.toc_s).abs(),
        max_age_s: MAX_NAVIGATION_AGE_S,
    }
}

pub fn is_beidou_navigation_valid(
    navigation: &BeidouBroadcastNavigationData,
    reference_time_s: f64,
) -> bool {
    beidou_navigation_age(navigation, reference_time_s).is_valid()
}

pub fn select_best_beidou_navigation<'a>(
    navigations: &'a [BeidouBroadcastNavigationData],
    sat: SatId,
    reference_time_s: f64,
) -> Option<&'a BeidouBroadcastNavigationData> {
    navigations.iter().filter(|navigation| navigation.sat == sat).min_by(|left, right| {
        let left_age = beidou_navigation_age(left, reference_time_s);
        let right_age = beidou_navigation_age(right, reference_time_s);
        let left_score = left_age.toe_age_s.max(left_age.toc_age_s);
        let right_score = right_age.toe_age_s.max(right_age.toc_age_s);
        left_score.partial_cmp(&right_score).unwrap_or(Ordering::Equal)
    })
}

pub fn sat_state_beidou_b1i_at_receive_time(
    navigation: &BeidouBroadcastNavigationData,
    receive_sow_s: f64,
    signal_travel_time_s: f64,
) -> BeidouSatState {
    sat_state_beidou_b1i(navigation, receive_sow_s - signal_travel_time_s, signal_travel_time_s)
}

pub fn sat_state_beidou_b1i_from_observation(
    navigation: &BeidouBroadcastNavigationData,
    receive_sow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> BeidouSatState {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_sow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_sow_s - signal_travel_time_s);
    sat_state_beidou_b1i(navigation, transmit_sow_s, signal_travel_time_s)
}

fn satellite_clock_correction_from_anomaly(
    navigation: &BeidouBroadcastNavigationData,
    t_tx_s: f64,
    anomaly: BroadcastOrbitAnomaly,
) -> BeidouSatelliteClockCorrection {
    let dt = wrap_time(t_tx_s - navigation.clock.toc_s);
    let base_bias_s =
        navigation.clock.af0 + navigation.clock.af1 * dt + navigation.clock.af2 * dt * dt;
    let sin_e = anomaly.kepler.sin_eccentric_anomaly;
    let relativistic_s =
        RELATIVISTIC_F * navigation.ephemeris.e * navigation.ephemeris.sqrt_a * sin_e;
    let relativistic_drift_s_per_s = RELATIVISTIC_F
        * navigation.ephemeris.e
        * navigation.ephemeris.sqrt_a
        * anomaly.kepler.cos_eccentric_anomaly
        * anomaly.kepler.eccentric_anomaly_rate_rad_s;

    BeidouSatelliteClockCorrection {
        bias_s: base_bias_s + relativistic_s - navigation.clock.tgd1_s,
        drift_s_per_s: navigation.clock.af1
            + 2.0 * navigation.clock.af2 * dt
            + relativistic_drift_s_per_s,
        drift_rate_s_per_s2: 2.0 * navigation.clock.af2,
        base_bias_s,
        relativistic_s,
        tgd1_s: navigation.clock.tgd1_s,
        tgd2_s: navigation.clock.tgd2_s,
    }
}

fn wrap_time(t: f64) -> f64 {
    wrap_gnss_week_seconds(t)
}

fn beidou_orbit_constants() -> BroadcastOrbitConstants {
    BroadcastOrbitConstants {
        gravitational_parameter_m3_s2: MU,
        earth_rotation_rate_rad_s: OMEGA_E_DOT,
    }
}

fn beidou_broadcast_orbit(eph: &BeidouEphemeris) -> BroadcastKeplerianOrbit {
    BroadcastKeplerianOrbit {
        toe_s: eph.toe_s,
        sqrt_a_m: eph.sqrt_a,
        eccentricity: eph.e,
        inclination_rad: eph.i0,
        inclination_rate_rad_s: eph.idot,
        right_ascension_rad: eph.omega0,
        right_ascension_rate_rad_s: eph.omegadot,
        argument_of_perigee_rad: eph.w,
        mean_anomaly_rad: eph.m0,
        mean_motion_delta_rad_s: eph.delta_n,
        latitude_cosine_correction_rad: eph.cuc,
        latitude_sine_correction_rad: eph.cus,
        radius_cosine_correction_m: eph.crc,
        radius_sine_correction_m: eph.crs,
        inclination_cosine_correction_rad: eph.cic,
        inclination_sine_correction_rad: eph.cis,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bijux_gnss_core::api::{GpsTime, Seconds};

    const TEST_OMEGA_E_DOT: f64 = 7.292_115_0e-5;
    const TEST_MU: f64 = 3.986_004_418e14;
    const TEST_RELATIVISTIC_F: f64 = -4.442_807_309e-10;

    fn sample_navigation() -> BeidouBroadcastNavigationData {
        BeidouBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            bdt: BeidouSystemTime { week: 987, sow_s: 64_800 },
            urai: 2,
            signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
            clock: BeidouClockCorrection {
                toc_s: 64_800.0,
                aodc: 3,
                af0: -1.8e-4,
                af1: 2.2e-12,
                af2: -2.6e-19,
                tgd1_s: -1.1e-9,
                tgd2_s: 2.2e-9,
            },
            ephemeris: BeidouEphemeris {
                sat: SatId { constellation: Constellation::Beidou, prn: 11 },
                aode: 3,
                toe_s: 64_800.0,
                sqrt_a: 5_282.625_128,
                e: 0.002_34,
                i0: 0.958,
                idot: -1.9e-10,
                omega0: 0.87,
                omegadot: -6.2e-9,
                w: -0.42,
                m0: 1.12,
                delta_n: 4.3e-9,
                cuc: -2.3e-6,
                cus: 3.1e-6,
                crc: 145.0,
                crs: -82.0,
                cic: 2.6e-7,
                cis: -2.2e-7,
            },
            ionosphere: BeidouIonosphericCorrection {
                alpha0: 1.0e-8,
                alpha1: -2.0e-8,
                alpha2: 3.0e-8,
                alpha3: -4.0e-8,
                beta0: 1.2e5,
                beta1: -2.4e5,
                beta2: 3.6e5,
                beta3: -4.8e5,
            },
        }
    }

    fn reference_state(
        navigation: &BeidouBroadcastNavigationData,
        transmit_sow_s: f64,
        signal_travel_time_s: f64,
    ) -> BeidouSatState {
        let eph = &navigation.ephemeris;
        let a = eph.sqrt_a * eph.sqrt_a;
        let n0 = (TEST_MU / (a * a * a)).sqrt();
        let n = n0 + eph.delta_n;
        let tk = reference_wrap_time(transmit_sow_s - eph.toe_s);
        let m = eph.m0 + n * tk;
        let e_anom = reference_solve_kepler(m, eph.e);
        let sin_e = e_anom.sin();
        let cos_e = e_anom.cos();
        let v = ((1.0 - eph.e * eph.e).sqrt() * sin_e).atan2(cos_e - eph.e);
        let e_anom_rate = n / (1.0 - eph.e * cos_e);
        let v_rate = (1.0 - eph.e * eph.e).sqrt() * e_anom_rate / (1.0 - eph.e * cos_e);
        let phi = v + eph.w;
        let phi_rate = v_rate;
        let sin2phi = (2.0 * phi).sin();
        let cos2phi = (2.0 * phi).cos();
        let sin2phi_rate = 2.0 * phi_rate * cos2phi;
        let cos2phi_rate = -2.0 * phi_rate * sin2phi;
        let u = phi + eph.cuc * cos2phi + eph.cus * sin2phi;
        let u_rate = phi_rate + eph.cuc * cos2phi_rate + eph.cus * sin2phi_rate;
        let r = a * (1.0 - eph.e * cos_e) + eph.crc * cos2phi + eph.crs * sin2phi;
        let r_rate =
            a * eph.e * sin_e * e_anom_rate + eph.crc * cos2phi_rate + eph.crs * sin2phi_rate;
        let i = eph.i0 + eph.idot * tk + eph.cic * cos2phi + eph.cis * sin2phi;
        let i_rate = eph.idot + eph.cic * cos2phi_rate + eph.cis * sin2phi_rate;
        let x_orb = r * u.cos();
        let y_orb = r * u.sin();
        let x_orb_rate = r_rate * u.cos() - r * u.sin() * u_rate;
        let y_orb_rate = r_rate * u.sin() + r * u.cos() * u_rate;
        let omega =
            eph.omega0 + (eph.omegadot - TEST_OMEGA_E_DOT) * tk - TEST_OMEGA_E_DOT * eph.toe_s;
        let omega_rate = eph.omegadot - TEST_OMEGA_E_DOT;
        let cos_omega = omega.cos();
        let sin_omega = omega.sin();
        let cos_i = i.cos();
        let sin_i = i.sin();

        let mut x_m = x_orb * omega.cos() - y_orb * i.cos() * omega.sin();
        let mut y_m = x_orb * omega.sin() + y_orb * i.cos() * omega.cos();
        let z_m = y_orb * i.sin();
        let x_rate_mps = x_orb_rate * cos_omega
            - x_orb * sin_omega * omega_rate
            - y_orb_rate * cos_i * sin_omega
            + y_orb * sin_i * i_rate * sin_omega
            - y_orb * cos_i * cos_omega * omega_rate;
        let y_rate_mps = x_orb_rate * sin_omega
            + x_orb * cos_omega * omega_rate
            + y_orb_rate * cos_i * cos_omega
            - y_orb * sin_i * i_rate * cos_omega
            - y_orb * cos_i * sin_omega * omega_rate;
        let z_rate_mps = y_orb_rate * sin_i + y_orb * cos_i * i_rate;

        let rotation_rad = TEST_OMEGA_E_DOT * signal_travel_time_s;
        let rotated_x_m = rotation_rad.cos() * x_m + rotation_rad.sin() * y_m;
        let rotated_y_m = -rotation_rad.sin() * x_m + rotation_rad.cos() * y_m;
        let vx_mps = rotation_rad.cos() * x_rate_mps + rotation_rad.sin() * y_rate_mps;
        let vy_mps = -rotation_rad.sin() * x_rate_mps + rotation_rad.cos() * y_rate_mps;
        let vz_mps = z_rate_mps;
        x_m = rotated_x_m;
        y_m = rotated_y_m;

        let dt = reference_wrap_time(transmit_sow_s - navigation.clock.toc_s);
        let base_bias_s =
            navigation.clock.af0 + navigation.clock.af1 * dt + navigation.clock.af2 * dt * dt;
        let relativistic_s = TEST_RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
        let relativistic_drift_s_per_s =
            TEST_RELATIVISTIC_F * eph.e * eph.sqrt_a * cos_e * e_anom_rate;

        BeidouSatState {
            x_m,
            y_m,
            z_m,
            vx_mps,
            vy_mps,
            vz_mps,
            clock_correction: BeidouSatelliteClockCorrection {
                bias_s: base_bias_s + relativistic_s - navigation.clock.tgd1_s,
                drift_s_per_s: navigation.clock.af1
                    + 2.0 * navigation.clock.af2 * dt
                    + relativistic_drift_s_per_s,
                drift_rate_s_per_s2: 2.0 * navigation.clock.af2,
                base_bias_s,
                relativistic_s,
                tgd1_s: navigation.clock.tgd1_s,
                tgd2_s: navigation.clock.tgd2_s,
            },
        }
    }

    fn reference_solve_kepler(m: f64, e: f64) -> f64 {
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
        e_anom
    }

    fn reference_wrap_time(mut t: f64) -> f64 {
        while t > 302_400.0 {
            t -= 604_800.0;
        }
        while t < -302_400.0 {
            t += 604_800.0;
        }
        t
    }

    #[test]
    fn beidou_satellite_state_matches_independent_reference() {
        let navigation = sample_navigation();
        let transmit_sow_s = 65_432.123;
        let signal_travel_time_s = 0.081_4;

        let state = sat_state_beidou_b1i(&navigation, transmit_sow_s, signal_travel_time_s);
        let expected = reference_state(&navigation, transmit_sow_s, signal_travel_time_s);

        assert!((state.x_m - expected.x_m).abs() < 1.0e-6);
        assert!((state.y_m - expected.y_m).abs() < 1.0e-6);
        assert!((state.z_m - expected.z_m).abs() < 1.0e-6);
        assert!((state.vx_mps - expected.vx_mps).abs() < 1.0e-6);
        assert!((state.vy_mps - expected.vy_mps).abs() < 1.0e-6);
        assert!((state.vz_mps - expected.vz_mps).abs() < 1.0e-6);
        assert!((state.clock_correction.bias_s - expected.clock_correction.bias_s).abs() < 1.0e-15);
        assert!(
            (state.clock_correction.drift_s_per_s - expected.clock_correction.drift_s_per_s).abs()
                < 1.0e-18
        );
    }

    #[test]
    fn beidou_navigation_selection_prefers_fresh_navigation() {
        let fresh = sample_navigation();
        let mut stale = sample_navigation();
        stale.clock.toc_s -= 18_000.0;
        stale.ephemeris.toe_s -= 18_000.0;

        let age = beidou_navigation_age(&fresh, 65_000.0);
        let candidates = [stale, fresh.clone()];
        let selected =
            select_best_beidou_navigation(&candidates, fresh.sat, 65_000.0).expect("selected nav");

        assert!(age.is_valid());
        assert_eq!(selected.clock.toc_s, fresh.clock.toc_s);
        assert_eq!(selected.ephemeris.toe_s, fresh.ephemeris.toe_s);
    }

    #[test]
    fn beidou_state_from_observation_uses_explicit_signal_timing() {
        let navigation = sample_navigation();
        let signal_timing = ObsSignalTiming {
            signal_travel_time_s: Seconds(0.081_4),
            transmit_gps_time: GpsTime { week: navigation.bdt.week as u32, tow_s: 65_432.123 },
        };

        let direct_state = sat_state_beidou_b1i(
            &navigation,
            signal_timing.transmit_gps_time.tow_s,
            signal_timing.signal_travel_time_s.0,
        );
        let observation_state = sat_state_beidou_b1i_from_observation(
            &navigation,
            65_432.205,
            24_000_000.0,
            Some(signal_timing),
        );
        let clock = beidou_satellite_clock_correction_b1i(
            &navigation,
            signal_timing.transmit_gps_time.tow_s,
        );

        assert!((direct_state.x_m - observation_state.x_m).abs() < 1.0e-9);
        assert!((direct_state.y_m - observation_state.y_m).abs() < 1.0e-9);
        assert!((direct_state.z_m - observation_state.z_m).abs() < 1.0e-9);
        assert!((direct_state.clock_correction.bias_s - clock.bias_s).abs() < 1.0e-18);
    }
}
