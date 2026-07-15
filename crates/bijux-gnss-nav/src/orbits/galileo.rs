#![allow(missing_docs)]

use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, GpsTime, Llh, ObsSignalTiming, SatId, SigId};
use bijux_gnss_signal::api::signal_registry;

use crate::models::nequick::model::GalileoNequickModel;
use crate::orbits::broadcast_orbit::{
    earth_rotation_correction, propagate_broadcast_orbit, solve_broadcast_orbit_anomaly,
    wrap_gnss_week_seconds, BroadcastKeplerianOrbit, BroadcastOrbitConstants,
};

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

impl GalileoIonosphericCorrection {
    pub fn nequick_model(&self) -> GalileoNequickModel {
        GalileoNequickModel::new([self.ai0, self.ai1, self.ai2])
    }
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

impl GalileoBroadcastNavigationData {
    pub fn nequick_model(&self) -> GalileoNequickModel {
        self.ionosphere.nequick_model()
    }

    pub fn nequick_delay_m(
        &self,
        signal: SigId,
        receiver: Llh,
        satellite: Llh,
        gps_time: GpsTime,
    ) -> Option<f64> {
        if signal.sat != self.sat || signal.sat.constellation != Constellation::Galileo {
            return None;
        }
        let carrier_hz = signal_registry(signal.sat.constellation, signal.band, signal.code)?
            .spec
            .carrier_hz
            .value();
        self.nequick_model().delay_m_at_gps_time(gps_time, receiver, satellite, carrier_hz)
    }
}

pub fn galileo_satellite_clock_correction_e1(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
) -> GalileoSatelliteClockCorrection {
    let anomaly = solve_broadcast_orbit_anomaly(
        galileo_broadcast_orbit(&navigation.ephemeris),
        t_tx_s,
        galileo_orbit_constants(),
    );
    satellite_clock_correction_from_sin_e(navigation, t_tx_s, anomaly.kepler.sin_eccentric_anomaly)
}

pub fn galileo_earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
) -> GalileoEarthRotationCorrection {
    let correction = earth_rotation_correction(x_m, y_m, signal_travel_time_s, OMEGA_E_DOT);
    GalileoEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad: correction.rotation_rad,
        delta_x_m: correction.delta_x_m,
        delta_y_m: correction.delta_y_m,
    }
}

pub fn sat_state_galileo_e1(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
    tau_s: f64,
) -> GalileoSatState {
    let orbit_state = propagate_broadcast_orbit(
        galileo_broadcast_orbit(&navigation.ephemeris),
        t_tx_s,
        tau_s,
        galileo_orbit_constants(),
    );

    GalileoSatState {
        x_m: orbit_state.x_m,
        y_m: orbit_state.y_m,
        z_m: orbit_state.z_m,
        clock_correction: satellite_clock_correction_from_sin_e(
            navigation,
            t_tx_s,
            orbit_state.sin_eccentric_anomaly,
        ),
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
    sat_state_galileo_e1(navigation, receive_tow_s - signal_travel_time_s, signal_travel_time_s)
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

fn satellite_clock_correction_from_sin_e(
    navigation: &GalileoBroadcastNavigationData,
    t_tx_s: f64,
    sin_e: f64,
) -> GalileoSatelliteClockCorrection {
    let dt = wrap_time(t_tx_s - navigation.clock.t0c_s);
    let base_bias_s =
        navigation.clock.af0 + navigation.clock.af1 * dt + navigation.clock.af2 * dt * dt;
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

fn wrap_time(t: f64) -> f64 {
    wrap_gnss_week_seconds(t)
}

fn galileo_orbit_constants() -> BroadcastOrbitConstants {
    BroadcastOrbitConstants {
        gravitational_parameter_m3_s2: MU,
        earth_rotation_rate_rad_s: OMEGA_E_DOT,
    }
}

fn galileo_broadcast_orbit(eph: &GalileoEphemeris) -> BroadcastKeplerianOrbit {
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
    use bijux_gnss_core::api::{GpsTime, Llh, Seconds, SigId, SignalBand, SignalCode};

    const TEST_OMEGA_E_DOT: f64 = 7.292_115_146_7e-5;
    const TEST_MU: f64 = 3.986_004_418e14;
    const TEST_RELATIVISTIC_F: f64 = -4.442_807_309e-10;

    fn sample_navigation() -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Galileo, prn: 19 },
            iodnav: 0x1A5,
            gst: GalileoSystemTime { week: 2222, tow_s: 456_789 },
            sisa_e1_e5b: 77,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s: 66_000.0,
                af0: -1.7e-4,
                af1: 2.5e-12,
                af2: -3.0e-19,
                bgd_e1_e5a_s: -1.1e-9,
                bgd_e1_e5b_s: 2.4e-9,
            },
            ephemeris: GalileoEphemeris {
                sat: SatId { constellation: Constellation::Galileo, prn: 19 },
                iodnav: 0x1A5,
                toe_s: 64_800.0,
                sqrt_a: 5_440.612_319,
                e: 0.001_23,
                i0: 0.953,
                idot: -2.1e-10,
                omega0: 1.17,
                omegadot: -5.8e-9,
                w: -0.37,
                m0: 0.84,
                delta_n: 4.7e-9,
                cuc: -3.2e-6,
                cus: 4.1e-6,
                crc: 178.0,
                crs: -91.0,
                cic: 1.9e-7,
                cis: -2.4e-7,
            },
            ionosphere: GalileoIonosphericCorrection {
                ai0: 121.129_893,
                ai1: 0.351_254_133,
                ai2: 0.013_463_534_8,
                disturbance_flags: GalileoIonosphericDisturbanceFlags {
                    region_1: false,
                    region_2: false,
                    region_3: false,
                    region_4: false,
                    region_5: false,
                },
            },
        }
    }

    fn reference_state(
        navigation: &GalileoBroadcastNavigationData,
        transmit_tow_s: f64,
        signal_travel_time_s: f64,
    ) -> GalileoSatState {
        let eph = &navigation.ephemeris;
        let a = eph.sqrt_a * eph.sqrt_a;
        let n0 = (TEST_MU / (a * a * a)).sqrt();
        let n = n0 + eph.delta_n;
        let tk = reference_wrap_time(transmit_tow_s - eph.toe_s);
        let m = eph.m0 + n * tk;
        let e_anom = reference_solve_kepler(m, eph.e);
        let sin_e = e_anom.sin();
        let cos_e = e_anom.cos();
        let v = ((1.0 - eph.e * eph.e).sqrt() * sin_e).atan2(cos_e - eph.e);
        let phi = v + eph.w;
        let sin2phi = (2.0 * phi).sin();
        let cos2phi = (2.0 * phi).cos();
        let u = phi + eph.cuc * cos2phi + eph.cus * sin2phi;
        let r = a * (1.0 - eph.e * cos_e) + eph.crc * cos2phi + eph.crs * sin2phi;
        let i = eph.i0 + eph.idot * tk + eph.cic * cos2phi + eph.cis * sin2phi;
        let x_orb = r * u.cos();
        let y_orb = r * u.sin();
        let omega =
            eph.omega0 + (eph.omegadot - TEST_OMEGA_E_DOT) * tk - TEST_OMEGA_E_DOT * eph.toe_s;

        let mut x_m = x_orb * omega.cos() - y_orb * i.cos() * omega.sin();
        let mut y_m = x_orb * omega.sin() + y_orb * i.cos() * omega.cos();
        let z_m = y_orb * i.sin();

        let rotation_rad = TEST_OMEGA_E_DOT * signal_travel_time_s;
        let rotated_x_m = rotation_rad.cos() * x_m + rotation_rad.sin() * y_m;
        let rotated_y_m = -rotation_rad.sin() * x_m + rotation_rad.cos() * y_m;
        x_m = rotated_x_m;
        y_m = rotated_y_m;

        let dt = reference_wrap_time(transmit_tow_s - navigation.clock.t0c_s);
        let base_bias_s =
            navigation.clock.af0 + navigation.clock.af1 * dt + navigation.clock.af2 * dt * dt;
        let relativistic_s = TEST_RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;

        GalileoSatState {
            x_m,
            y_m,
            z_m,
            clock_correction: GalileoSatelliteClockCorrection {
                bias_s: base_bias_s + relativistic_s - navigation.clock.bgd_e1_e5b_s,
                drift_s_per_s: navigation.clock.af1 + 2.0 * navigation.clock.af2 * dt,
                drift_rate_s_per_s2: 2.0 * navigation.clock.af2,
                base_bias_s,
                relativistic_s,
                bgd_e1_e5a_s: navigation.clock.bgd_e1_e5a_s,
                bgd_e1_e5b_s: navigation.clock.bgd_e1_e5b_s,
            },
        }
    }

    #[test]
    fn ionospheric_correction_exposes_nequick_model() {
        let navigation = sample_navigation();
        let model = navigation.ionosphere.nequick_model();
        let receiver = Llh { lat_deg: -3.0, lon_deg: 40.19, alt_m: -23.32 };
        let satellite = Llh { lat_deg: -41.43, lon_deg: 76.65, alt_m: 20_157_673.93 };

        let stec_tecu = model
            .stec_tecu(4, 0.0, receiver, satellite)
            .expect("official-like validation geometry should converge");
        assert!((stec_tecu - 18.26).abs() <= 0.05);
    }

    #[test]
    fn broadcast_navigation_nequick_delay_matches_model_delay() {
        let navigation = sample_navigation();
        let signal = SigId { sat: navigation.sat, band: SignalBand::E1, code: SignalCode::E1B };
        let receiver = Llh { lat_deg: -3.0, lon_deg: 40.19, alt_m: -23.32 };
        let satellite = Llh { lat_deg: -41.43, lon_deg: 76.65, alt_m: 20_157_673.93 };
        let gps_time = GpsTime { week: 2295, tow_s: 0.0 };
        let carrier_hz = bijux_gnss_signal::api::signal_registry(
            Constellation::Galileo,
            SignalBand::E1,
            SignalCode::E1B,
        )
        .expect("Galileo E1 signal registry entry")
        .spec
        .carrier_hz
        .value();

        let expected_delay_m = navigation
            .nequick_model()
            .delay_m_at_gps_time(gps_time, receiver, satellite, carrier_hz)
            .expect("direct model delay");
        let navigation_delay_m = navigation
            .nequick_delay_m(signal, receiver, satellite, gps_time)
            .expect("broadcast navigation delay");

        assert!((navigation_delay_m - expected_delay_m).abs() < 1.0e-9);
        assert!(navigation_delay_m > 0.0);
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
    fn galileo_satellite_state_matches_independent_reference() {
        let navigation = sample_navigation();
        let transmit_tow_s = 65_432.123;
        let signal_travel_time_s = 0.079_2;

        let state = sat_state_galileo_e1(&navigation, transmit_tow_s, signal_travel_time_s);
        let expected = reference_state(&navigation, transmit_tow_s, signal_travel_time_s);

        assert!((state.x_m - expected.x_m).abs() < 1.0e-6);
        assert!((state.y_m - expected.y_m).abs() < 1.0e-6);
        assert!((state.z_m - expected.z_m).abs() < 1.0e-6);
        assert!((state.clock_correction.bias_s - expected.clock_correction.bias_s).abs() < 1.0e-15);
        assert!(
            (state.clock_correction.drift_s_per_s - expected.clock_correction.drift_s_per_s).abs()
                < 1.0e-20
        );
        assert!(
            (state.clock_correction.relativistic_s - expected.clock_correction.relativistic_s)
                .abs()
                < 1.0e-18
        );
    }

    #[test]
    fn galileo_earth_rotation_correction_matches_rotation_matrix() {
        let x_m = 16_400_000.0;
        let y_m = 20_800_000.0;
        let signal_travel_time_s = 0.081;
        let rotation_rad = TEST_OMEGA_E_DOT * signal_travel_time_s;
        let expected_x_m = rotation_rad.cos() * x_m + rotation_rad.sin() * y_m;
        let expected_y_m = -rotation_rad.sin() * x_m + rotation_rad.cos() * y_m;

        let correction = galileo_earth_rotation_correction(x_m, y_m, signal_travel_time_s);

        assert!((correction.rotation_rad - rotation_rad).abs() < 1.0e-18);
        assert!((x_m + correction.delta_x_m - expected_x_m).abs() < 1.0e-9);
        assert!((y_m + correction.delta_y_m - expected_y_m).abs() < 1.0e-9);
    }

    #[test]
    fn receive_time_helper_matches_direct_state_evaluation() {
        let navigation = sample_navigation();
        let receive_tow_s = 65_432.200;
        let signal_travel_time_s = 0.077;

        let receive_time_state =
            sat_state_galileo_e1_at_receive_time(&navigation, receive_tow_s, signal_travel_time_s);
        let direct_state = sat_state_galileo_e1(
            &navigation,
            receive_tow_s - signal_travel_time_s,
            signal_travel_time_s,
        );

        assert!((receive_time_state.x_m - direct_state.x_m).abs() < 1.0e-9);
        assert!((receive_time_state.y_m - direct_state.y_m).abs() < 1.0e-9);
        assert!((receive_time_state.z_m - direct_state.z_m).abs() < 1.0e-9);
        assert!(
            (receive_time_state.clock_correction.bias_s - direct_state.clock_correction.bias_s)
                .abs()
                < 1.0e-18
        );
    }

    #[test]
    fn observation_helper_uses_explicit_signal_timing() {
        let navigation = sample_navigation();
        let signal_timing = ObsSignalTiming {
            signal_travel_time_s: Seconds(0.078),
            transmit_gps_time: GpsTime { week: navigation.gst.week as u32, tow_s: 65_432.122 },
        };
        let state = sat_state_galileo_e1_from_observation(
            &navigation,
            65_432.200,
            24_000_000.0,
            Some(signal_timing),
        );
        let expected = sat_state_galileo_e1(
            &navigation,
            signal_timing.transmit_gps_time.tow_s,
            signal_timing.signal_travel_time_s.0,
        );

        assert!((state.x_m - expected.x_m).abs() < 1.0e-9);
        assert!((state.y_m - expected.y_m).abs() < 1.0e-9);
        assert!((state.z_m - expected.z_m).abs() < 1.0e-9);
    }

    #[test]
    fn galileo_navigation_age_and_selection_follow_reference_time() {
        let fresh = sample_navigation();
        let mut stale = sample_navigation();
        stale.clock.t0c_s -= 18_000.0;
        stale.ephemeris.toe_s -= 18_000.0;

        let fresh_age = galileo_navigation_age(&fresh, 65_000.0);
        let stale_age = galileo_navigation_age(&stale, 65_000.0);

        assert!(fresh_age.is_valid());
        assert!(stale_age.is_stale());
        assert!(is_galileo_navigation_valid(&fresh, 65_000.0));
        assert!(!is_galileo_navigation_valid(&stale, 65_000.0));

        let candidates = [stale.clone(), fresh.clone()];
        let selected = select_best_galileo_navigation(&candidates, fresh.sat, 65_000.0)
            .expect("best Galileo navigation");

        assert_eq!(selected.iodnav, fresh.iodnav);
        assert_eq!(selected.clock.t0c_s, fresh.clock.t0c_s);
    }
}
