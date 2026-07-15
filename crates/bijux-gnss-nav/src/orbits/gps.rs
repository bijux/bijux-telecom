#![allow(missing_docs)]

use std::cmp::Ordering;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Llh, ObsSignalTiming, SatId, Seconds};

use crate::models::atmosphere::{IonosphereModel, KlobucharCoefficients, KlobucharModel};
#[cfg(test)]
use crate::orbits::broadcast_orbit::solve_kepler as solve_broadcast_kepler;
use crate::orbits::broadcast_orbit::{
    earth_rotation_correction, propagate_broadcast_orbit, solve_broadcast_orbit_anomaly,
    wrap_gnss_week_seconds, BroadcastKeplerianOrbit, BroadcastOrbitConstants,
};

const OMEGA_E_DOT: f64 = 7.292_115_146_7e-5;
const MU: f64 = 3.986_005e14;
const RELATIVISTIC_F: f64 = -4.442_807_633e-10;
const MAX_EPHEMERIS_AGE_S: f64 = 7200.0;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsEphemeris {
    pub sat: SatId,
    pub iodc: u16,
    pub iode: u8,
    pub week: u32,
    pub sv_health: u8,
    pub toe_s: f64,
    pub toc_s: f64,
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
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub tgd: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsBroadcastNavigationData {
    pub ephemerides: Vec<GpsEphemeris>,
    #[serde(default)]
    pub klobuchar: Option<KlobucharCoefficients>,
}

impl GpsBroadcastNavigationData {
    pub fn klobuchar_model(&self) -> Option<KlobucharModel> {
        self.klobuchar.map(KlobucharModel::new)
    }

    pub fn klobuchar_delay_l1_m(
        &self,
        receiver: Llh,
        az_deg: f64,
        el_deg: f64,
        receive_tow_s: f64,
    ) -> Option<f64> {
        self.klobuchar_model()
            .map(|model| model.delay_m(receiver, az_deg, el_deg, Seconds(receive_tow_s)))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsSatelliteClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub drift_rate_s_per_s2: f64,
    pub base_bias_s: f64,
    pub relativistic_s: f64,
    pub group_delay_s: f64,
}

impl GpsSatelliteClockCorrection {
    pub fn from_bias_s(bias_s: f64) -> Self {
        Self {
            bias_s,
            drift_s_per_s: 0.0,
            drift_rate_s_per_s2: 0.0,
            base_bias_s: bias_s,
            relativistic_s: 0.0,
            group_delay_s: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_correction: GpsSatelliteClockCorrection,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GpsEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GpsEphemerisAge {
    pub reference_time_s: f64,
    pub toe_age_s: f64,
    pub toc_age_s: f64,
    pub max_age_s: f64,
}

impl GpsEphemerisAge {
    pub fn is_valid(&self) -> bool {
        self.toe_age_s <= self.max_age_s && self.toc_age_s <= self.max_age_s
    }

    pub fn is_stale(&self) -> bool {
        !self.is_valid()
    }
}

pub fn gps_satellite_clock_correction(
    eph: &GpsEphemeris,
    t_tx_s: f64,
) -> GpsSatelliteClockCorrection {
    let anomaly =
        solve_broadcast_orbit_anomaly(gps_broadcast_orbit(eph), t_tx_s, gps_orbit_constants());
    satellite_clock_correction_from_sin_e(eph, t_tx_s, anomaly.kepler.sin_eccentric_anomaly)
}

pub fn gps_earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
) -> GpsEarthRotationCorrection {
    let correction = earth_rotation_correction(x_m, y_m, signal_travel_time_s, OMEGA_E_DOT);
    GpsEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad: correction.rotation_rad,
        delta_x_m: correction.delta_x_m,
        delta_y_m: correction.delta_y_m,
    }
}

pub fn sat_state_gps_l1ca_at_receive_time(
    eph: &GpsEphemeris,
    receive_tow_s: f64,
    signal_travel_time_s: f64,
) -> GpsSatState {
    sat_state_gps_l1ca(eph, receive_tow_s - signal_travel_time_s, signal_travel_time_s)
}

pub fn sat_state_gps_l1ca_from_observation(
    eph: &GpsEphemeris,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> GpsSatState {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_tow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_tow_s - signal_travel_time_s);
    sat_state_gps_l1ca(eph, transmit_tow_s, signal_travel_time_s)
}

pub fn sat_state_gps_l1ca(eph: &GpsEphemeris, t_tx_s: f64, tau_s: f64) -> GpsSatState {
    let orbit_state =
        propagate_broadcast_orbit(gps_broadcast_orbit(eph), t_tx_s, tau_s, gps_orbit_constants());
    let clock =
        satellite_clock_correction_from_sin_e(eph, t_tx_s, orbit_state.sin_eccentric_anomaly);

    GpsSatState {
        x_m: orbit_state.x_m,
        y_m: orbit_state.y_m,
        z_m: orbit_state.z_m,
        clock_correction: clock,
    }
}

pub fn gps_ephemeris_age(eph: &GpsEphemeris, reference_time_s: f64) -> GpsEphemerisAge {
    GpsEphemerisAge {
        reference_time_s,
        toe_age_s: wrap_time(reference_time_s - eph.toe_s).abs(),
        toc_age_s: wrap_time(reference_time_s - eph.toc_s).abs(),
        max_age_s: MAX_EPHEMERIS_AGE_S,
    }
}

pub fn is_ephemeris_valid(eph: &GpsEphemeris, t_s: f64) -> bool {
    gps_ephemeris_age(eph, t_s).is_valid()
}

pub fn select_best_ephemeris<'a>(
    ephs: &'a [GpsEphemeris],
    sat: SatId,
    reference_time_s: f64,
) -> Option<&'a GpsEphemeris> {
    ephs.iter().filter(|eph| eph.sat == sat).min_by(|left, right| {
        let left_age = gps_ephemeris_age(left, reference_time_s);
        let right_age = gps_ephemeris_age(right, reference_time_s);
        let left_score = left_age.toe_age_s.max(left_age.toc_age_s);
        let right_score = right_age.toe_age_s.max(right_age.toc_age_s);
        left_score.partial_cmp(&right_score).unwrap_or(Ordering::Equal)
    })
}

#[cfg(test)]
fn solve_kepler(m: f64, e: f64) -> (f64, f64, f64) {
    let solution = solve_broadcast_kepler(m, e);
    (solution.eccentric_anomaly_rad, solution.sin_eccentric_anomaly, solution.cos_eccentric_anomaly)
}

fn wrap_time(t: f64) -> f64 {
    wrap_gnss_week_seconds(t)
}

fn gps_orbit_constants() -> BroadcastOrbitConstants {
    BroadcastOrbitConstants {
        gravitational_parameter_m3_s2: MU,
        earth_rotation_rate_rad_s: OMEGA_E_DOT,
    }
}

fn gps_broadcast_orbit(eph: &GpsEphemeris) -> BroadcastKeplerianOrbit {
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

fn satellite_clock_correction_from_sin_e(
    eph: &GpsEphemeris,
    t_tx_s: f64,
    sin_e: f64,
) -> GpsSatelliteClockCorrection {
    let dt = wrap_time(t_tx_s - eph.toc_s);
    let base_bias_s = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt;
    let relativistic_s = RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
    let group_delay_s = eph.tgd;
    GpsSatelliteClockCorrection {
        bias_s: base_bias_s + relativistic_s - group_delay_s,
        drift_s_per_s: eph.af1 + 2.0 * eph.af2 * dt,
        drift_rate_s_per_s2: 2.0 * eph.af2,
        base_bias_s,
        relativistic_s,
        group_delay_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::formats::lnav_bits::{
        bit_sync_from_prompt, compute_parity, decode_words, demodulate_gps_l1ca_navigation_bits,
    };
    use bijux_gnss_core::api::{Constellation, Llh, SatId, Seconds};

    fn encode_word(data: u32, prev_d29: u8, prev_d30: u8) -> [u8; 30] {
        let mut bits = [0_u8; 30];
        for (i, bit) in bits.iter_mut().enumerate().take(24) {
            let shift = 23 - i;
            *bit = ((data >> shift) & 1) as u8;
        }
        let (p1, p2, p3, p4, p5, p6) = compute_parity(&bits[..24], prev_d29, prev_d30);
        bits[24] = p1;
        bits[25] = p2;
        bits[26] = p3;
        bits[27] = p4;
        bits[28] = p5;
        bits[29] = p6;
        bits
    }

    #[test]
    fn parity_roundtrip() {
        let data = 0xABCDE;
        let bits = encode_word(data, 0, 0);
        let mut signed = Vec::new();
        for b in bits {
            signed.push(if b == 1 { 1 } else { -1 });
        }
        let words = decode_words(&signed);
        assert_eq!(words.len(), 1);
        assert!(words[0].parity_ok);
        assert_eq!(words[0].data, data);
    }

    #[test]
    fn kepler_solution_close_for_circular() {
        let m = 1.0;
        let e = 0.0;
        let (e_anom, _, _) = solve_kepler(m, e);
        assert!((e_anom - m).abs() < 1e-10);
    }

    #[test]
    fn sat_state_basic() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.0,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.0,
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
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let state = sat_state_gps_l1ca(&eph, 0.0, 0.0);
        let radius = (state.x_m * state.x_m + state.y_m * state.y_m + state.z_m * state.z_m).sqrt();
        assert!((radius - 26_560_000.0).abs() < 5_000_000.0);
    }

    #[test]
    fn gps_satellite_state_matches_independent_reference() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            iodc: 0x123,
            iode: 0x45,
            week: 2200,
            sv_health: 0,
            toe_s: 345_600.0,
            toc_s: 345_520.0,
            sqrt_a: 5_153.795_477_5,
            e: 0.0123,
            i0: 0.947,
            idot: 2.1e-10,
            omega0: 1.35,
            omegadot: -8.4e-9,
            w: 0.71,
            m0: -0.23,
            delta_n: 4.7e-9,
            cuc: 1.2e-6,
            cus: -2.4e-6,
            crc: 145.0,
            crs: -83.0,
            cic: -1.7e-7,
            cis: 2.3e-7,
            af0: 2.4e-4,
            af1: -1.1e-12,
            af2: 4.0e-20,
            tgd: -7.2e-9,
        };
        let transmit_tow_s = 348_123.456;
        let signal_travel_time_s = 0.073_5;

        let state = sat_state_gps_l1ca(&eph, transmit_tow_s, signal_travel_time_s);
        let expected = reference_gps_satellite_state(&eph, transmit_tow_s, signal_travel_time_s);

        assert!((state.x_m - expected.x_m).abs() < 1.0e-6);
        assert!((state.y_m - expected.y_m).abs() < 1.0e-6);
        assert!((state.z_m - expected.z_m).abs() < 1.0e-6);
        assert!((state.clock_correction.bias_s - expected.clock_correction.bias_s).abs() < 1.0e-18);
        assert!(
            (state.clock_correction.relativistic_s - expected.clock_correction.relativistic_s)
                .abs()
                < 1.0e-18
        );
    }

    #[test]
    fn earth_rotation_correction_is_identity_for_zero_travel_time() {
        let correction = gps_earth_rotation_correction(12_345.0, -67_890.0, 0.0);

        assert_eq!(correction.signal_travel_time_s, 0.0);
        assert_eq!(correction.rotation_rad, 0.0);
        assert_eq!(correction.delta_x_m, 0.0);
        assert_eq!(correction.delta_y_m, 0.0);
    }

    #[test]
    fn earth_rotation_correction_matches_explicit_rotation_matrix() {
        let x_m = 15_600_000.0;
        let y_m = 21_700_000.0;
        let tau_s = 0.082;
        let rotation_rad = OMEGA_E_DOT * tau_s;
        let cos_rot = rotation_rad.cos();
        let sin_rot = rotation_rad.sin();
        let expected_x_m = cos_rot * x_m + sin_rot * y_m;
        let expected_y_m = -sin_rot * x_m + cos_rot * y_m;

        let correction = gps_earth_rotation_correction(x_m, y_m, tau_s);

        assert!((correction.rotation_rad - rotation_rad).abs() < 1.0e-18);
        assert!((x_m + correction.delta_x_m - expected_x_m).abs() < 1.0e-9);
        assert!((y_m + correction.delta_y_m - expected_y_m).abs() < 1.0e-9);
        assert!(correction.delta_x_m.abs() > 0.01);
        assert!(correction.delta_y_m.abs() > 0.01);
    }

    #[test]
    fn receive_time_helper_matches_transmit_time_state_evaluation() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 3 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.4,
            omegadot: 0.0,
            w: 0.2,
            m0: 0.7,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let receive_tow_s = 345_600.0;
        let tau_s = 0.074;

        let from_receive_time = sat_state_gps_l1ca_at_receive_time(&eph, receive_tow_s, tau_s);
        let from_transmit_time = sat_state_gps_l1ca(&eph, receive_tow_s - tau_s, tau_s);

        assert!((from_receive_time.x_m - from_transmit_time.x_m).abs() < 1.0e-9);
        assert!((from_receive_time.y_m - from_transmit_time.y_m).abs() < 1.0e-9);
        assert!((from_receive_time.z_m - from_transmit_time.z_m).abs() < 1.0e-9);
        assert!(
            (from_receive_time.clock_correction.bias_s
                - from_transmit_time.clock_correction.bias_s)
                .abs()
                < 1.0e-18
        );
    }

    #[test]
    fn observation_helper_uses_explicit_transmit_time_when_available() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            iodc: 0,
            iode: 0,
            week: 2200,
            sv_health: 0,
            toe_s: 345_600.0,
            toc_s: 345_600.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 2.1,
            omegadot: 0.0,
            w: 0.1,
            m0: 0.4,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let receive_tow_s = 345_600.07;
        let tau_s = 0.073;
        let timing = ObsSignalTiming {
            signal_travel_time_s: bijux_gnss_core::api::Seconds(tau_s),
            transmit_gps_time: bijux_gnss_core::api::GpsTime {
                week: eph.week,
                tow_s: receive_tow_s - tau_s,
            },
        };

        let from_observation = sat_state_gps_l1ca_from_observation(
            &eph,
            receive_tow_s,
            tau_s * SPEED_OF_LIGHT_MPS,
            Some(timing),
        );
        let from_explicit_timing = sat_state_gps_l1ca(&eph, timing.transmit_gps_time.tow_s, tau_s);

        assert!((from_observation.x_m - from_explicit_timing.x_m).abs() < 1.0e-9);
        assert!((from_observation.y_m - from_explicit_timing.y_m).abs() < 1.0e-9);
        assert!((from_observation.z_m - from_explicit_timing.z_m).abs() < 1.0e-9);
    }

    #[test]
    fn bit_sync_detects_offset() {
        let mut prompt = vec![1.0_f32; 5];
        prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        let result = bit_sync_from_prompt(&prompt);
        assert_eq!(result.bit_start_ms, 5);
        assert!(result.sync_confidence > 0.0, "{result:?}");
        assert_eq!(result.bits.len(), 2);
        assert_eq!(result.bits[0], -1);
        assert_eq!(result.bits[1], 1);
    }

    #[test]
    fn demodulated_navigation_bits_preserve_windows_and_prompt_sums() {
        let mut prompt = vec![0.25_f32; 3];
        prompt.extend(std::iter::repeat_n(1.5_f32, 20));
        prompt.extend(std::iter::repeat_n(-0.5_f32, 20));

        let demodulation = demodulate_gps_l1ca_navigation_bits(&prompt);

        assert_eq!(demodulation.bit_start_ms, 3);
        assert!(demodulation.sync_confidence > 0.0, "{demodulation:?}");
        assert_eq!(demodulation.complete_window_count, 2);
        assert_eq!(demodulation.bits.len(), 2);
        assert_eq!(demodulation.bits[0].bit_index, 0);
        assert_eq!(demodulation.bits[0].start_prompt_index, 3);
        assert_eq!(demodulation.bits[0].end_prompt_index_exclusive, 23);
        assert_eq!(demodulation.bits[0].sign, 1);
        assert!(demodulation.bits[0].confidence > 0.9, "{demodulation:?}");
        assert!((demodulation.bits[0].prompt_sum - 30.0).abs() <= f32::EPSILON);
        assert_eq!(demodulation.bits[1].bit_index, 1);
        assert_eq!(demodulation.bits[1].start_prompt_index, 23);
        assert_eq!(demodulation.bits[1].end_prompt_index_exclusive, 43);
        assert_eq!(demodulation.bits[1].sign, -1);
        assert!(demodulation.bits[1].confidence > 0.9, "{demodulation:?}");
        assert!((demodulation.bits[1].prompt_sum + 10.0).abs() <= f32::EPSILON);
    }

    #[test]
    fn relativistic_term_nonzero_for_eccentric() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
            e: 0.1,
            i0: 0.94,
            idot: 0.0,
            omega0: 1.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.1,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let state = sat_state_gps_l1ca(&eph, 1000.0, 0.0);
        assert!(state.clock_correction.relativistic_s.abs() > 0.0);
    }

    #[test]
    fn satellite_clock_correction_applies_polynomial_relativistic_and_group_delay_terms() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 0.0,
            toc_s: 100.0,
            sqrt_a: 5153.7954775,
            e: 0.02,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.1,
            omegadot: 0.0,
            w: 0.2,
            m0: 0.3,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 1.0e-4,
            af1: -2.0e-12,
            af2: 3.0e-20,
            tgd: 8.0e-9,
        };
        let correction = gps_satellite_clock_correction(&eph, 1_600.0);
        let dt = 1_500.0;
        let expected_base_bias = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt;
        assert!((correction.base_bias_s - expected_base_bias).abs() < 1e-18);
        assert!(correction.relativistic_s.abs() > 0.0);
        assert!((correction.group_delay_s - eph.tgd).abs() < 1e-18);
        let expected_bias = expected_base_bias + correction.relativistic_s - eph.tgd;
        assert!((correction.bias_s - expected_bias).abs() < 1e-18);
    }

    #[test]
    fn satellite_clock_correction_reports_drift_and_drift_rate() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 0.0,
            toc_s: 50.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
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
            af0: 0.0,
            af1: 2.5e-12,
            af2: -7.5e-20,
            tgd: 0.0,
        };
        let correction = gps_satellite_clock_correction(&eph, 650.0);
        let dt = 600.0;
        let expected_drift = eph.af1 + 2.0 * eph.af2 * dt;
        let expected_drift_rate = 2.0 * eph.af2;
        assert!((correction.drift_s_per_s - expected_drift).abs() < 1e-24);
        assert!((correction.drift_rate_s_per_s2 - expected_drift_rate).abs() < 1e-30);
    }

    #[test]
    fn broadcast_navigation_returns_no_klobuchar_delay_without_coefficients() {
        let navigation = GpsBroadcastNavigationData { ephemerides: Vec::new(), klobuchar: None };

        assert_eq!(
            navigation.klobuchar_delay_l1_m(
                Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 },
                120.0,
                30.0,
                50_400.0,
            ),
            None
        );
        assert!(navigation.klobuchar_model().is_none());
    }

    #[test]
    fn broadcast_navigation_klobuchar_delay_matches_model_delay() {
        let coefficients = KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        );
        let navigation =
            GpsBroadcastNavigationData { ephemerides: Vec::new(), klobuchar: Some(coefficients) };
        let receiver = Llh { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 };
        let model_delay_m =
            KlobucharModel::new(coefficients).delay_m(receiver, 120.0, 30.0, Seconds(50_400.0));
        let navigation_delay_m = navigation
            .klobuchar_delay_l1_m(receiver, 120.0, 30.0, 50_400.0)
            .expect("broadcast navigation delay");

        assert!((navigation_delay_m - model_delay_m).abs() < 1.0e-12);
        assert!(navigation_delay_m > 0.0);
    }

    fn sample_eph() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            toe_s: 100_000.0,
            toc_s: 100_000.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
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
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        }
    }

    #[test]
    fn ephemeris_age_stays_valid_at_limit() {
        let eph = sample_eph();

        let age = gps_ephemeris_age(&eph, eph.toe_s + MAX_EPHEMERIS_AGE_S);

        assert_eq!(age.toe_age_s, MAX_EPHEMERIS_AGE_S);
        assert_eq!(age.toc_age_s, MAX_EPHEMERIS_AGE_S);
        assert!(age.is_valid());
        assert!(!age.is_stale());
        assert!(is_ephemeris_valid(&eph, eph.toe_s + MAX_EPHEMERIS_AGE_S));
    }

    #[test]
    fn ephemeris_age_rejects_stale_toe() {
        let mut eph = sample_eph();
        eph.toc_s = eph.toe_s + 30.0;

        let age = gps_ephemeris_age(&eph, eph.toe_s + MAX_EPHEMERIS_AGE_S + 1.0);

        assert!(age.toe_age_s > age.max_age_s);
        assert!(age.toc_age_s <= age.max_age_s);
        assert!(!age.is_valid());
        assert!(age.is_stale());
        assert!(!is_ephemeris_valid(&eph, eph.toe_s + MAX_EPHEMERIS_AGE_S + 1.0));
    }

    #[test]
    fn ephemeris_age_rejects_stale_toc() {
        let mut eph = sample_eph();
        eph.toe_s = eph.toc_s + 30.0;

        let age = gps_ephemeris_age(&eph, eph.toc_s + MAX_EPHEMERIS_AGE_S + 1.0);

        assert!(age.toe_age_s <= age.max_age_s);
        assert!(age.toc_age_s > age.max_age_s);
        assert!(!age.is_valid());
    }

    #[test]
    fn ephemeris_age_wraps_week_rollover() {
        let mut eph = sample_eph();
        eph.toe_s = 604_799.0;
        eph.toc_s = 604_799.0;

        let age = gps_ephemeris_age(&eph, 1.0);

        assert_eq!(age.toe_age_s, 2.0);
        assert_eq!(age.toc_age_s, 2.0);
        assert!(age.is_valid());
    }

    #[test]
    fn select_best_ephemeris_prefers_closest_reference_time() {
        let mut stale = sample_eph();
        stale.toe_s = 597_600.0;
        stale.toc_s = 597_600.0;

        let mut current = sample_eph();
        current.toe_s = 603_000.0;
        current.toc_s = 603_000.0;

        let ephemerides = [stale.clone(), current.clone()];
        let selected =
            select_best_ephemeris(&ephemerides, current.sat, 15.0).expect("select best ephemeris");

        assert_eq!(selected.toe_s, current.toe_s);
        assert_eq!(selected.toc_s, current.toc_s);
    }

    fn reference_gps_satellite_state(
        eph: &GpsEphemeris,
        transmit_tow_s: f64,
        signal_travel_time_s: f64,
    ) -> GpsSatState {
        let semi_major_axis_m = eph.sqrt_a * eph.sqrt_a;
        let nominal_mean_motion_rad_s = (MU / semi_major_axis_m.powi(3)).sqrt();
        let corrected_mean_motion_rad_s = nominal_mean_motion_rad_s + eph.delta_n;
        let time_from_ephemeris_s = reference_wrap_time(transmit_tow_s - eph.toe_s);
        let mean_anomaly_rad = eph.m0 + corrected_mean_motion_rad_s * time_from_ephemeris_s;
        let eccentric_anomaly_rad = reference_solve_kepler(mean_anomaly_rad, eph.e);
        let sin_e = eccentric_anomaly_rad.sin();
        let cos_e = eccentric_anomaly_rad.cos();
        let true_anomaly_rad = ((1.0 - eph.e * eph.e).sqrt() * sin_e).atan2(cos_e - eph.e);
        let argument_of_latitude_rad = true_anomaly_rad + eph.w;
        let sin_double_latitude = (2.0 * argument_of_latitude_rad).sin();
        let cos_double_latitude = (2.0 * argument_of_latitude_rad).cos();
        let corrected_argument_of_latitude_rad = argument_of_latitude_rad
            + eph.cuc * cos_double_latitude
            + eph.cus * sin_double_latitude;
        let corrected_radius_m = semi_major_axis_m * (1.0 - eph.e * cos_e)
            + eph.crc * cos_double_latitude
            + eph.crs * sin_double_latitude;
        let corrected_inclination_rad = eph.i0
            + eph.idot * time_from_ephemeris_s
            + eph.cic * cos_double_latitude
            + eph.cis * sin_double_latitude;
        let orbital_x_m = corrected_radius_m * corrected_argument_of_latitude_rad.cos();
        let orbital_y_m = corrected_radius_m * corrected_argument_of_latitude_rad.sin();
        let corrected_right_ascension_rad = eph.omega0
            + (eph.omegadot - OMEGA_E_DOT) * time_from_ephemeris_s
            - OMEGA_E_DOT * eph.toe_s;
        let x_unrotated_m = orbital_x_m * corrected_right_ascension_rad.cos()
            - orbital_y_m * corrected_inclination_rad.cos() * corrected_right_ascension_rad.sin();
        let y_unrotated_m = orbital_x_m * corrected_right_ascension_rad.sin()
            + orbital_y_m * corrected_inclination_rad.cos() * corrected_right_ascension_rad.cos();
        let z_m = orbital_y_m * corrected_inclination_rad.sin();
        let rotation_rad = OMEGA_E_DOT * signal_travel_time_s;
        let x_m = rotation_rad.cos() * x_unrotated_m + rotation_rad.sin() * y_unrotated_m;
        let y_m = -rotation_rad.sin() * x_unrotated_m + rotation_rad.cos() * y_unrotated_m;
        let clock_correction = reference_gps_clock_correction(eph, transmit_tow_s, sin_e);

        GpsSatState { x_m, y_m, z_m, clock_correction }
    }

    fn reference_gps_clock_correction(
        eph: &GpsEphemeris,
        transmit_tow_s: f64,
        sin_e: f64,
    ) -> GpsSatelliteClockCorrection {
        let clock_time_s = reference_wrap_time(transmit_tow_s - eph.toc_s);
        let base_bias_s = eph.af0 + eph.af1 * clock_time_s + eph.af2 * clock_time_s * clock_time_s;
        let relativistic_s = RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
        GpsSatelliteClockCorrection {
            bias_s: base_bias_s + relativistic_s - eph.tgd,
            drift_s_per_s: eph.af1 + 2.0 * eph.af2 * clock_time_s,
            drift_rate_s_per_s2: 2.0 * eph.af2,
            base_bias_s,
            relativistic_s,
            group_delay_s: eph.tgd,
        }
    }

    fn reference_solve_kepler(mean_anomaly_rad: f64, eccentricity: f64) -> f64 {
        let mut eccentric_anomaly_rad = mean_anomaly_rad;
        for _ in 0..20 {
            let residual = eccentric_anomaly_rad
                - eccentricity * eccentric_anomaly_rad.sin()
                - mean_anomaly_rad;
            let derivative = 1.0 - eccentricity * eccentric_anomaly_rad.cos();
            let step = residual / derivative;
            eccentric_anomaly_rad -= step;
            if step.abs() < 1.0e-14 {
                break;
            }
        }
        eccentric_anomaly_rad
    }

    fn reference_wrap_time(mut seconds: f64) -> f64 {
        while seconds > 302_400.0 {
            seconds -= 604_800.0;
        }
        while seconds < -302_400.0 {
            seconds += 604_800.0;
        }
        seconds
    }
}
