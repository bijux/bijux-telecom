#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::SatId;

const OMEGA_E_DOT: f64 = 7.292_115_146_7e-5;
const MU: f64 = 3.986_005e14;
const RELATIVISTIC_F: f64 = -4.442_807_633e-10;
const MAX_EPHEMERIS_AGE_S: f64 = 7200.0;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsEphemeris {
    pub sat: SatId,
    pub iodc: u16,
    pub iode: u8,
    pub week: u16,
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
pub struct GpsSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s: f64,
    pub relativistic_s: f64,
}

pub fn sat_state_gps_l1ca(eph: &GpsEphemeris, t_tx_s: f64, tau_s: f64) -> GpsSatState {
    let a = eph.sqrt_a * eph.sqrt_a;
    let n0 = (MU / (a * a * a)).sqrt();
    let n = n0 + eph.delta_n;
    let mut tk = t_tx_s - eph.toe_s;
    tk = wrap_time(tk);

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

    let mut x = x_orb * cos_omega - y_orb * cos_i * sin_omega;
    let mut y = x_orb * sin_omega + y_orb * cos_i * cos_omega;
    let z = y_orb * sin_i;

    let rot = OMEGA_E_DOT * tau_s;
    if rot.abs() > 0.0 {
        let cos_rot = rot.cos();
        let sin_rot = rot.sin();
        let xr = cos_rot * x + sin_rot * y;
        let yr = -sin_rot * x + cos_rot * y;
        x = xr;
        y = yr;
    }

    let dt = wrap_time(t_tx_s - eph.toc_s);
    let relativistic = RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
    let clock_bias = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + relativistic - eph.tgd;
    let clock_drift = eph.af1 + 2.0 * eph.af2 * dt;

    GpsSatState {
        x_m: x,
        y_m: y,
        z_m: z,
        clock_bias_s: clock_bias,
        clock_drift_s: clock_drift,
        relativistic_s: relativistic,
    }
}

pub fn is_ephemeris_valid(eph: &GpsEphemeris, t_s: f64) -> bool {
    let toe_age = wrap_time(t_s - eph.toe_s).abs();
    let toc_age = wrap_time(t_s - eph.toc_s).abs();
    toe_age <= MAX_EPHEMERIS_AGE_S && toc_age <= MAX_EPHEMERIS_AGE_S
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::formats::lnav_bits::{bit_sync_from_prompt, compute_parity, decode_words};
    use bijux_gnss_core::api::{Constellation, SatId};

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
    fn bit_sync_detects_offset() {
        let mut prompt = vec![1.0_f32; 5];
        prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
        prompt.extend(std::iter::repeat_n(1.0_f32, 20));
        let result = bit_sync_from_prompt(&prompt);
        assert_eq!(result.bit_start_ms, 5);
        assert_eq!(result.bits.len(), 2);
        assert_eq!(result.bits[0], -1);
        assert_eq!(result.bits[1], 1);
    }

    #[test]
    fn relativistic_term_nonzero_for_eccentric() {
        let eph = GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            iodc: 0,
            iode: 0,
            week: 0,
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
        assert!(state.relativistic_s.abs() > 0.0);
    }
}
