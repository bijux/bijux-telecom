#[cfg(test)]
mod tests {
    use super::*;

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
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
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
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
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
