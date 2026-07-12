#![allow(missing_docs)]

//! GLONASS L1 standard-precision spreading code generation.
//!
//! The open-service GLONASS L1 FDMA signal uses a 511-chip maximum-length
//! sequence clocked at 511 kchips/s. All satellites share the same spreading
//! code; satellites are distinguished by carrier frequency channel instead.

/// Number of chips in one GLONASS L1 standard-precision code period.
pub const GLONASS_L1_ST_CODE_CHIPS: usize = 511;

/// Code rate of the GLONASS L1 standard-precision ranging code.
pub const GLONASS_L1_ST_CODE_RATE_HZ: f64 = 511_000.0;

/// Generate one full-period GLONASS L1 standard-precision spreading code.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_glonass_l1_st_code() -> Vec<i8> {
    generate_glonass_l1_st_code_chips(GLONASS_L1_ST_CODE_CHIPS)
}

/// Generate a GLONASS L1 standard-precision spreading code sequence of
/// arbitrary length.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_glonass_l1_st_code_chips(chip_count: usize) -> Vec<i8> {
    let mut register = [1u8; 9];
    let mut code = Vec::with_capacity(chip_count);

    for _ in 0..chip_count {
        // The ICD defines the output at the seventh stage of the 9-stage
        // register with polynomial x^9 + x^5 + 1 and all-ones initialization.
        let chip = register[6];
        code.push(if chip == 0 { 1 } else { -1 });

        let feedback = register[4] ^ register[8];
        shift_register(&mut register, feedback);
    }

    code
}

/// Sample the GLONASS L1 standard-precision code at an arbitrary sample rate
/// from a chip-phase origin.
pub fn sample_glonass_l1_st_code(
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, crate::error::SignalError> {
    let code = generate_glonass_l1_st_code();
    crate::dsp::signal::sample_code(
        &code,
        sample_rate_hz,
        GLONASS_L1_ST_CODE_RATE_HZ,
        start_chip_phase,
        sample_count,
    )
}

fn shift_register(register: &mut [u8; 9], feedback: u8) {
    for index in (1..register.len()).rev() {
        register[index] = register[index - 1];
    }
    register[0] = feedback;
}

#[cfg(test)]
mod tests {
    use super::{generate_glonass_l1_st_code_chips, GLONASS_L1_ST_CODE_CHIPS};

    #[test]
    fn glonass_l1_reference_prefix_matches_icd_sequence() {
        let code = generate_glonass_l1_st_code_chips(9);
        let bits =
            code.into_iter().map(|chip| if chip > 0 { '0' } else { '1' }).collect::<String>();

        assert_eq!(bits, "111111100");
    }

    #[test]
    fn glonass_l1_period_repeats_after_511_chips() {
        let two_periods = generate_glonass_l1_st_code_chips(GLONASS_L1_ST_CODE_CHIPS * 2);
        let (first_period, second_period) = two_periods.split_at(GLONASS_L1_ST_CODE_CHIPS);

        assert_eq!(first_period, second_period);
    }
}
