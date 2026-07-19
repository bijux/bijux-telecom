#![allow(missing_docs)]

//! BeiDou B1I ranging-code generation.
//!
//! Clean-room implementation derived from the public BDS-SIS-ICD-2.0 signal definition.

use crate::dsp::signal::sample_code;
use crate::error::SignalError;

/// Number of chips in one published BeiDou B1I code period.
pub const BEIDOU_B1I_CODE_CHIPS: usize = 2046;

/// Code rate of the BeiDou B1I ranging code.
pub const BEIDOU_B1I_CODE_RATE_HZ: f64 = 2_046_000.0;

/// Published phase assignment for one BeiDou B1I PRN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BeidouB1iCodeAssignment {
    pub prn: u8,
    pub g2_taps: (u8, u8),
}

impl BeidouB1iCodeAssignment {
    fn zero_based_g2_taps(self) -> (usize, usize) {
        (usize::from(self.g2_taps.0 - 1), usize::from(self.g2_taps.1 - 1))
    }
}

const BEIDOU_B1I_CODE_ASSIGNMENTS: [BeidouB1iCodeAssignment; 37] = [
    BeidouB1iCodeAssignment { prn: 1, g2_taps: (1, 3) },
    BeidouB1iCodeAssignment { prn: 2, g2_taps: (1, 4) },
    BeidouB1iCodeAssignment { prn: 3, g2_taps: (1, 5) },
    BeidouB1iCodeAssignment { prn: 4, g2_taps: (1, 6) },
    BeidouB1iCodeAssignment { prn: 5, g2_taps: (1, 8) },
    BeidouB1iCodeAssignment { prn: 6, g2_taps: (1, 9) },
    BeidouB1iCodeAssignment { prn: 7, g2_taps: (1, 10) },
    BeidouB1iCodeAssignment { prn: 8, g2_taps: (1, 11) },
    BeidouB1iCodeAssignment { prn: 9, g2_taps: (2, 7) },
    BeidouB1iCodeAssignment { prn: 10, g2_taps: (3, 4) },
    BeidouB1iCodeAssignment { prn: 11, g2_taps: (3, 5) },
    BeidouB1iCodeAssignment { prn: 12, g2_taps: (3, 6) },
    BeidouB1iCodeAssignment { prn: 13, g2_taps: (3, 8) },
    BeidouB1iCodeAssignment { prn: 14, g2_taps: (3, 9) },
    BeidouB1iCodeAssignment { prn: 15, g2_taps: (3, 10) },
    BeidouB1iCodeAssignment { prn: 16, g2_taps: (3, 11) },
    BeidouB1iCodeAssignment { prn: 17, g2_taps: (4, 5) },
    BeidouB1iCodeAssignment { prn: 18, g2_taps: (4, 6) },
    BeidouB1iCodeAssignment { prn: 19, g2_taps: (4, 8) },
    BeidouB1iCodeAssignment { prn: 20, g2_taps: (4, 9) },
    BeidouB1iCodeAssignment { prn: 21, g2_taps: (4, 10) },
    BeidouB1iCodeAssignment { prn: 22, g2_taps: (4, 11) },
    BeidouB1iCodeAssignment { prn: 23, g2_taps: (5, 6) },
    BeidouB1iCodeAssignment { prn: 24, g2_taps: (5, 8) },
    BeidouB1iCodeAssignment { prn: 25, g2_taps: (5, 9) },
    BeidouB1iCodeAssignment { prn: 26, g2_taps: (5, 10) },
    BeidouB1iCodeAssignment { prn: 27, g2_taps: (5, 11) },
    BeidouB1iCodeAssignment { prn: 28, g2_taps: (6, 8) },
    BeidouB1iCodeAssignment { prn: 29, g2_taps: (6, 9) },
    BeidouB1iCodeAssignment { prn: 30, g2_taps: (6, 10) },
    BeidouB1iCodeAssignment { prn: 31, g2_taps: (6, 11) },
    BeidouB1iCodeAssignment { prn: 32, g2_taps: (8, 9) },
    BeidouB1iCodeAssignment { prn: 33, g2_taps: (8, 10) },
    BeidouB1iCodeAssignment { prn: 34, g2_taps: (8, 11) },
    BeidouB1iCodeAssignment { prn: 35, g2_taps: (9, 10) },
    BeidouB1iCodeAssignment { prn: 36, g2_taps: (9, 11) },
    BeidouB1iCodeAssignment { prn: 37, g2_taps: (10, 11) },
];

/// Return the published BeiDou B1I phase assignment for one PRN.
pub fn beidou_b1i_code_assignment(
    prn: u8,
) -> Result<&'static BeidouB1iCodeAssignment, SignalError> {
    prn_index(prn).map(|index| &BEIDOU_B1I_CODE_ASSIGNMENTS[index])
}

/// Return the published BeiDou B1I phase assignments for PRNs 1 through 37.
pub fn beidou_b1i_code_assignments() -> &'static [BeidouB1iCodeAssignment; 37] {
    &BEIDOU_B1I_CODE_ASSIGNMENTS
}

/// Generate one full-period BeiDou B1I ranging code for a given PRN.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_beidou_b1i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let (tap1, tap2) = beidou_b1i_code_assignment(prn)?.zero_based_g2_taps();
    let mut g1 = [0_i8, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0];
    let mut g2 = [0_i8, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0];
    let mut code = Vec::with_capacity(BEIDOU_B1I_CODE_CHIPS);

    for _ in 0..BEIDOU_B1I_CODE_CHIPS {
        let chip = g1[10] ^ g2[tap1] ^ g2[tap2];
        code.push(if chip == 0 { 1 } else { -1 });

        let g1_feedback = g1[0] ^ g1[6] ^ g1[7] ^ g1[8] ^ g1[9] ^ g1[10];
        let g2_feedback = g2[0] ^ g2[1] ^ g2[2] ^ g2[3] ^ g2[4] ^ g2[7] ^ g2[8] ^ g2[10];

        shift_register(&mut g1, g1_feedback);
        shift_register(&mut g2, g2_feedback);
    }

    Ok(code)
}

/// Generate a BeiDou B1I ranging-code sequence of arbitrary length by repeating the
/// published 2046-chip truncated period.
pub fn generate_beidou_b1i_code_chips(prn: u8, chip_count: usize) -> Result<Vec<i8>, SignalError> {
    let period = generate_beidou_b1i_code(prn)?;
    let mut code = Vec::with_capacity(chip_count);
    while code.len() < chip_count {
        let remaining = chip_count - code.len();
        code.extend(period.iter().copied().take(remaining));
    }
    Ok(code)
}

/// Sample the BeiDou B1I ranging code at an arbitrary sample rate from a chip-phase origin.
pub fn sample_beidou_b1i_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_beidou_b1i_code(prn)?;
    sample_code(&code, sample_rate_hz, BEIDOU_B1I_CODE_RATE_HZ, start_chip_phase, sample_count)
}

fn shift_register(register: &mut [i8; 11], feedback: i8) {
    for index in (1..register.len()).rev() {
        register[index] = register[index - 1];
    }
    register[0] = feedback & 1;
}

fn prn_index(prn: u8) -> Result<usize, SignalError> {
    match prn {
        1..=37 => Ok(usize::from(prn - 1)),
        _ => Err(SignalError::UnsupportedPrn(prn)),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        beidou_b1i_code_assignment, beidou_b1i_code_assignments, generate_beidou_b1i_code,
        generate_beidou_b1i_code_chips, BEIDOU_B1I_CODE_CHIPS,
    };
    use crate::error::SignalError;

    #[test]
    fn beidou_b1i_assignments_follow_published_catalog_bounds() {
        let assignments = beidou_b1i_code_assignments();

        assert_eq!(assignments.len(), 37);
        assert_eq!(assignments.first().expect("prn 1").g2_taps, (1, 3));
        assert_eq!(assignments.last().expect("prn 37").g2_taps, (10, 11));
    }

    #[test]
    fn beidou_b1i_generator_rejects_out_of_range_prns() {
        assert_eq!(beidou_b1i_code_assignment(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_beidou_b1i_code(38), Err(SignalError::UnsupportedPrn(38)));
    }

    #[test]
    fn beidou_b1i_primary_code_is_balanced_and_bipolar() {
        let code = generate_beidou_b1i_code(11).expect("valid B1I PRN");
        let positive = code.iter().filter(|chip| **chip == 1).count();
        let negative = code.iter().filter(|chip| **chip == -1).count();

        assert_eq!(code.len(), BEIDOU_B1I_CODE_CHIPS);
        assert_eq!(positive, negative);
        assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn beidou_b1i_arbitrary_length_repeats_truncated_period() {
        let two_periods =
            generate_beidou_b1i_code_chips(19, BEIDOU_B1I_CODE_CHIPS * 2).expect("valid B1I PRN");
        let (first_period, second_period) = two_periods.split_at(BEIDOU_B1I_CODE_CHIPS);

        assert_eq!(first_period, second_period);
    }
}
