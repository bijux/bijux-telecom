#![allow(missing_docs)]

//! BeiDou B2I ranging-code generation.
//!
//! The public BDS-SIS-ICD-2.0 defines B1I and B2I as sharing the same
//! 2046-chip truncated Gold-code family and PRN phase assignments.

use crate::codes::beidou_b1i::{
    beidou_b1i_code_assignment, beidou_b1i_code_assignments, generate_beidou_b1i_code,
    sample_beidou_b1i_code, BeidouB1iCodeAssignment, BEIDOU_B1I_CODE_CHIPS,
    BEIDOU_B1I_CODE_RATE_HZ,
};
use crate::error::SignalError;

/// Number of chips in one published BeiDou B2I code period.
pub const BEIDOU_B2I_CODE_CHIPS: usize = BEIDOU_B1I_CODE_CHIPS;

/// Code rate of the BeiDou B2I ranging code.
pub const BEIDOU_B2I_CODE_RATE_HZ: f64 = BEIDOU_B1I_CODE_RATE_HZ;

/// Published phase assignment for one BeiDou B2I PRN.
pub type BeidouB2iCodeAssignment = BeidouB1iCodeAssignment;

/// Return the published BeiDou B2I phase assignment for one PRN.
pub fn beidou_b2i_code_assignment(
    prn: u8,
) -> Result<&'static BeidouB2iCodeAssignment, SignalError> {
    beidou_b1i_code_assignment(prn)
}

/// Return the published BeiDou B2I phase assignments for PRNs 1 through 37.
pub fn beidou_b2i_code_assignments() -> &'static [BeidouB2iCodeAssignment; 37] {
    beidou_b1i_code_assignments()
}

/// Generate one full-period BeiDou B2I ranging code for a given PRN.
pub fn generate_beidou_b2i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    generate_beidou_b1i_code(prn)
}

/// Generate a BeiDou B2I ranging-code sequence of arbitrary length by repeating the
/// published 2046-chip truncated period.
pub fn generate_beidou_b2i_code_chips(prn: u8, chip_count: usize) -> Result<Vec<i8>, SignalError> {
    let period = generate_beidou_b2i_code(prn)?;
    let mut code = Vec::with_capacity(chip_count);
    while code.len() < chip_count {
        let remaining = chip_count - code.len();
        code.extend(period.iter().copied().take(remaining));
    }
    Ok(code)
}

/// Sample the BeiDou B2I ranging code at an arbitrary sample rate from a chip-phase origin.
pub fn sample_beidou_b2i_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    sample_beidou_b1i_code(prn, sample_rate_hz, start_chip_phase, sample_count)
}

#[cfg(test)]
mod tests {
    use super::{
        beidou_b2i_code_assignment, beidou_b2i_code_assignments, generate_beidou_b2i_code,
        generate_beidou_b2i_code_chips, BEIDOU_B2I_CODE_CHIPS,
    };
    use crate::codes::beidou_b1i::generate_beidou_b1i_code;
    use crate::error::SignalError;

    #[test]
    fn beidou_b2i_assignments_follow_published_catalog_bounds() {
        let assignments = beidou_b2i_code_assignments();

        assert_eq!(assignments.len(), 37);
        assert_eq!(assignments.first().expect("prn 1").g2_taps, (1, 3));
        assert_eq!(assignments.last().expect("prn 37").g2_taps, (10, 11));
    }

    #[test]
    fn beidou_b2i_generator_rejects_out_of_range_prns() {
        assert_eq!(beidou_b2i_code_assignment(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_beidou_b2i_code(38), Err(SignalError::UnsupportedPrn(38)));
    }

    #[test]
    fn beidou_b2i_primary_code_matches_published_shared_open_service_family() {
        let b1 = generate_beidou_b1i_code(11).expect("valid B1I PRN");
        let b2 = generate_beidou_b2i_code(11).expect("valid B2I PRN");

        assert_eq!(b2.len(), BEIDOU_B2I_CODE_CHIPS);
        assert_eq!(b2, b1);
    }

    #[test]
    fn beidou_b2i_arbitrary_length_repeats_truncated_period() {
        let two_periods =
            generate_beidou_b2i_code_chips(19, BEIDOU_B2I_CODE_CHIPS * 2).expect("valid B2I PRN");
        let (first_period, second_period) = two_periods.split_at(BEIDOU_B2I_CODE_CHIPS);

        assert_eq!(first_period, second_period);
    }
}
