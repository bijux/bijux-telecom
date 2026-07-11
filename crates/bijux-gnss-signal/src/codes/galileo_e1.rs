#![allow(missing_docs)]

use crate::codes::galileo_e1_tables::{E1B_HEX, E1C_HEX, E1C_SECONDARY_CODE};
use crate::error::SignalError;

pub const GALILEO_E1_PRIMARY_CODE_CHIPS: usize = 4092;
pub const GALILEO_E1_SECONDARY_CODE_CHIPS: usize = 25;
pub const GALILEO_E1_CODE_RATE_HZ: f64 = 1_023_000.0;
pub const GALILEO_E1_PRIMARY_PERIOD_MS: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GalileoE1Channel {
    E1B,
    E1C,
}

pub fn generate_galileo_e1b_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    generate_galileo_e1_primary_code(prn, GalileoE1Channel::E1B)
}

pub fn generate_galileo_e1c_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    generate_galileo_e1_primary_code(prn, GalileoE1Channel::E1C)
}

pub fn generate_galileo_e1_primary_code(
    prn: u8,
    channel: GalileoE1Channel,
) -> Result<Vec<i8>, SignalError> {
    let index = usize::from(prn.saturating_sub(1));
    let hex = match channel {
        GalileoE1Channel::E1B => E1B_HEX.get(index),
        GalileoE1Channel::E1C => E1C_HEX.get(index),
    }
    .ok_or(SignalError::UnsupportedPrn(prn))?;

    let mut code = Vec::with_capacity(GALILEO_E1_PRIMARY_CODE_CHIPS);
    for ch in hex.bytes() {
        let nibble = (ch as char).to_digit(16).ok_or(SignalError::UnsupportedPrn(prn))? as u8;
        for bit in (0..4).rev() {
            code.push(1 - (2 * ((nibble >> bit) & 1) as i8));
        }
    }
    debug_assert_eq!(code.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    Ok(code)
}

pub fn galileo_e1c_secondary_code() -> [i8; GALILEO_E1_SECONDARY_CODE_CHIPS] {
    let mut code = [0_i8; GALILEO_E1_SECONDARY_CODE_CHIPS];
    debug_assert_eq!(E1C_SECONDARY_CODE.len(), GALILEO_E1_SECONDARY_CODE_CHIPS);
    for (chip, bit) in code.iter_mut().zip(E1C_SECONDARY_CODE.bytes()) {
        *chip = 1 - (2 * (bit - b'0') as i8);
    }
    code
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primary_code_decode_lengths_match_icd() {
        let e1b = generate_galileo_e1b_code(1).expect("valid Galileo E1-B PRN");
        let e1c = generate_galileo_e1c_code(1).expect("valid Galileo E1-C PRN");

        assert_eq!(e1b.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
        assert_eq!(e1c.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
        assert!(e1b.iter().all(|chip| *chip == -1 || *chip == 1));
        assert!(e1c.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn primary_code_rejects_out_of_range_prns() {
        assert_eq!(
            generate_galileo_e1_primary_code(0, GalileoE1Channel::E1B),
            Err(SignalError::UnsupportedPrn(0))
        );
        assert_eq!(
            generate_galileo_e1_primary_code(51, GalileoE1Channel::E1C),
            Err(SignalError::UnsupportedPrn(51))
        );
    }

    #[test]
    fn secondary_code_matches_published_length_and_signs() {
        let secondary = galileo_e1c_secondary_code();
        assert_eq!(secondary.len(), GALILEO_E1_SECONDARY_CODE_CHIPS);
        assert_eq!(secondary[0], 1);
        assert_eq!(secondary[2], -1);
        assert_eq!(secondary[24], 1);
    }
}
