#![allow(missing_docs)]

//! BeiDou D1 navigation-data and Neumann-Hoffman epoch-modulation helpers.
//!
//! Clean-room implementation derived from the public BDS-SIS-ICD-2.0 open-service
//! definition for B1I/B2I MEO/IGSO signals.

use crate::error::SignalError;

/// Number of one-millisecond primary-code epochs covered by one D1 navigation symbol.
pub const BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL: usize = 20;
/// D1 navigation symbol period in seconds.
pub const BEIDOU_D1_NAV_SYMBOL_PERIOD_S: f64 = 0.020;
/// D1 NH overlay length in one-millisecond primary-code epochs.
pub const BEIDOU_D1_SECONDARY_CODE_CHIPS: usize = BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL;
/// D1 NH overlay chip period in seconds.
pub const BEIDOU_D1_SECONDARY_CHIP_PERIOD_S: f64 = 0.001;

// ICD bits use 0/1 polarity; convert to the crate's bipolar {-1, +1} convention.
const BEIDOU_D1_NH_CODE: [i8; BEIDOU_D1_SECONDARY_CODE_CHIPS] =
    [1, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1];

/// Return the published D1 NH overlay as bipolar chips.
pub fn beidou_d1_neumann_hoffman_code() -> &'static [i8; BEIDOU_D1_SECONDARY_CODE_CHIPS] {
    &BEIDOU_D1_NH_CODE
}

/// Return the D1 NH chip applied during one primary-code epoch.
pub fn beidou_d1_nh_chip(primary_code_period_index: usize) -> i8 {
    BEIDOU_D1_NH_CODE[primary_code_period_index % BEIDOU_D1_NH_CODE.len()]
}

/// Return the epoch index inside the current 20 ms D1 symbol.
pub fn beidou_d1_symbol_epoch(primary_code_period_index: usize) -> usize {
    primary_code_period_index % BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL
}

/// Return the zero-based D1 navigation-symbol index for one primary-code epoch.
pub fn beidou_d1_data_symbol_index(primary_code_period_index: usize) -> usize {
    primary_code_period_index / BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL
}

/// Return the bipolar epoch symbol after combining D1 navigation data and NH overlay.
pub fn beidou_d1_epoch_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    Ok(beidou_d1_nh_chip(primary_code_period_index)
        * data_symbol_at_period(data_symbols, primary_code_period_index)?)
}

fn data_symbol_at_period(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    if data_symbols.is_empty() {
        return Err(SignalError::EmptyNavigationSymbolStream);
    }
    let symbol_index = beidou_d1_data_symbol_index(primary_code_period_index) % data_symbols.len();
    validate_data_symbol(data_symbols[symbol_index])
}

fn validate_data_symbol(symbol: i8) -> Result<i8, SignalError> {
    match symbol {
        -1 | 1 => Ok(symbol),
        other => Err(SignalError::InvalidNavigationSymbol(other)),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        beidou_d1_data_symbol_index, beidou_d1_epoch_symbol, beidou_d1_neumann_hoffman_code,
        beidou_d1_nh_chip, beidou_d1_symbol_epoch, BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL,
    };
    use crate::error::SignalError;

    #[test]
    fn nh_helpers_follow_twenty_millisecond_boundaries() {
        assert_eq!(beidou_d1_neumann_hoffman_code().len(), BEIDOU_D1_PRIMARY_EPOCHS_PER_SYMBOL);
        assert_eq!(beidou_d1_nh_chip(0), 1);
        assert_eq!(beidou_d1_nh_chip(5), -1);
        assert_eq!(beidou_d1_nh_chip(19), 1);
        assert_eq!(beidou_d1_nh_chip(20), 1);
        assert_eq!(beidou_d1_symbol_epoch(0), 0);
        assert_eq!(beidou_d1_symbol_epoch(19), 19);
        assert_eq!(beidou_d1_symbol_epoch(20), 0);
    }

    #[test]
    fn epoch_symbol_combines_navigation_data_and_nh_overlay() {
        assert_eq!(beidou_d1_data_symbol_index(0), 0);
        assert_eq!(beidou_d1_data_symbol_index(19), 0);
        assert_eq!(beidou_d1_data_symbol_index(20), 1);
        assert_eq!(beidou_d1_epoch_symbol(&[1, -1], 0), Ok(1));
        assert_eq!(beidou_d1_epoch_symbol(&[1, -1], 5), Ok(-1));
        assert_eq!(beidou_d1_epoch_symbol(&[1, -1], 20), Ok(-1));
        assert_eq!(beidou_d1_epoch_symbol(&[-1], 0), Ok(-1));
        assert_eq!(beidou_d1_epoch_symbol(&[-1], 5), Ok(1));
    }

    #[test]
    fn epoch_symbol_rejects_empty_or_invalid_data_symbols() {
        assert_eq!(beidou_d1_epoch_symbol(&[], 0), Err(SignalError::EmptyNavigationSymbolStream));
        assert_eq!(beidou_d1_epoch_symbol(&[0], 0), Err(SignalError::InvalidNavigationSymbol(0)));
    }
}
