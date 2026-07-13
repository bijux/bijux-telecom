#![allow(missing_docs)]

//! GPS L5-I primary-code and navigation-symbol helpers.
//!
//! Clean-room implementation derived from the public IS-GPS-705 signal definition.

use crate::dsp::signal::{code_value_at_phase, sample_code};
use crate::error::SignalError;

/// Number of chips in one GPS L5 primary-code period.
pub const GPS_L5_PRIMARY_CODE_CHIPS: usize = 10_230;
/// GPS L5 primary-code chip rate.
pub const GPS_L5_PRIMARY_CODE_RATE_HZ: f64 = 10_230_000.0;
/// Number of one-millisecond primary-code epochs in one GPS L5-I data symbol.
pub const GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL: usize = 10;
/// Transmitted L5-I chips covered by one data symbol.
pub const GPS_L5_I_SYMBOL_CHIPS: usize =
    GPS_L5_PRIMARY_CODE_CHIPS * GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL;

const GPS_L5_XA_SHORT_CYCLE_CHIPS: usize = 8190;
const GPS_L5_REGISTER_LENGTH: usize = 13;
const GPS_L5_I_NEUMANN_HOFFMAN_CODE: [i8; GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL] =
    [1, 1, 1, 1, -1, -1, 1, -1, 1, -1];

type RegisterState = [u8; GPS_L5_REGISTER_LENGTH];

/// Published GPS L5-I assignment metadata for one PRN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsL5ICodeAssignment {
    pub prn: u8,
    pub xb_advance_chips: u16,
    pub xb_initial_state_bits: u16,
}

const GPS_L5_I_CODE_ASSIGNMENTS: [GpsL5ICodeAssignment; 63] = [
    GpsL5ICodeAssignment { prn: 1, xb_advance_chips: 266, xb_initial_state_bits: 0b0101011100100 },
    GpsL5ICodeAssignment { prn: 2, xb_advance_chips: 365, xb_initial_state_bits: 0b1100000110101 },
    GpsL5ICodeAssignment { prn: 3, xb_advance_chips: 804, xb_initial_state_bits: 0b0100000001000 },
    GpsL5ICodeAssignment { prn: 4, xb_advance_chips: 1138, xb_initial_state_bits: 0b1011000100110 },
    GpsL5ICodeAssignment { prn: 5, xb_advance_chips: 1509, xb_initial_state_bits: 0b1110111010111 },
    GpsL5ICodeAssignment { prn: 6, xb_advance_chips: 1559, xb_initial_state_bits: 0b0110011111010 },
    GpsL5ICodeAssignment { prn: 7, xb_advance_chips: 1756, xb_initial_state_bits: 0b1010010011111 },
    GpsL5ICodeAssignment { prn: 8, xb_advance_chips: 2084, xb_initial_state_bits: 0b1011110100100 },
    GpsL5ICodeAssignment { prn: 9, xb_advance_chips: 2170, xb_initial_state_bits: 0b1111100101011 },
    GpsL5ICodeAssignment {
        prn: 10,
        xb_advance_chips: 2303,
        xb_initial_state_bits: 0b0111111011110,
    },
    GpsL5ICodeAssignment {
        prn: 11,
        xb_advance_chips: 2527,
        xb_initial_state_bits: 0b0000100111010,
    },
    GpsL5ICodeAssignment {
        prn: 12,
        xb_advance_chips: 2687,
        xb_initial_state_bits: 0b1110011111001,
    },
    GpsL5ICodeAssignment {
        prn: 13,
        xb_advance_chips: 2930,
        xb_initial_state_bits: 0b0001110011100,
    },
    GpsL5ICodeAssignment {
        prn: 14,
        xb_advance_chips: 3471,
        xb_initial_state_bits: 0b0100000100111,
    },
    GpsL5ICodeAssignment {
        prn: 15,
        xb_advance_chips: 3940,
        xb_initial_state_bits: 0b0110101011010,
    },
    GpsL5ICodeAssignment {
        prn: 16,
        xb_advance_chips: 4132,
        xb_initial_state_bits: 0b0001111001001,
    },
    GpsL5ICodeAssignment {
        prn: 17,
        xb_advance_chips: 4332,
        xb_initial_state_bits: 0b0100110001111,
    },
    GpsL5ICodeAssignment {
        prn: 18,
        xb_advance_chips: 4924,
        xb_initial_state_bits: 0b1111000011110,
    },
    GpsL5ICodeAssignment {
        prn: 19,
        xb_advance_chips: 5343,
        xb_initial_state_bits: 0b1100100011111,
    },
    GpsL5ICodeAssignment {
        prn: 20,
        xb_advance_chips: 5443,
        xb_initial_state_bits: 0b0110101101101,
    },
    GpsL5ICodeAssignment {
        prn: 21,
        xb_advance_chips: 5641,
        xb_initial_state_bits: 0b0010000001000,
    },
    GpsL5ICodeAssignment {
        prn: 22,
        xb_advance_chips: 5816,
        xb_initial_state_bits: 0b1110111101111,
    },
    GpsL5ICodeAssignment {
        prn: 23,
        xb_advance_chips: 5898,
        xb_initial_state_bits: 0b1000011111110,
    },
    GpsL5ICodeAssignment {
        prn: 24,
        xb_advance_chips: 5918,
        xb_initial_state_bits: 0b1100010110100,
    },
    GpsL5ICodeAssignment {
        prn: 25,
        xb_advance_chips: 5955,
        xb_initial_state_bits: 0b1101001101101,
    },
    GpsL5ICodeAssignment {
        prn: 26,
        xb_advance_chips: 6243,
        xb_initial_state_bits: 0b1010110010110,
    },
    GpsL5ICodeAssignment {
        prn: 27,
        xb_advance_chips: 6345,
        xb_initial_state_bits: 0b0101011011110,
    },
    GpsL5ICodeAssignment {
        prn: 28,
        xb_advance_chips: 6477,
        xb_initial_state_bits: 0b0111101010110,
    },
    GpsL5ICodeAssignment {
        prn: 29,
        xb_advance_chips: 6518,
        xb_initial_state_bits: 0b0101111100001,
    },
    GpsL5ICodeAssignment {
        prn: 30,
        xb_advance_chips: 6875,
        xb_initial_state_bits: 0b1000010110111,
    },
    GpsL5ICodeAssignment {
        prn: 31,
        xb_advance_chips: 7168,
        xb_initial_state_bits: 0b0001010011110,
    },
    GpsL5ICodeAssignment {
        prn: 32,
        xb_advance_chips: 7187,
        xb_initial_state_bits: 0b0000010111001,
    },
    GpsL5ICodeAssignment {
        prn: 33,
        xb_advance_chips: 7329,
        xb_initial_state_bits: 0b1101010000001,
    },
    GpsL5ICodeAssignment {
        prn: 34,
        xb_advance_chips: 7577,
        xb_initial_state_bits: 0b1101111111001,
    },
    GpsL5ICodeAssignment {
        prn: 35,
        xb_advance_chips: 7720,
        xb_initial_state_bits: 0b1111011011100,
    },
    GpsL5ICodeAssignment {
        prn: 36,
        xb_advance_chips: 7777,
        xb_initial_state_bits: 0b1001011001000,
    },
    GpsL5ICodeAssignment {
        prn: 37,
        xb_advance_chips: 8057,
        xb_initial_state_bits: 0b0011010010000,
    },
    GpsL5ICodeAssignment {
        prn: 38,
        xb_advance_chips: 5358,
        xb_initial_state_bits: 0b0101100000110,
    },
    GpsL5ICodeAssignment {
        prn: 39,
        xb_advance_chips: 3550,
        xb_initial_state_bits: 0b1001001100101,
    },
    GpsL5ICodeAssignment {
        prn: 40,
        xb_advance_chips: 3412,
        xb_initial_state_bits: 0b1100111001010,
    },
    GpsL5ICodeAssignment { prn: 41, xb_advance_chips: 819, xb_initial_state_bits: 0b0111011011001 },
    GpsL5ICodeAssignment {
        prn: 42,
        xb_advance_chips: 4608,
        xb_initial_state_bits: 0b0011101101100,
    },
    GpsL5ICodeAssignment {
        prn: 43,
        xb_advance_chips: 3698,
        xb_initial_state_bits: 0b0011011111010,
    },
    GpsL5ICodeAssignment { prn: 44, xb_advance_chips: 962, xb_initial_state_bits: 0b1001011010001 },
    GpsL5ICodeAssignment {
        prn: 45,
        xb_advance_chips: 3001,
        xb_initial_state_bits: 0b1001010111111,
    },
    GpsL5ICodeAssignment {
        prn: 46,
        xb_advance_chips: 4441,
        xb_initial_state_bits: 0b0111000111101,
    },
    GpsL5ICodeAssignment {
        prn: 47,
        xb_advance_chips: 4937,
        xb_initial_state_bits: 0b0000001000100,
    },
    GpsL5ICodeAssignment {
        prn: 48,
        xb_advance_chips: 3717,
        xb_initial_state_bits: 0b1000101010001,
    },
    GpsL5ICodeAssignment {
        prn: 49,
        xb_advance_chips: 4730,
        xb_initial_state_bits: 0b0011010001001,
    },
    GpsL5ICodeAssignment {
        prn: 50,
        xb_advance_chips: 7291,
        xb_initial_state_bits: 0b1000111110001,
    },
    GpsL5ICodeAssignment {
        prn: 51,
        xb_advance_chips: 2279,
        xb_initial_state_bits: 0b1011100101001,
    },
    GpsL5ICodeAssignment {
        prn: 52,
        xb_advance_chips: 7613,
        xb_initial_state_bits: 0b0100101011010,
    },
    GpsL5ICodeAssignment {
        prn: 53,
        xb_advance_chips: 5723,
        xb_initial_state_bits: 0b0000001000010,
    },
    GpsL5ICodeAssignment {
        prn: 54,
        xb_advance_chips: 7030,
        xb_initial_state_bits: 0b0110001101110,
    },
    GpsL5ICodeAssignment {
        prn: 55,
        xb_advance_chips: 1475,
        xb_initial_state_bits: 0b0000011001110,
    },
    GpsL5ICodeAssignment {
        prn: 56,
        xb_advance_chips: 2593,
        xb_initial_state_bits: 0b1110111011110,
    },
    GpsL5ICodeAssignment {
        prn: 57,
        xb_advance_chips: 2904,
        xb_initial_state_bits: 0b0001000010011,
    },
    GpsL5ICodeAssignment {
        prn: 58,
        xb_advance_chips: 2056,
        xb_initial_state_bits: 0b0000010100001,
    },
    GpsL5ICodeAssignment {
        prn: 59,
        xb_advance_chips: 2757,
        xb_initial_state_bits: 0b0100001100001,
    },
    GpsL5ICodeAssignment {
        prn: 60,
        xb_advance_chips: 3756,
        xb_initial_state_bits: 0b0100101001001,
    },
    GpsL5ICodeAssignment {
        prn: 61,
        xb_advance_chips: 6205,
        xb_initial_state_bits: 0b0011110011110,
    },
    GpsL5ICodeAssignment {
        prn: 62,
        xb_advance_chips: 5053,
        xb_initial_state_bits: 0b1011000110001,
    },
    GpsL5ICodeAssignment {
        prn: 63,
        xb_advance_chips: 6437,
        xb_initial_state_bits: 0b0101111001011,
    },
];

/// Return the published GPS L5-I assignment for one PRN.
pub fn gps_l5_i_code_assignment(prn: u8) -> Result<&'static GpsL5ICodeAssignment, SignalError> {
    GPS_L5_I_CODE_ASSIGNMENTS
        .iter()
        .find(|assignment| assignment.prn == prn)
        .ok_or(SignalError::UnsupportedPrn(prn))
}

/// Return the published GPS L5-I assignments for all supported PRNs.
pub fn gps_l5_i_code_assignments() -> &'static [GpsL5ICodeAssignment; 63] {
    &GPS_L5_I_CODE_ASSIGNMENTS
}

/// Return the GPS L5-I Neumann-Hoffman secondary code as bipolar values.
pub fn gps_l5_i_neumann_hoffman_code() -> &'static [i8; GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL] {
    &GPS_L5_I_NEUMANN_HOFFMAN_CODE
}

/// Return the one-millisecond primary-code epoch index within the current L5-I data symbol.
pub fn gps_l5_i_symbol_epoch(primary_code_period_index: usize) -> usize {
    primary_code_period_index % GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL
}

/// Return the navigation-symbol index covering one primary-code epoch.
pub fn gps_l5_i_data_symbol_index(primary_code_period_index: usize) -> usize {
    primary_code_period_index / GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL
}

/// Generate one full-period GPS L5-I primary code for a given PRN.
///
/// Returns chips in `{-1, +1}`.
pub fn generate_gps_l5_i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let assignment = gps_l5_i_code_assignment(prn)?;
    let mut xa = [1_u8; GPS_L5_REGISTER_LENGTH];
    let mut xb = [1_u8; GPS_L5_REGISTER_LENGTH];
    advance_l5_register(&mut xb, usize::from(assignment.xb_advance_chips), step_xb);

    let mut code = Vec::with_capacity(GPS_L5_PRIMARY_CODE_CHIPS);
    for chip_index in 0..GPS_L5_PRIMARY_CODE_CHIPS {
        if chip_index == GPS_L5_XA_SHORT_CYCLE_CHIPS {
            xa = [1_u8; GPS_L5_REGISTER_LENGTH];
        }

        let chip = xa[GPS_L5_REGISTER_LENGTH - 1] ^ xb[GPS_L5_REGISTER_LENGTH - 1];
        code.push(if chip == 0 { 1 } else { -1 });

        step_xa(&mut xa);
        step_xb(&mut xb);
    }

    Ok(code)
}

/// Generate GPS L5-I transmitted chips by repeating the primary code and applying
/// the Neumann-Hoffman overlay and data-symbol timing.
pub fn generate_gps_l5_i_chips(
    prn: u8,
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    let code = generate_gps_l5_i_code(prn)?;
    generate_gps_l5_i_chips_from_code(&code, start_chip, chip_count, data_symbols)
}

/// Generate transmitted L5-I chips from one explicit primary code.
pub fn generate_gps_l5_i_chips_from_code(
    code: &[i8],
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    validate_navigation_symbol_stream(data_symbols)?;

    let mut chips = Vec::with_capacity(chip_count);
    for offset in 0..chip_count {
        let chip_phase = (start_chip + offset) as f64;
        chips.push(gps_l5_i_value(code, chip_phase, data_symbols)? as i8);
    }
    Ok(chips)
}

/// Sample the GPS L5-I transmitted chip stream at an arbitrary sample rate.
pub fn sample_gps_l5_i(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<f32>, SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !start_chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let code = generate_gps_l5_i_code(prn)?;
    sample_gps_l5_i_from_code(&code, sample_rate_hz, start_chip_phase, sample_count, data_symbols)
}

/// Sample the transmitted L5-I signal from one explicit primary code.
pub fn sample_gps_l5_i_from_code(
    code: &[i8],
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<f32>, SignalError> {
    if !sample_rate_hz.is_finite() || sample_rate_hz <= 0.0 {
        return Err(SignalError::InvalidSampleRate);
    }
    if !start_chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }
    validate_navigation_symbol_stream(data_symbols)?;

    let chips_per_sample = GPS_L5_PRIMARY_CODE_RATE_HZ / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        samples.push(gps_l5_i_value(code, chip_phase, data_symbols)?);
    }
    Ok(samples)
}

/// Sample the transmitted L5-I value at one emitted-chip phase.
pub fn gps_l5_i_value(
    code: &[i8],
    chip_phase: f64,
    data_symbols: &[i8],
) -> Result<f32, SignalError> {
    validate_navigation_symbol_stream(data_symbols)?;
    if !chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let code_length = GPS_L5_PRIMARY_CODE_CHIPS as f64;
    let normalized_chip_phase = chip_phase.max(0.0);
    let primary_code_period_index = (normalized_chip_phase / code_length).floor() as usize;
    let primary_chip_phase = chip_phase.rem_euclid(code_length);

    Ok(code_value_at_phase(code, primary_chip_phase)?
        * current_navigation_symbol(data_symbols, primary_code_period_index)? as f32)
}

/// Sample the GPS L5-I primary code at an arbitrary sample rate from a chip-phase origin.
pub fn sample_gps_l5_i_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_gps_l5_i_code(prn)?;
    sample_code(&code, sample_rate_hz, GPS_L5_PRIMARY_CODE_RATE_HZ, start_chip_phase, sample_count)
}

/// Return the transmitted L5-I sign for one primary-code epoch.
pub fn gps_l5_i_epoch_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    validate_navigation_symbol_stream(data_symbols)?;
    current_navigation_symbol(data_symbols, primary_code_period_index)
}

fn current_navigation_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    let data_symbol_index =
        gps_l5_i_data_symbol_index(primary_code_period_index) % data_symbols.len();
    let data_symbol = validate_navigation_symbol(data_symbols[data_symbol_index])?;
    Ok(data_symbol
        * GPS_L5_I_NEUMANN_HOFFMAN_CODE[gps_l5_i_symbol_epoch(primary_code_period_index)])
}

fn validate_navigation_symbol_stream(data_symbols: &[i8]) -> Result<(), SignalError> {
    if data_symbols.is_empty() {
        return Err(SignalError::EmptyNavigationSymbolStream);
    }
    for symbol in data_symbols {
        validate_navigation_symbol(*symbol)?;
    }
    Ok(())
}

fn validate_navigation_symbol(symbol: i8) -> Result<i8, SignalError> {
    match symbol {
        -1 | 1 => Ok(symbol),
        other => Err(SignalError::InvalidNavigationSymbol(other)),
    }
}

fn advance_l5_register(
    register: &mut RegisterState,
    chip_count: usize,
    step: fn(&mut RegisterState),
) {
    for _ in 0..chip_count {
        step(register);
    }
}

fn step_xa(register: &mut RegisterState) {
    let feedback = register[8] ^ register[9] ^ register[11] ^ register[12];
    shift_register(register, feedback);
}

fn step_xb(register: &mut RegisterState) {
    let feedback = register[0]
        ^ register[2]
        ^ register[3]
        ^ register[5]
        ^ register[6]
        ^ register[7]
        ^ register[11]
        ^ register[12];
    shift_register(register, feedback);
}

fn shift_register(register: &mut RegisterState, feedback: u8) {
    for index in (1..GPS_L5_REGISTER_LENGTH).rev() {
        register[index] = register[index - 1];
    }
    register[0] = feedback & 1;
}

#[cfg(test)]
fn register_state_to_bits(register: &RegisterState) -> u16 {
    let mut bits = 0_u16;
    for bit in register {
        bits = (bits << 1) | u16::from(*bit);
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::{
        generate_gps_l5_i_chips, generate_gps_l5_i_code, gps_l5_i_code_assignment,
        gps_l5_i_code_assignments, gps_l5_i_data_symbol_index, gps_l5_i_epoch_symbol,
        gps_l5_i_neumann_hoffman_code, gps_l5_i_symbol_epoch, register_state_to_bits, step_xb,
        GpsL5ICodeAssignment, GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL, GPS_L5_I_SYMBOL_CHIPS,
        GPS_L5_PRIMARY_CODE_CHIPS, GPS_L5_PRIMARY_CODE_RATE_HZ,
    };
    use crate::error::SignalError;

    #[test]
    fn gps_l5_i_assignments_cover_published_prn_range() {
        let assignments = gps_l5_i_code_assignments();
        let first = assignments.first().expect("first assignment");
        let last = assignments.last().expect("last assignment");

        assert_eq!(assignments.len(), 63);
        assert_eq!(first.prn, 1);
        assert_eq!(last.prn, 63);
    }

    #[test]
    fn gps_l5_i_assignments_match_selected_official_rows() {
        assert_eq!(
            gps_l5_i_code_assignment(1),
            Ok(&GpsL5ICodeAssignment {
                prn: 1,
                xb_advance_chips: 266,
                xb_initial_state_bits: 0b0101011100100,
            })
        );
        assert_eq!(
            gps_l5_i_code_assignment(37),
            Ok(&GpsL5ICodeAssignment {
                prn: 37,
                xb_advance_chips: 8057,
                xb_initial_state_bits: 0b0011010010000,
            })
        );
        assert_eq!(
            gps_l5_i_code_assignment(63),
            Ok(&GpsL5ICodeAssignment {
                prn: 63,
                xb_advance_chips: 6437,
                xb_initial_state_bits: 0b0101111001011,
            })
        );
    }

    #[test]
    fn gps_l5_i_generator_rejects_unsupported_prns() {
        assert_eq!(gps_l5_i_code_assignment(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_gps_l5_i_code(64), Err(SignalError::UnsupportedPrn(64)));
    }

    #[test]
    fn gps_l5_i_primary_code_is_bipolar_and_has_expected_length() {
        let code = generate_gps_l5_i_code(7).expect("valid L5-I PRN");

        assert_eq!(code.len(), GPS_L5_PRIMARY_CODE_CHIPS);
        assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
    }

    #[test]
    fn gps_l5_i_xb_advances_land_on_published_initial_states() {
        for prn in [1_u8, 20, 37, 63] {
            let assignment = gps_l5_i_code_assignment(prn).expect("published assignment");
            let mut xb = [1_u8; 13];
            for _ in 0..assignment.xb_advance_chips {
                step_xb(&mut xb);
            }
            assert_eq!(register_state_to_bits(&xb), assignment.xb_initial_state_bits, "prn={prn}");
        }
    }

    #[test]
    fn gps_l5_i_primary_code_prefix_matches_published_initial_xb_complements() {
        let code = generate_gps_l5_i_code(7).expect("valid L5-I PRN");
        let assignment = gps_l5_i_code_assignment(7).expect("published assignment");

        let expected =
            (0..13)
                .map(|bit_index| {
                    if ((assignment.xb_initial_state_bits >> bit_index) & 1) == 0 {
                        -1
                    } else {
                        1
                    }
                })
                .collect::<Vec<i8>>();

        assert_eq!(&code[..13], expected.as_slice());
    }

    #[test]
    fn gps_l5_i_epoch_symbol_applies_neumann_hoffman_sequence_and_symbol_timing() {
        let nh = gps_l5_i_neumann_hoffman_code();

        assert_eq!(gps_l5_i_epoch_symbol(&[1], 0).expect("first epoch"), nh[0]);
        assert_eq!(gps_l5_i_epoch_symbol(&[1], 4).expect("fifth epoch"), nh[4]);
        assert_eq!(gps_l5_i_epoch_symbol(&[-1], 0).expect("negative data symbol"), -nh[0]);
        assert_eq!(gps_l5_i_epoch_symbol(&[1, -1], 10).expect("second data symbol"), -nh[0]);
        assert_eq!(gps_l5_i_symbol_epoch(17), 7);
        assert_eq!(gps_l5_i_data_symbol_index(17), 1);
    }

    #[test]
    fn gps_l5_i_transmitted_chip_generation_respects_epoch_and_symbol_boundaries() {
        let code = generate_gps_l5_i_code(7).expect("valid L5-I PRN");
        let chips =
            generate_gps_l5_i_chips(7, 0, GPS_L5_I_SYMBOL_CHIPS + 2, &[1, -1]).expect("chips");

        assert_eq!(chips[0], code[0]);
        assert_eq!(chips[GPS_L5_PRIMARY_CODE_CHIPS * 4], -code[0]);
        assert_eq!(chips[GPS_L5_PRIMARY_CODE_CHIPS * 10], -code[0]);
        assert_eq!(chips[GPS_L5_I_SYMBOL_CHIPS], -code[0]);
    }

    #[test]
    fn gps_l5_i_transmitted_chip_generation_rejects_invalid_navigation_symbols() {
        assert_eq!(
            generate_gps_l5_i_chips(7, 0, 1, &[0]),
            Err(SignalError::InvalidNavigationSymbol(0))
        );
    }

    #[test]
    fn gps_l5_i_neumann_hoffman_code_has_expected_period() {
        let nh = gps_l5_i_neumann_hoffman_code();

        assert_eq!(nh.len(), GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL);
        assert_eq!(nh, &[1, 1, 1, 1, -1, -1, 1, -1, 1, -1]);
    }

    #[test]
    fn gps_l5_i_primary_code_samples_at_chip_rate() {
        let code = generate_gps_l5_i_code(7).expect("valid L5-I PRN");
        let samples =
            crate::codes::gps_l5::sample_gps_l5_i(7, GPS_L5_PRIMARY_CODE_RATE_HZ, 0.0, 8, &[1])
                .expect("samples");

        assert_eq!(samples, code[..8].iter().copied().map(f32::from).collect::<Vec<f32>>());
    }
}
