#![allow(missing_docs)]

//! GPS L5-I primary-code and modulation helpers.
//!
//! Clean-room implementation derived from the public IS-GPS-705J signal definition.

use crate::dsp::signal::{code_value_at_phase, sample_code};
use crate::error::SignalError;

pub const GPS_L5_CODE_CHIPS: usize = 10_230;
pub const GPS_L5_PRIMARY_CODE_CHIPS: usize = GPS_L5_CODE_CHIPS;
pub const GPS_L5_CODE_RATE_HZ: f64 = 10_230_000.0;
pub const GPS_L5_PRIMARY_CODE_RATE_HZ: f64 = GPS_L5_CODE_RATE_HZ;
pub const GPS_L5I_SYMBOL_CODE_EPOCHS: usize = 10;
pub const GPS_L5_I_PRIMARY_EPOCHS_PER_SYMBOL: usize = GPS_L5I_SYMBOL_CODE_EPOCHS;
pub const GPS_L5I_SYMBOL_CHIPS: usize = GPS_L5_CODE_CHIPS * GPS_L5I_SYMBOL_CODE_EPOCHS;
pub const GPS_L5_I_SYMBOL_CHIPS: usize = GPS_L5I_SYMBOL_CHIPS;

const GPS_L5_XA_CHIPS: usize = 8_190;
const GPS_L5_XB_CHIPS: usize = 8_191;
const GPS_L5_REGISTER_STAGES: usize = 13;
const GPS_L5I_POWER_SCALE: f32 = std::f32::consts::FRAC_1_SQRT_2;
const GPS_L5I_NH_CODE: [i8; GPS_L5I_SYMBOL_CODE_EPOCHS] = [1, 1, 1, 1, -1, -1, 1, -1, 1, -1];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsL5ICodeAssignment {
    pub prn: u8,
    pub xb_advance_chips: usize,
    pub xb_initial_state_bits: &'static str,
}

impl GpsL5ICodeAssignment {
    fn parsed_initial_xb_state(self) -> [u8; GPS_L5_REGISTER_STAGES] {
        let mut parsed = [0_u8; GPS_L5_REGISTER_STAGES];
        for (slot, bit) in parsed.iter_mut().zip(self.xb_initial_state_bits.bytes()) {
            *slot = match bit {
                b'0' => 0,
                b'1' => 1,
                _ => unreachable!("table contains only binary digits"),
            };
        }
        parsed
    }
}

const GPS_L5_I_CODE_ASSIGNMENTS: [GpsL5ICodeAssignment; 63] = [
    GpsL5ICodeAssignment { prn: 1, xb_advance_chips: 266, xb_initial_state_bits: "0101011100100" },
    GpsL5ICodeAssignment { prn: 2, xb_advance_chips: 365, xb_initial_state_bits: "1100000110101" },
    GpsL5ICodeAssignment { prn: 3, xb_advance_chips: 804, xb_initial_state_bits: "0100000001000" },
    GpsL5ICodeAssignment { prn: 4, xb_advance_chips: 1138, xb_initial_state_bits: "1011000100110" },
    GpsL5ICodeAssignment { prn: 5, xb_advance_chips: 1509, xb_initial_state_bits: "1110111010111" },
    GpsL5ICodeAssignment { prn: 6, xb_advance_chips: 1559, xb_initial_state_bits: "0110011111010" },
    GpsL5ICodeAssignment { prn: 7, xb_advance_chips: 1756, xb_initial_state_bits: "1010010011111" },
    GpsL5ICodeAssignment { prn: 8, xb_advance_chips: 2084, xb_initial_state_bits: "1011110100100" },
    GpsL5ICodeAssignment { prn: 9, xb_advance_chips: 2170, xb_initial_state_bits: "1111100101011" },
    GpsL5ICodeAssignment {
        prn: 10,
        xb_advance_chips: 2303,
        xb_initial_state_bits: "0111111011110",
    },
    GpsL5ICodeAssignment {
        prn: 11,
        xb_advance_chips: 2527,
        xb_initial_state_bits: "0000100111010",
    },
    GpsL5ICodeAssignment {
        prn: 12,
        xb_advance_chips: 2687,
        xb_initial_state_bits: "1110011111001",
    },
    GpsL5ICodeAssignment {
        prn: 13,
        xb_advance_chips: 2930,
        xb_initial_state_bits: "0001110011100",
    },
    GpsL5ICodeAssignment {
        prn: 14,
        xb_advance_chips: 3471,
        xb_initial_state_bits: "0100000100111",
    },
    GpsL5ICodeAssignment {
        prn: 15,
        xb_advance_chips: 3940,
        xb_initial_state_bits: "0110101011010",
    },
    GpsL5ICodeAssignment {
        prn: 16,
        xb_advance_chips: 4132,
        xb_initial_state_bits: "0001111001001",
    },
    GpsL5ICodeAssignment {
        prn: 17,
        xb_advance_chips: 4332,
        xb_initial_state_bits: "0100110001111",
    },
    GpsL5ICodeAssignment {
        prn: 18,
        xb_advance_chips: 4924,
        xb_initial_state_bits: "1111000011110",
    },
    GpsL5ICodeAssignment {
        prn: 19,
        xb_advance_chips: 5343,
        xb_initial_state_bits: "1100100011111",
    },
    GpsL5ICodeAssignment {
        prn: 20,
        xb_advance_chips: 5443,
        xb_initial_state_bits: "0110101101101",
    },
    GpsL5ICodeAssignment {
        prn: 21,
        xb_advance_chips: 5641,
        xb_initial_state_bits: "0010000001000",
    },
    GpsL5ICodeAssignment {
        prn: 22,
        xb_advance_chips: 5816,
        xb_initial_state_bits: "1110111101111",
    },
    GpsL5ICodeAssignment {
        prn: 23,
        xb_advance_chips: 5898,
        xb_initial_state_bits: "1000011111110",
    },
    GpsL5ICodeAssignment {
        prn: 24,
        xb_advance_chips: 5918,
        xb_initial_state_bits: "1100010110100",
    },
    GpsL5ICodeAssignment {
        prn: 25,
        xb_advance_chips: 5955,
        xb_initial_state_bits: "1101001101101",
    },
    GpsL5ICodeAssignment {
        prn: 26,
        xb_advance_chips: 6243,
        xb_initial_state_bits: "1010110010110",
    },
    GpsL5ICodeAssignment {
        prn: 27,
        xb_advance_chips: 6345,
        xb_initial_state_bits: "0101011011110",
    },
    GpsL5ICodeAssignment {
        prn: 28,
        xb_advance_chips: 6477,
        xb_initial_state_bits: "0111101010110",
    },
    GpsL5ICodeAssignment {
        prn: 29,
        xb_advance_chips: 6518,
        xb_initial_state_bits: "0101111100001",
    },
    GpsL5ICodeAssignment {
        prn: 30,
        xb_advance_chips: 6875,
        xb_initial_state_bits: "1000010110111",
    },
    GpsL5ICodeAssignment {
        prn: 31,
        xb_advance_chips: 7168,
        xb_initial_state_bits: "0001010011110",
    },
    GpsL5ICodeAssignment {
        prn: 32,
        xb_advance_chips: 7187,
        xb_initial_state_bits: "0000010111001",
    },
    GpsL5ICodeAssignment {
        prn: 33,
        xb_advance_chips: 7329,
        xb_initial_state_bits: "1101010000001",
    },
    GpsL5ICodeAssignment {
        prn: 34,
        xb_advance_chips: 7577,
        xb_initial_state_bits: "1101111111001",
    },
    GpsL5ICodeAssignment {
        prn: 35,
        xb_advance_chips: 7720,
        xb_initial_state_bits: "1111011011100",
    },
    GpsL5ICodeAssignment {
        prn: 36,
        xb_advance_chips: 7777,
        xb_initial_state_bits: "1001011001000",
    },
    GpsL5ICodeAssignment {
        prn: 37,
        xb_advance_chips: 8057,
        xb_initial_state_bits: "0011010010000",
    },
    GpsL5ICodeAssignment {
        prn: 38,
        xb_advance_chips: 5358,
        xb_initial_state_bits: "0101100000110",
    },
    GpsL5ICodeAssignment {
        prn: 39,
        xb_advance_chips: 3550,
        xb_initial_state_bits: "1001001100101",
    },
    GpsL5ICodeAssignment {
        prn: 40,
        xb_advance_chips: 3412,
        xb_initial_state_bits: "1100111001010",
    },
    GpsL5ICodeAssignment { prn: 41, xb_advance_chips: 819, xb_initial_state_bits: "0111011011001" },
    GpsL5ICodeAssignment {
        prn: 42,
        xb_advance_chips: 4608,
        xb_initial_state_bits: "0011101101100",
    },
    GpsL5ICodeAssignment {
        prn: 43,
        xb_advance_chips: 3698,
        xb_initial_state_bits: "0011011111010",
    },
    GpsL5ICodeAssignment { prn: 44, xb_advance_chips: 962, xb_initial_state_bits: "1001011010001" },
    GpsL5ICodeAssignment {
        prn: 45,
        xb_advance_chips: 3001,
        xb_initial_state_bits: "1001010111111",
    },
    GpsL5ICodeAssignment {
        prn: 46,
        xb_advance_chips: 4441,
        xb_initial_state_bits: "0111000111101",
    },
    GpsL5ICodeAssignment {
        prn: 47,
        xb_advance_chips: 4937,
        xb_initial_state_bits: "0000001000100",
    },
    GpsL5ICodeAssignment {
        prn: 48,
        xb_advance_chips: 3717,
        xb_initial_state_bits: "1000101010001",
    },
    GpsL5ICodeAssignment {
        prn: 49,
        xb_advance_chips: 4730,
        xb_initial_state_bits: "0011010001001",
    },
    GpsL5ICodeAssignment {
        prn: 50,
        xb_advance_chips: 7291,
        xb_initial_state_bits: "1000111110001",
    },
    GpsL5ICodeAssignment {
        prn: 51,
        xb_advance_chips: 2279,
        xb_initial_state_bits: "1011100101001",
    },
    GpsL5ICodeAssignment {
        prn: 52,
        xb_advance_chips: 7613,
        xb_initial_state_bits: "0100101011010",
    },
    GpsL5ICodeAssignment {
        prn: 53,
        xb_advance_chips: 5723,
        xb_initial_state_bits: "0000001000010",
    },
    GpsL5ICodeAssignment {
        prn: 54,
        xb_advance_chips: 7030,
        xb_initial_state_bits: "0110001101110",
    },
    GpsL5ICodeAssignment {
        prn: 55,
        xb_advance_chips: 1475,
        xb_initial_state_bits: "0000011001110",
    },
    GpsL5ICodeAssignment {
        prn: 56,
        xb_advance_chips: 2593,
        xb_initial_state_bits: "1110111011110",
    },
    GpsL5ICodeAssignment {
        prn: 57,
        xb_advance_chips: 2904,
        xb_initial_state_bits: "0001000010011",
    },
    GpsL5ICodeAssignment {
        prn: 58,
        xb_advance_chips: 2056,
        xb_initial_state_bits: "0000010100001",
    },
    GpsL5ICodeAssignment {
        prn: 59,
        xb_advance_chips: 2757,
        xb_initial_state_bits: "0100001100001",
    },
    GpsL5ICodeAssignment {
        prn: 60,
        xb_advance_chips: 3756,
        xb_initial_state_bits: "0100101001001",
    },
    GpsL5ICodeAssignment {
        prn: 61,
        xb_advance_chips: 6205,
        xb_initial_state_bits: "0011110011110",
    },
    GpsL5ICodeAssignment {
        prn: 62,
        xb_advance_chips: 5053,
        xb_initial_state_bits: "1011000110001",
    },
    GpsL5ICodeAssignment {
        prn: 63,
        xb_advance_chips: 6437,
        xb_initial_state_bits: "0101111001011",
    },
];

pub fn gps_l5_i_code_assignment(prn: u8) -> Result<&'static GpsL5ICodeAssignment, SignalError> {
    prn_index(prn).map(|index| &GPS_L5_I_CODE_ASSIGNMENTS[index])
}

pub fn gps_l5_i_code_assignments() -> &'static [GpsL5ICodeAssignment; 63] {
    &GPS_L5_I_CODE_ASSIGNMENTS
}

pub fn generate_gps_l5_i_code(prn: u8) -> Result<Vec<i8>, SignalError> {
    let xb_state = gps_l5_i_code_assignment(prn)?.parsed_initial_xb_state();
    let xa = generate_gps_l5_xa_sequence();
    let xb = generate_gps_l5_xb_sequence(&xb_state);

    let mut code = Vec::with_capacity(GPS_L5_CODE_CHIPS);
    for chip_index in 0..GPS_L5_CODE_CHIPS {
        let xa_chip = xa[chip_index % GPS_L5_XA_CHIPS];
        let xb_chip = xb[chip_index % GPS_L5_XB_CHIPS];
        code.push(if xa_chip == xb_chip { 1 } else { -1 });
    }
    Ok(code)
}

pub fn sample_gps_l5_i_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    let code = generate_gps_l5_i_code(prn)?;
    sample_code(&code, sample_rate_hz, GPS_L5_CODE_RATE_HZ, start_chip_phase, sample_count)
}

pub fn sample_gps_l5_i_primary_code(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
) -> Result<Vec<f32>, SignalError> {
    sample_gps_l5_i_code(prn, sample_rate_hz, start_chip_phase, sample_count)
}

pub fn gps_l5_i_nh_chip(primary_code_period_index: usize) -> i8 {
    GPS_L5I_NH_CODE[primary_code_period_index % GPS_L5I_NH_CODE.len()]
}

pub fn gps_l5_i_neumann_hoffman_code() -> &'static [i8; GPS_L5I_SYMBOL_CODE_EPOCHS] {
    &GPS_L5I_NH_CODE
}

pub fn gps_l5_i_symbol_epoch(primary_code_period_index: usize) -> usize {
    primary_code_period_index % GPS_L5I_SYMBOL_CODE_EPOCHS
}

pub fn gps_l5_i_data_symbol_index(primary_code_period_index: usize) -> usize {
    primary_code_period_index / GPS_L5I_SYMBOL_CODE_EPOCHS
}

pub fn gps_l5_i_epoch_symbol(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    Ok(gps_l5_i_nh_chip(primary_code_period_index)
        * l5_i_data_symbol_at_period(data_symbols, primary_code_period_index)?)
}

pub fn generate_gps_l5_i_chips(
    prn: u8,
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    let code = generate_gps_l5_i_code(prn)?;
    generate_gps_l5_i_chips_from_code(&code, start_chip, chip_count, data_symbols)
}

pub fn generate_gps_l5_i_chips_from_code(
    code: &[i8],
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    validate_l5_i_data_symbols(data_symbols)?;
    if code.len() != GPS_L5_CODE_CHIPS {
        return Err(SignalError::EmptyCodeSequence);
    }

    let mut chips = Vec::with_capacity(chip_count);
    for offset in 0..chip_count {
        let chip_phase = (start_chip + offset) as f64;
        let primary_code_period_index = (start_chip + offset) / GPS_L5_CODE_CHIPS;
        let data_symbol = l5_i_data_symbol_at_period(data_symbols, primary_code_period_index)?;
        chips.push(
            gps_l5_i_value(code, chip_phase, primary_code_period_index, data_symbol)?.signum()
                as i8,
        );
    }
    Ok(chips)
}

pub fn sample_gps_l5_i(
    prn: u8,
    sample_rate_hz: f64,
    start_chip_phase: f64,
    sample_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<f32>, SignalError> {
    let code = generate_gps_l5_i_code(prn)?;
    sample_gps_l5_i_from_code(&code, sample_rate_hz, start_chip_phase, sample_count, data_symbols)
}

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
    validate_l5_i_data_symbols(data_symbols)?;
    if code.len() != GPS_L5_CODE_CHIPS {
        return Err(SignalError::EmptyCodeSequence);
    }

    let chips_per_sample = GPS_L5_CODE_RATE_HZ / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        let primary_code_period_index =
            chip_phase.div_euclid(GPS_L5_CODE_CHIPS as f64).floor() as usize;
        let data_symbol = l5_i_data_symbol_at_period(data_symbols, primary_code_period_index)?;
        samples.push(gps_l5_i_value(code, chip_phase, primary_code_period_index, data_symbol)?);
    }
    Ok(samples)
}

pub fn gps_l5_i_value(
    code: &[i8],
    chip_phase: f64,
    primary_code_period_index: usize,
    data_symbol: i8,
) -> Result<f32, SignalError> {
    validate_l5_i_data_symbol(data_symbol)?;
    let primary = code_value_at_phase(code, chip_phase)?;
    Ok(primary
        * gps_l5_i_epoch_symbol(&[data_symbol], primary_code_period_index)? as f32
        * GPS_L5I_POWER_SCALE)
}

fn generate_gps_l5_xa_sequence() -> [u8; GPS_L5_XA_CHIPS] {
    let mut sequence = [0_u8; GPS_L5_XA_CHIPS];
    let mut register = [1_u8; GPS_L5_REGISTER_STAGES];
    for chip in &mut sequence {
        *chip = register[GPS_L5_REGISTER_STAGES - 1];
        advance_gps_l5_register(&mut register, &[8, 9, 11, 12]);
    }
    sequence
}

fn generate_gps_l5_xb_sequence(
    initial_state: &[u8; GPS_L5_REGISTER_STAGES],
) -> [u8; GPS_L5_XB_CHIPS] {
    let mut sequence = [0_u8; GPS_L5_XB_CHIPS];
    let mut register = *initial_state;
    for chip in &mut sequence {
        *chip = register[GPS_L5_REGISTER_STAGES - 1];
        advance_gps_l5_register(&mut register, &[0, 2, 3, 5, 6, 7, 11, 12]);
    }
    sequence
}

fn advance_gps_l5_register(register: &mut [u8; GPS_L5_REGISTER_STAGES], feedback_taps: &[usize]) {
    let mut feedback = 0_u8;
    for tap in feedback_taps {
        feedback ^= register[*tap];
    }
    for destination in (1..register.len()).rev() {
        register[destination] = register[destination - 1];
    }
    register[0] = feedback;
}

fn l5_i_data_symbol_at_period(
    data_symbols: &[i8],
    primary_code_period_index: usize,
) -> Result<i8, SignalError> {
    let symbol_index = gps_l5_i_data_symbol_index(primary_code_period_index) % data_symbols.len();
    validate_l5_i_data_symbol(data_symbols[symbol_index])
}

fn validate_l5_i_data_symbols(data_symbols: &[i8]) -> Result<(), SignalError> {
    if data_symbols.is_empty() {
        return Err(SignalError::EmptyNavigationSymbolStream);
    }
    for symbol in data_symbols {
        validate_l5_i_data_symbol(*symbol)?;
    }
    Ok(())
}

fn validate_l5_i_data_symbol(symbol: i8) -> Result<i8, SignalError> {
    match symbol {
        -1 | 1 => Ok(symbol),
        other => Err(SignalError::InvalidNavigationSymbol(other)),
    }
}

fn prn_index(prn: u8) -> Result<usize, SignalError> {
    match prn {
        1..=63 => Ok(usize::from(prn - 1)),
        _ => Err(SignalError::UnsupportedPrn(prn)),
    }
}

#[cfg(test)]
mod tests {
    use super::{
        generate_gps_l5_i_chips, generate_gps_l5_i_code, gps_l5_i_code_assignment,
        gps_l5_i_code_assignments, gps_l5_i_data_symbol_index, gps_l5_i_epoch_symbol,
        gps_l5_i_neumann_hoffman_code, gps_l5_i_nh_chip, gps_l5_i_symbol_epoch, sample_gps_l5_i,
        sample_gps_l5_i_primary_code, GpsL5ICodeAssignment, GPS_L5I_SYMBOL_CHIPS,
        GPS_L5_CODE_RATE_HZ,
    };
    use crate::error::SignalError;

    #[test]
    fn assignments_cover_supported_prns() {
        assert_eq!(gps_l5_i_code_assignments().len(), 63);
        assert_eq!(gps_l5_i_code_assignments()[0].prn, 1);
        assert_eq!(gps_l5_i_code_assignments()[62].prn, 63);
    }

    #[test]
    fn selected_assignment_rows_match_expected_values() {
        for expected in [
            GpsL5ICodeAssignment {
                prn: 1,
                xb_advance_chips: 266,
                xb_initial_state_bits: "0101011100100",
            },
            GpsL5ICodeAssignment {
                prn: 38,
                xb_advance_chips: 5358,
                xb_initial_state_bits: "0101100000110",
            },
            GpsL5ICodeAssignment {
                prn: 63,
                xb_advance_chips: 6437,
                xb_initial_state_bits: "0101111001011",
            },
        ] {
            assert_eq!(
                *gps_l5_i_code_assignment(expected.prn).expect("published assignment"),
                expected
            );
        }
    }

    #[test]
    fn primary_code_rejects_out_of_range_prns() {
        assert_eq!(generate_gps_l5_i_code(0), Err(SignalError::UnsupportedPrn(0)));
        assert_eq!(generate_gps_l5_i_code(64), Err(SignalError::UnsupportedPrn(64)));
    }

    #[test]
    fn first_thirteen_primary_chips_match_assignment_state_signs() {
        for prn in [1, 20, 37, 63] {
            let assignment = gps_l5_i_code_assignment(prn).expect("published assignment");
            let code = generate_gps_l5_i_code(prn).expect("valid GPS L5-I PRN");
            for (chip_index, xb_bit) in assignment.xb_initial_state_bits.bytes().enumerate() {
                let expected_chip = if xb_bit == b'0' { -1 } else { 1 };
                assert_eq!(code[chip_index], expected_chip, "prn={prn} chip={chip_index}");
            }
        }
    }

    #[test]
    fn nh_and_symbol_helpers_follow_ten_millisecond_boundaries() {
        assert_eq!(gps_l5_i_neumann_hoffman_code(), &[1, 1, 1, 1, -1, -1, 1, -1, 1, -1]);
        assert_eq!(gps_l5_i_nh_chip(0), 1);
        assert_eq!(gps_l5_i_nh_chip(4), -1);
        assert_eq!(gps_l5_i_symbol_epoch(0), 0);
        assert_eq!(gps_l5_i_symbol_epoch(9), 9);
        assert_eq!(gps_l5_i_symbol_epoch(10), 0);
        assert_eq!(gps_l5_i_data_symbol_index(0), 0);
        assert_eq!(gps_l5_i_data_symbol_index(9), 0);
        assert_eq!(gps_l5_i_data_symbol_index(10), 1);
        assert_eq!(gps_l5_i_epoch_symbol(&[1, -1], 0), Ok(1));
        assert_eq!(gps_l5_i_epoch_symbol(&[1, -1], 4), Ok(-1));
        assert_eq!(gps_l5_i_epoch_symbol(&[1, -1], 10), Ok(-1));
    }

    #[test]
    fn chip_rate_sampling_matches_generated_chip_stream() {
        let chips = generate_gps_l5_i_chips(12, 0, 32, &[1]).expect("generated chips");
        let samples =
            sample_gps_l5_i(12, GPS_L5_CODE_RATE_HZ, 0.0, 32, &[1]).expect("chip-rate samples");
        let primary = sample_gps_l5_i_primary_code(12, GPS_L5_CODE_RATE_HZ, 0.0, 32)
            .expect("primary-code samples");
        let expected = chips
            .into_iter()
            .map(|chip| chip as f32 * std::f32::consts::FRAC_1_SQRT_2)
            .collect::<Vec<f32>>();

        assert_eq!(samples, expected);
        assert_eq!(primary.len(), 32);
    }

    #[test]
    fn modulation_rejects_empty_or_non_bipolar_symbols() {
        assert_eq!(
            generate_gps_l5_i_chips(12, 0, 1, &[]),
            Err(SignalError::EmptyNavigationSymbolStream)
        );
        assert_eq!(
            sample_gps_l5_i(12, GPS_L5_CODE_RATE_HZ, 0.0, 1, &[0]),
            Err(SignalError::InvalidNavigationSymbol(0))
        );
    }

    #[test]
    fn l5_i_symbol_boundaries_repeat_cyclic_data() {
        let chips = generate_gps_l5_i_chips(7, 0, GPS_L5I_SYMBOL_CHIPS * 2, &[1, -1])
            .expect("generated L5-I chips");
        assert_eq!(chips[0], 1);
        assert_eq!(chips[GPS_L5I_SYMBOL_CHIPS], -1);
    }
}
