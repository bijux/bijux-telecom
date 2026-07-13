//! GPS L2C CM/CL time-division multiplexing helpers.

use super::gps_l2c_cl::{generate_gps_l2c_cl_code, GPS_L2C_CL_CODE_CHIPS};
use super::gps_l2c_cm::{generate_gps_l2c_cm_code, GPS_L2C_CM_CODE_CHIPS};
use crate::dsp::signal::code_value_at_phase;
use crate::error::SignalError;

/// GPS L2C transmitted chip rate after CM/CL multiplexing.
pub const GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ: f64 = 1_023_000.0;
/// GPS L2C transmitted period in multiplexed chips for one full CL cycle.
pub const GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS: usize = GPS_L2C_CL_CODE_CHIPS * 2;
/// GPS L2C transmitted chips covered by one 20 ms CNAV symbol.
pub const GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS: usize = GPS_L2C_CM_CODE_CHIPS * 2;

/// One transmitted L2C chip slot in the alternating CM/CL stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsL2cTimeMultiplexedComponent {
    /// Data-bearing CM chip slot.
    CmData,
    /// Pilot CL chip slot.
    ClPilot,
}

/// Return which L2C component occupies a transmitted chip slot.
pub fn gps_l2c_time_multiplexed_component(
    chip_phase: f64,
) -> Result<GpsL2cTimeMultiplexedComponent, SignalError> {
    if !chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }
    let transmitted_chip_index =
        chip_phase.rem_euclid(GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS as f64).floor() as usize;
    if transmitted_chip_index % 2 == 0 {
        Ok(GpsL2cTimeMultiplexedComponent::CmData)
    } else {
        Ok(GpsL2cTimeMultiplexedComponent::ClPilot)
    }
}

/// Generate multiplexed GPS L2C transmitted chips for one PRN.
///
/// `data_symbols` supplies the CNAV symbol stream as bipolar values (`-1` or `1`).
/// One symbol spans 20 ms, or `GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS` transmitted chips.
/// When more chips are requested than the symbol stream length covers, symbols repeat cyclically.
pub fn generate_gps_l2c_time_multiplexed_chips(
    prn: u8,
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    let cm_code = generate_gps_l2c_cm_code(prn)?;
    let cl_code = generate_gps_l2c_cl_code(prn)?;
    generate_gps_l2c_time_multiplexed_chips_from_components(
        &cm_code,
        &cl_code,
        start_chip,
        chip_count,
        data_symbols,
    )
}

/// Generate multiplexed L2C chips from explicit CM and CL component codes.
pub fn generate_gps_l2c_time_multiplexed_chips_from_components(
    cm_code: &[i8],
    cl_code: &[i8],
    start_chip: usize,
    chip_count: usize,
    data_symbols: &[i8],
) -> Result<Vec<i8>, SignalError> {
    validate_navigation_symbol_stream(data_symbols)?;

    let mut chips = Vec::with_capacity(chip_count);
    for offset in 0..chip_count {
        let chip_phase = (start_chip + offset) as f64;
        chips.push(gps_l2c_time_multiplexed_value(cm_code, cl_code, chip_phase, data_symbols)? as i8);
    }
    Ok(chips)
}

/// Sample the multiplexed GPS L2C transmitted chip stream at an arbitrary sample rate.
pub fn sample_gps_l2c_time_multiplexed(
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

    let cm_code = generate_gps_l2c_cm_code(prn)?;
    let cl_code = generate_gps_l2c_cl_code(prn)?;
    sample_gps_l2c_time_multiplexed_from_components(
        &cm_code,
        &cl_code,
        sample_rate_hz,
        start_chip_phase,
        sample_count,
        data_symbols,
    )
}

/// Sample the multiplexed L2C stream from explicit CM and CL component codes.
pub fn sample_gps_l2c_time_multiplexed_from_components(
    cm_code: &[i8],
    cl_code: &[i8],
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

    let chips_per_sample = GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ / sample_rate_hz;
    let mut samples = Vec::with_capacity(sample_count);
    for sample_index in 0..sample_count {
        let chip_phase = start_chip_phase + sample_index as f64 * chips_per_sample;
        samples.push(gps_l2c_time_multiplexed_value(cm_code, cl_code, chip_phase, data_symbols)?);
    }
    Ok(samples)
}

/// Sample the multiplexed L2C value at a transmitted-chip phase.
pub fn gps_l2c_time_multiplexed_value(
    cm_code: &[i8],
    cl_code: &[i8],
    chip_phase: f64,
    data_symbols: &[i8],
) -> Result<f32, SignalError> {
    validate_navigation_symbol_stream(data_symbols)?;
    if !chip_phase.is_finite() {
        return Err(SignalError::InvalidCodePhase);
    }

    let code_length = GPS_L2C_TIME_MULTIPLEXED_CODE_CHIPS as f64;
    let wrapped_chip_phase = chip_phase.rem_euclid(code_length);
    let transmitted_chip_index = wrapped_chip_phase.floor() as usize;
    let transmitted_chip_phase = wrapped_chip_phase.fract();
    let component_chip_phase = (transmitted_chip_index / 2) as f64 + transmitted_chip_phase;

    match gps_l2c_time_multiplexed_component(chip_phase)? {
        GpsL2cTimeMultiplexedComponent::CmData => {
            Ok(code_value_at_phase(cm_code, component_chip_phase)?
                * current_navigation_symbol(data_symbols, transmitted_chip_index)? as f32)
        }
        GpsL2cTimeMultiplexedComponent::ClPilot => {
            code_value_at_phase(cl_code, component_chip_phase)
        }
    }
}

fn current_navigation_symbol(
    data_symbols: &[i8],
    transmitted_chip_index: usize,
) -> Result<i8, SignalError> {
    let symbol_index =
        (transmitted_chip_index / GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS) % data_symbols.len();
    validate_navigation_symbol(data_symbols[symbol_index])
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

#[cfg(test)]
mod tests {
    use super::{
        generate_gps_l2c_time_multiplexed_chips, gps_l2c_time_multiplexed_component,
        gps_l2c_time_multiplexed_value, sample_gps_l2c_time_multiplexed,
        GpsL2cTimeMultiplexedComponent, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
        GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS,
    };
    use crate::codes::gps_l2c_cl::generate_gps_l2c_cl_code;
    use crate::codes::gps_l2c_cm::generate_gps_l2c_cm_code;
    use crate::error::SignalError;

    #[test]
    fn time_multiplexed_component_alternates_between_cm_and_cl_slots() {
        assert_eq!(
            gps_l2c_time_multiplexed_component(0.0).expect("valid phase"),
            GpsL2cTimeMultiplexedComponent::CmData
        );
        assert_eq!(
            gps_l2c_time_multiplexed_component(1.0).expect("valid phase"),
            GpsL2cTimeMultiplexedComponent::ClPilot
        );
        assert_eq!(
            gps_l2c_time_multiplexed_component(2.0).expect("valid phase"),
            GpsL2cTimeMultiplexedComponent::CmData
        );
    }

    #[test]
    fn chip_generation_interleaves_cm_and_cl_streams() {
        let cm = generate_gps_l2c_cm_code(38).expect("valid CM code");
        let cl = generate_gps_l2c_cl_code(38).expect("valid CL code");
        let chips = generate_gps_l2c_time_multiplexed_chips(38, 0, 8, &[1])
            .expect("valid multiplexed chips");

        assert_eq!(chips, vec![cm[0], cl[0], cm[1], cl[1], cm[2], cl[2], cm[3], cl[3]]);
    }

    #[test]
    fn multiplexed_value_applies_navigation_symbols_only_to_cm_slots() {
        let cm = generate_gps_l2c_cm_code(38).expect("valid CM code");
        let cl = generate_gps_l2c_cl_code(38).expect("valid CL code");
        let first_symbol_cm =
            gps_l2c_time_multiplexed_value(&cm, &cl, 0.0, &[1, -1]).expect("first CM slot");
        let first_symbol_cl =
            gps_l2c_time_multiplexed_value(&cm, &cl, 1.0, &[1, -1]).expect("first CL slot");
        let next_symbol_cm = gps_l2c_time_multiplexed_value(
            &cm,
            &cl,
            GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS as f64,
            &[1, -1],
        )
        .expect("second CM symbol slot");
        let next_symbol_cl = gps_l2c_time_multiplexed_value(
            &cm,
            &cl,
            (GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS + 1) as f64,
            &[1, -1],
        )
        .expect("second CL symbol slot");

        assert_eq!(first_symbol_cm, cm[0] as f32);
        assert_eq!(first_symbol_cl, cl[0] as f32);
        assert_eq!(next_symbol_cm, -(cm[0] as f32));
        assert_eq!(next_symbol_cl, cl[0] as f32);
    }

    #[test]
    fn chip_rate_sampling_matches_exact_transmitted_chips() {
        let expected =
            generate_gps_l2c_time_multiplexed_chips(38, 0, 8, &[1, -1]).expect("expected chips");
        let samples = sample_gps_l2c_time_multiplexed(
            38,
            GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
            0.0,
            8,
            &[1, -1],
        )
        .expect("multiplexed samples");

        assert_eq!(samples, expected.into_iter().map(f32::from).collect::<Vec<f32>>());
    }

    #[test]
    fn sampling_rejects_empty_navigation_symbol_streams() {
        assert_eq!(
            sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 1, &[]),
            Err(SignalError::EmptyNavigationSymbolStream)
        );
    }

    #[test]
    fn sampling_rejects_invalid_navigation_symbols() {
        assert_eq!(
            sample_gps_l2c_time_multiplexed(
                38,
                GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
                0.0,
                1,
                &[0]
            ),
            Err(SignalError::InvalidNavigationSymbol(0))
        );
    }
}
