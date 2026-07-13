#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code_range, generate_gps_l2c_cm_code_chips, sample_gps_l2c_time_multiplexed,
    SignalError, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS,
};

#[test]
fn public_api_applies_navigation_symbols_only_to_cm_slots() {
    let cm = generate_gps_l2c_cm_code_chips(38, 1).expect("valid L2C CM PRN");
    let cl = generate_gps_l2c_cl_code_range(38, 0, 1).expect("valid L2C CL PRN");

    let first_symbol = sample_gps_l2c_time_multiplexed(
        38,
        GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
        0.0,
        2,
        &[1, -1],
    )
    .expect("first CNAV symbol");
    let second_symbol = sample_gps_l2c_time_multiplexed(
        38,
        GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
        GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS as f64,
        2,
        &[1, -1],
    )
    .expect("second CNAV symbol");

    assert_eq!(first_symbol, vec![cm[0] as f32, cl[0] as f32]);
    assert_eq!(second_symbol, vec![-(cm[0] as f32), cl[0] as f32]);
}

#[test]
fn public_api_rejects_empty_or_non_bipolar_navigation_symbols() {
    assert_eq!(
        sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 1, &[]),
        Err(SignalError::EmptyNavigationSymbolStream)
    );
    assert_eq!(
        sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 1, &[0]),
        Err(SignalError::InvalidNavigationSymbol(0))
    );
}
