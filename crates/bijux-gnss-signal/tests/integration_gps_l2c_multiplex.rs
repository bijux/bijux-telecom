#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code, generate_gps_l2c_cm_code, generate_gps_l2c_time_multiplexed_chips,
    gps_l2c_time_multiplexed_component, periodic_correlation, sample_gps_l2c_time_multiplexed,
    SignalError,
    GpsL2cTimeMultiplexedComponent, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
    GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS,
};

#[test]
fn public_api_reports_l2c_component_slots() {
    assert_eq!(
        gps_l2c_time_multiplexed_component(0.0).expect("valid L2C phase"),
        GpsL2cTimeMultiplexedComponent::CmData
    );
    assert_eq!(
        gps_l2c_time_multiplexed_component(1.0).expect("valid L2C phase"),
        GpsL2cTimeMultiplexedComponent::ClPilot
    );
    assert_eq!(
        gps_l2c_time_multiplexed_component(2.0).expect("valid L2C phase"),
        GpsL2cTimeMultiplexedComponent::CmData
    );
}

#[test]
fn public_api_interleaves_cm_and_cl_chips_at_transmitted_rate() {
    let cm = generate_gps_l2c_cm_code(38).expect("valid GPS L2C CM PRN");
    let cl = generate_gps_l2c_cl_code(38).expect("valid GPS L2C CL PRN");
    let chips = generate_gps_l2c_time_multiplexed_chips(38, 0, 8, &[1])
        .expect("valid GPS L2C multiplexed chips");
    let samples = sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 8, &[1])
        .expect("valid GPS L2C multiplexed samples");

    let expected = vec![cm[0], cl[0], cm[1], cl[1], cm[2], cl[2], cm[3], cl[3]];

    assert_eq!(chips, expected);
    assert_eq!(
        samples,
        expected.into_iter().map(f32::from).collect::<Vec<f32>>()
    );
}

#[test]
fn public_api_applies_navigation_symbols_only_to_cm_slots() {
    let cm = generate_gps_l2c_cm_code(38).expect("valid GPS L2C CM PRN");
    let cl = generate_gps_l2c_cl_code(38).expect("valid GPS L2C CL PRN");
    let samples = sample_gps_l2c_time_multiplexed(
        38,
        GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
        GPS_L2C_TIME_MULTIPLEXED_SYMBOL_CHIPS as f64,
        4,
        &[1, -1],
    )
    .expect("valid GPS L2C multiplexed samples");

    assert_eq!(samples[0], -(cm[0] as f32));
    assert_eq!(samples[1], cl[10_230] as f32);
    assert_eq!(samples[2], -(cm[1] as f32));
    assert_eq!(samples[3], cl[10_231] as f32);
}

#[test]
fn public_api_multiplexed_replica_has_full_zero_shift_correlation_peak() {
    let chips = generate_gps_l2c_time_multiplexed_chips(38, 0, 20_460, &[1])
        .expect("valid GPS L2C multiplexed chips");
    let correlation = periodic_correlation(&chips, &chips).expect("correlation trace");

    assert_eq!(correlation[0], 20_460);
}

#[test]
fn public_api_shifted_multiplex_replica_loses_the_main_peak() {
    let reference = generate_gps_l2c_time_multiplexed_chips(38, 0, 20_460, &[1])
        .expect("reference GPS L2C multiplexed chips");
    let shifted = generate_gps_l2c_time_multiplexed_chips(38, 1, 20_460, &[1])
        .expect("shifted GPS L2C multiplexed chips");
    let exact = periodic_correlation(&reference, &reference).expect("exact correlation");
    let shifted_correlation =
        periodic_correlation(&reference, &shifted).expect("shifted correlation");

    assert_eq!(exact[0], 20_460);
    assert!(shifted_correlation[0].abs() < exact[0]);
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
