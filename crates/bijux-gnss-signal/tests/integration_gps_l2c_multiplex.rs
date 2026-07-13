#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code, generate_gps_l2c_cm_code, generate_gps_l2c_time_multiplexed_chips,
    gps_l2c_time_multiplexed_component, sample_gps_l2c_time_multiplexed,
    GpsL2cTimeMultiplexedComponent, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
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
