#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code_range, generate_gps_l2c_cm_code_chips,
    generate_gps_l2c_time_multiplexed_chips, gps_l2c_time_multiplexed_component,
    sample_gps_l2c_time_multiplexed, GpsL2cTimeMultiplexedComponent,
    GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ,
};

#[test]
fn public_api_reports_l2c_transmitted_chip_ownership() {
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
    assert_eq!(
        gps_l2c_time_multiplexed_component(3.0).expect("valid phase"),
        GpsL2cTimeMultiplexedComponent::ClPilot
    );
}

#[test]
fn public_api_interleaves_cm_and_cl_chips_at_chip_rate() {
    let cm = generate_gps_l2c_cm_code_chips(38, 4).expect("valid L2C CM PRN");
    let cl = generate_gps_l2c_cl_code_range(38, 0, 4).expect("valid L2C CL PRN");
    let expected = vec![cm[0], cl[0], cm[1], cl[1], cm[2], cl[2], cm[3], cl[3]];

    let generated =
        generate_gps_l2c_time_multiplexed_chips(38, 0, 8, &[1]).expect("valid multiplexed chips");
    let sampled =
        sample_gps_l2c_time_multiplexed(38, GPS_L2C_TIME_MULTIPLEXED_CODE_RATE_HZ, 0.0, 8, &[1])
            .expect("chip-rate multiplexed samples")
            .into_iter()
            .map(|sample| sample as i8)
            .collect::<Vec<_>>();

    assert_eq!(generated, expected);
    assert_eq!(sampled, expected);
}
