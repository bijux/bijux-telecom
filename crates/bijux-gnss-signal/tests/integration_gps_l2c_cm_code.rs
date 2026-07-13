#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cm_code, gps_l2c_cm_code_assignment, sample_gps_l2c_cm_code,
    GPS_L2C_CM_CODE_CHIPS, GPS_L2C_CM_CODE_RATE_HZ,
};

#[test]
fn public_api_generates_full_gps_l2c_cm_periods() {
    let code = generate_gps_l2c_cm_code(63).expect("valid L2C CM PRN");

    assert_eq!(code.len(), GPS_L2C_CM_CODE_CHIPS);
    assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
}

#[test]
fn public_api_exposes_supplemental_gps_l2c_cm_prn_assignments() {
    let assignment = gps_l2c_cm_code_assignment(210).expect("supplemental L2C CM PRN");

    assert_eq!(assignment.prn, 210);
    assert_eq!(assignment.initial_state_octal, 0o140633660);
    assert_eq!(assignment.end_state_octal, 0o465052527);
}

#[test]
fn public_api_samples_gps_l2c_cm_at_chip_rate() {
    let samples =
        sample_gps_l2c_cm_code(38, GPS_L2C_CM_CODE_RATE_HZ, 0.0, 6).expect("valid L2C CM PRN");
    let code = generate_gps_l2c_cm_code(38).expect("valid L2C CM PRN");

    assert_eq!(samples, code.iter().take(6).map(|chip| f32::from(*chip)).collect::<Vec<_>>());
}
