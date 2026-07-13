#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code, generate_gps_l2c_cl_code_range, gps_l2c_cl_code_assignment,
    sample_gps_l2c_cl_code, GPS_L2C_CL_CODE_CHIPS, GPS_L2C_CL_CODE_RATE_HZ,
};

#[test]
fn public_api_generates_full_gps_l2c_cl_periods() {
    let code = generate_gps_l2c_cl_code(63).expect("valid L2C CL PRN");

    assert_eq!(code.len(), GPS_L2C_CL_CODE_CHIPS);
    assert!(code.iter().all(|chip| *chip == -1 || *chip == 1));
}

#[test]
fn public_api_exposes_supplemental_gps_l2c_cl_prn_assignments() {
    let assignment = gps_l2c_cl_code_assignment(210).expect("supplemental L2C CL PRN");

    assert_eq!(assignment.prn, 210);
    assert_eq!(assignment.initial_state_octal, 0o513322453);
    assert_eq!(assignment.end_state_octal, 0o113765506);
}

#[test]
fn public_api_generates_wrapped_gps_l2c_cl_ranges() {
    let wrapped = generate_gps_l2c_cl_code_range(38, GPS_L2C_CL_CODE_CHIPS - 5, 12)
        .expect("valid L2C CL PRN");
    let full = generate_gps_l2c_cl_code(38).expect("valid L2C CL PRN");

    let mut expected = full[GPS_L2C_CL_CODE_CHIPS - 5..].to_vec();
    expected.extend_from_slice(&full[..7]);

    assert_eq!(wrapped, expected);
}

#[test]
fn public_api_samples_gps_l2c_cl_at_chip_rate() {
    let samples =
        sample_gps_l2c_cl_code(38, GPS_L2C_CL_CODE_RATE_HZ, 0.0, 6).expect("valid L2C CL PRN");
    let code = generate_gps_l2c_cl_code_range(38, 0, 6).expect("valid L2C CL PRN");

    assert_eq!(samples, code.iter().map(|chip| f32::from(*chip)).collect::<Vec<_>>());
}
