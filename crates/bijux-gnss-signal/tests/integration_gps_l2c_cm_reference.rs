mod support;

use bijux_gnss_signal::api::{
    generate_gps_l2c_cm_code, gps_l2c_cm_code_assignment, gps_l2c_cm_code_assignments,
    sample_gps_l2c_cm_code, GPS_L2C_CM_CODE_RATE_HZ,
};

use support::gps_l2c_cm_reference::{assert_code_matches_reference, load_reference_catalog};
use support::period_reference::assert_period_repetition;

#[test]
fn gps_l2c_cm_public_assignments_match_reference_catalog() {
    let catalog = load_reference_catalog();
    let assignments = gps_l2c_cm_code_assignments();

    assert_eq!(assignments.len(), catalog.published_prn_count);

    for reference in &catalog.code {
        let assignment =
            gps_l2c_cm_code_assignment(reference.prn).expect("published GPS L2C CM PRN");
        assert_eq!(assignment.prn, reference.prn);
        assert_eq!(
            format!("{:09o}", assignment.initial_state_octal),
            reference.initial_state_octal,
            "initial state mismatch for PRN {}",
            reference.prn
        );
        assert_eq!(
            format!("{:09o}", assignment.end_state_octal),
            reference.end_state_octal,
            "end state mismatch for PRN {}",
            reference.prn
        );
    }
}

#[test]
fn gps_l2c_cm_public_codes_match_reference_catalog() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let code = generate_gps_l2c_cm_code(reference.prn).expect("published GPS L2C CM PRN");
        assert_code_matches_reference(&catalog, reference.prn, &code);
        assert_period_repetition(
            &code,
            catalog.chip_length,
            &format!("GPS L2C CM PRN {}", reference.prn),
        );
    }
}

#[test]
fn gps_l2c_cm_public_samples_match_reference_prefixes() {
    let catalog = load_reference_catalog();

    for prn in [1_u8, 38, 159, 210] {
        let reference = catalog.code_reference(prn);
        let sample_count = 16;
        let samples = sample_gps_l2c_cm_code(prn, GPS_L2C_CM_CODE_RATE_HZ, 0.0, sample_count)
            .expect("published GPS L2C CM PRN");
        let expected = reference
            .bit_prefix
            .chars()
            .take(sample_count)
            .map(|bit| if bit == '1' { -1.0 } else { 1.0 })
            .collect::<Vec<f32>>();

        assert_eq!(samples, expected, "sample mismatch for PRN {prn}");
    }
}
