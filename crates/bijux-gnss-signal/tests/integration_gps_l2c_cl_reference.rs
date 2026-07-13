mod support;

use bijux_gnss_signal::api::{
    generate_gps_l2c_cl_code, generate_gps_l2c_cl_code_range, gps_l2c_cl_code_assignment,
    gps_l2c_cl_code_assignments, sample_gps_l2c_cl_code, GPS_L2C_CL_CODE_CHIPS,
    GPS_L2C_CL_CODE_RATE_HZ,
};

use support::gps_l2c_cl_reference::{assert_code_matches_reference, load_reference_catalog};

#[test]
fn gps_l2c_cl_public_assignments_match_reference_catalog() {
    let catalog = load_reference_catalog();
    let assignments = gps_l2c_cl_code_assignments();

    assert_eq!(assignments.len(), catalog.published_prn_count);

    for reference in &catalog.code {
        let assignment = gps_l2c_cl_code_assignment(reference.prn).expect("published GPS L2C CL PRN");
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
fn gps_l2c_cl_public_codes_match_reference_catalog() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let code = generate_gps_l2c_cl_code(reference.prn).expect("published GPS L2C CL PRN");
        assert_code_matches_reference(&catalog, reference.prn, &code);
    }
}

#[test]
fn gps_l2c_cl_public_ranges_match_reference_interior_windows() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let range = generate_gps_l2c_cl_code_range(
            reference.prn,
            catalog.interior_window_start,
            catalog.interior_window_length,
        )
        .expect("published GPS L2C CL PRN");
        let expected = reference
            .interior_window_bits
            .chars()
            .map(|bit| if bit == '1' { -1 } else { 1 })
            .collect::<Vec<i8>>();

        assert_eq!(range, expected, "interior range mismatch for PRN {}", reference.prn);
    }
}

#[test]
fn gps_l2c_cl_public_ranges_wrap_against_reference_edges() {
    let catalog = load_reference_catalog();
    let wrap_tail = 32;
    let wrap_head = 32;
    let wrap_start = GPS_L2C_CL_CODE_CHIPS - wrap_tail;

    for reference in &catalog.code {
        let wrapped = generate_gps_l2c_cl_code_range(reference.prn, wrap_start, wrap_tail + wrap_head)
            .expect("published GPS L2C CL PRN");
        let expected = reference
            .bit_suffix
            .chars()
            .skip(reference.bit_suffix.len() - wrap_tail)
            .chain(reference.bit_prefix.chars().take(wrap_head))
            .map(|bit| if bit == '1' { -1 } else { 1 })
            .collect::<Vec<i8>>();

        assert_eq!(wrapped, expected, "wrapped range mismatch for PRN {}", reference.prn);
    }
}

#[test]
fn gps_l2c_cl_public_samples_match_reference_prefixes() {
    let catalog = load_reference_catalog();

    for prn in [1_u8, 38, 159, 210] {
        let reference = catalog.code_reference(prn);
        let sample_count = 16;
        let samples = sample_gps_l2c_cl_code(prn, GPS_L2C_CL_CODE_RATE_HZ, 0.0, sample_count)
            .expect("published GPS L2C CL PRN");
        let expected = reference
            .bit_prefix
            .chars()
            .take(sample_count)
            .map(|bit| if bit == '1' { -1.0 } else { 1.0 })
            .collect::<Vec<f32>>();

        assert_eq!(samples, expected, "sample mismatch for PRN {prn}");
    }
}
