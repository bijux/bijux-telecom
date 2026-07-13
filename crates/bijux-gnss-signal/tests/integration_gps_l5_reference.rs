mod support;

use bijux_gnss_signal::api::{
    generate_gps_l5_i_code, generate_gps_l5_q_code, gps_l5_i_code_assignment, gps_l5_i_neumann_hoffman_code,
    gps_l5_q_code_assignment, gps_l5_q_neumann_hoffman_code,
};

use support::gps_l5_reference::{assert_primary_code_matches_reference, load_reference_catalog};
use support::period_reference::{
    assert_code_period_reference, assert_period_repetition, logical_bits_from_bipolar, ReferenceWindow,
};

#[test]
fn gps_l5_i_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=63 {
        let assignment = gps_l5_i_code_assignment(prn).expect("published GPS L5-I PRN");
        let reference = catalog.primary_code_reference("I", prn);
        let code = generate_gps_l5_i_code(prn).expect("published GPS L5-I PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "GPS L5-I code");

        assert_eq!(assignment.xb_advance_chips, reference.xb_advance_chips);
        assert_eq!(assignment.xb_initial_state_bits, reference.xb_initial_state_bits);
        assert_primary_code_matches_reference(&catalog, "I", prn, &logical_bits);
        assert_period_repetition(&code, catalog.code_length, &format!("GPS L5-I PRN {prn}"));
    }
}

#[test]
fn gps_l5_q_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=63 {
        let assignment = gps_l5_q_code_assignment(prn).expect("published GPS L5-Q PRN");
        let reference = catalog.primary_code_reference("Q", prn);
        let code = generate_gps_l5_q_code(prn).expect("published GPS L5-Q PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "GPS L5-Q code");

        assert_eq!(assignment.xb_advance_chips, reference.xb_advance_chips);
        assert_eq!(assignment.xb_initial_state_bits, reference.xb_initial_state_bits);
        assert_primary_code_matches_reference(&catalog, "Q", prn, &logical_bits);
        assert_period_repetition(&code, catalog.code_length, &format!("GPS L5-Q PRN {prn}"));
    }
}

#[test]
fn gps_l5_i_secondary_overlay_matches_published_sequence() {
    let catalog = load_reference_catalog();
    let code = gps_l5_i_neumann_hoffman_code();
    let logical_bits = logical_bits_from_bipolar(code, "GPS L5-I NH overlay");
    let windows = vec![
        ReferenceWindow {
            start: 0,
            bits: "00001101".to_owned(),
        },
        ReferenceWindow {
            start: 1,
            bits: "00011010".to_owned(),
        },
        ReferenceWindow {
            start: code.len() - 8,
            bits: "00110101".to_owned(),
        },
        ReferenceWindow {
            start: code.len() - 4,
            bits: "01010000".to_owned(),
        },
    ];

    assert_eq!(logical_bits, catalog.l5i_nh_bits);
    assert_code_period_reference(
        &logical_bits,
        code.len(),
        &catalog.l5i_nh_sha256,
        &windows,
        "GPS L5-I NH overlay",
    );
    assert_period_repetition(code, code.len(), "GPS L5-I NH overlay");
}

#[test]
fn gps_l5_q_secondary_overlay_matches_published_sequence() {
    let catalog = load_reference_catalog();
    let code = gps_l5_q_neumann_hoffman_code();
    let logical_bits = logical_bits_from_bipolar(code, "GPS L5-Q NH overlay");
    let windows = vec![
        ReferenceWindow {
            start: 0,
            bits: "00000100".to_owned(),
        },
        ReferenceWindow {
            start: 6,
            bits: "00110101".to_owned(),
        },
        ReferenceWindow {
            start: code.len() - 8,
            bits: "01001110".to_owned(),
        },
        ReferenceWindow {
            start: code.len() - 4,
            bits: "11100000".to_owned(),
        },
    ];

    assert_eq!(logical_bits, catalog.l5q_nh_bits);
    assert_code_period_reference(
        &logical_bits,
        code.len(),
        &catalog.l5q_nh_sha256,
        &windows,
        "GPS L5-Q NH overlay",
    );
    assert_period_repetition(code, code.len(), "GPS L5-Q NH overlay");
}
