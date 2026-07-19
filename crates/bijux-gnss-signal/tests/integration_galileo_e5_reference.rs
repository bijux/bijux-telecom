mod support;

use bijux_gnss_signal::api::{
    galileo_e5a_i_code_assignment, galileo_e5a_i_secondary_code, galileo_e5a_q_code_assignment,
    galileo_e5a_q_secondary_code, galileo_e5b_i_code_assignment, galileo_e5b_i_secondary_code,
    galileo_e5b_q_code_assignment, galileo_e5b_q_secondary_code, generate_galileo_e5a_i_code,
    generate_galileo_e5a_q_code, generate_galileo_e5b_i_code, generate_galileo_e5b_q_code,
};

use support::galileo_e5_reference::{
    assert_primary_code_matches_reference, assert_secondary_code_matches_reference,
    load_reference_catalog,
};
use support::period_reference::{assert_period_repetition, logical_bits_from_bipolar};

#[test]
fn galileo_e5a_i_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let assignment = galileo_e5a_i_code_assignment(prn).expect("published Galileo E5a-I PRN");
        let reference = catalog.primary_code_reference("E5AI", prn);
        let code = generate_galileo_e5a_i_code(prn).expect("published Galileo E5a-I PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5a-I code");

        assert_eq!(assignment.register_2_start_octal, reference.register_2_start_octal);
        assert_primary_code_matches_reference(&catalog, "E5AI", prn, &logical_bits);
        assert_period_repetition(
            &code,
            catalog.primary_code_length,
            &format!("Galileo E5a-I PRN {prn}"),
        );
    }
}

#[test]
fn galileo_e5a_q_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let assignment = galileo_e5a_q_code_assignment(prn).expect("published Galileo E5a-Q PRN");
        let reference = catalog.primary_code_reference("E5AQ", prn);
        let code = generate_galileo_e5a_q_code(prn).expect("published Galileo E5a-Q PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5a-Q code");

        assert_eq!(assignment.register_2_start_octal, reference.register_2_start_octal);
        assert_primary_code_matches_reference(&catalog, "E5AQ", prn, &logical_bits);
        assert_period_repetition(
            &code,
            catalog.primary_code_length,
            &format!("Galileo E5a-Q PRN {prn}"),
        );
    }
}

#[test]
fn galileo_e5b_i_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let assignment = galileo_e5b_i_code_assignment(prn).expect("published Galileo E5b-I PRN");
        let reference = catalog.primary_code_reference("E5BI", prn);
        let code = generate_galileo_e5b_i_code(prn).expect("published Galileo E5b-I PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5b-I code");

        assert_eq!(assignment.register_2_start_octal, reference.register_2_start_octal);
        assert_primary_code_matches_reference(&catalog, "E5BI", prn, &logical_bits);
        assert_period_repetition(
            &code,
            catalog.primary_code_length,
            &format!("Galileo E5b-I PRN {prn}"),
        );
    }
}

#[test]
fn galileo_e5b_q_primary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let assignment = galileo_e5b_q_code_assignment(prn).expect("published Galileo E5b-Q PRN");
        let reference = catalog.primary_code_reference("E5BQ", prn);
        let code = generate_galileo_e5b_q_code(prn).expect("published Galileo E5b-Q PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5b-Q code");

        assert_eq!(assignment.register_2_start_octal, reference.register_2_start_octal);
        assert_primary_code_matches_reference(&catalog, "E5BQ", prn, &logical_bits);
        assert_period_repetition(
            &code,
            catalog.primary_code_length,
            &format!("Galileo E5b-Q PRN {prn}"),
        );
    }
}

#[test]
fn galileo_e5a_i_secondary_code_matches_immutable_reference_catalog() {
    let catalog = load_reference_catalog();
    let code = galileo_e5a_i_secondary_code();
    let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5a-I secondary code");

    assert_secondary_code_matches_reference(&catalog, "E5AI", None, &logical_bits);
    assert_period_repetition(&code, code.len(), "Galileo E5a-I secondary code");
}

#[test]
fn galileo_e5a_q_secondary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let code = galileo_e5a_q_secondary_code(prn).expect("published Galileo E5a-Q PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5a-Q secondary code");

        assert_secondary_code_matches_reference(&catalog, "E5AQ", Some(prn), &logical_bits);
        assert_period_repetition(&code, code.len(), &format!("Galileo E5a-Q secondary PRN {prn}"));
    }
}

#[test]
fn galileo_e5b_i_secondary_code_matches_immutable_reference_catalog() {
    let catalog = load_reference_catalog();
    let code = galileo_e5b_i_secondary_code();
    let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5b-I secondary code");

    assert_secondary_code_matches_reference(&catalog, "E5BI", None, &logical_bits);
    assert_period_repetition(&code, code.len(), "Galileo E5b-I secondary code");
}

#[test]
fn galileo_e5b_q_secondary_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let code = galileo_e5b_q_secondary_code(prn).expect("published Galileo E5b-Q PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "Galileo E5b-Q secondary code");

        assert_secondary_code_matches_reference(&catalog, "E5BQ", Some(prn), &logical_bits);
        assert_period_repetition(&code, code.len(), &format!("Galileo E5b-Q secondary PRN {prn}"));
    }
}
