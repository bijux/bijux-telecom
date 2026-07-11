use bijux_gnss_signal::api::{
    galileo_e1c_secondary_code, generate_galileo_e1b_code, generate_galileo_e1c_code,
    GALILEO_E1_PRIMARY_CODE_CHIPS, GALILEO_E1_SECONDARY_CODE_CHIPS,
};

#[test]
fn galileo_e1_primary_codes_are_prn_specific() {
    let prn_one = generate_galileo_e1b_code(1).expect("valid PRN 1");
    let prn_two = generate_galileo_e1b_code(2).expect("valid PRN 2");

    assert_eq!(prn_one.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    assert_eq!(prn_two.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    assert_ne!(prn_one, prn_two);
}

#[test]
fn galileo_e1_data_and_pilot_primary_codes_differ() {
    let data = generate_galileo_e1b_code(7).expect("valid E1-B PRN");
    let pilot = generate_galileo_e1c_code(7).expect("valid E1-C PRN");

    assert_eq!(data.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    assert_eq!(pilot.len(), GALILEO_E1_PRIMARY_CODE_CHIPS);
    assert_ne!(data, pilot);
}

#[test]
fn galileo_e1c_secondary_code_is_shared_across_prns() {
    let secondary = galileo_e1c_secondary_code();

    assert_eq!(secondary.len(), GALILEO_E1_SECONDARY_CODE_CHIPS);
    assert!(secondary.iter().all(|chip| *chip == -1 || *chip == 1));
}
