use bijux_gnss_signal::api::{
    galileo_e1c_secondary_code, generate_galileo_e1b_code, generate_galileo_e1c_code,
    sample_galileo_e1_boc11_code, sample_galileo_e1_cboc, GalileoE1Channel,
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

#[test]
fn galileo_e1_boc11_sampling_preserves_chip_transition_shape() {
    let samples = sample_galileo_e1_boc11_code(1, GalileoE1Channel::E1B, 4_092_000.0, 0.0, 8)
        .expect("valid E1-B BOC(1,1) sampling");

    assert_eq!(samples.len(), 8);
    assert_eq!(samples[0], samples[1]);
    assert_eq!(samples[2], samples[3]);
    assert_eq!(samples[0], -samples[2]);
}

#[test]
fn galileo_e1_cboc_sampling_emits_nonzero_samples() {
    let samples =
        sample_galileo_e1_cboc(11, 4_092_000.0, 0.0, 16, 0, 1).expect("valid E1 CBOC sampling");

    assert_eq!(samples.len(), 16);
    assert!(samples.iter().any(|sample| sample.abs() > f32::EPSILON));
}
