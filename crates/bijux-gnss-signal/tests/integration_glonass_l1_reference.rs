mod support;

use bijux_gnss_signal::api::generate_glonass_l1_st_code;

use support::period_reference::{
    assert_code_period_reference, assert_period_repetition, logical_bits_from_bipolar,
    ReferenceWindow,
};

const GLONASS_L1_ST_SHA256: &str =
    "f9ced31ae34538d0a7040df20f674d55aafde247cbc078680e3fe40a5d50420e";

#[test]
fn glonass_l1_standard_precision_code_matches_immutable_reference_vectors() {
    let code = generate_glonass_l1_st_code();
    let logical_bits = logical_bits_from_bipolar(&code, "GLONASS L1 ST code");
    let windows = vec![
        ReferenceWindow { start: 0, bits: "11111110000011110111110001011100".to_owned() },
        ReferenceWindow { start: 239, bits: "00100110101001100110000000110001".to_owned() },
        ReferenceWindow {
            start: code.len() - 32,
            bits: "11011011001101000011101111000011".to_owned(),
        },
        ReferenceWindow {
            start: code.len() - 16,
            bits: "00111011110000111111111000001111".to_owned(),
        },
    ];

    assert_code_period_reference(
        &logical_bits,
        code.len(),
        GLONASS_L1_ST_SHA256,
        &windows,
        "GLONASS L1 ST code",
    );
    assert_period_repetition(&code, code.len(), "GLONASS L1 ST code");
}
