mod support;

use bijux_gnss_signal::api::{
    beidou_d1_neumann_hoffman_code, generate_beidou_b1i_code, generate_beidou_b2i_code,
};

use support::beidou_b1i_reference::{assert_code_matches_reference, load_reference_catalog};
use support::period_reference::{
    assert_code_period_reference, assert_period_repetition, logical_bits_from_bipolar, ReferenceWindow,
};

const BEIDOU_D1_SECONDARY_BITS: &str = "00000100110101001110";
const BEIDOU_D1_SECONDARY_SHA256: &str =
    "52b2d75177a955ef5c35b781ad325f4ed5ac117b7ac66b63054707809245f056";

#[test]
fn beidou_b1i_codes_match_immutable_reference_catalog() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let code = generate_beidou_b1i_code(reference.prn).expect("valid BeiDou B1I PRN");
        let logical_bits = logical_bits_from_bipolar(&code, "BeiDou B1I code");

        assert_code_matches_reference(&catalog, reference.prn, &logical_bits);
        assert_period_repetition(&code, catalog.code_length, &format!("BeiDou B1I PRN {}", reference.prn));
    }
}

#[test]
fn beidou_b2i_codes_match_shared_open_service_reference_catalog() {
    let catalog = load_reference_catalog();

    for reference in &catalog.code {
        let b1 = generate_beidou_b1i_code(reference.prn).expect("valid BeiDou B1I PRN");
        let b2 = generate_beidou_b2i_code(reference.prn).expect("valid BeiDou B2I PRN");
        let logical_bits = logical_bits_from_bipolar(&b2, "BeiDou B2I code");

        assert_eq!(b2, b1, "BeiDou B2I PRN {} must share the B1I open-service family", reference.prn);
        assert_code_matches_reference(&catalog, reference.prn, &logical_bits);
        assert_period_repetition(&b2, catalog.code_length, &format!("BeiDou B2I PRN {}", reference.prn));
    }
}

#[test]
fn beidou_d1_secondary_overlay_matches_published_sequence() {
    let code = beidou_d1_neumann_hoffman_code();
    let logical_bits = logical_bits_from_bipolar(code, "BeiDou D1 NH overlay");
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

    assert_eq!(logical_bits, BEIDOU_D1_SECONDARY_BITS);
    assert_code_period_reference(
        &logical_bits,
        code.len(),
        BEIDOU_D1_SECONDARY_SHA256,
        &windows,
        "BeiDou D1 NH overlay",
    );
    assert_period_repetition(code, code.len(), "BeiDou D1 NH overlay");
}
