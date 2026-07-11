mod support;

use bijux_gnss_signal::api::{generate_galileo_e1c_code, GalileoE1Channel};

use support::galileo_e1_reference::{
    assert_primary_code_matches_reference, load_reference_catalog,
};

#[test]
fn galileo_e1c_primary_codes_match_independent_reference_catalog() {
    let catalog = load_reference_catalog();

    for prn in 1..=50 {
        let code = generate_galileo_e1c_code(prn).expect("published Galileo E1-C PRN");
        assert_primary_code_matches_reference(&catalog, GalileoE1Channel::E1C, prn, &code);
    }
}
