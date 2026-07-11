mod support;

use bijux_gnss_signal::api::galileo_e1c_secondary_code;

use support::galileo_e1_reference::{
    assert_secondary_code_matches_reference, load_reference_catalog,
};

#[test]
fn galileo_e1c_secondary_code_matches_reference_catalog() {
    let catalog = load_reference_catalog();
    let secondary = galileo_e1c_secondary_code();
    assert_secondary_code_matches_reference(&catalog, &secondary);
}
