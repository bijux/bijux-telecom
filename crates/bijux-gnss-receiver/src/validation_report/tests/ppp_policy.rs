use super::*;

#[test]
fn build_ppp_config_maps_precise_product_policy() {
    let mut profile = ReceiverConfig::default();
    profile.navigation.ppp.precise_product_missing_satellite_action =
        "bridge_with_broadcast".to_string();
    profile.navigation.ppp.precise_product_out_of_coverage_action =
        "inflate_satellite_state".to_string();
    profile.navigation.ppp.precise_product_insufficient_support_action =
        "reset_satellite_state".to_string();
    profile.navigation.ppp.precise_product_orbit_gap_action = "inflate_satellite_state".to_string();
    profile.navigation.ppp.precise_product_orbit_flag_action = "refuse_satellite".to_string();
    profile.navigation.ppp.precise_product_clock_gap_action = "reset_satellite_state".to_string();
    profile.navigation.ppp.precise_product_clock_jump_action = "refuse_satellite".to_string();
    profile.navigation.ppp.precise_product_state_inflation = 40.0;

    let config = build_ppp_config(&profile);

    assert_eq!(
        config.precise_product_policy.missing_satellite_action,
        PppPreciseProductAction::BridgeWithBroadcast
    );
    assert_eq!(
        config.precise_product_policy.out_of_coverage_action,
        PppPreciseProductAction::InflateSatelliteState
    );
    assert_eq!(
        config.precise_product_policy.insufficient_support_action,
        PppPreciseProductAction::ResetSatelliteState
    );
    assert_eq!(
        config.precise_product_policy.orbit_gap_action,
        PppPreciseProductAction::InflateSatelliteState
    );
    assert_eq!(
        config.precise_product_policy.orbit_flag_action,
        PppPreciseProductAction::RefuseSatellite
    );
    assert_eq!(
        config.precise_product_policy.clock_gap_action,
        PppPreciseProductAction::ResetSatelliteState
    );
    assert_eq!(
        config.precise_product_policy.clock_jump_action,
        PppPreciseProductAction::RefuseSatellite
    );
    assert_eq!(config.precise_product_policy.satellite_state_inflation, 40.0);
}
