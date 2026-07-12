#![allow(missing_docs)]

use bijux_gnss_infra::api::apply_sweep_value;
use bijux_gnss_receiver::api::{NavigationWeightingMode, ReceiverConfig};

#[test]
fn apply_sweep_value_accepts_elevation_cn0_weighting_mode() {
    let mut profile = ReceiverConfig::default();

    apply_sweep_value(&mut profile, "navigation.weighting.mode", "elevation_cn0")
        .expect("combined weighting mode should parse");

    assert_eq!(profile.navigation.weighting.mode, NavigationWeightingMode::ElevationCn0);
}
