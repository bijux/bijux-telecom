#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{check_dual_frequency_observations, validate_obs_epochs, SignalBand};
use bijux_gnss_nav::api::parse_rinex_gps_observation_dataset;

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

#[test]
fn public_rinex_2_import_exposes_valid_l1_l2_observation_channels() {
    let dataset = parse_rinex_gps_observation_dataset(&fixture("georinex_14601736_20180622.obs"))
        .expect("parse public RINEX 2 observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::L1, SignalBand::L2]
    );
    validate_obs_epochs(&dataset.epochs).expect("public RINEX 2 epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert!(report.complete_pairs > 0);
    assert_eq!(report.complete_pairs, report.l1_l2_pairs);
}

#[test]
fn public_rinex_3_import_exposes_valid_l1_l2_observation_channels() {
    let dataset = parse_rinex_gps_observation_dataset(&fixture("glab_gage_20100305.obs"))
        .expect("parse public RINEX 3 observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::L1, SignalBand::L2]
    );
    validate_obs_epochs(&dataset.epochs).expect("public RINEX 3 epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert!(report.complete_pairs > 0);
    assert_eq!(report.complete_pairs, report.l1_l2_pairs);
}
