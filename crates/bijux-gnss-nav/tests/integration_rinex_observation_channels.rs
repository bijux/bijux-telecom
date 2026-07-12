#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{
    check_dual_frequency_observations, validate_obs_epochs, SignalBand, SignalCode,
};
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
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::Py]
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
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::Py]
    );
    validate_obs_epochs(&dataset.epochs).expect("public RINEX 3 epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert!(report.complete_pairs > 0);
    assert_eq!(report.complete_pairs, report.l1_l2_pairs);
}

#[test]
fn synthetic_rinex_3_import_exposes_l2c_observation_channel() {
    let data = [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "gps-station", "MARKER NAME"),
        format!("{:<60}{}", "G    4 C1C L1C C2L L2L", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
            "G01", 20345678.123, 123456.250, 20345680.750, 123450.500
        ),
    ]
    .join("\n");
    let dataset =
        parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 L2C data");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::L2C]
    );
    validate_obs_epochs(&dataset.epochs).expect("synthetic RINEX 3 L2C epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert_eq!(report.complete_pairs, 1);
    assert_eq!(report.l1_l2_pairs, 1);
}

#[test]
fn public_rinex_2_import_exposes_valid_l1_l2_l5_observation_channels() {
    let dataset = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public RINEX 2 L5 observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::L1, SignalBand::L2, SignalBand::L5]
    );
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::Py, SignalCode::Unknown]
    );
    validate_obs_epochs(&dataset.epochs).expect("public RINEX 2 L1/L2/L5 epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert!(report.complete_pairs > 0);
    assert!(report.l1_l2_pairs > 0);
    assert!(report.l1_l5_pairs > 0);
}

#[test]
fn synthetic_rinex_3_import_exposes_l5_observation_channel() {
    let data = [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "gps-station", "MARKER NAME"),
        format!("{:<60}{}", "G    5 C1C L1C C5Q L5Q S5Q", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
            "G01", 20345678.123, 123456.250, 20345680.750, 123450.500, 48.0
        ),
    ]
    .join("\n");
    let dataset =
        parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 L5 data");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::L1, SignalBand::L5]
    );
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::Unknown]
    );
    validate_obs_epochs(&dataset.epochs).expect("synthetic RINEX 3 L5 epochs must validate");

    let report = check_dual_frequency_observations(&dataset.epochs);
    assert_eq!(report.complete_pairs, 1);
    assert_eq!(report.l1_l5_pairs, 1);
}
