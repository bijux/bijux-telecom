#![allow(missing_docs)]

use bijux_gnss_core::api::{validate_obs_epochs, SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    parse_rinex_beidou_observation_dataset,
};

fn synthetic_beidou_rinex() -> String {
    [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "beidou-station", "MARKER NAME"),
        format!("{:<60}{}", "C    5 C2I L2I C7I L7I S7I", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
            "C11", 24_345_678.125, 123_456.250, 24_345_679.875, 123_450.500, 46.5
        ),
    ]
    .join("\n")
}

#[test]
fn rinex_beidou_import_exposes_b1_b2_channels() {
    let dataset = parse_rinex_beidou_observation_dataset(&synthetic_beidou_rinex())
        .expect("parse synthetic BeiDou RINEX observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::B1, SignalBand::B2]
    );
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::B1I, SignalCode::B2I]
    );
    validate_obs_epochs(&dataset.epochs).expect("synthetic BeiDou epochs must validate");
}

#[test]
fn rinex_beidou_import_supports_dual_frequency_combinations() {
    let dataset = parse_rinex_beidou_observation_dataset(&synthetic_beidou_rinex())
        .expect("parse synthetic BeiDou RINEX observations");

    let combinations = combinations_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(combinations[0].band_1, SignalBand::B1);
    assert_eq!(combinations[0].band_2, SignalBand::B2);
}
