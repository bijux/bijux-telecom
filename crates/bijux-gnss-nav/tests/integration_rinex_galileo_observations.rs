#![allow(missing_docs)]

use bijux_gnss_core::api::{validate_obs_epochs, SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    parse_rinex_galileo_observation_dataset,
};

fn synthetic_galileo_rinex() -> String {
    [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "galileo-station", "MARKER NAME"),
        format!("{:<60}{}", "E    5 C1C L1C C5Q L5Q S5Q", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
            "E11", 24_345_678.125, 123_456.250, 24_345_679.875, 123_450.500, 47.0
        ),
    ]
    .join("\n")
}

#[test]
fn rinex_galileo_import_exposes_e1_e5_channels() {
    let dataset = parse_rinex_galileo_observation_dataset(&synthetic_galileo_rinex())
        .expect("parse synthetic Galileo RINEX observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::E1, SignalBand::E5]
    );
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::E1B, SignalCode::E5a]
    );
    validate_obs_epochs(&dataset.epochs).expect("synthetic Galileo epochs must validate");
}

#[test]
fn rinex_galileo_import_supports_dual_frequency_combinations() {
    let dataset = parse_rinex_galileo_observation_dataset(&synthetic_galileo_rinex())
        .expect("parse synthetic Galileo RINEX observations");

    let combinations = combinations_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(combinations[0].band_1, SignalBand::E1);
    assert_eq!(combinations[0].band_2, SignalBand::E5);
}
