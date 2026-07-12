#![allow(missing_docs)]

use bijux_gnss_core::api::{signal_cycles_to_meters, validate_obs_epochs, SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    measured_ionosphere_from_obs_epochs, parse_rinex_galileo_observation_dataset,
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

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(measured_ionosphere.len(), 1);
    assert_eq!(combinations[0].band_1, SignalBand::E1);
    assert_eq!(combinations[0].band_2, SignalBand::E5);
    assert_eq!(measured_ionosphere[0].band_1, SignalBand::E1);
    assert_eq!(measured_ionosphere[0].band_2, SignalBand::E5);
}

#[test]
fn rinex_galileo_phase_combinations_follow_signal_specific_wavelengths() {
    let dataset = parse_rinex_galileo_observation_dataset(&synthetic_galileo_rinex())
        .expect("parse synthetic Galileo RINEX observations");
    let epoch = &dataset.epochs[0];
    let e1 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::E1).expect("E1 observation");
    let e5 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::E5).expect("E5 observation");

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::E1, SignalBand::E5);

    let e1_phase_m = signal_cycles_to_meters(e1.carrier_phase_cycles, e1.metadata.signal).0;
    let e5_phase_m = signal_cycles_to_meters(e5.carrier_phase_cycles, e5.metadata.signal).0;
    let f1_2 = e1.metadata.signal.carrier_hz.value().powi(2);
    let f2_2 = e5.metadata.signal.carrier_hz.value().powi(2);
    let expected_if_phase_m = (f1_2 * e1_phase_m - f2_2 * e5_phase_m) / (f1_2 - f2_2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert!(
        (combinations[0].geometry_free_phase_m.expect("geometry-free phase")
            - (e1_phase_m - e5_phase_m))
            .abs()
            < 1.0e-9
    );
    assert!(
        (combinations[0].if_phase_m.expect("iono-free combination phase") - expected_if_phase_m)
            .abs()
            < 1.0e-9
    );
    assert_eq!(measured_ionosphere.len(), 1);
    assert_eq!(measured_ionosphere[0].code_status, "ok");
    assert_eq!(measured_ionosphere[0].phase_status, "ok");
    assert_eq!(measured_ionosphere[0].phase_arc_reset, true);
    assert!(
        (measured_ionosphere[0].code_geometry_free_m.expect("geometry-free code")
            - (e5.pseudorange_m.0 - e1.pseudorange_m.0))
            .abs()
            < 1.0e-9
    );
    assert!(
        (measured_ionosphere[0].phase_geometry_free_m.expect("geometry-free phase")
            - (e1_phase_m - e5_phase_m))
            .abs()
            < 1.0e-9
    );
    assert!(
        (iono_free_phase[0].phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-9
    );
}
