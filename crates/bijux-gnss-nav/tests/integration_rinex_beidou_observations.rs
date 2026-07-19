#![allow(missing_docs)]

use bijux_gnss_core::api::{SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    measured_ionosphere_from_obs_epochs, parse_rinex_beidou_observation_dataset,
};
use bijux_gnss_signal::api::{signal_cycles_to_meters, validate_obs_epochs};

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

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(measured_ionosphere.len(), 1);
    assert_eq!(combinations[0].band_1, SignalBand::B1);
    assert_eq!(combinations[0].band_2, SignalBand::B2);
    assert_eq!(measured_ionosphere[0].band_1, SignalBand::B1);
    assert_eq!(measured_ionosphere[0].band_2, SignalBand::B2);
}

#[test]
fn rinex_beidou_phase_combinations_follow_signal_specific_wavelengths() {
    let dataset = parse_rinex_beidou_observation_dataset(&synthetic_beidou_rinex())
        .expect("parse synthetic BeiDou RINEX observations");
    let epoch = &dataset.epochs[0];
    let b1 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::B1).expect("B1 observation");
    let b2 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::B2).expect("B2 observation");

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::B1, SignalBand::B2);

    let b1_phase_m = signal_cycles_to_meters(b1.carrier_phase_cycles, b1.metadata.signal).0;
    let b2_phase_m = signal_cycles_to_meters(b2.carrier_phase_cycles, b2.metadata.signal).0;
    let f1_2 = b1.metadata.signal.carrier_hz.value().powi(2);
    let f2_2 = b2.metadata.signal.carrier_hz.value().powi(2);
    let expected_if_phase_m = (f1_2 * b1_phase_m - f2_2 * b2_phase_m) / (f1_2 - f2_2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert!(
        (combinations[0].geometry_free_phase_m.expect("geometry-free phase")
            - (b1_phase_m - b2_phase_m))
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
    assert!(measured_ionosphere[0].phase_arc_reset);
    assert!(
        (measured_ionosphere[0].code_geometry_free_m.expect("geometry-free code")
            - (b2.pseudorange_m.0 - b1.pseudorange_m.0))
            .abs()
            < 1.0e-9
    );
    assert!(
        (measured_ionosphere[0].phase_geometry_free_m.expect("geometry-free phase")
            - (b1_phase_m - b2_phase_m))
            .abs()
            < 1.0e-9
    );
    assert!(
        (iono_free_phase[0].phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-9
    );
}
