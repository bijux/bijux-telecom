#![allow(missing_docs)]

use bijux_gnss_core::api::{SignalBand, SignalCode};
use bijux_gnss_nav::api::{
    combinations_from_obs_epochs, iono_free_code_from_obs_epochs, iono_free_phase_from_obs_epochs,
    measured_ionosphere_from_obs_epochs, parse_rinex_gps_observation_dataset,
};
use bijux_gnss_signal::api::{signal_cycles_to_meters, validate_obs_epochs};

fn synthetic_gps_l1_l2_rinex() -> String {
    [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "gps-station", "MARKER NAME"),
        format!("{:<60}{}", "G    4 C1C L1C C2W L2W", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
            "G11", 24_345_678.125, 123_456.250, 24_345_681.875, 123_450.500
        ),
    ]
    .join("\n")
}

#[test]
fn rinex_gps_l1_l2_import_supports_measured_ionosphere() {
    let dataset = parse_rinex_gps_observation_dataset(&synthetic_gps_l1_l2_rinex())
        .expect("parse synthetic GPS L1/L2 RINEX observations");

    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.band).collect::<Vec<_>>(),
        vec![SignalBand::L1, SignalBand::L2]
    );
    assert_eq!(
        dataset.observation_channels.iter().map(|channel| channel.code).collect::<Vec<_>>(),
        vec![SignalCode::Ca, SignalCode::Py]
    );
    validate_obs_epochs(&dataset.epochs).expect("synthetic GPS L1/L2 epochs must validate");

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);
    let iono_free_code =
        iono_free_code_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_code.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(measured_ionosphere.len(), 1);
    assert_eq!(measured_ionosphere[0].band_1, SignalBand::L1);
    assert_eq!(measured_ionosphere[0].band_2, SignalBand::L2);
    assert_eq!(measured_ionosphere[0].code_status, "ok");
    assert_eq!(measured_ionosphere[0].phase_status, "ok");
}

#[test]
fn rinex_gps_l1_l2_measured_ionosphere_uses_signal_specific_wavelengths() {
    let dataset = parse_rinex_gps_observation_dataset(&synthetic_gps_l1_l2_rinex())
        .expect("parse synthetic GPS L1/L2 RINEX observations");
    let epoch = &dataset.epochs[0];
    let l1 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let l2 =
        epoch.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L2).expect("L2 observation");

    let combinations =
        combinations_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);
    let iono_free_phase =
        iono_free_phase_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);
    let measured_ionosphere =
        measured_ionosphere_from_obs_epochs(&dataset.epochs, SignalBand::L1, SignalBand::L2);

    let l1_phase_m = signal_cycles_to_meters(l1.carrier_phase_cycles, l1.metadata.signal).0;
    let l2_phase_m = signal_cycles_to_meters(l2.carrier_phase_cycles, l2.metadata.signal).0;
    let f1_2 = l1.metadata.signal.carrier_hz.value().powi(2);
    let f2_2 = l2.metadata.signal.carrier_hz.value().powi(2);
    let expected_if_phase_m = (f1_2 * l1_phase_m - f2_2 * l2_phase_m) / (f1_2 - f2_2);

    assert_eq!(combinations.len(), 1);
    assert_eq!(iono_free_phase.len(), 1);
    assert_eq!(measured_ionosphere.len(), 1);
    assert!(
        (combinations[0].geometry_free_phase_m.expect("geometry-free phase")
            - (l1_phase_m - l2_phase_m))
            .abs()
            < 1.0e-9
    );
    assert!(
        (combinations[0].if_phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-9
    );
    assert!(
        (measured_ionosphere[0].code_geometry_free_m.expect("geometry-free code")
            - (l2.pseudorange_m.0 - l1.pseudorange_m.0))
            .abs()
            < 1.0e-9
    );
    assert!(
        (measured_ionosphere[0].phase_geometry_free_m.expect("geometry-free phase")
            - (l1_phase_m - l2_phase_m))
            .abs()
            < 1.0e-9
    );
    assert!(
        (iono_free_phase[0].phase_m.expect("iono-free phase") - expected_if_phase_m).abs() < 1.0e-9
    );
}
