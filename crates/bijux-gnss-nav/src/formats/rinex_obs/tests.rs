use std::path::PathBuf;

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_signal::api::{
    check_dual_frequency_observations, signal_spec_beidou_b1i, signal_spec_beidou_b2i,
    signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_gps_l2c, signal_spec_gps_l5,
    validate_obs_epochs,
};

use super::{
    format_rinex_observation_dataset, parse_observation_cells, parse_rinex_2_observation_records,
    parse_rinex_beidou_observation_dataset, parse_rinex_galileo_observation_dataset,
    parse_rinex_gps_observation_dataset, parse_rinex_observation_dataset,
    parse_rinex_observation_header, RinexCodeBiasState, RinexObservationRecord,
    RinexObservationTimeSystem,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("read RINEX fixture {}", path.display()))
}

mod header_parsing;
mod raw_datasets;
mod record_parsing;

#[test]
fn parse_public_rinex_2_gps_observation_epochs() {
    let data = fixture("georinex_14601736_20180622.obs");
    let dataset =
        parse_rinex_gps_observation_dataset(&data).expect("parse RINEX GPS observation data");

    assert!((dataset.version - 2.11).abs() < 1.0e-12);
    assert_eq!(dataset.marker_name.as_deref(), Some("st"));
    assert_eq!(dataset.interval_s, Some(15.0));
    assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Unknown);
    assert_eq!(dataset.code_bias_status.source, None);
    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[0].band, SignalBand::L1);
    assert_eq!(dataset.observation_channels[0].code, SignalCode::Ca);
    assert_eq!(dataset.observation_channels[0].pseudorange_observation_type, "C1");
    assert_eq!(
        dataset.observation_channels[0].carrier_phase_observation_type.as_deref(),
        Some("L1")
    );
    assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::Py);
    assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "P2");
    assert_eq!(
        dataset.observation_channels[1].carrier_phase_observation_type.as_deref(),
        Some("L2")
    );
    assert_eq!(dataset.epochs.len(), 3);
    validate_obs_epochs(&dataset.epochs).expect("imported dual-frequency epochs must validate");

    let first_epoch = &dataset.epochs[0];
    assert_eq!(first_epoch.gps_week, Some(2006));
    assert!(first_epoch.sats.len() > 5);
    assert!(first_epoch.valid);
    assert!(!first_epoch.discontinuity);

    let g03_l1 = first_epoch
        .sats
        .iter()
        .find(|sat| {
            sat.signal_id.sat == SatId { constellation: Constellation::Gps, prn: 3 }
                && sat.signal_id.band == SignalBand::L1
        })
        .expect("G03 L1 observation");
    assert!((g03_l1.pseudorange_m.0 - 22_719_526.844).abs() < 1.0e-6);
    assert!((g03_l1.carrier_phase_cycles.0 - 119_391_903.878).abs() < 1.0e-6);
    assert_eq!(g03_l1.timing, None);

    let any_l2 = first_epoch
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L2)
        .expect("at least one L2 observation");
    assert!(any_l2.lock_flags.code_lock);
    assert!(any_l2.lock_flags.carrier_lock);

    let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
    assert!(dual_frequency.complete_pairs > 0);
    assert_eq!(dual_frequency.complete_pairs, dual_frequency.l1_l2_pairs);
}

#[test]
fn parse_rinex_3_gps_observation_epoch() {
    let data = [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "gps-station", "MARKER NAME"),
        format!("{:<60}{}", "G    4 C1C L1C C2P L2P", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  2".to_string(),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
            "G01", 20345678.123, 123456.250, 20345680.750, 123450.500
        ),
        format!(
            "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
            "G02", 21345678.456, 223456.500, 21345681.125, 223450.750
        ),
    ]
    .join("\n");
    let dataset = parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 data");

    assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Unknown);
    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.epochs.len(), 1);
    assert_eq!(dataset.epochs[0].sats.len(), 4);
    assert_eq!(
        dataset.epochs[0].sats[0].signal_id.sat,
        SatId { constellation: Constellation::Gps, prn: 1 }
    );
    let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
    assert_eq!(dual_frequency.complete_pairs, 2);
}

#[test]
fn parse_rinex_3_l2c_observation_epoch() {
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

    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::L2C);
    assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "C2L");
    assert_eq!(dataset.epochs.len(), 1);
    assert_eq!(dataset.epochs[0].sats.len(), 2);
    let l2 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L2)
        .expect("L2C observation");
    assert_eq!(l2.signal_id.code, SignalCode::L2C);
    assert_eq!(l2.metadata.signal, signal_spec_gps_l2c());
}

#[test]
fn parse_rinex_3_l5_observation_epoch() {
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
            "G01", 20345678.123, 123456.250, 20345680.750, 123450.500, 49.5
        ),
    ]
    .join("\n");
    let dataset =
        parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 L5 data");

    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[1].band, SignalBand::L5);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::Unknown);
    assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "C5Q");
    assert_eq!(
        dataset.observation_channels[1].carrier_phase_observation_type.as_deref(),
        Some("L5Q")
    );
    assert_eq!(
        dataset.observation_channels[1].signal_strength_observation_type.as_deref(),
        Some("S5Q")
    );
    assert_eq!(dataset.epochs.len(), 1);
    assert_eq!(dataset.epochs[0].sats.len(), 2);
    let l5 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L5)
        .expect("L5 observation");
    assert_eq!(l5.signal_id.code, SignalCode::Unknown);
    assert_eq!(l5.metadata.signal, signal_spec_gps_l5());
    assert_eq!(l5.cn0_dbhz, 49.5);
}

#[test]
fn parse_rinex_3_galileo_e1_e5_observation_epoch() {
    let data = [
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
            "E11", 24345678.123, 223456.250, 24345680.750, 223450.500, 47.0
        ),
    ]
    .join("\n");
    let dataset = parse_rinex_galileo_observation_dataset(&data)
        .expect("parse synthetic RINEX 3 Galileo E1/E5 data");

    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[0].band, SignalBand::E1);
    assert_eq!(dataset.observation_channels[0].code, SignalCode::E1B);
    assert_eq!(dataset.observation_channels[1].band, SignalBand::E5);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::E5a);
    assert_eq!(dataset.epochs.len(), 1);
    assert_eq!(dataset.epochs[0].sats.len(), 2);
    let e1 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::E1)
        .expect("E1 observation");
    let e5 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::E5)
        .expect("E5 observation");
    assert_eq!(e1.metadata.signal, signal_spec_galileo_e1b());
    assert_eq!(e5.metadata.signal, signal_spec_galileo_e5a());
    assert_eq!(e5.cn0_dbhz, 47.0);
    let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
    assert_eq!(dual_frequency.complete_pairs, 1);
    assert_eq!(dual_frequency.e1_e5_pairs, 1);
}

#[test]
fn parse_rinex_3_beidou_b1_b2_observation_epoch() {
    let data = [
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
    .join("\n");
    let dataset = parse_rinex_beidou_observation_dataset(&data)
        .expect("parse synthetic RINEX 3 BeiDou B1/B2 data");

    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[0].band, SignalBand::B1);
    assert_eq!(dataset.observation_channels[0].code, SignalCode::B1I);
    assert_eq!(dataset.observation_channels[1].band, SignalBand::B2);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::B2I);
    assert_eq!(dataset.epochs.len(), 1);
    assert_eq!(dataset.epochs[0].sats.len(), 2);
    let b1 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::B1)
        .expect("B1 observation");
    let b2 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::B2)
        .expect("B2 observation");
    assert_eq!(b1.metadata.signal, signal_spec_beidou_b1i());
    assert_eq!(b2.metadata.signal, signal_spec_beidou_b2i());
    assert_eq!(b2.cn0_dbhz, 46.5);
    let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
    assert_eq!(dual_frequency.complete_pairs, 1);
    assert_eq!(dual_frequency.b1_b2_pairs, 1);
}

#[test]
fn parse_public_rinex_3_mixed_gps_observation_epochs() {
    let data = fixture("glab_gage_20100305.obs");
    let dataset = parse_rinex_gps_observation_dataset(&data).expect("parse public RINEX 3 data");

    assert!((dataset.version - 3.01).abs() < 1.0e-12);
    assert_eq!(dataset.marker_name.as_deref(), Some("MRKR"));
    assert_eq!(dataset.interval_s, Some(30.0));
    assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Applied);
    assert_eq!(
        dataset.code_bias_status.source.as_deref(),
        Some("G CC2NONCC          p1c1bias.hist @ goby.nrl.navy.mil")
    );
    assert_eq!(dataset.observation_channels.len(), 2);
    assert_eq!(dataset.observation_channels[0].band, SignalBand::L1);
    assert_eq!(dataset.observation_channels[0].code, SignalCode::Ca);
    assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
    assert_eq!(dataset.observation_channels[1].code, SignalCode::Py);
    assert_eq!(dataset.epochs.len(), 2);
    validate_obs_epochs(&dataset.epochs).expect("imported mixed-band epochs must validate");
    assert!(dataset.epochs[0].sats.len() > 10);
    let g07_l1 = dataset.epochs[0]
        .sats
        .iter()
        .find(|sat| {
            sat.signal_id.sat == SatId { constellation: Constellation::Gps, prn: 7 }
                && sat.signal_id.band == SignalBand::L1
        })
        .expect("G07 L1 observation");
    assert!((g07_l1.pseudorange_m.0 - 22_227_666.760).abs() < 1.0e-6);
    let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
    assert!(dual_frequency.complete_pairs > 0);
    assert_eq!(dual_frequency.complete_pairs, dual_frequency.l1_l2_pairs);
}
