use super::*;

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
