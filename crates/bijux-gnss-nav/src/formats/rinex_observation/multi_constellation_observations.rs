use super::*;

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
