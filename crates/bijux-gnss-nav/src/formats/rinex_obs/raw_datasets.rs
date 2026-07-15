use super::*;

#[test]
fn raw_observation_dataset_parses_rinex_2_records() {
    let data = [
        format!(
            "{:<60}{}",
            "     2.11           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "raw-station", "MARKER NAME"),
        format!("{:<60}{}", "     3    C1    L1    S1", "# / TYPES OF OBSERV"),
        format!("{:<60}{}", "", "END OF HEADER"),
        format!(
            "{:>3}{:>3}{:>3}{:>3}{:>3}{:>11.7}  {:>1}{:>3}{}",
            22, 5, 14, 0, 0, 0.0, 1, 2, "G01R02"
        ),
        format!(
            "{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            20_000_000.0, 1, 9, 100_000.0, " ", 8, 45.0, " ", 7
        ),
        format!(
            "{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            21_000_000.0, " ", 6, 110_000.0, " ", 5, 44.0, " ", 4
        ),
    ]
    .join("\n");

    let dataset = parse_rinex_observation_dataset(&data).expect("raw RINEX 2 dataset");

    assert!((dataset.version - 2.11).abs() < 1.0e-12);
    assert_eq!(dataset.marker_name.as_deref(), Some("raw-station"));
    assert_eq!(dataset.observation_types_v2.as_deref().expect("types"), ["C1", "L1", "S1"]);
    assert_eq!(dataset.records.len(), 1);
    let RinexObservationRecord::Epoch(epoch) = &dataset.records[0] else {
        panic!("expected epoch");
    };
    assert_eq!(epoch.event_flag, 1);
    assert_eq!(epoch.satellites.len(), 2);
    assert_eq!(epoch.satellites[0].system, 'G');
    assert_eq!(epoch.satellites[1].system, 'R');
    assert_eq!(epoch.satellites[0].observations[0].signal_strength_indicator, Some(9));
}

#[test]
fn rinex_2_raw_dataset_round_trips_measurements_and_flags() {
    let data = [
        format!(
            "{:<60}{}",
            "     2.11           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "raw-station", "MARKER NAME"),
        format!("{:<60}{}", "     6    C1    L1    S1    C2    L2    S2", "# / TYPES OF OBSERV"),
        format!("{:<60}{}", "", "END OF HEADER"),
        format!(
            "{:>3}{:>3}{:>3}{:>3}{:>3}{:>11.7}  {:>1}{:>3}{}",
            22, 5, 14, 0, 0, 0.0, 0, 1, "G01"
        ),
        format!(
            "{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            20_000_000.125,
            1,
            9,
            100_000.250,
            " ",
            8,
            45.500,
            " ",
            7,
            21_000_000.875,
            " ",
            6,
            110_000.500,
            " ",
            5
        ),
        format!("{:>14.3}{}{}", 44.250, " ", 4),
    ]
    .join("\n");
    let dataset = parse_rinex_observation_dataset(&data).expect("raw RINEX 2 dataset");

    let encoded = format_rinex_observation_dataset(&dataset).expect("format RINEX 2 dataset");
    let reparsed =
        parse_rinex_observation_dataset(&encoded).expect("reparse formatted RINEX 2 dataset");

    assert_eq!(reparsed, dataset);
}

#[test]
fn raw_observation_dataset_parses_rinex_4_mixed_epochs_and_events() {
    let data = [
        format!(
            "{:<60}{}",
            "     4.02           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "mixed-station", "MARKER NAME"),
        format!("{:<60}{}", "G    3 C1C L1C S1C", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "E    2 C1C L1C", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  2  -0.000000123".to_string(),
        format!(
            "{:<3}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            "G01", 20_000_000.0, 1, 9, 100_000.0, " ", 8, 45.0, " ", 7
        ),
        format!("{:<3}{:>14.3}{}{}{:>14.3}{}{}", "E11", 24_000_000.0, " ", 6, 200_000.0, " ", 5),
        "> 2022 05 14 00 00 30.0000000  4  1".to_string(),
        format!("{:<60}{}", "event marker", "COMMENT"),
    ]
    .join("\n");

    let dataset = parse_rinex_observation_dataset(&data).expect("raw RINEX 4 dataset");

    assert!((dataset.version - 4.02).abs() < 1.0e-12);
    assert_eq!(dataset.records.len(), 2);
    let RinexObservationRecord::Epoch(epoch) = &dataset.records[0] else {
        panic!("expected epoch");
    };
    assert_eq!(epoch.receiver_clock_offset_s, Some(-0.000000123));
    assert_eq!(epoch.satellites.len(), 2);
    assert_eq!(epoch.satellites[0].system, 'G');
    assert_eq!(epoch.satellites[0].observations[0].loss_of_lock_indicator, Some(1));
    assert_eq!(epoch.satellites[1].system, 'E');
    let RinexObservationRecord::Event(event) = &dataset.records[1] else {
        panic!("expected event");
    };
    assert_eq!(event.event_flag, 4);
    assert_eq!(
        event.records,
        ["event marker                                                COMMENT"]
    );
}

#[test]
fn rinex_4_raw_dataset_round_trips_mixed_epochs_and_events() {
    let data = [
        format!(
            "{:<60}{}",
            "     4.02           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "mixed-station", "MARKER NAME"),
        format!("{:<60}{}", "G    6 C1C L1C S1C C5Q L5Q S5Q", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "E    2 C1C L1C", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
        "> 2022 05 14 00 00 00.0000000  0  2  -0.000000123".to_string(),
        format!(
            "{:<3}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            "G01",
            20_000_000.125,
            1,
            9,
            100_000.250,
            " ",
            8,
            45.500,
            " ",
            7,
            21_000_000.875,
            " ",
            6
        ),
        format!("{:>14.3}{}{}{:>14.3}{}{}", 110_000.500, " ", 5, 44.250, " ", 4),
        format!(
            "{:<3}{:>14.3}{}{}{:>14.3}{}{}",
            "E11", 24_000_000.000, " ", 6, 200_000.000, " ", 5
        ),
        "> 2022 05 14 00 00 30.0000000  4  1".to_string(),
        format!("{:<60}{}", "event marker", "COMMENT"),
    ]
    .join("\n");
    let dataset = parse_rinex_observation_dataset(&data).expect("raw RINEX 4 dataset");

    let encoded = format_rinex_observation_dataset(&dataset).expect("format RINEX 4 dataset");
    let reparsed =
        parse_rinex_observation_dataset(&encoded).expect("reparse formatted RINEX 4 dataset");

    assert_eq!(reparsed, dataset);
}

#[test]
fn public_rinex_2_raw_dataset_round_trips_supported_records() {
    let data = fixture("georinex_14601736_20180622.obs");
    let dataset = parse_rinex_observation_dataset(&data).expect("parse public RINEX 2 data");

    let encoded = format_rinex_observation_dataset(&dataset).expect("format public RINEX 2");
    let reparsed =
        parse_rinex_observation_dataset(&encoded).expect("reparse formatted RINEX 2 data");

    assert_eq!(reparsed, dataset);
}

#[test]
fn public_rinex_3_raw_dataset_round_trips_supported_records() {
    let data = fixture("glab_gage_20100305.obs");
    let dataset = parse_rinex_observation_dataset(&data).expect("parse public RINEX 3 data");

    let encoded = format_rinex_observation_dataset(&dataset).expect("format public RINEX 3");
    let reparsed =
        parse_rinex_observation_dataset(&encoded).expect("reparse formatted RINEX 3 data");

    assert_eq!(reparsed, dataset);
}
