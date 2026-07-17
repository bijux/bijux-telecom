use super::*;

#[test]
fn observation_cells_preserve_loss_of_lock_and_signal_strength_flags() {
    let line = format!("{:>14}{}{}{:16}{:>14}{}{}", 20_345_678.123, 1, 9, "", 123_456.250, " ", 7);
    let cells = parse_observation_cells(&[&line], 3, 0).expect("observation cells");

    assert_eq!(cells[0].value, Some(20_345_678.123));
    assert_eq!(cells[0].loss_of_lock_indicator, Some(1));
    assert_eq!(cells[0].signal_strength_indicator, Some(9));
    assert_eq!(cells[1].value, None);
    assert_eq!(cells[1].loss_of_lock_indicator, None);
    assert_eq!(cells[1].signal_strength_indicator, None);
    assert_eq!(cells[2].value, Some(123_456.250));
    assert_eq!(cells[2].loss_of_lock_indicator, None);
    assert_eq!(cells[2].signal_strength_indicator, Some(7));
}

#[test]
fn rinex_2_raw_records_preserve_events_and_continuations() {
    let observation_types =
        ["C1", "L1", "S1", "C2", "L2", "S2"].into_iter().map(str::to_string).collect::<Vec<_>>();
    let epoch_header = format!(
        "{:>3}{:>3}{:>3}{:>3}{:>3}{:>11.7}  {:>1}{:>3}{}",
        22, 5, 14, 0, 0, 0.0, 0, 13, "G01G02G03G04G05G06G07G08G09G10G11G12"
    );
    let epoch_continuation = format!("{:32}{}", "", "R01");
    let mut lines = vec![epoch_header, epoch_continuation];
    for satellite_index in 0..13 {
        lines.push(format!(
            "{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}{:>14.3}{}{}",
            20_000_000.0 + satellite_index as f64,
            1,
            9,
            100_000.0 + satellite_index as f64,
            " ",
            8,
            45.0,
            " ",
            7,
            21_000_000.0 + satellite_index as f64,
            " ",
            6,
            110_000.0 + satellite_index as f64,
            " ",
            5,
        ));
        lines.push(format!("{:>14.3}{}{}", 44.0, " ", 4));
    }
    lines.push(format!("{:28}{:>1}{:>3}", "", 4, 1));
    lines.push(format!("{:<60}{}", "event marker", "COMMENT"));
    let line_refs = lines.iter().map(String::as_str).collect::<Vec<_>>();

    let records = parse_rinex_2_observation_records(
        &line_refs,
        &observation_types,
        RinexObservationTimeSystem::Gps,
    )
    .expect("RINEX 2 raw records");

    assert_eq!(records.len(), 2);
    let RinexObservationRecord::Epoch(epoch) = &records[0] else {
        panic!("expected epoch record");
    };
    assert_eq!(epoch.epoch_time.year, 2022);
    assert_eq!(epoch.event_flag, 0);
    assert_eq!(epoch.satellites.len(), 13);
    assert_eq!(epoch.satellites[12].system, 'R');
    assert_eq!(epoch.satellites[0].observations.len(), 6);
    assert_eq!(epoch.satellites[0].observations[0].loss_of_lock_indicator, Some(1));
    assert_eq!(epoch.satellites[0].observations[0].signal_strength_indicator, Some(9));
    assert_eq!(epoch.satellites[0].observations[5].value, Some(44.0));
    assert_eq!(epoch.satellites[0].observations[5].signal_strength_indicator, Some(4));

    let RinexObservationRecord::Event(event) = &records[1] else {
        panic!("expected event record");
    };
    assert_eq!(event.event_flag, 4);
    assert_eq!(
        event.records,
        ["event marker                                                COMMENT"]
    );
}
