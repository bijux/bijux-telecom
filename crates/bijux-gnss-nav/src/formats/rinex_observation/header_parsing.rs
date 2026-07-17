use super::*;

#[test]
fn parse_public_rinex_2_observation_header() {
    let data = fixture("georinex_14601736_20180622.obs");
    let (header, header_line_count) =
        parse_rinex_observation_header(&data).expect("parse RINEX observation header");

    assert_eq!(header_line_count, 33);
    assert!((header.version - 2.11).abs() < 1.0e-12);
    assert_eq!(header.marker_name.as_deref(), Some("st"));
    assert_eq!(
        header.approx_position_ecef_m,
        Some((-4_647_137.5830, 2_562_189.6255, -3_526_626.7006))
    );
    assert_eq!(header.interval_s, Some(15.0));

    let gps_observation_types =
        header.gps_observation_types().expect("RINEX 2 observation types present");
    assert_eq!(gps_observation_types, ["C1", "C2", "C8", "L1", "L2", "L8", "P2"]);
}

#[test]
fn parse_rinex_3_observation_type_header() {
    let data = [
        format!(
            "{:<60}{}",
            "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
        ),
        format!("{:<60}{}", "G    5 C1C L1C D1C S1C C5Q", "SYS / # / OBS TYPES"),
        format!("{:<60}{}", "", "END OF HEADER"),
    ]
    .join("\n");
    let (header, header_line_count) =
        parse_rinex_observation_header(&data).expect("parse RINEX 3 observation header");

    assert_eq!(header_line_count, 3);
    assert_eq!(
        header.gps_observation_types().expect("GPS observation types"),
        ["C1C", "L1C", "D1C", "S1C", "C5Q"]
    );
}
