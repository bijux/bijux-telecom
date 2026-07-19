#![allow(missing_docs)]

use bijux_gnss_core::api::{GpsTime, LeapSeconds, TaiTime};
use bijux_gnss_nav::api::{
    format_utc_civil_time, gps_to_utc_civil_with_offset, parse_utc_civil_time,
    tai_to_utc_civil_with_offset, utc_civil_to_gps_with_offset, utc_civil_to_tai_with_offset,
    GnssTimeSystem, TimeOffsetSource,
};

fn assert_seconds_close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1.0e-9, "actual={actual} expected={expected}");
}

#[test]
fn public_utc_civil_time_preserves_inserted_second_text_and_ordering() {
    let leap = LeapSeconds::default_table();
    let before = parse_utc_civil_time("2016-12-31T23:59:59Z").expect("before");
    let inserted = parse_utc_civil_time("2016-12-31T23:59:60.75Z").expect("inserted");
    let after = parse_utc_civil_time("2017-01-01T00:00:00Z").expect("after");

    inserted.validate_leap_second(&leap).expect("declared leap second");

    assert!(before < inserted);
    assert!(inserted < after);
    assert_eq!(format_utc_civil_time(inserted), "2016-12-31T23:59:60.75Z");
}

#[test]
fn public_gpst_conversion_round_trips_inserted_utc_second() {
    let leap = LeapSeconds::default_table();
    let before = utc_civil_to_gps_with_offset(
        parse_utc_civil_time("2016-12-31T23:59:59Z").expect("before"),
        &leap,
    )
    .expect("before gps");
    let inserted = utc_civil_to_gps_with_offset(
        parse_utc_civil_time("2016-12-31T23:59:60.25Z").expect("inserted"),
        &leap,
    )
    .expect("inserted gps");
    let after = utc_civil_to_gps_with_offset(
        parse_utc_civil_time("2017-01-01T00:00:00Z").expect("after"),
        &leap,
    )
    .expect("after gps");
    let round_trip = gps_to_utc_civil_with_offset(inserted.time, &leap);
    let after_from_gps = gps_to_utc_civil_with_offset(GpsTime { week: 1930, tow_s: 18.0 }, &leap);

    assert_eq!(inserted.offset.source, TimeOffsetSource::LeapSecondTable);
    assert_eq!(inserted.offset.from, GnssTimeSystem::Utc);
    assert_eq!(inserted.offset.to, GnssTimeSystem::Gpst);
    assert_seconds_close(inserted.time.to_seconds() - before.time.to_seconds(), 1.25);
    assert_seconds_close(after.time.to_seconds() - inserted.time.to_seconds(), 0.75);
    assert_eq!(round_trip.time.format_utc(), "2016-12-31T23:59:60.25Z");
    assert_eq!(after_from_gps.time.format_utc(), "2017-01-01T00:00:00Z");
}

#[test]
fn public_tai_conversion_round_trips_inserted_utc_second() {
    let leap = LeapSeconds::default_table();
    let before = utc_civil_to_tai_with_offset(
        parse_utc_civil_time("2016-12-31T23:59:59Z").expect("before"),
        &leap,
    )
    .expect("before tai");
    let inserted = utc_civil_to_tai_with_offset(
        parse_utc_civil_time("2016-12-31T23:59:60.5Z").expect("inserted"),
        &leap,
    )
    .expect("inserted tai");
    let after = utc_civil_to_tai_with_offset(
        parse_utc_civil_time("2017-01-01T00:00:00Z").expect("after"),
        &leap,
    )
    .expect("after tai");
    let round_trip = tai_to_utc_civil_with_offset(inserted.time, &leap);
    let after_from_tai = tai_to_utc_civil_with_offset(TaiTime { tai_s: 1_483_228_837.0 }, &leap);

    assert_eq!(inserted.offset.source, TimeOffsetSource::LeapSecondTable);
    assert_eq!(inserted.offset.from, GnssTimeSystem::Utc);
    assert_eq!(inserted.offset.to, GnssTimeSystem::Tai);
    assert_seconds_close(inserted.time.tai_s - before.time.tai_s, 1.5);
    assert_seconds_close(after.time.tai_s - inserted.time.tai_s, 0.5);
    assert_eq!(round_trip.time.format_utc(), "2016-12-31T23:59:60.5Z");
    assert_eq!(after_from_tai.time.format_utc(), "2017-01-01T00:00:00Z");
}
