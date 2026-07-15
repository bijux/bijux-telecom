#![allow(missing_docs)]

use bijux_gnss_core::api::{GpsTime, LeapSeconds, TaiTime, UtcTime};
use bijux_gnss_nav::api::{
    beidou_to_gps_with_offset, galileo_to_gps_with_offset, glonass_to_gps_with_offset,
    gps_to_beidou_with_offset, gps_to_galileo_with_offset, gps_to_glonass_with_offset,
    gps_to_utc_with_offset, tai_to_utc_with_offset, utc_to_glonass_with_offset,
    utc_to_gps_with_offset, utc_to_tai_with_offset, BeidouTime, GalileoGpsTimeOffset, GalileoTime,
    GlonassTime, GlonassUtcOffset, GnssTimeSystem, TimeOffsetSource,
};

fn assert_seconds_close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1.0e-9, "actual={actual} expected={expected}");
}

#[test]
fn public_time_conversions_cover_leap_boundary_vectors() {
    let leap = LeapSeconds::default_table();
    let before = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_799.0 }, &leap);
    let after = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_800.0 }, &leap);
    let after_utc = gps_to_utc_with_offset(after.time, &leap);
    let after_tai = utc_to_tai_with_offset(after_utc.time, &leap);
    let tai_round_trip = tai_to_utc_with_offset(TaiTime { tai_s: after_tai.time.tai_s }, &leap);

    assert_eq!(before.offset.source, TimeOffsetSource::LeapSecondTable);
    assert_eq!(before.offset.offset_s, 17.0);
    assert_eq!(after.offset.source, TimeOffsetSource::LeapSecondTable);
    assert_eq!(after.offset.offset_s, 18.0);
    assert_eq!(after_utc.offset.from, GnssTimeSystem::Gpst);
    assert_eq!(after_utc.offset.to, GnssTimeSystem::Utc);
    assert_eq!(after_utc.time.unix_s, 1_483_228_800.0);
    assert_eq!(after_tai.offset.offset_s, 37.0);
    assert_eq!(tai_round_trip.offset.offset_s, -37.0);
    assert_eq!(tai_round_trip.time, after_utc.time);
}

#[test]
fn public_galileo_and_beidou_conversions_preserve_epoch_boundaries() {
    let galileo_offset =
        GalileoGpsTimeOffset::broadcast(0.75, "integration gst-gpst boundary vector");
    let gps_week_end = GpsTime { week: 2_300, tow_s: 604_799.5 };
    let galileo = gps_to_galileo_with_offset(gps_week_end, &galileo_offset);
    let galileo_round_trip = galileo_to_gps_with_offset(galileo.time, &galileo_offset);

    assert_eq!(galileo.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
    assert_eq!(galileo.offset.from, GnssTimeSystem::Gpst);
    assert_eq!(galileo.offset.to, GnssTimeSystem::Gst);
    assert_eq!(galileo.time, GalileoTime { week: 2_301, tow_s: 0.25 });
    assert_seconds_close(galileo_round_trip.time.to_seconds(), gps_week_end.to_seconds());

    let bdt_epoch = BeidouTime { week: 0, tow_s: 0.0 };
    let gps_at_bdt_epoch = beidou_to_gps_with_offset(bdt_epoch);
    let bdt_round_trip = gps_to_beidou_with_offset(gps_at_bdt_epoch.time);

    assert_eq!(gps_at_bdt_epoch.offset.source, TimeOffsetSource::FixedSystemDefinition);
    assert_eq!(gps_at_bdt_epoch.offset.offset_s, 14.0);
    assert_eq!(gps_at_bdt_epoch.time, GpsTime { week: 1_356, tow_s: 14.0 });
    assert_eq!(bdt_round_trip.time, bdt_epoch);
}

#[test]
fn public_glonass_conversions_preserve_utc_and_gps_boundaries() {
    let leap = LeapSeconds::default_table();
    let glonass_offset = GlonassUtcOffset::broadcast(0.25, "integration utc su boundary vector");
    let utc = UtcTime { unix_s: 1_483_308_000.0 };
    let glonass = utc_to_glonass_with_offset(utc, &glonass_offset);

    assert_eq!(glonass.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
    assert_eq!(glonass.offset.from, GnssTimeSystem::Utc);
    assert_eq!(glonass.offset.to, GnssTimeSystem::Glonass);
    assert_eq!(glonass.offset.offset_s, 10_800.25);
    assert_eq!(glonass.time, GlonassTime { day_index: 17_168, seconds_of_day: 3_600.25 });

    let gps = glonass_to_gps_with_offset(glonass.time, &leap, &glonass_offset);
    let glonass_round_trip = gps_to_glonass_with_offset(gps.time, &leap, &glonass_offset);

    assert_eq!(gps.offset.source, TimeOffsetSource::CompositeOffsetModel);
    assert_eq!(glonass_round_trip.offset.source, TimeOffsetSource::CompositeOffsetModel);
    assert_seconds_close(glonass_round_trip.time.seconds_of_day, glonass.time.seconds_of_day);
    assert_eq!(glonass_round_trip.time.day_index, glonass.time.day_index);
}
