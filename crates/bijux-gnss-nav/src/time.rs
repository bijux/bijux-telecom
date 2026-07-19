#![allow(missing_docs)]

use bijux_gnss_core::api::{gps_to_utc, utc_to_gps, GpsTime, LeapSeconds, TaiTime, UtcTime};
use serde::{Deserialize, Serialize};

pub mod rollover;

const UTC_TAI_EPOCH_OFFSET_S: f64 = 19.0;
const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;
const WEEK_SECONDS: f64 = 604_800.0;
const BEIDOU_EPOCH_GPS_SECONDS: f64 = 1_356.0 * WEEK_SECONDS + 14.0;
const BEIDOU_GPS_OFFSET_S: f64 = 14.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssTimeSystem {
    Gpst,
    Gst,
    Bdt,
    Glonass,
    Utc,
    Tai,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeOffsetSource {
    LeapSecondTable,
    FixedSystemDefinition,
    BroadcastNavigationMessage,
    CompositeOffsetModel,
    ReferenceUtcDay,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeOffsetEvidence {
    pub from: GnssTimeSystem,
    pub to: GnssTimeSystem,
    pub offset_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeConversion<T> {
    pub time: T,
    pub offset: TimeOffsetEvidence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct UtcCivilTime {
    pub year: i32,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub nanosecond: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum UtcCivilTimeError {
    InvalidFormat,
    InvalidDate,
    InvalidTime,
    LeapSecondNotDeclared,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoTime {
    pub week: u32,
    pub tow_s: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoGpsTimeOffset {
    pub gst_minus_gps_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BeidouTime {
    pub week: u32,
    pub tow_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassTime {
    pub day_index: i64,
    pub seconds_of_day: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GlonassUtcOffset {
    pub utc_su_minus_utc_s: f64,
    pub source: TimeOffsetSource,
    pub source_detail: String,
}

impl GalileoTime {
    pub fn from_seconds(total_seconds: f64) -> Self {
        let mut seconds = total_seconds.max(0.0);
        let week = (seconds / WEEK_SECONDS).floor() as u64;
        seconds -= week as f64 * WEEK_SECONDS;
        Self { week: week as u32, tow_s: seconds }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * WEEK_SECONDS + self.tow_s
    }
}

impl GalileoGpsTimeOffset {
    pub fn system_definition() -> Self {
        Self {
            gst_minus_gps_s: 0.0,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "GST steered to GPST within broadcast offset tolerance".to_string(),
        }
    }

    pub fn broadcast(gst_minus_gps_s: f64, source_detail: impl Into<String>) -> Self {
        Self {
            gst_minus_gps_s,
            source: TimeOffsetSource::BroadcastNavigationMessage,
            source_detail: source_detail.into(),
        }
    }
}

impl BeidouTime {
    pub fn from_seconds(total_seconds: f64) -> Self {
        let mut seconds = total_seconds.max(0.0);
        let week = (seconds / WEEK_SECONDS).floor() as u64;
        seconds -= week as f64 * WEEK_SECONDS;
        Self { week: week as u32, tow_s: seconds }
    }

    pub fn to_seconds(&self) -> f64 {
        self.week as f64 * WEEK_SECONDS + self.tow_s
    }
}

impl GlonassUtcOffset {
    pub fn broadcast(utc_su_minus_utc_s: f64, source_detail: impl Into<String>) -> Self {
        Self {
            utc_su_minus_utc_s,
            source: TimeOffsetSource::BroadcastNavigationMessage,
            source_detail: source_detail.into(),
        }
    }
}

impl UtcCivilTime {
    pub fn parse(value: &str) -> Result<Self, UtcCivilTimeError> {
        let value = value.strip_suffix('Z').ok_or(UtcCivilTimeError::InvalidFormat)?;
        let (date, time) = value.split_once('T').ok_or(UtcCivilTimeError::InvalidFormat)?;
        let mut date_parts = date.split('-');
        let year = parse_i32(date_parts.next())?;
        let month = parse_u8(date_parts.next())?;
        let day = parse_u8(date_parts.next())?;
        if date_parts.next().is_some() {
            return Err(UtcCivilTimeError::InvalidFormat);
        }

        let mut time_parts = time.split(':');
        let hour = parse_u8(time_parts.next())?;
        let minute = parse_u8(time_parts.next())?;
        let second_text = time_parts.next().ok_or(UtcCivilTimeError::InvalidFormat)?;
        if time_parts.next().is_some() {
            return Err(UtcCivilTimeError::InvalidFormat);
        }
        let (second_text, fraction_text) = match second_text.split_once('.') {
            Some((_, "")) => return Err(UtcCivilTimeError::InvalidFormat),
            Some((second, fraction)) => (second, fraction),
            None => (second_text, ""),
        };
        let second = parse_u8(Some(second_text))?;
        let nanosecond = parse_nanosecond(fraction_text)?;
        let time = Self { year, month, day, hour, minute, second, nanosecond };
        time.validate_fields()?;
        Ok(time)
    }

    pub fn format_utc(&self) -> String {
        if self.nanosecond == 0 {
            format!(
                "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}Z",
                self.year, self.month, self.day, self.hour, self.minute, self.second
            )
        } else {
            let mut fraction = format!("{:09}", self.nanosecond);
            while fraction.ends_with('0') {
                fraction.pop();
            }
            format!(
                "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{fraction}Z",
                self.year, self.month, self.day, self.hour, self.minute, self.second
            )
        }
    }

    pub fn from_utc_time(utc: UtcTime) -> Self {
        let mut whole_seconds = utc.unix_s.floor() as i64;
        let fractional_s = utc.unix_s - whole_seconds as f64;
        let mut nanosecond = (fractional_s * 1_000_000_000.0).round() as u32;
        if nanosecond == 1_000_000_000 {
            whole_seconds += 1;
            nanosecond = 0;
        }
        let (year, month, day, hour, minute, second) = civil_from_unix_seconds(whole_seconds);
        Self { year, month, day, hour, minute, second, nanosecond }
    }

    pub fn to_utc_time(&self) -> Result<UtcTime, UtcCivilTimeError> {
        self.validate_fields()?;
        if self.second == 60 {
            return Err(UtcCivilTimeError::LeapSecondNotDeclared);
        }
        Ok(UtcTime {
            unix_s: unix_seconds_from_civil(
                self.year,
                self.month,
                self.day,
                self.hour,
                self.minute,
                self.second,
            ) as f64
                + self.nanosecond as f64 / 1_000_000_000.0,
        })
    }

    pub fn validate_leap_second(&self, leap: &LeapSeconds) -> Result<(), UtcCivilTimeError> {
        self.validate_fields()?;
        if self.second != 60 {
            return Ok(());
        }
        let leap_boundary_unix_s =
            unix_seconds_from_civil(self.year, self.month, self.day, self.hour, self.minute, 59)
                + 1;
        if leap.entries.iter().any(|entry| entry.utc_unix_s == leap_boundary_unix_s) {
            Ok(())
        } else {
            Err(UtcCivilTimeError::LeapSecondNotDeclared)
        }
    }

    fn validate_fields(&self) -> Result<(), UtcCivilTimeError> {
        if !valid_date(self.year, self.month, self.day) {
            return Err(UtcCivilTimeError::InvalidDate);
        }
        if self.hour > 23 || self.minute > 59 || self.second > 60 {
            return Err(UtcCivilTimeError::InvalidTime);
        }
        if self.second == 60 && (self.hour != 23 || self.minute != 59) {
            return Err(UtcCivilTimeError::InvalidTime);
        }
        if self.nanosecond >= 1_000_000_000 {
            return Err(UtcCivilTimeError::InvalidTime);
        }
        Ok(())
    }
}

pub fn gps_time_from_utc(utc: UtcTime) -> GpsTime {
    utc_to_gps(utc, &LeapSeconds::default_table())
}

pub fn parse_utc_civil_time(value: &str) -> Result<UtcCivilTime, UtcCivilTimeError> {
    UtcCivilTime::parse(value)
}

pub fn format_utc_civil_time(time: UtcCivilTime) -> String {
    time.format_utc()
}

pub fn utc_to_gps_with_offset(utc: UtcTime, leap: &LeapSeconds) -> TimeConversion<GpsTime> {
    let offset_s = leap.offset_at_utc(utc.unix_s) as f64;
    TimeConversion {
        time: utc_to_gps(utc, leap),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Gpst,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "GPS-UTC from leap second table at UTC instant".to_string(),
        },
    }
}

pub fn gps_to_utc_with_offset(gps: GpsTime, leap: &LeapSeconds) -> TimeConversion<UtcTime> {
    let utc = gps_to_utc(gps, leap);
    let offset_s = -(leap.offset_at_utc(utc.unix_s) as f64);
    TimeConversion {
        time: utc,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Utc,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "UTC-GPS from leap second table after GPS-to-UTC resolution".to_string(),
        },
    }
}

pub fn utc_civil_to_gps_with_offset(
    civil: UtcCivilTime,
    leap: &LeapSeconds,
) -> Result<TimeConversion<GpsTime>, UtcCivilTimeError> {
    if civil.second != 60 {
        return Ok(utc_to_gps_with_offset(civil.to_utc_time()?, leap));
    }

    civil.validate_leap_second(leap)?;
    let leap_boundary_unix_s = leap_boundary_unix_seconds(civil);
    let entry = leap
        .entries
        .iter()
        .find(|entry| entry.utc_unix_s == leap_boundary_unix_s)
        .ok_or(UtcCivilTimeError::LeapSecondNotDeclared)?;
    let offset_s = entry.offset_s - 1;
    let gps_seconds = leap_boundary_unix_s as f64 - GPS_UNIX_EPOCH_OFFSET_S
        + offset_s as f64
        + civil.nanosecond as f64 / 1_000_000_000.0;

    Ok(TimeConversion {
        time: GpsTime::from_seconds(gps_seconds),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Gpst,
            offset_s: offset_s as f64,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "GPS-UTC across declared inserted UTC second".to_string(),
        },
    })
}

pub fn gps_to_utc_civil_with_offset(
    gps: GpsTime,
    leap: &LeapSeconds,
) -> TimeConversion<UtcCivilTime> {
    let gps_unix_like_s = gps.to_seconds() + GPS_UNIX_EPOCH_OFFSET_S;
    for entry in &leap.entries {
        let previous_offset_s = entry.offset_s - 1;
        let inserted_start_s = entry.utc_unix_s as f64 + previous_offset_s as f64;
        let inserted_end_s = entry.utc_unix_s as f64 + entry.offset_s as f64;
        if gps_unix_like_s >= inserted_start_s && gps_unix_like_s < inserted_end_s {
            let (year, month, day, hour, minute, _) = civil_from_unix_seconds(entry.utc_unix_s - 1);
            let nanosecond =
                ((gps_unix_like_s - inserted_start_s) * 1_000_000_000.0).round() as u32;
            return TimeConversion {
                time: UtcCivilTime { year, month, day, hour, minute, second: 60, nanosecond },
                offset: TimeOffsetEvidence {
                    from: GnssTimeSystem::Gpst,
                    to: GnssTimeSystem::Utc,
                    offset_s: -(previous_offset_s as f64),
                    source: TimeOffsetSource::LeapSecondTable,
                    source_detail: "UTC-GPS across declared inserted UTC second".to_string(),
                },
            };
        }
    }

    let conversion = gps_to_utc_with_offset(gps, leap);
    TimeConversion { time: UtcCivilTime::from_utc_time(conversion.time), offset: conversion.offset }
}

pub fn utc_to_tai_with_offset(utc: UtcTime, leap: &LeapSeconds) -> TimeConversion<TaiTime> {
    let offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.offset_at_utc(utc.unix_s) as f64;
    TimeConversion {
        time: TaiTime { tai_s: utc.unix_s + offset_s },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Tai,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "TAI-UTC from 1972 TAI epoch offset plus leap second table".to_string(),
        },
    }
}

pub fn tai_to_utc_with_offset(tai: TaiTime, leap: &LeapSeconds) -> TimeConversion<UtcTime> {
    let mut offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.latest_offset() as f64;
    let mut utc_s = tai.tai_s - offset_s;
    for _ in 0..3 {
        let adjusted_offset_s = UTC_TAI_EPOCH_OFFSET_S + leap.offset_at_utc(utc_s) as f64;
        if (adjusted_offset_s - offset_s).abs() <= f64::EPSILON {
            break;
        }
        offset_s = adjusted_offset_s;
        utc_s = tai.tai_s - offset_s;
    }
    TimeConversion {
        time: UtcTime { unix_s: utc_s },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Tai,
            to: GnssTimeSystem::Utc,
            offset_s: -offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "UTC-TAI from inverse leap second table resolution".to_string(),
        },
    }
}

pub fn utc_civil_to_tai_with_offset(
    civil: UtcCivilTime,
    leap: &LeapSeconds,
) -> Result<TimeConversion<TaiTime>, UtcCivilTimeError> {
    if civil.second != 60 {
        return Ok(utc_to_tai_with_offset(civil.to_utc_time()?, leap));
    }

    civil.validate_leap_second(leap)?;
    let leap_boundary_unix_s = leap_boundary_unix_seconds(civil);
    let entry = leap
        .entries
        .iter()
        .find(|entry| entry.utc_unix_s == leap_boundary_unix_s)
        .ok_or(UtcCivilTimeError::LeapSecondNotDeclared)?;
    let offset_s = UTC_TAI_EPOCH_OFFSET_S + (entry.offset_s - 1) as f64;
    Ok(TimeConversion {
        time: TaiTime {
            tai_s: leap_boundary_unix_s as f64
                + offset_s
                + civil.nanosecond as f64 / 1_000_000_000.0,
        },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Tai,
            offset_s,
            source: TimeOffsetSource::LeapSecondTable,
            source_detail: "TAI-UTC across declared inserted UTC second".to_string(),
        },
    })
}

pub fn tai_to_utc_civil_with_offset(
    tai: TaiTime,
    leap: &LeapSeconds,
) -> TimeConversion<UtcCivilTime> {
    for entry in &leap.entries {
        let previous_offset_s = entry.offset_s - 1;
        let inserted_start_s =
            entry.utc_unix_s as f64 + UTC_TAI_EPOCH_OFFSET_S + previous_offset_s as f64;
        let inserted_end_s =
            entry.utc_unix_s as f64 + UTC_TAI_EPOCH_OFFSET_S + entry.offset_s as f64;
        if tai.tai_s >= inserted_start_s && tai.tai_s < inserted_end_s {
            let (year, month, day, hour, minute, _) = civil_from_unix_seconds(entry.utc_unix_s - 1);
            let nanosecond = ((tai.tai_s - inserted_start_s) * 1_000_000_000.0).round() as u32;
            let offset_s = UTC_TAI_EPOCH_OFFSET_S + previous_offset_s as f64;
            return TimeConversion {
                time: UtcCivilTime { year, month, day, hour, minute, second: 60, nanosecond },
                offset: TimeOffsetEvidence {
                    from: GnssTimeSystem::Tai,
                    to: GnssTimeSystem::Utc,
                    offset_s: -offset_s,
                    source: TimeOffsetSource::LeapSecondTable,
                    source_detail: "UTC-TAI across declared inserted UTC second".to_string(),
                },
            };
        }
    }

    let conversion = tai_to_utc_with_offset(tai, leap);
    TimeConversion { time: UtcCivilTime::from_utc_time(conversion.time), offset: conversion.offset }
}

pub fn gps_to_galileo_with_offset(
    gps: GpsTime,
    offset: &GalileoGpsTimeOffset,
) -> TimeConversion<GalileoTime> {
    TimeConversion {
        time: GalileoTime::from_seconds(gps.to_seconds() + offset.gst_minus_gps_s),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Gst,
            offset_s: offset.gst_minus_gps_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn galileo_to_gps_with_offset(
    gst: GalileoTime,
    offset: &GalileoGpsTimeOffset,
) -> TimeConversion<GpsTime> {
    TimeConversion {
        time: GpsTime::from_seconds(gst.to_seconds() - offset.gst_minus_gps_s),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gst,
            to: GnssTimeSystem::Gpst,
            offset_s: -offset.gst_minus_gps_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn gps_to_beidou_with_offset(gps: GpsTime) -> TimeConversion<BeidouTime> {
    TimeConversion {
        time: BeidouTime::from_seconds(gps.to_seconds() - BEIDOU_EPOCH_GPS_SECONDS),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Bdt,
            offset_s: -BEIDOU_GPS_OFFSET_S,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "BDT epoch is 2006-01-01 UTC and BDT is GPST minus 14 seconds"
                .to_string(),
        },
    }
}

pub fn beidou_to_gps_with_offset(bdt: BeidouTime) -> TimeConversion<GpsTime> {
    TimeConversion {
        time: GpsTime::from_seconds(BEIDOU_EPOCH_GPS_SECONDS + bdt.to_seconds()),
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Bdt,
            to: GnssTimeSystem::Gpst,
            offset_s: BEIDOU_GPS_OFFSET_S,
            source: TimeOffsetSource::FixedSystemDefinition,
            source_detail: "GPST is BDT plus 14 seconds at the BeiDou time epoch".to_string(),
        },
    }
}

pub fn utc_to_glonass_with_offset(
    utc: UtcTime,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GlonassTime> {
    let glonass_seconds = utc.unix_s + 10_800.0 + offset.utc_su_minus_utc_s;
    let day_index = (glonass_seconds / 86_400.0).floor() as i64;
    let seconds_of_day = glonass_seconds - day_index as f64 * 86_400.0;
    TimeConversion {
        time: GlonassTime { day_index, seconds_of_day },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Utc,
            to: GnssTimeSystem::Glonass,
            offset_s: 10_800.0 + offset.utc_su_minus_utc_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn glonass_to_utc_with_offset(
    glonass: GlonassTime,
    offset: &GlonassUtcOffset,
) -> TimeConversion<UtcTime> {
    TimeConversion {
        time: UtcTime {
            unix_s: glonass.day_index as f64 * 86_400.0 + glonass.seconds_of_day
                - 10_800.0
                - offset.utc_su_minus_utc_s,
        },
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Glonass,
            to: GnssTimeSystem::Utc,
            offset_s: -10_800.0 - offset.utc_su_minus_utc_s,
            source: offset.source,
            source_detail: offset.source_detail.clone(),
        },
    }
}

pub fn gps_to_glonass_with_offset(
    gps: GpsTime,
    leap: &LeapSeconds,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GlonassTime> {
    let utc = gps_to_utc(gps, leap);
    let glonass = utc_to_glonass_with_offset(utc, offset);
    TimeConversion {
        time: glonass.time,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Gpst,
            to: GnssTimeSystem::Glonass,
            offset_s: glonass.offset.offset_s - leap.offset_at_utc(utc.unix_s) as f64,
            source: TimeOffsetSource::CompositeOffsetModel,
            source_detail: format!(
                "GPS-UTC from leap second table and GLONASS UTC relation: {}",
                offset.source_detail
            ),
        },
    }
}

pub fn glonass_to_gps_with_offset(
    glonass: GlonassTime,
    leap: &LeapSeconds,
    offset: &GlonassUtcOffset,
) -> TimeConversion<GpsTime> {
    let utc = glonass_to_utc_with_offset(glonass, offset);
    let gps = utc_to_gps(utc.time, leap);
    TimeConversion {
        time: gps,
        offset: TimeOffsetEvidence {
            from: GnssTimeSystem::Glonass,
            to: GnssTimeSystem::Gpst,
            offset_s: utc.offset.offset_s + leap.offset_at_utc(utc.time.unix_s) as f64,
            source: TimeOffsetSource::CompositeOffsetModel,
            source_detail: format!(
                "GLONASS UTC relation and GPS-UTC from leap second table: {}",
                offset.source_detail
            ),
        },
    }
}

fn parse_i32(value: Option<&str>) -> Result<i32, UtcCivilTimeError> {
    value
        .ok_or(UtcCivilTimeError::InvalidFormat)?
        .parse()
        .map_err(|_| UtcCivilTimeError::InvalidFormat)
}

fn parse_u8(value: Option<&str>) -> Result<u8, UtcCivilTimeError> {
    value
        .ok_or(UtcCivilTimeError::InvalidFormat)?
        .parse()
        .map_err(|_| UtcCivilTimeError::InvalidFormat)
}

fn parse_nanosecond(value: &str) -> Result<u32, UtcCivilTimeError> {
    if value.is_empty() {
        return Ok(0);
    }
    if value.len() > 9 || !value.bytes().all(|byte| byte.is_ascii_digit()) {
        return Err(UtcCivilTimeError::InvalidFormat);
    }
    let mut padded = value.to_string();
    while padded.len() < 9 {
        padded.push('0');
    }
    padded.parse().map_err(|_| UtcCivilTimeError::InvalidFormat)
}

fn leap_boundary_unix_seconds(civil: UtcCivilTime) -> i64 {
    unix_seconds_from_civil(civil.year, civil.month, civil.day, civil.hour, civil.minute, 59) + 1
}

fn valid_date(year: i32, month: u8, day: u8) -> bool {
    if !(1..=12).contains(&month) {
        return false;
    }
    (1..=days_in_month(year, month)).contains(&day)
}

fn days_in_month(year: i32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

fn leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

fn unix_seconds_from_civil(year: i32, month: u8, day: u8, hour: u8, minute: u8, second: u8) -> i64 {
    days_from_civil(year, month, day) * 86_400
        + hour as i64 * 3_600
        + minute as i64 * 60
        + second as i64
}

fn civil_from_unix_seconds(unix_s: i64) -> (i32, u8, u8, u8, u8, u8) {
    let days = unix_s.div_euclid(86_400);
    let seconds_of_day = unix_s.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    let hour = (seconds_of_day / 3_600) as u8;
    let minute = ((seconds_of_day % 3_600) / 60) as u8;
    let second = (seconds_of_day % 60) as u8;
    (year, month, day, hour, minute, second)
}

fn days_from_civil(year: i32, month: u8, day: u8) -> i64 {
    let year = year as i64 - i64::from(month <= 2);
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month = month as i64;
    let shifted_month = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * shifted_month + 2) / 5 + day as i64 - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

fn civil_from_days(days: i64) -> (i32, u8, u8) {
    let days = days + 719_468;
    let era = if days >= 0 { days } else { days - 146_096 } / 146_097;
    let day_of_era = days - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1_460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_prime = (5 * day_of_year + 2) / 153;
    let day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    let month = month_prime + if month_prime < 10 { 3 } else { -9 };
    let year = year + i64::from(month <= 2);
    (year as i32, month as u8, day as u8)
}

#[cfg(test)]
mod tests {
    use super::rollover::{
        gps_week_rollover, resolve_beidou_week_rollover, resolve_galileo_week_rollover,
        resolve_glonass_day_number, resolve_gps_week_rollover, resolve_truncated_week,
        GlonassDayResolution, RolloverResolutionError,
    };
    use super::{
        beidou_to_gps_with_offset, galileo_to_gps_with_offset, glonass_to_gps_with_offset,
        glonass_to_utc_with_offset, gps_to_beidou_with_offset, gps_to_galileo_with_offset,
        gps_to_glonass_with_offset, gps_to_utc_civil_with_offset, gps_to_utc_with_offset,
        tai_to_utc_civil_with_offset, tai_to_utc_with_offset, utc_civil_to_gps_with_offset,
        utc_civil_to_tai_with_offset, utc_to_glonass_with_offset, utc_to_gps_with_offset,
        utc_to_tai_with_offset, BeidouTime, GalileoGpsTimeOffset, GalileoTime, GlonassTime,
        GlonassUtcOffset, GnssTimeSystem, TimeOffsetSource, UtcCivilTime, UtcCivilTimeError,
    };
    use bijux_gnss_core::api::{GpsTime, LeapSeconds, TaiTime, UtcTime};

    #[test]
    fn gps_week_rollover_keeps_same_cycle_when_reference_matches() {
        assert_eq!(gps_week_rollover(161, 2209), 2209);
    }

    #[test]
    fn gps_week_rollover_uses_previous_cycle_near_lower_boundary() {
        assert_eq!(gps_week_rollover(1023, 1024), 1023);
    }

    #[test]
    fn gps_week_rollover_uses_next_cycle_near_upper_boundary() {
        assert_eq!(gps_week_rollover(0, 1023), 1024);
    }

    #[test]
    fn gps_week_rollover_prefers_nearest_week_across_multiple_cycles() {
        assert_eq!(gps_week_rollover(7, 3070), 3079);
    }

    #[test]
    fn gps_week_rollover_checked_reports_resolution_evidence() {
        let resolution = resolve_gps_week_rollover(7, 3070).expect("resolved week");

        assert_eq!(resolution.truncated_week, 7);
        assert_eq!(resolution.reference_week, 3070);
        assert_eq!(resolution.cycle_weeks, 1024);
        assert_eq!(resolution.week, 3079);
        assert_eq!(resolution.distance_weeks, 9);
    }

    #[test]
    fn truncated_week_resolution_refuses_exact_half_cycle_ambiguity() {
        assert_eq!(
            resolve_gps_week_rollover(512, 1024),
            Err(RolloverResolutionError::AmbiguousReference)
        );
        assert_eq!(
            resolve_truncated_week(2048, 4096, 4096),
            Err(RolloverResolutionError::AmbiguousReference)
        );
    }

    #[test]
    fn galileo_and_beidou_week_rollover_use_declared_cycles() {
        let galileo = resolve_galileo_week_rollover(3, 4094).expect("galileo rollover");
        let beidou = resolve_beidou_week_rollover(4, 8190).expect("beidou rollover");

        assert_eq!(galileo.week, 4099);
        assert_eq!(galileo.cycle_weeks, 4096);
        assert_eq!(beidou.week, 8196);
        assert_eq!(beidou.cycle_weeks, 8192);
    }

    #[test]
    fn glonass_day_resolution_uses_reference_day_across_boundary() {
        let previous = resolve_glonass_day_number(1461, 1461).expect("previous day");
        let current = resolve_glonass_day_number(1, 1461).expect("current day");
        let next = resolve_glonass_day_number(2, 1461).expect("next day");

        assert_eq!(
            previous,
            GlonassDayResolution {
                day_number: 1461,
                reference_day_index: 1461,
                cycle_days: 1461,
                day_index: 1460,
                distance_days: -1,
            }
        );
        assert_eq!(current.day_index, 1461);
        assert_eq!(current.distance_days, 0);
        assert_eq!(next.day_index, 1462);
        assert_eq!(next.distance_days, 1);
    }

    #[test]
    fn glonass_day_resolution_rejects_invalid_day_numbers() {
        assert_eq!(
            resolve_glonass_day_number(0, 1461),
            Err(RolloverResolutionError::TruncatedValueOutOfRange)
        );
        assert_eq!(
            resolve_glonass_day_number(1462, 1461),
            Err(RolloverResolutionError::TruncatedValueOutOfRange)
        );
    }

    #[test]
    fn utc_to_gps_reports_leap_second_offset_before_and_after_boundary() {
        let leap = LeapSeconds::default_table();
        let before = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_799.0 }, &leap);
        let after = utc_to_gps_with_offset(UtcTime { unix_s: 1_483_228_800.0 }, &leap);

        assert_eq!(before.offset.from, GnssTimeSystem::Utc);
        assert_eq!(before.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(before.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(before.offset.offset_s, 17.0);
        assert_eq!(after.offset.offset_s, 18.0);
        assert!((after.time.to_seconds() - before.time.to_seconds() - 2.0).abs() < 1.0e-9);
    }

    #[test]
    fn gps_to_utc_reports_resolved_leap_second_offset() {
        let leap = LeapSeconds::default_table();
        let conversion = gps_to_utc_with_offset(GpsTime { week: 1930, tow_s: 18.0 }, &leap);

        assert_eq!(conversion.offset.from, GnssTimeSystem::Gpst);
        assert_eq!(conversion.offset.to, GnssTimeSystem::Utc);
        assert_eq!(conversion.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(conversion.offset.offset_s, -18.0);
        assert_eq!(conversion.time.unix_s, 1_483_228_800.0);
    }

    #[test]
    fn utc_tai_conversion_reports_historical_leap_table_offsets() {
        let leap = LeapSeconds::default_table();
        let utc = UtcTime { unix_s: 1_483_228_800.0 };
        let tai = utc_to_tai_with_offset(utc, &leap);
        let round_trip = tai_to_utc_with_offset(tai.time, &leap);

        assert_eq!(tai.offset.from, GnssTimeSystem::Utc);
        assert_eq!(tai.offset.to, GnssTimeSystem::Tai);
        assert_eq!(tai.offset.source, TimeOffsetSource::LeapSecondTable);
        assert_eq!(tai.offset.offset_s, 37.0);
        assert_eq!(tai.time.tai_s, 1_483_228_837.0);
        assert_eq!(round_trip.offset.offset_s, -37.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn tai_to_utc_uses_epoch_offset_at_historical_instant() {
        let leap = LeapSeconds::default_table();
        let conversion = tai_to_utc_with_offset(TaiTime { tai_s: 315_964_819.0 }, &leap);

        assert_eq!(conversion.offset.offset_s, -19.0);
        assert_eq!(conversion.time.unix_s, 315_964_800.0);
    }

    #[test]
    fn galileo_system_definition_offset_preserves_gps_time() {
        let offset = GalileoGpsTimeOffset::system_definition();
        let gps = GpsTime { week: 2_300, tow_s: 604_799.5 };
        let gst = gps_to_galileo_with_offset(gps, &offset);
        let round_trip = galileo_to_gps_with_offset(gst.time, &offset);

        assert_eq!(gst.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(gst.offset.from, GnssTimeSystem::Gpst);
        assert_eq!(gst.offset.to, GnssTimeSystem::Gst);
        assert_eq!(gst.offset.offset_s, 0.0);
        assert_eq!(gst.time, GalileoTime { week: 2_300, tow_s: 604_799.5 });
        assert_eq!(round_trip.time, gps);
    }

    #[test]
    fn galileo_broadcast_offset_crosses_week_boundary_with_evidence() {
        let offset = GalileoGpsTimeOffset::broadcast(0.75, "galileo inav word gst-gps offset");
        let gps = GpsTime { week: 2_300, tow_s: 604_799.5 };
        let gst = gps_to_galileo_with_offset(gps, &offset);
        let round_trip = galileo_to_gps_with_offset(gst.time, &offset);

        assert_eq!(gst.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
        assert_eq!(gst.offset.offset_s, 0.75);
        assert_eq!(gst.time, GalileoTime { week: 2_301, tow_s: 0.25 });
        assert_eq!(round_trip.offset.from, GnssTimeSystem::Gst);
        assert_eq!(round_trip.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(round_trip.offset.offset_s, -0.75);
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }

    #[test]
    fn beidou_epoch_maps_to_gps_week_with_fixed_offset_evidence() {
        let bdt = BeidouTime { week: 0, tow_s: 0.0 };
        let gps = beidou_to_gps_with_offset(bdt);
        let round_trip = gps_to_beidou_with_offset(gps.time);

        assert_eq!(gps.offset.from, GnssTimeSystem::Bdt);
        assert_eq!(gps.offset.to, GnssTimeSystem::Gpst);
        assert_eq!(gps.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(gps.offset.offset_s, 14.0);
        assert_eq!(gps.time, GpsTime { week: 1_356, tow_s: 14.0 });
        assert_eq!(round_trip.offset.offset_s, -14.0);
        assert_eq!(round_trip.time, bdt);
    }

    #[test]
    fn beidou_conversion_crosses_week_boundary_without_losing_epoch_offset() {
        let gps = GpsTime { week: 1_357, tow_s: 10.0 };
        let bdt = gps_to_beidou_with_offset(gps);
        let round_trip = beidou_to_gps_with_offset(bdt.time);

        assert_eq!(bdt.time, BeidouTime { week: 0, tow_s: 604_796.0 });
        assert_eq!(bdt.offset.source, TimeOffsetSource::FixedSystemDefinition);
        assert_eq!(bdt.offset.offset_s, -14.0);
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }

    #[test]
    fn glonass_utc_conversion_reports_broadcast_utc_relation() {
        let offset = GlonassUtcOffset::broadcast(0.0, "glonass tau_c broadcast value");
        let utc = UtcTime { unix_s: 1_483_228_800.0 };
        let glonass = utc_to_glonass_with_offset(utc, &offset);
        let round_trip = glonass_to_utc_with_offset(glonass.time, &offset);

        assert_eq!(glonass.offset.from, GnssTimeSystem::Utc);
        assert_eq!(glonass.offset.to, GnssTimeSystem::Glonass);
        assert_eq!(glonass.offset.source, TimeOffsetSource::BroadcastNavigationMessage);
        assert_eq!(glonass.offset.offset_s, 10_800.0);
        assert_eq!(glonass.time.seconds_of_day, 10_800.0);
        assert_eq!(round_trip.offset.offset_s, -10_800.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn glonass_conversion_crosses_moscow_day_boundary() {
        let offset = GlonassUtcOffset::broadcast(0.0, "glonass tau_c broadcast value");
        let utc = UtcTime { unix_s: 1_483_308_000.0 };
        let glonass = utc_to_glonass_with_offset(utc, &offset);
        let round_trip = glonass_to_utc_with_offset(glonass.time, &offset);

        assert_eq!(glonass.time.seconds_of_day, 3_600.0);
        assert_eq!(round_trip.time, utc);
    }

    #[test]
    fn gps_glonass_conversion_combines_leap_and_utc_relation_evidence() {
        let leap = LeapSeconds::default_table();
        let offset = GlonassUtcOffset::broadcast(0.25, "glonass utc su relation");
        let gps = GpsTime { week: 1_930, tow_s: 18.0 };
        let glonass = gps_to_glonass_with_offset(gps, &leap, &offset);
        let round_trip = glonass_to_gps_with_offset(glonass.time, &leap, &offset);

        assert_eq!(glonass.offset.source, TimeOffsetSource::CompositeOffsetModel);
        assert_eq!(glonass.offset.offset_s, 10_782.25);
        assert_eq!(glonass.time, GlonassTime { day_index: 17_167, seconds_of_day: 10_800.25 });
        assert!((round_trip.time.to_seconds() - gps.to_seconds()).abs() < 1.0e-9);
    }

    #[test]
    fn utc_civil_time_parses_and_formats_declared_leap_second() {
        let leap = LeapSeconds::default_table();
        let civil = UtcCivilTime::parse("2016-12-31T23:59:60Z").expect("leap second parses");

        civil.validate_leap_second(&leap).expect("declared leap second");
        assert_eq!(civil.second, 60);
        assert_eq!(civil.format_utc(), "2016-12-31T23:59:60Z");
    }

    #[test]
    fn utc_civil_time_rejects_undeclared_leap_second() {
        let leap = LeapSeconds::default_table();
        let civil = UtcCivilTime::parse("2016-12-30T23:59:60Z").expect("syntax parses");

        assert_eq!(
            civil.validate_leap_second(&leap),
            Err(UtcCivilTimeError::LeapSecondNotDeclared)
        );
    }

    #[test]
    fn utc_civil_time_orders_inserted_second_before_following_midnight() {
        let before = UtcCivilTime::parse("2016-12-31T23:59:59Z").expect("before");
        let inserted = UtcCivilTime::parse("2016-12-31T23:59:60Z").expect("inserted");
        let after = UtcCivilTime::parse("2017-01-01T00:00:00Z").expect("after");

        assert!(before < inserted);
        assert!(inserted < after);
    }

    #[test]
    fn utc_civil_time_preserves_fractional_ordinary_seconds() {
        let civil = UtcCivilTime::parse("2026-07-15T12:34:56.125Z").expect("fractional time");
        let utc = civil.to_utc_time().expect("ordinary utc");
        let formatted = UtcCivilTime::from_utc_time(utc).format_utc();

        assert_eq!(civil.nanosecond, 125_000_000);
        assert_eq!(formatted, "2026-07-15T12:34:56.125Z");
    }

    #[test]
    fn utc_civil_leap_second_maps_to_unique_gps_second() {
        let leap = LeapSeconds::default_table();
        let before = utc_civil_to_gps_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:59Z").expect("before"),
            &leap,
        )
        .expect("before gps");
        let inserted = utc_civil_to_gps_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:60Z").expect("inserted"),
            &leap,
        )
        .expect("inserted gps");
        let after = utc_civil_to_gps_with_offset(
            UtcCivilTime::parse("2017-01-01T00:00:00Z").expect("after"),
            &leap,
        )
        .expect("after gps");

        assert_eq!(inserted.offset.offset_s, 17.0);
        assert!((inserted.time.to_seconds() - before.time.to_seconds() - 1.0).abs() < 1.0e-9);
        assert!((after.time.to_seconds() - inserted.time.to_seconds() - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn gps_time_inside_leap_second_formats_as_inserted_utc_second() {
        let leap = LeapSeconds::default_table();
        let inserted = utc_civil_to_gps_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:60.5Z").expect("inserted"),
            &leap,
        )
        .expect("inserted gps");
        let round_trip = gps_to_utc_civil_with_offset(inserted.time, &leap);
        let after = gps_to_utc_civil_with_offset(GpsTime { week: 1930, tow_s: 18.0 }, &leap);

        assert_eq!(round_trip.time.format_utc(), "2016-12-31T23:59:60.5Z");
        assert_eq!(round_trip.offset.offset_s, -17.0);
        assert_eq!(after.time.format_utc(), "2017-01-01T00:00:00Z");
    }

    #[test]
    fn utc_civil_leap_second_maps_to_unique_tai_second() {
        let leap = LeapSeconds::default_table();
        let before = utc_civil_to_tai_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:59Z").expect("before"),
            &leap,
        )
        .expect("before tai");
        let inserted = utc_civil_to_tai_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:60Z").expect("inserted"),
            &leap,
        )
        .expect("inserted tai");
        let after = utc_civil_to_tai_with_offset(
            UtcCivilTime::parse("2017-01-01T00:00:00Z").expect("after"),
            &leap,
        )
        .expect("after tai");

        assert_eq!(inserted.offset.offset_s, 36.0);
        assert!((inserted.time.tai_s - before.time.tai_s - 1.0).abs() < 1.0e-9);
        assert!((after.time.tai_s - inserted.time.tai_s - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn tai_time_inside_leap_second_formats_as_inserted_utc_second() {
        let leap = LeapSeconds::default_table();
        let inserted = utc_civil_to_tai_with_offset(
            UtcCivilTime::parse("2016-12-31T23:59:60.25Z").expect("inserted"),
            &leap,
        )
        .expect("inserted tai");
        let round_trip = tai_to_utc_civil_with_offset(inserted.time, &leap);
        let after = tai_to_utc_civil_with_offset(TaiTime { tai_s: 1_483_228_837.0 }, &leap);

        assert_eq!(round_trip.time.format_utc(), "2016-12-31T23:59:60.25Z");
        assert_eq!(round_trip.offset.offset_s, -36.0);
        assert_eq!(after.time.format_utc(), "2017-01-01T00:00:00Z");
    }
}
