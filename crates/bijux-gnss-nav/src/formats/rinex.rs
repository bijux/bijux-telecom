#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::OnceLock;

use bijux_gnss_core::api::{
    gps_to_utc, Constellation, GlonassSlot, GpsTime, IoError, LeapSeconds, ObsEpoch, ObsSatellite,
    ParseError, SatId, SignalBand, SignalCode,
};
use regex::Regex;

use crate::formats::rinex_obs::{
    format_rinex_observation_dataset, RinexObservationDataset, RinexObservationEpoch,
    RinexObservationEpochTime, RinexObservationRecord, RinexObservationTimeSystem,
    RinexObservationValue, RinexSatelliteObservation,
};
use crate::models::atmosphere::KlobucharCoefficients;
use crate::orbits::beidou::{
    BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
    BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
};
use crate::orbits::galileo::{
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime,
};
use crate::orbits::glonass::{
    GlonassBroadcastNavigationFrame, GlonassFrameTime, GlonassImmediateHealth,
    GlonassImmediateNavigationData, GlonassSatelliteType, GlonassStateVector, GlonassSystemTime,
};
use crate::orbits::gps::GpsBroadcastNavigationData;
use crate::orbits::gps::GpsEphemeris;
use crate::time::{
    beidou_to_gps_with_offset, galileo_to_gps_with_offset, gps_to_beidou_with_offset,
    utc_civil_to_gps_with_offset, BeidouTime, GalileoGpsTimeOffset, GalileoTime, UtcCivilTime,
    UtcCivilTimeError,
};

const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;

fn push_rinex_line(output: &mut String, line: &str) {
    if line.len() > 80 {
        output.push_str(&line[..80]);
    } else {
        output.push_str(line);
    }
    output.push('\n');
}

#[derive(Debug, Clone, PartialEq)]
struct RinexNavHeader {
    version: f64,
    is_mixed: bool,
    klobuchar: Option<KlobucharCoefficients>,
    time_system_corrections: Vec<RinexNavigationTimeSystemCorrection>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexNavigationTimeSystemCorrection {
    pub code: String,
    pub a0_s: f64,
    pub a1_s_per_s: f64,
    pub reference_time_s: u32,
    pub reference_week: u32,
    pub provider: Option<String>,
    pub utc_id: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RinexBroadcastNavigationDataset {
    pub version: f64,
    pub klobuchar: Option<KlobucharCoefficients>,
    pub time_system_corrections: Vec<RinexNavigationTimeSystemCorrection>,
    pub gps: Vec<GpsEphemeris>,
    pub galileo: Vec<GalileoBroadcastNavigationData>,
    pub beidou: Vec<BeidouBroadcastNavigationData>,
    pub glonass: Vec<GlonassBroadcastNavigationFrame>,
}

fn rinex_nav_float_regex() -> &'static Regex {
    static REGEX: OnceLock<Regex> = OnceLock::new();
    REGEX.get_or_init(|| {
        Regex::new(r"[+-]?\d+(?:\.\d*)?(?:[DdEe][+-]?\d+)?").expect("valid RINEX NAV float regex")
    })
}

fn parse_rinex_float(field: &str) -> Result<f64, ParseError> {
    field
        .trim()
        .replace(['D', 'd'], "E")
        .parse::<f64>()
        .map_err(|err| ParseError { message: format!("invalid RINEX NAV float '{field}': {err}") })
}

fn parse_rinex_numeric_fields(line: &str) -> Result<Vec<f64>, ParseError> {
    rinex_nav_float_regex().find_iter(line).map(|field| parse_rinex_float(field.as_str())).collect()
}

fn parse_rinex_nav_header(data: &str) -> Result<(RinexNavHeader, usize), ParseError> {
    let mut has_type = false;
    let mut has_end = false;
    let mut header = None;
    let mut header_line_count = 0usize;
    let mut klobuchar_alpha = None;
    let mut klobuchar_beta = None;

    for line in data.lines() {
        header_line_count += 1;
        if header.is_none() && line.contains("RINEX VERSION / TYPE") {
            let version =
                line.get(..9).unwrap_or_default().trim().parse::<f64>().map_err(|err| {
                    ParseError {
                        message: format!(
                            "invalid RINEX NAV version '{}': {err}",
                            &line[..9.min(line.len())]
                        ),
                    }
                })?;
            has_type = line.contains("NAVIGATION DATA") || line.contains("NAV DATA");
            header = Some(RinexNavHeader {
                version,
                is_mixed: line.contains("M (MIXED)"),
                klobuchar: None,
                time_system_corrections: Vec::new(),
            });
        }
        if line.contains("ION ALPHA") {
            klobuchar_alpha = Some(parse_rinex_klobuchar_header_fields(line, "ION ALPHA")?);
        } else if line.contains("ION BETA") {
            klobuchar_beta = Some(parse_rinex_klobuchar_header_fields(line, "ION BETA")?);
        } else if line.contains("IONOSPHERIC CORR") {
            match line.get(..4).unwrap_or_default().trim() {
                "GPSA" => {
                    klobuchar_alpha =
                        Some(parse_rinex_klobuchar_header_fields(line, "GPSA IONOSPHERIC CORR")?);
                }
                "GPSB" => {
                    klobuchar_beta =
                        Some(parse_rinex_klobuchar_header_fields(line, "GPSB IONOSPHERIC CORR")?);
                }
                _ => {}
            }
        } else if line.contains("TIME SYSTEM CORR") {
            let correction = parse_rinex_time_system_correction(line)?;
            header.get_or_insert(RinexNavHeader {
                version: 0.0,
                is_mixed: false,
                klobuchar: None,
                time_system_corrections: Vec::new(),
            });
            if let Some(header) = header.as_mut() {
                header.time_system_corrections.push(correction);
            }
        }
        if line.contains("END OF HEADER") {
            has_end = true;
            break;
        }
    }

    match (header, has_type, has_end) {
        (Some(mut header), true, true) => {
            header.klobuchar = match (klobuchar_alpha, klobuchar_beta) {
                (Some(alpha), Some(beta)) => Some(KlobucharCoefficients::new(alpha, beta)),
                (None, None) => None,
                _ => {
                    return Err(ParseError {
                        message: "incomplete GPS Klobuchar coefficients in RINEX NAV header"
                            .to_string(),
                    });
                }
            };
            Ok((header, header_line_count))
        }
        _ => Err(ParseError { message: "invalid or incomplete RINEX NAV header".to_string() }),
    }
}

fn parse_rinex_time_system_correction(
    line: &str,
) -> Result<RinexNavigationTimeSystemCorrection, ParseError> {
    let content = line.get(..60).unwrap_or_default();
    let code = content.get(..4).unwrap_or_default().trim().to_string();
    if code.is_empty() {
        return Err(ParseError { message: "RINEX TIME SYSTEM CORR is missing code".to_string() });
    }
    let numeric_fields = parse_rinex_numeric_fields(content.get(4..).unwrap_or_default())?;
    if numeric_fields.len() < 4 {
        return Err(ParseError {
            message: format!(
                "RINEX TIME SYSTEM CORR {code} requires code, a0, a1, reference time, and reference week"
            ),
        });
    }
    let text_fields = content
        .split_whitespace()
        .skip(1)
        .filter(|field| field.chars().all(|char| char.is_ascii_alphabetic()))
        .collect::<Vec<_>>();
    Ok(RinexNavigationTimeSystemCorrection {
        code,
        a0_s: numeric_fields[0],
        a1_s_per_s: numeric_fields[1],
        reference_time_s: numeric_fields[2].round().max(0.0) as u32,
        reference_week: numeric_fields[3].round().max(0.0) as u32,
        provider: text_fields.first().map(|value| (*value).to_string()),
        utc_id: text_fields.get(1).map(|value| (*value).to_string()),
    })
}

fn parse_rinex_klobuchar_header_fields(line: &str, label: &str) -> Result<[f64; 4], ParseError> {
    let fields = parse_rinex_numeric_fields(line)?;
    if fields.len() < 4 {
        return Err(ParseError {
            message: format!("{label} requires 4 numeric fields, found {}", fields.len()),
        });
    }
    Ok([fields[0], fields[1], fields[2], fields[3]])
}

fn pad_rinex_nav_record_fields(mut fields: Vec<f64>) -> [f64; 4] {
    fields.resize(4, 0.0);
    [fields[0], fields[1], fields[2], fields[3]]
}

#[cfg(test)]
fn parse_rinex_epoch_utc(
    year: i32,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: f64,
) -> Result<bijux_gnss_core::api::UtcTime, ParseError> {
    let civil = parse_rinex_epoch_utc_civil(year, month, day, hour, minute, second)?;
    civil.to_utc_time().map_err(rinex_utc_civil_error)
}

fn parse_rinex_epoch_utc_civil(
    year: i32,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: f64,
) -> Result<UtcCivilTime, ParseError> {
    if !second.is_finite() {
        return Err(ParseError { message: "RINEX NAV seconds must be finite".to_string() });
    }
    let whole_seconds = second.floor();
    if !(0.0..=60.0).contains(&whole_seconds) {
        return Err(ParseError { message: format!("invalid RINEX NAV second {second}") });
    }
    let mut whole_seconds = whole_seconds as u8;
    let mut nanos = ((second - whole_seconds as f64) * 1_000_000_000.0).round() as u32;
    if nanos == 1_000_000_000 {
        whole_seconds = whole_seconds.saturating_add(1);
        nanos = 0;
    }
    let civil =
        UtcCivilTime { year, month, day, hour, minute, second: whole_seconds, nanosecond: nanos };
    civil.validate_leap_second(&LeapSeconds::default_table()).map_err(rinex_utc_civil_error)?;
    Ok(civil)
}

fn rinex_utc_civil_error(err: UtcCivilTimeError) -> ParseError {
    ParseError { message: format!("invalid RINEX NAV UTC epoch: {err:?}") }
}

fn civil_day_of_year(year: i32, month: u8, day: u8) -> Result<u16, ParseError> {
    if !(1..=12).contains(&month) {
        return Err(ParseError { message: format!("invalid RINEX NAV month {month}") });
    }
    let month_lengths = [
        31,
        if is_gregorian_leap_year(year) { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let month_length = month_lengths[usize::from(month - 1)];
    if day == 0 || day > month_length {
        return Err(ParseError { message: format!("invalid RINEX NAV day {day}") });
    }
    let elapsed_days =
        month_lengths[..usize::from(month - 1)].iter().map(|days| u16::from(*days)).sum::<u16>();
    Ok(elapsed_days + u16::from(day))
}

fn is_gregorian_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

fn format_rinex_nav_float(value: f64) -> String {
    format!("{value:>19.12E}").replace('E', "D")
}

fn format_rinex_header_float(value: f64) -> String {
    format!("{value:>12.4E}").replace('E', "D")
}

fn format_rinex_klobuchar_header_line(prefix: &str, coefficients: [f64; 4]) -> String {
    format!(
        "{prefix:<4}{}{}{}{}       IONOSPHERIC CORR",
        format_rinex_header_float(coefficients[0]),
        format_rinex_header_float(coefficients[1]),
        format_rinex_header_float(coefficients[2]),
        format_rinex_header_float(coefficients[3]),
    )
}

fn format_rinex_time_system_correction_header_line(
    correction: &RinexNavigationTimeSystemCorrection,
) -> String {
    format!(
        "{:<4}{}{}{:>7}{:>5} {:>3} {:>3}   TIME SYSTEM CORR",
        correction.code,
        format!("{:>17.10E}", correction.a0_s).replace('E', "D"),
        format!("{:>16.9E}", correction.a1_s_per_s).replace('E', "D"),
        correction.reference_time_s,
        correction.reference_week,
        correction.provider.as_deref().unwrap_or(""),
        correction.utc_id.as_deref().unwrap_or(""),
    )
}

fn gps_time_to_utc_datetime(gps_time: GpsTime) -> Result<time::OffsetDateTime, IoError> {
    let utc = gps_to_utc(gps_time, &LeapSeconds::default_table());
    time::OffsetDateTime::from_unix_timestamp_nanos((utc.unix_s * 1_000_000_000.0).round() as i128)
        .map_err(|err| IoError { message: format!("invalid GPS-to-UTC conversion: {err}") })
}

fn format_gps_rinex_nav_record(eph: &GpsEphemeris) -> Result<Vec<String>, IoError> {
    let utc = gps_time_to_utc_datetime(GpsTime { week: eph.week, tow_s: eph.toc_s })?;
    Ok(vec![
        format!(
            "G{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            eph.sat.prn,
            utc.year(),
            u8::from(utc.month()),
            utc.day(),
            utc.hour(),
            utc.minute(),
            utc.second(),
            format_rinex_nav_float(eph.af0),
            format_rinex_nav_float(eph.af1),
            format_rinex_nav_float(eph.af2),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.iode as f64),
            format_rinex_nav_float(eph.crs),
            format_rinex_nav_float(eph.delta_n),
            format_rinex_nav_float(eph.m0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.cuc),
            format_rinex_nav_float(eph.e),
            format_rinex_nav_float(eph.cus),
            format_rinex_nav_float(eph.sqrt_a),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.toe_s),
            format_rinex_nav_float(eph.cic),
            format_rinex_nav_float(eph.omega0),
            format_rinex_nav_float(eph.cis),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.i0),
            format_rinex_nav_float(eph.crc),
            format_rinex_nav_float(eph.w),
            format_rinex_nav_float(eph.omegadot),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.idot),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(eph.week as f64),
            format_rinex_nav_float(0.0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.sv_accuracy.map(f64::from).unwrap_or(0.0)),
            format_rinex_nav_float(eph.sv_health as f64),
            format_rinex_nav_float(eph.tgd),
            format_rinex_nav_float(eph.iodc as f64),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(eph.toe_s),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
        ),
    ])
}

fn format_galileo_rinex_nav_record(
    nav: &GalileoBroadcastNavigationData,
) -> Result<Vec<String>, IoError> {
    let gps = galileo_to_gps_with_offset(
        GalileoTime { week: u32::from(nav.gst.week), tow_s: nav.clock.t0c_s },
        &GalileoGpsTimeOffset::system_definition(),
    )
    .time;
    let utc = gps_time_to_utc_datetime(gps)?;
    Ok(vec![
        format!(
            "E{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            nav.sat.prn,
            utc.year(),
            u8::from(utc.month()),
            utc.day(),
            utc.hour(),
            utc.minute(),
            utc.second(),
            format_rinex_nav_float(nav.clock.af0),
            format_rinex_nav_float(nav.clock.af1),
            format_rinex_nav_float(nav.clock.af2),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(f64::from(nav.iodnav)),
            format_rinex_nav_float(nav.ephemeris.crs),
            format_rinex_nav_float(nav.ephemeris.delta_n),
            format_rinex_nav_float(nav.ephemeris.m0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.cuc),
            format_rinex_nav_float(nav.ephemeris.e),
            format_rinex_nav_float(nav.ephemeris.cus),
            format_rinex_nav_float(nav.ephemeris.sqrt_a),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.toe_s),
            format_rinex_nav_float(nav.ephemeris.cic),
            format_rinex_nav_float(nav.ephemeris.omega0),
            format_rinex_nav_float(nav.ephemeris.cis),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.i0),
            format_rinex_nav_float(nav.ephemeris.crc),
            format_rinex_nav_float(nav.ephemeris.w),
            format_rinex_nav_float(nav.ephemeris.omegadot),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.idot),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(f64::from(nav.gst.week)),
            format_rinex_nav_float(f64::from(nav.sisa_e1_e5b)),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(galileo_health_word(nav) as f64),
            format_rinex_nav_float(nav.clock.bgd_e1_e5a_s),
            format_rinex_nav_float(nav.clock.bgd_e1_e5b_s),
            format_rinex_nav_float(f64::from(nav.gst.tow_s)),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
        ),
    ])
}

fn galileo_health_word(nav: &GalileoBroadcastNavigationData) -> u8 {
    nav.signal_health.e5b_signal_health
        | (nav.signal_health.e1b_signal_health << 2)
        | (u8::from(!nav.signal_health.e5b_data_valid) << 4)
        | (u8::from(!nav.signal_health.e1b_data_valid) << 5)
}

fn format_beidou_rinex_nav_record(
    nav: &BeidouBroadcastNavigationData,
) -> Result<Vec<String>, IoError> {
    let gps = beidou_to_gps_with_offset(BeidouTime {
        week: u32::from(nav.bdt.week),
        tow_s: nav.clock.toc_s,
    })
    .time;
    let utc = gps_time_to_utc_datetime(gps)?;
    Ok(vec![
        format!(
            "C{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            nav.sat.prn,
            utc.year(),
            u8::from(utc.month()),
            utc.day(),
            utc.hour(),
            utc.minute(),
            utc.second(),
            format_rinex_nav_float(nav.clock.af0),
            format_rinex_nav_float(nav.clock.af1),
            format_rinex_nav_float(nav.clock.af2),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(f64::from(nav.ephemeris.aode)),
            format_rinex_nav_float(nav.ephemeris.crs),
            format_rinex_nav_float(nav.ephemeris.delta_n),
            format_rinex_nav_float(nav.ephemeris.m0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.cuc),
            format_rinex_nav_float(nav.ephemeris.e),
            format_rinex_nav_float(nav.ephemeris.cus),
            format_rinex_nav_float(nav.ephemeris.sqrt_a),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.toe_s),
            format_rinex_nav_float(nav.ephemeris.cic),
            format_rinex_nav_float(nav.ephemeris.omega0),
            format_rinex_nav_float(nav.ephemeris.cis),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.i0),
            format_rinex_nav_float(nav.ephemeris.crc),
            format_rinex_nav_float(nav.ephemeris.w),
            format_rinex_nav_float(nav.ephemeris.omegadot),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(nav.ephemeris.idot),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(f64::from(nav.bdt.week)),
            format_rinex_nav_float(0.0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(f64::from(nav.urai)),
            format_rinex_nav_float(f64::from(u8::from(
                !nav.signal_health.autonomous_satellite_good,
            ))),
            format_rinex_nav_float(nav.clock.tgd1_s),
            format_rinex_nav_float(nav.clock.tgd2_s),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(f64::from(nav.bdt.sow_s)),
            format_rinex_nav_float(f64::from(nav.clock.aodc)),
            format_rinex_nav_float(0.0),
            format_rinex_nav_float(0.0),
        ),
    ])
}

fn format_glonass_rinex_nav_record(
    nav: &GlonassBroadcastNavigationFrame,
) -> Result<Vec<String>, IoError> {
    let date = glonass_reference_date(nav.immediate.system_time.map(|time| time.day_number))?;
    let second = if nav.immediate.frame_time.half_minute { 30 } else { 0 };
    let state = nav.immediate.state_vector;
    Ok(vec![
        format!(
            "R{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            nav.sat.prn,
            date.year(),
            u8::from(date.month()),
            date.day(),
            nav.immediate.frame_time.hour,
            nav.immediate.frame_time.minute,
            second,
            format_rinex_nav_float(-nav.immediate.clock_bias_s),
            format_rinex_nav_float(nav.immediate.relative_frequency_bias),
            format_rinex_nav_float(f64::from(nav.immediate.ephemeris_reference_time_s)),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(state.x_m / 1_000.0),
            format_rinex_nav_float(state.vx_mps / 1_000.0),
            format_rinex_nav_float(state.ax_mps2 / 1_000.0),
            format_rinex_nav_float(f64::from(nav.immediate.health.status_code)),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(state.y_m / 1_000.0),
            format_rinex_nav_float(state.vy_mps / 1_000.0),
            format_rinex_nav_float(state.ay_mps2 / 1_000.0),
            format_rinex_nav_float(0.0),
        ),
        format!(
            "    {}{}{}{}",
            format_rinex_nav_float(state.z_m / 1_000.0),
            format_rinex_nav_float(state.vz_mps / 1_000.0),
            format_rinex_nav_float(state.az_mps2 / 1_000.0),
            format_rinex_nav_float(f64::from(nav.immediate.immediate_data_age_days)),
        ),
    ])
}

fn glonass_reference_date(day_number: Option<u16>) -> Result<time::Date, IoError> {
    let day_number = day_number.unwrap_or(1);
    let year = if day_number <= 365 { 2022 } else { 2020 };
    time::Date::from_ordinal_date(year, day_number).map_err(|err| IoError {
        message: format!("invalid GLONASS day number {day_number}: {err}"),
    })
}

fn parse_rinex_nav_year(year: i32, version: f64) -> i32 {
    if version >= 3.0 || year >= 100 {
        year
    } else if year >= 80 {
        1900 + year
    } else {
        2000 + year
    }
}

fn rinex_nav_record_line_count(version: f64, line: &str) -> Result<usize, ParseError> {
    if version < 3.0 {
        return Ok(8);
    }
    let system = line
        .chars()
        .next()
        .ok_or_else(|| ParseError { message: "empty RINEX NAV record line".to_string() })?;
    match system {
        'G' | 'E' | 'C' | 'J' | 'I' | 'S' => Ok(8),
        'R' => Ok(4),
        _ => Err(ParseError { message: format!("unsupported RINEX NAV constellation '{system}'") }),
    }
}

fn parse_gps_rinex_nav_record(
    header: &RinexNavHeader,
    lines: &[&str],
) -> Result<GpsEphemeris, ParseError> {
    if lines.len() != 8 {
        return Err(ParseError {
            message: format!("GPS RINEX NAV record requires 8 lines, found {}", lines.len()),
        });
    }

    let first_line = lines[0];
    let (sat, first_line_payload) = if header.version >= 3.0 {
        let sat = first_line
            .get(..3)
            .unwrap_or_default()
            .trim()
            .strip_prefix('G')
            .ok_or_else(|| ParseError {
                message: format!(
                    "unsupported RINEX NAV satellite identifier '{}'",
                    &first_line[..3.min(first_line.len())]
                ),
            })?
            .parse::<u8>()
            .map_err(|err| ParseError {
                message: format!(
                    "invalid GPS satellite identifier '{}': {err}",
                    &first_line[..3.min(first_line.len())]
                ),
            })?;
        (sat, first_line.get(3..).unwrap_or_default())
    } else {
        let sat = first_line.get(..2).unwrap_or_default().trim().parse::<u8>().map_err(|err| {
            ParseError {
                message: format!(
                    "invalid GPS satellite identifier '{}': {err}",
                    &first_line[..2.min(first_line.len())]
                ),
            }
        })?;
        (sat, first_line.get(2..).unwrap_or_default())
    };

    let line1 = parse_rinex_numeric_fields(first_line_payload)?;
    if line1.len() < 9 {
        return Err(ParseError {
            message: format!(
                "GPS RINEX NAV first line requires 9 numeric fields, found {}",
                line1.len()
            ),
        });
    }
    let line2 = parse_rinex_numeric_fields(lines[1])?;
    let line3 = parse_rinex_numeric_fields(lines[2])?;
    let line4 = parse_rinex_numeric_fields(lines[3])?;
    let line5 = parse_rinex_numeric_fields(lines[4])?;
    let line6 = parse_rinex_numeric_fields(lines[5])?;
    let line7 = parse_rinex_numeric_fields(lines[6])?;

    for (line_index, fields, min_required) in [
        (2, &line2, 4usize),
        (3, &line3, 4usize),
        (4, &line4, 4usize),
        (5, &line5, 4usize),
        (6, &line6, 4usize),
        (7, &line7, 2usize),
    ] {
        if fields.len() < min_required {
            return Err(ParseError {
                message: format!(
                    "GPS RINEX NAV line {line_index} requires at least {min_required} numeric fields, found {}",
                    fields.len()
                ),
            });
        }
    }
    let line2 = pad_rinex_nav_record_fields(line2);
    let line3 = pad_rinex_nav_record_fields(line3);
    let line4 = pad_rinex_nav_record_fields(line4);
    let line5 = pad_rinex_nav_record_fields(line5);
    let line6 = pad_rinex_nav_record_fields(line6);
    let line7 = pad_rinex_nav_record_fields(line7);

    let year = parse_rinex_nav_year(line1[0] as i32, header.version);
    let utc = parse_rinex_epoch_utc_civil(
        year,
        line1[1] as u8,
        line1[2] as u8,
        line1[3] as u8,
        line1[4] as u8,
        line1[5],
    )?;
    let toc = utc_civil_to_gps_with_offset(utc, &LeapSeconds::default_table())
        .map_err(rinex_utc_civil_error)?
        .time;
    let week = line6[2].round().max(0.0) as u32;

    Ok(GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: sat },
        iodc: line7[3].round().max(0.0) as u16,
        iode: line2[0].round().max(0.0) as u8,
        week: if week == 0 { toc.week } else { week },
        sv_health: line7[1].round().max(0.0) as u8,
        sv_accuracy: Some(line7[0].round().max(0.0) as u8),
        toe_s: line4[0].rem_euclid(604_800.0),
        toc_s: toc.tow_s,
        sqrt_a: line3[3],
        e: line3[1],
        i0: line5[0],
        idot: line6[0],
        omega0: line4[2],
        omegadot: line5[3],
        w: line5[2],
        m0: line2[3],
        delta_n: line2[2],
        cuc: line3[0],
        cus: line3[2],
        crc: line5[1],
        crs: line2[1],
        cic: line4[1],
        cis: line4[3],
        af0: line1[6],
        af1: line1[7],
        af2: line1[8],
        tgd: line7[2],
    })
}

fn parse_galileo_rinex_nav_record(
    header: &RinexNavHeader,
    lines: &[&str],
) -> Result<GalileoBroadcastNavigationData, ParseError> {
    if lines.len() != 8 {
        return Err(ParseError {
            message: format!("Galileo RINEX NAV record requires 8 lines, found {}", lines.len()),
        });
    }

    let first_line = lines[0];
    let prn = first_line
        .get(..3)
        .unwrap_or_default()
        .trim()
        .strip_prefix('E')
        .ok_or_else(|| ParseError {
            message: format!(
                "unsupported Galileo RINEX NAV satellite identifier '{}'",
                &first_line[..3.min(first_line.len())]
            ),
        })?
        .parse::<u8>()
        .map_err(|err| ParseError {
            message: format!(
                "invalid Galileo satellite identifier '{}': {err}",
                &first_line[..3.min(first_line.len())]
            ),
        })?;
    let line1 = parse_rinex_numeric_fields(first_line.get(3..).unwrap_or_default())?;
    if line1.len() < 9 {
        return Err(ParseError {
            message: format!(
                "Galileo RINEX NAV first line requires 9 numeric fields, found {}",
                line1.len()
            ),
        });
    }
    let line2 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[1])?);
    let line3 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[2])?);
    let line4 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[3])?);
    let line5 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[4])?);
    let line6 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[5])?);
    let line7 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[6])?);

    let year = parse_rinex_nav_year(line1[0] as i32, header.version);
    let toc = parse_rinex_epoch_utc_civil(
        year,
        line1[1] as u8,
        line1[2] as u8,
        line1[3] as u8,
        line1[4] as u8,
        line1[5],
    )?;
    let toc_gps = utc_civil_to_gps_with_offset(toc, &LeapSeconds::default_table())
        .map_err(rinex_utc_civil_error)?
        .time;
    let sat = SatId { constellation: Constellation::Galileo, prn };
    let iodnav = line2[0].round().max(0.0) as u16;
    let health = line7[0].round().max(0.0) as u8;

    Ok(GalileoBroadcastNavigationData {
        sat,
        iodnav,
        gst: GalileoSystemTime {
            week: line6[2].round().max(0.0) as u16,
            tow_s: line7[3].round().max(0.0) as u32,
        },
        sisa_e1_e5b: line6[3].round().max(0.0) as u8,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: health & 0b11,
            e1b_signal_health: (health >> 2) & 0b11,
            e5b_data_valid: (health & 0b0001_0000) == 0,
            e1b_data_valid: (health & 0b0010_0000) == 0,
        },
        clock: GalileoClockCorrection {
            t0c_s: toc_gps.tow_s,
            af0: line1[6],
            af1: line1[7],
            af2: line1[8],
            bgd_e1_e5a_s: line7[1],
            bgd_e1_e5b_s: line7[2],
        },
        ephemeris: GalileoEphemeris {
            sat,
            iodnav,
            toe_s: line4[0].rem_euclid(604_800.0),
            sqrt_a: line3[3],
            e: line3[1],
            i0: line5[0],
            idot: line6[0],
            omega0: line4[2],
            omegadot: line5[3],
            w: line5[2],
            m0: line2[3],
            delta_n: line2[2],
            cuc: line3[0],
            cus: line3[2],
            crc: line5[1],
            crs: line2[1],
            cic: line4[1],
            cis: line4[3],
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 0.0,
            ai1: 0.0,
            ai2: 0.0,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    })
}

fn parse_beidou_rinex_nav_record(
    header: &RinexNavHeader,
    lines: &[&str],
) -> Result<BeidouBroadcastNavigationData, ParseError> {
    if lines.len() != 8 {
        return Err(ParseError {
            message: format!("BeiDou RINEX NAV record requires 8 lines, found {}", lines.len()),
        });
    }

    let first_line = lines[0];
    let prn = first_line
        .get(..3)
        .unwrap_or_default()
        .trim()
        .strip_prefix('C')
        .ok_or_else(|| ParseError {
            message: format!(
                "unsupported BeiDou RINEX NAV satellite identifier '{}'",
                &first_line[..3.min(first_line.len())]
            ),
        })?
        .parse::<u8>()
        .map_err(|err| ParseError {
            message: format!(
                "invalid BeiDou satellite identifier '{}': {err}",
                &first_line[..3.min(first_line.len())]
            ),
        })?;
    let line1 = parse_rinex_numeric_fields(first_line.get(3..).unwrap_or_default())?;
    if line1.len() < 9 {
        return Err(ParseError {
            message: format!(
                "BeiDou RINEX NAV first line requires 9 numeric fields, found {}",
                line1.len()
            ),
        });
    }
    let line2 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[1])?);
    let line3 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[2])?);
    let line4 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[3])?);
    let line5 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[4])?);
    let line6 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[5])?);
    let line7 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[6])?);
    let line8 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[7])?);

    let year = parse_rinex_nav_year(line1[0] as i32, header.version);
    let toc = parse_rinex_epoch_utc_civil(
        year,
        line1[1] as u8,
        line1[2] as u8,
        line1[3] as u8,
        line1[4] as u8,
        line1[5],
    )?;
    let toc_gps = utc_civil_to_gps_with_offset(toc, &LeapSeconds::default_table())
        .map_err(rinex_utc_civil_error)?
        .time;
    let toc_bdt = gps_to_beidou_with_offset(toc_gps).time;
    let sat = SatId { constellation: Constellation::Beidou, prn };
    let aode = line2[0].round().max(0.0) as u8;
    let transmit_time_s = line8[0].rem_euclid(604_800.0);

    Ok(BeidouBroadcastNavigationData {
        sat,
        bdt: BeidouSystemTime {
            week: line6[2].round().max(0.0) as u16,
            sow_s: transmit_time_s.round().max(0.0) as u32,
        },
        urai: line7[0].round().max(0.0) as u8,
        signal_health: BeidouSignalHealth { autonomous_satellite_good: line7[1].round() == 0.0 },
        clock: BeidouClockCorrection {
            toc_s: toc_bdt.tow_s,
            aodc: line8[1].round().max(0.0) as u8,
            af0: line1[6],
            af1: line1[7],
            af2: line1[8],
            tgd1_s: line7[2],
            tgd2_s: line7[3],
        },
        ephemeris: BeidouEphemeris {
            sat,
            aode,
            toe_s: line4[0].rem_euclid(604_800.0),
            sqrt_a: line3[3],
            e: line3[1],
            i0: line5[0],
            idot: line6[0],
            omega0: line4[2],
            omegadot: line5[3],
            w: line5[2],
            m0: line2[3],
            delta_n: line2[2],
            cuc: line3[0],
            cus: line3[2],
            crc: line5[1],
            crs: line2[1],
            cic: line4[1],
            cis: line4[3],
        },
        ionosphere: BeidouIonosphericCorrection {
            alpha0: 0.0,
            alpha1: 0.0,
            alpha2: 0.0,
            alpha3: 0.0,
            beta0: 0.0,
            beta1: 0.0,
            beta2: 0.0,
            beta3: 0.0,
        },
    })
}

fn parse_glonass_rinex_nav_record(
    header: &RinexNavHeader,
    lines: &[&str],
) -> Result<GlonassBroadcastNavigationFrame, ParseError> {
    if lines.len() != 4 {
        return Err(ParseError {
            message: format!("GLONASS RINEX NAV record requires 4 lines, found {}", lines.len()),
        });
    }

    let first_line = lines[0];
    let prn = first_line
        .get(..3)
        .unwrap_or_default()
        .trim()
        .strip_prefix('R')
        .ok_or_else(|| ParseError {
            message: format!(
                "unsupported GLONASS RINEX NAV satellite identifier '{}'",
                &first_line[..3.min(first_line.len())]
            ),
        })?
        .parse::<u8>()
        .map_err(|err| ParseError {
            message: format!(
                "invalid GLONASS satellite identifier '{}': {err}",
                &first_line[..3.min(first_line.len())]
            ),
        })?;
    let slot = GlonassSlot::new(prn).ok_or_else(|| ParseError {
        message: format!("GLONASS satellite identifier R{prn:02} is outside supported slot range"),
    })?;
    let line1 = parse_rinex_numeric_fields(first_line.get(3..).unwrap_or_default())?;
    if line1.len() < 9 {
        return Err(ParseError {
            message: format!(
                "GLONASS RINEX NAV first line requires 9 numeric fields, found {}",
                line1.len()
            ),
        });
    }
    let line2 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[1])?);
    let line3 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[2])?);
    let line4 = pad_rinex_nav_record_fields(parse_rinex_numeric_fields(lines[3])?);

    let year = parse_rinex_nav_year(line1[0] as i32, header.version);
    let toc = parse_rinex_epoch_utc_civil(
        year,
        line1[1] as u8,
        line1[2] as u8,
        line1[3] as u8,
        line1[4] as u8,
        line1[5],
    )?;
    let frame_time =
        GlonassFrameTime { hour: toc.hour, minute: toc.minute, half_minute: toc.second >= 30 };
    let ephemeris_reference_time_s = line1[8].round().max(0.0) as u32;
    let day_number = civil_day_of_year(year, toc.month, toc.day)?;
    let sat = SatId { constellation: Constellation::Glonass, prn };
    let health_status = line2[3].round().max(0.0) as u8;
    let immediate = GlonassImmediateNavigationData {
        sat,
        frame_time,
        ephemeris_reference_time_s,
        tb_update_interval_min: 30,
        tb_is_odd: Some((ephemeris_reference_time_s / 900) % 2 == 1),
        state_vector: GlonassStateVector {
            x_m: line2[0] * 1_000.0,
            y_m: line3[0] * 1_000.0,
            z_m: line4[0] * 1_000.0,
            vx_mps: line2[1] * 1_000.0,
            vy_mps: line3[1] * 1_000.0,
            vz_mps: line4[1] * 1_000.0,
            ax_mps2: line2[2] * 1_000.0,
            ay_mps2: line3[2] * 1_000.0,
            az_mps2: line4[2] * 1_000.0,
        },
        relative_frequency_bias: line1[7],
        // RINEX stores -TauN; the typed model stores TauN.
        clock_bias_s: -line1[6],
        l2_l1_delay_s: None,
        health: GlonassImmediateHealth {
            line_unhealthy: health_status != 0,
            status_code: health_status,
        },
        immediate_data_age_days: line4[3].round().max(0.0) as u8,
        satellite_type: GlonassSatelliteType::GlonassM,
        reported_slot: Some(slot),
        system_time: Some(GlonassSystemTime { day_number, four_year_interval: None }),
        resolved_day_index: None,
        accuracy_code: None,
    };

    Ok(GlonassBroadcastNavigationFrame {
        sat,
        immediate,
        system_time: None,
        almanac_entries: Vec::new(),
    })
}

pub fn parse_rinex_nav(data: &str) -> Result<Vec<GpsEphemeris>, ParseError> {
    Ok(parse_rinex_broadcast_navigation(data)?.ephemerides)
}

pub fn parse_rinex_navigation_dataset(
    data: &str,
) -> Result<RinexBroadcastNavigationDataset, ParseError> {
    let (header, header_line_count) = parse_rinex_nav_header(data)?;
    let lines = data
        .lines()
        .skip(header_line_count)
        .filter(|line| !line.trim().is_empty())
        .collect::<Vec<_>>();
    let mut gps = Vec::new();
    let mut galileo = Vec::new();
    let mut beidou = Vec::new();
    let mut glonass = Vec::new();
    let mut index = 0usize;

    while index < lines.len() {
        let line = lines[index];
        let line_count = rinex_nav_record_line_count(header.version, line)?;
        if index + line_count > lines.len() {
            return Err(ParseError {
                message: format!(
                    "truncated RINEX NAV record at line {}: expected {} lines, found {}",
                    header_line_count + index + 1,
                    line_count,
                    lines.len() - index
                ),
            });
        }
        let record = &lines[index..index + line_count];
        if header.version < 3.0 || line.starts_with('G') {
            gps.push(parse_gps_rinex_nav_record(&header, record)?);
        } else if line.starts_with('E') {
            galileo.push(parse_galileo_rinex_nav_record(&header, record)?);
        } else if line.starts_with('C') {
            beidou.push(parse_beidou_rinex_nav_record(&header, record)?);
        } else if line.starts_with('R') {
            glonass.push(parse_glonass_rinex_nav_record(&header, record)?);
        }
        index += line_count;
    }

    Ok(RinexBroadcastNavigationDataset {
        version: header.version,
        klobuchar: header.klobuchar,
        time_system_corrections: header.time_system_corrections,
        gps,
        galileo,
        beidou,
        glonass,
    })
}

pub fn parse_rinex_broadcast_navigation(
    data: &str,
) -> Result<GpsBroadcastNavigationData, ParseError> {
    let dataset = parse_rinex_navigation_dataset(data)?;
    Ok(GpsBroadcastNavigationData { ephemerides: dataset.gps, klobuchar: dataset.klobuchar })
}

pub fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], _strict: bool) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError { message: e.to_string() })?;
    let mut writer = BufWriter::new(file);
    let dataset = rinex_observation_dataset_from_epochs(epochs)?;
    writer
        .write_all(format_rinex_observation_dataset(&dataset)?.as_bytes())
        .map_err(|e| IoError { message: e.to_string() })?;

    writer.flush().map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
}

fn rinex_observation_dataset_from_epochs(
    epochs: &[ObsEpoch],
) -> Result<RinexObservationDataset, IoError> {
    let observation_types_by_system = rinex_observation_types_by_system(epochs);
    let mut records = Vec::with_capacity(epochs.len());

    for epoch in epochs {
        let mut by_satellite: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for satellite in &epoch.sats {
            if rinex_observation_codes_for_satellite(satellite).is_some()
                && rinex_system_for_constellation(satellite.signal_id.sat.constellation).is_some()
                && observation_is_exportable(satellite)
            {
                by_satellite.entry(satellite.signal_id.sat).or_default().push(satellite);
            }
        }

        let mut satellites = Vec::with_capacity(by_satellite.len());
        for (satellite_id, signals) in by_satellite {
            let Some(system) = rinex_system_for_constellation(satellite_id.constellation) else {
                continue;
            };
            let Some(observation_types) = observation_types_by_system.get(&system) else {
                continue;
            };
            let mut observations = vec![blank_rinex_observation_cell(); observation_types.len()];

            for signal in signals {
                let Some(codes) = rinex_observation_codes_for_satellite(signal) else {
                    continue;
                };
                insert_rinex_observation_cell(
                    &mut observations,
                    observation_types,
                    codes[0],
                    rinex_value_cell(signal.pseudorange_m.0, None, None)?,
                );
                if signal.lock_flags.carrier_lock {
                    insert_rinex_observation_cell(
                        &mut observations,
                        observation_types,
                        codes[1],
                        rinex_value_cell(
                            signal.carrier_phase_cycles.0,
                            signal.lock_flags.cycle_slip.then_some(1),
                            None,
                        )?,
                    );
                }
                insert_rinex_observation_cell(
                    &mut observations,
                    observation_types,
                    codes[2],
                    rinex_value_cell(signal.doppler_hz.0, None, None)?,
                );
                if signal.cn0_dbhz.is_finite() && signal.cn0_dbhz > 0.0 {
                    insert_rinex_observation_cell(
                        &mut observations,
                        observation_types,
                        codes[3],
                        rinex_value_cell(signal.cn0_dbhz, None, None)?,
                    );
                }
            }

            if observations.iter().any(|observation| observation.value.is_some()) {
                satellites.push(RinexSatelliteObservation {
                    system,
                    satellite: satellite_id,
                    observations,
                });
            }
        }

        records.push(RinexObservationRecord::Epoch(RinexObservationEpoch {
            epoch_time: rinex_observation_epoch_time(epoch)?,
            receive_gps_time: rinex_observation_gps_time(epoch),
            event_flag: if epoch.discontinuity { 1 } else { 0 },
            receiver_clock_offset_s: None,
            satellites,
        }));
    }

    Ok(RinexObservationDataset {
        version: 3.04,
        marker_name: None,
        approx_position_ecef_m: None,
        interval_s: None,
        time_system: RinexObservationTimeSystem::Gps,
        code_bias_status_by_system: BTreeMap::new(),
        observation_types_v2: None,
        observation_types_by_system,
        records,
    })
}

fn blank_rinex_observation_cell() -> RinexObservationValue {
    RinexObservationValue {
        value: None,
        loss_of_lock_indicator: None,
        signal_strength_indicator: None,
    }
}

fn rinex_observation_types_by_system(epochs: &[ObsEpoch]) -> BTreeMap<char, Vec<String>> {
    let mut by_system = BTreeMap::new();
    for epoch in epochs {
        for satellite in &epoch.sats {
            let Some(system) =
                rinex_system_for_constellation(satellite.signal_id.sat.constellation)
            else {
                continue;
            };
            let Some(codes) = rinex_observation_codes_for_satellite(satellite) else {
                continue;
            };
            let observation_types = by_system.entry(system).or_insert_with(Vec::new);
            for code in codes {
                if !observation_types.iter().any(|existing| existing == code) {
                    observation_types.push(code.to_string());
                }
            }
        }
    }
    by_system
}

fn insert_rinex_observation_cell(
    observations: &mut [RinexObservationValue],
    observation_types: &[String],
    observation_type: &str,
    cell: RinexObservationValue,
) {
    if let Some(index) =
        observation_types.iter().position(|candidate| candidate == observation_type)
    {
        if observations[index].value.is_none() {
            observations[index] = cell;
        }
    }
}

fn rinex_value_cell(
    value: f64,
    loss_of_lock_indicator: Option<u8>,
    signal_strength_indicator: Option<u8>,
) -> Result<RinexObservationValue, IoError> {
    if !value.is_finite() {
        return Err(IoError { message: "RINEX observation value must be finite".to_string() });
    }
    Ok(RinexObservationValue {
        value: Some(value),
        loss_of_lock_indicator,
        signal_strength_indicator,
    })
}

fn observation_is_exportable(satellite: &ObsSatellite) -> bool {
    matches!(
        satellite.observation_status,
        bijux_gnss_core::api::ObservationStatus::Accepted
            | bijux_gnss_core::api::ObservationStatus::Weak
            | bijux_gnss_core::api::ObservationStatus::Inconsistent
    ) && satellite.pseudorange_m.0.is_finite()
        && satellite.pseudorange_m.0 > 0.0
}

fn rinex_observation_epoch_time(epoch: &ObsEpoch) -> Result<RinexObservationEpochTime, IoError> {
    let gps_time = rinex_observation_gps_time(epoch);
    let unix_like_s = gps_time.to_seconds() + GPS_UNIX_EPOCH_OFFSET_S;
    let datetime =
        time::OffsetDateTime::from_unix_timestamp_nanos((unix_like_s * 1_000_000_000.0) as i128)
            .map_err(|err| IoError {
                message: format!("invalid RINEX observation epoch: {err}"),
            })?;
    Ok(RinexObservationEpochTime {
        year: datetime.year(),
        month: u8::from(datetime.month()),
        day: datetime.day(),
        hour: datetime.hour(),
        minute: datetime.minute(),
        second: datetime.second() as f64 + datetime.nanosecond() as f64 / 1_000_000_000.0,
    })
}

fn rinex_observation_gps_time(epoch: &ObsEpoch) -> GpsTime {
    match (epoch.gps_week, epoch.tow_s) {
        (Some(week), Some(tow_s)) => GpsTime { week, tow_s: tow_s.0 },
        _ => GpsTime::from_seconds(epoch.t_rx_s.0),
    }
}

fn rinex_system_for_constellation(constellation: Constellation) -> Option<char> {
    match constellation {
        Constellation::Gps => Some('G'),
        Constellation::Glonass => Some('R'),
        Constellation::Galileo => Some('E'),
        Constellation::Beidou => Some('C'),
        Constellation::Unknown => None,
    }
}

fn rinex_observation_codes_for_satellite(satellite: &ObsSatellite) -> Option<[&'static str; 4]> {
    match (
        satellite.signal_id.sat.constellation,
        satellite.signal_id.band,
        satellite.signal_id.code,
    ) {
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => Some(["C1C", "L1C", "D1C", "S1C"]),
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C) => Some(["C2L", "L2L", "D2L", "S2L"]),
        (Constellation::Gps, SignalBand::L2, SignalCode::Py) => Some(["C2W", "L2W", "D2W", "S2W"]),
        (Constellation::Gps, SignalBand::L5, _) => Some(["C5Q", "L5Q", "D5Q", "S5Q"]),
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
            Some(["C1C", "L1C", "D1C", "S1C"])
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            Some(["C5Q", "L5Q", "D5Q", "S5Q"])
        }
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
            Some(["C2I", "L2I", "D2I", "S2I"])
        }
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            Some(["C7I", "L7I", "D7I", "S7I"])
        }
        _ => None,
    }
}

pub fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], _strict: bool) -> Result<(), IoError> {
    write_rinex_broadcast_navigation(
        path,
        &GpsBroadcastNavigationData { ephemerides: ephs.to_vec(), klobuchar: None },
        _strict,
    )
}

pub fn write_rinex_broadcast_navigation(
    path: &Path,
    navigation: &GpsBroadcastNavigationData,
    _strict: bool,
) -> Result<(), IoError> {
    write_rinex_navigation_dataset(
        path,
        &RinexBroadcastNavigationDataset {
            version: 3.04,
            klobuchar: navigation.klobuchar,
            time_system_corrections: Vec::new(),
            gps: navigation.ephemerides.clone(),
            galileo: Vec::new(),
            beidou: Vec::new(),
            glonass: Vec::new(),
        },
        _strict,
    )
}

pub fn write_rinex_navigation_dataset(
    path: &Path,
    dataset: &RinexBroadcastNavigationDataset,
    _strict: bool,
) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError { message: e.to_string() })?;
    let mut writer = BufWriter::new(file);
    writer
        .write_all(format_rinex_navigation_dataset(dataset)?.as_bytes())
        .map_err(|e| IoError { message: e.to_string() })?;
    writer.flush().map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
}

pub fn format_rinex_navigation_dataset(
    dataset: &RinexBroadcastNavigationDataset,
) -> Result<String, IoError> {
    let mut output = String::new();
    let version = if dataset.version >= 3.0 { dataset.version } else { 3.05 };
    let has_non_gps =
        !dataset.galileo.is_empty() || !dataset.beidou.is_empty() || !dataset.glonass.is_empty();
    let system_label = if has_non_gps { "M (MIXED)" } else { "G (GPS)" };
    push_rinex_line(
        &mut output,
        &format!(
            "{version:>9.2}           NAVIGATION DATA     {system_label:<20}RINEX VERSION / TYPE"
        ),
    );
    push_rinex_line(&mut output, "bijux-gnss                              PGM / RUN BY / DATE");
    if let Some(klobuchar) = dataset.klobuchar {
        push_rinex_line(&mut output, &format_rinex_klobuchar_header_line("GPSA", klobuchar.alpha));
        push_rinex_line(&mut output, &format_rinex_klobuchar_header_line("GPSB", klobuchar.beta));
    }
    for correction in &dataset.time_system_corrections {
        push_rinex_line(&mut output, &format_rinex_time_system_correction_header_line(correction));
    }
    push_rinex_line(
        &mut output,
        "                                                            END OF HEADER",
    );

    for eph in &dataset.gps {
        for line in format_gps_rinex_nav_record(eph)? {
            push_rinex_line(&mut output, &line);
        }
    }
    for nav in &dataset.galileo {
        for line in format_galileo_rinex_nav_record(nav)? {
            push_rinex_line(&mut output, &line);
        }
    }
    for nav in &dataset.beidou {
        for line in format_beidou_rinex_nav_record(nav)? {
            push_rinex_line(&mut output, &line);
        }
    }
    for nav in &dataset.glonass {
        for line in format_glonass_rinex_nav_record(nav)? {
            push_rinex_line(&mut output, &line);
        }
    }

    Ok(output)
}

pub fn parse_rinex_obs_header(data: &str) -> Result<(), ParseError> {
    let mut has_version = false;
    let mut has_end = false;
    for line in data.lines() {
        if line.contains("RINEX VERSION / TYPE") {
            has_version = true;
        }
        if line.contains("END OF HEADER") {
            has_end = true;
            break;
        }
    }
    if has_version && has_end {
        Ok(())
    } else {
        Err(ParseError { message: "invalid or incomplete RINEX header".to_string() })
    }
}

#[cfg(test)]
mod tests;
