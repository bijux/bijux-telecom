#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::OnceLock;

use bijux_gnss_core::api::{
    gps_to_utc, Constellation, GpsTime, IoError, LeapSeconds, ObsEpoch, ObsSatellite, ParseError,
    SatId, SignalBand, SignalCode,
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
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::GpsBroadcastNavigationData;
use crate::orbits::gps::GpsEphemeris;
use crate::time::{
    gps_to_beidou_with_offset, utc_civil_to_gps_with_offset, UtcCivilTime, UtcCivilTimeError,
};

const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;

fn write_header_line(writer: &mut BufWriter<File>, line: &str) -> Result<(), IoError> {
    let mut out = line.to_string();
    if out.len() > 80 {
        out.truncate(80);
    }
    writeln!(writer, "{out}").map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
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

fn gps_time_to_utc_datetime(gps_time: GpsTime) -> Result<time::OffsetDateTime, IoError> {
    let utc = gps_to_utc(gps_time, &LeapSeconds::default_table());
    time::OffsetDateTime::from_unix_timestamp_nanos((utc.unix_s * 1_000_000_000.0).round() as i128)
        .map_err(|err| IoError { message: format!("invalid GPS-to-UTC conversion: {err}") })
}

fn write_rinex_nav_record(writer: &mut BufWriter<File>, eph: &GpsEphemeris) -> Result<(), IoError> {
    let utc = gps_time_to_utc_datetime(GpsTime { week: eph.week, tow_s: eph.toc_s })?;
    write_header_line(
        writer,
        &format!(
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
    )?;
    for line in [
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
            format_rinex_nav_float(0.0),
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
    ] {
        write_header_line(writer, &line)?;
    }
    Ok(())
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
    let glonass = Vec::new();
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
    let file = File::create(path).map_err(|e| IoError { message: e.to_string() })?;
    let mut writer = BufWriter::new(file);
    write_header_line(
        &mut writer,
        "     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE",
    )?;
    write_header_line(&mut writer, "bijux-gnss                              PGM / RUN BY / DATE")?;
    if let Some(klobuchar) = navigation.klobuchar {
        write_header_line(
            &mut writer,
            &format_rinex_klobuchar_header_line("GPSA", klobuchar.alpha),
        )?;
        write_header_line(
            &mut writer,
            &format_rinex_klobuchar_header_line("GPSB", klobuchar.beta),
        )?;
    }
    write_header_line(
        &mut writer,
        "                                                            END OF HEADER",
    )?;

    for eph in &navigation.ephemerides {
        write_rinex_nav_record(&mut writer, eph)?;
    }

    writer.flush().map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
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
mod tests {
    use super::{
        format_rinex_nav_float, parse_rinex_broadcast_navigation, parse_rinex_epoch_utc,
        parse_rinex_epoch_utc_civil, parse_rinex_float, parse_rinex_nav, parse_rinex_nav_header,
        parse_rinex_navigation_dataset, parse_rinex_numeric_fields,
        write_rinex_broadcast_navigation, write_rinex_nav, write_rinex_obs,
    };
    use crate::formats::rinex_obs::{parse_rinex_observation_dataset, RinexObservationRecord};
    use crate::models::atmosphere::KlobucharCoefficients;
    use crate::orbits::beidou::BeidouSystemTime;
    use crate::orbits::galileo::GalileoSystemTime;
    use crate::orbits::gps::{GpsBroadcastNavigationData, GpsEphemeris};
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationStatus, ReceiverRole, SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    fn sample_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            iodc: 97,
            iode: 11,
            week: 2209,
            sv_health: 0,
            toe_s: 345_600.0,
            toc_s: 504_018.0,
            sqrt_a: 5_153.795_477_5,
            e: 1.234_567_890_123e-2,
            i0: 9.4e-1,
            idot: 7.8e-10,
            omega0: 1.5,
            omegadot: -8.9e-9,
            w: 2.1e-1,
            m0: 6.0e-1,
            delta_n: 4.5e-9,
            cuc: 1.2e-6,
            cus: 2.3e-6,
            crc: 321.0,
            crs: 25.0,
            cic: 4.5e-8,
            cis: 5.6e-8,
            af0: -1.234_567_890_123e-4,
            af1: 2.345_678_901_234e-12,
            af2: 0.0,
            tgd: -1.9e-8,
        }
    }

    fn rinex_obs_output_path(name: &str) -> std::path::PathBuf {
        let output_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../artifacts/rust-test/rinex");
        std::fs::create_dir_all(&output_dir).expect("create RINEX test artifact directory");
        output_dir.join(format!("{name}-{}.rnx", std::process::id()))
    }

    fn observation(
        constellation: Constellation,
        prn: u8,
        band: SignalBand,
        code: SignalCode,
        pseudorange_m: f64,
        carrier_phase_cycles: f64,
        doppler_hz: f64,
        cn0_dbhz: f64,
        cycle_slip: bool,
    ) -> ObsSatellite {
        ObsSatellite {
            signal_id: SigId { sat: SatId { constellation, prn }, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(doppler_hz),
            doppler_var_hz2: 1.0,
            cn0_dbhz,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata::default(),
        }
    }

    #[test]
    fn write_rinex_obs_emits_supported_measurements() {
        let epoch = ObsEpoch {
            t_rx_s: Seconds(0.0),
            source_time: Default::default(),
            gps_week: Some(2209),
            tow_s: Some(Seconds(504_000.0)),
            epoch_idx: 0,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                observation(
                    Constellation::Gps,
                    1,
                    SignalBand::L1,
                    SignalCode::Ca,
                    20_000_000.125,
                    100_000.250,
                    -500.125,
                    45.5,
                    true,
                ),
                observation(
                    Constellation::Gps,
                    1,
                    SignalBand::L2,
                    SignalCode::L2C,
                    21_000_000.875,
                    110_000.500,
                    -400.500,
                    44.25,
                    false,
                ),
                observation(
                    Constellation::Galileo,
                    11,
                    SignalBand::E1,
                    SignalCode::E1B,
                    24_000_000.000,
                    200_000.000,
                    -250.000,
                    47.0,
                    false,
                ),
            ],
            decision: Default::default(),
            decision_reason: None,
            manifest: None,
        };
        let output_path = rinex_obs_output_path("write-supported-observations");

        write_rinex_obs(&output_path, &[epoch], true).expect("write RINEX observation file");
        let encoded = std::fs::read_to_string(&output_path).expect("read RINEX observation file");
        let dataset =
            parse_rinex_observation_dataset(&encoded).expect("parse written RINEX observations");

        assert_eq!(
            dataset.observation_types_by_system[&'G'],
            ["C1C", "L1C", "D1C", "S1C", "C2L", "L2L", "D2L", "S2L"]
        );
        assert_eq!(dataset.observation_types_by_system[&'E'], ["C1C", "L1C", "D1C", "S1C"]);
        let RinexObservationRecord::Epoch(parsed_epoch) = &dataset.records[0] else {
            panic!("expected observation epoch");
        };
        let gps = parsed_epoch
            .satellites
            .iter()
            .find(|satellite| satellite.system == 'G' && satellite.satellite.prn == 1)
            .expect("GPS satellite record");
        assert_eq!(gps.observations[0].value, Some(20_000_000.125));
        assert_eq!(gps.observations[1].value, Some(100_000.250));
        assert_eq!(gps.observations[1].loss_of_lock_indicator, Some(1));
        assert_eq!(gps.observations[4].value, Some(21_000_000.875));
        assert_eq!(gps.observations[7].value, Some(44.250));
    }

    #[test]
    fn rinex_nav_header_reports_version_and_mixed_flag() {
        let data = "\
     3.04           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
";

        let (header, line_count) = parse_rinex_nav_header(data).expect("valid nav header");

        assert_eq!(header.version, 3.04);
        assert!(header.is_mixed);
        assert_eq!(header.klobuchar, None);
        assert_eq!(line_count, 3);
    }

    #[test]
    fn parse_rinex_nav_header_reads_rinex_2_klobuchar_coefficients() {
        let data = "\
     2.11           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
 1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07          ION ALPHA
 1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06          ION BETA
                                                            END OF HEADER
";

        let (header, _) = parse_rinex_nav_header(data).expect("RINEX 2 header");

        assert_eq!(
            header.klobuchar,
            Some(KlobucharCoefficients::new(
                [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
                [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
            ))
        );
    }

    #[test]
    fn parse_rinex_nav_header_reads_rinex_3_klobuchar_coefficients() {
        let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPSA  1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07       IONOSPHERIC CORR
GPSB  1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06       IONOSPHERIC CORR
                                                            END OF HEADER
";

        let (header, _) = parse_rinex_nav_header(data).expect("RINEX 3 header");

        assert_eq!(
            header.klobuchar,
            Some(KlobucharCoefficients::new(
                [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
                [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
            ))
        );
    }

    #[test]
    fn parse_rinex_nav_header_reads_time_system_corrections() {
        let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GAUT -1.8626451492D-09-8.881784197D-16 432000 2209 GAL UTC TIME SYSTEM CORR
BDUT  4.6566128731D-10 0.000000000D+00 345600 0850 BDS UTC TIME SYSTEM CORR
                                                            END OF HEADER
";

        let (header, _) = parse_rinex_nav_header(data).expect("RINEX NAV header");

        assert_eq!(header.version, 3.05);
        assert!(header.is_mixed);
        assert_eq!(header.time_system_corrections.len(), 2);
        assert_eq!(header.time_system_corrections[0].code, "GAUT");
        assert!((header.time_system_corrections[0].a0_s + 1.862_645_149_2e-9).abs() < 1.0e-20);
        assert_eq!(header.time_system_corrections[0].reference_time_s, 432_000);
        assert_eq!(header.time_system_corrections[0].reference_week, 2209);
        assert_eq!(header.time_system_corrections[0].provider.as_deref(), Some("GAL"));
        assert_eq!(header.time_system_corrections[0].utc_id.as_deref(), Some("UTC"));
        assert_eq!(header.time_system_corrections[1].code, "BDUT");
    }

    #[test]
    fn rinex_nav_float_parses_fortran_exponents() {
        let value = parse_rinex_float("-1.981128007174D-04").expect("D exponent float");

        assert!((value + 1.981_128_007_174e-4).abs() < 1.0e-16);
    }

    #[test]
    fn rinex_nav_float_formats_fortran_exponents() {
        let field = format_rinex_nav_float(-1.981_128_007_174e-4);

        assert_eq!(field.len(), 19);
        assert!(field.contains('D'));
        assert!(!field.contains('E'));
    }

    #[test]
    fn rinex_nav_numeric_fields_extract_adjacent_values() {
        let fields = parse_rinex_numeric_fields(
            "    1.700000000000D+01-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00",
        )
        .expect("numeric fields");

        assert_eq!(fields.len(), 4);
        assert_eq!(fields[0], 17.0);
        assert!((fields[1] + 1.234_567_890_123e-4).abs() < 1.0e-16);
        assert!((fields[2] - 2.345_678_901_234e-12).abs() < 1.0e-24);
        assert_eq!(fields[3], 0.0);
    }

    #[test]
    fn rinex_nav_epoch_conversion_preserves_utc_in_gps_time() {
        let utc = parse_rinex_epoch_utc(2022, 5, 13, 20, 0, 0.0).expect("utc epoch");
        let gps = crate::time::gps_time_from_utc(utc);

        assert_eq!(gps.week, 2209);
        assert!((gps.tow_s - 504_018.0).abs() < 1.0e-9);
    }

    #[test]
    fn rinex_nav_epoch_conversion_preserves_declared_leap_second_in_gps_time() {
        let civil = parse_rinex_epoch_utc_civil(2016, 12, 31, 23, 59, 60.5).expect("leap epoch");
        let gps = crate::time::utc_civil_to_gps_with_offset(
            civil,
            &bijux_gnss_core::api::LeapSeconds::default_table(),
        )
        .expect("gps conversion")
        .time;

        assert_eq!(civil.format_utc(), "2016-12-31T23:59:60.5Z");
        assert_eq!(gps.week, 1930);
        assert!((gps.tow_s - 17.5).abs() < 1.0e-9);
        assert!(parse_rinex_epoch_utc(2016, 12, 31, 23, 59, 60.0).is_err());
    }

    #[test]
    fn parse_rinex_nav_reads_gps_rinex_3_ephemeris() {
        let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let ephs = parse_rinex_nav(data).expect("RINEX NAV parse");

        assert_eq!(ephs.len(), 1);
        let eph = &ephs[0];
        assert_eq!(eph.sat, SatId { constellation: Constellation::Gps, prn: 1 });
        assert_eq!(eph.week, 2209);
        assert_eq!(eph.toe_s, 345_600.0);
        assert!((eph.toc_s - 504_018.0).abs() < 1.0e-9);
        assert_eq!(eph.iode, 11);
        assert_eq!(eph.iodc, 97);
        assert!((eph.af0 + 1.234_567_890_123e-4).abs() < 1.0e-16);
        assert!((eph.af1 - 2.345_678_901_234e-12).abs() < 1.0e-24);
        assert!((eph.sqrt_a - 5_153.795_477_5).abs() < 1.0e-12);
        assert!((eph.e - 1.234_567_890_123e-2).abs() < 1.0e-15);
        assert!((eph.delta_n - 4.5e-9).abs() < 1.0e-18);
        assert!((eph.tgd + 1.9e-8).abs() < 1.0e-18);
    }

    #[test]
    fn parse_rinex_navigation_dataset_preserves_gps_and_header_corrections() {
        let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPGA  1.0000000000D-09 2.000000000D-15 345600 2209 GPS GAL TIME SYSTEM CORR
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let dataset = parse_rinex_navigation_dataset(data).expect("parse mixed navigation dataset");

        assert_eq!(dataset.version, 3.05);
        assert_eq!(dataset.gps.len(), 1);
        assert!(dataset.galileo.is_empty());
        assert!(dataset.beidou.is_empty());
        assert!(dataset.glonass.is_empty());
        assert_eq!(dataset.time_system_corrections.len(), 1);
        assert_eq!(dataset.time_system_corrections[0].code, "GPGA");
        assert_eq!(dataset.time_system_corrections[0].provider.as_deref(), Some("GPS"));
        assert_eq!(dataset.time_system_corrections[0].utc_id.as_deref(), Some("GAL"));
    }

    #[test]
    fn parse_rinex_navigation_dataset_reads_galileo_records() {
        let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
E19 1980 01 06 18 20 00-1.700000000000D-04 2.500000000000D-12-3.000000000000D-19
    4.210000000000D+02-9.100000000000D+01 4.700000000000D-09 8.400000000000D-01
   -3.200000000000D-06 1.230000000000D-03 4.100000000000D-06 5.440612319000D+03
    6.480000000000D+04 1.900000000000D-07 1.170000000000D+00-2.400000000000D-07
    9.530000000000D-01 1.780000000000D+02-3.700000000000D-01-5.800000000000D-09
   -2.100000000000D-10 5.000000000000D+00 0.000000000000D+00 7.700000000000D+01
    3.000000000000D+00-1.100000000000D-09 2.400000000000D-09 6.500000000000D+04
    0.000000000000D+00 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let dataset = parse_rinex_navigation_dataset(data).expect("parse Galileo navigation");

        assert!(dataset.gps.is_empty());
        assert_eq!(dataset.galileo.len(), 1);
        let galileo = &dataset.galileo[0];
        assert_eq!(galileo.sat, SatId { constellation: Constellation::Galileo, prn: 19 });
        assert_eq!(galileo.iodnav, 421);
        assert_eq!(galileo.gst, GalileoSystemTime { week: 0, tow_s: 65_000 });
        assert_eq!(galileo.sisa_e1_e5b, 77);
        assert_eq!(galileo.signal_health.e5b_signal_health, 3);
        assert!((galileo.clock.t0c_s - 66_000.0).abs() < 1.0e-9);
        assert!((galileo.clock.af0 + 1.7e-4).abs() < 1.0e-16);
        assert!((galileo.clock.bgd_e1_e5a_s + 1.1e-9).abs() < 1.0e-20);
        assert!((galileo.ephemeris.sqrt_a - 5_440.612_319).abs() < 1.0e-12);
        assert!((galileo.ephemeris.toe_s - 64_800.0).abs() < 1.0e-9);
    }

    #[test]
    fn parse_rinex_navigation_dataset_reads_beidou_records() {
        let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
C11 2006 01 01 18 00 00-1.800000000000D-04 2.200000000000D-12-2.600000000000D-19
    3.000000000000D+00-8.200000000000D+01 4.300000000000D-09 1.120000000000D+00
   -2.300000000000D-06 2.340000000000D-03 3.100000000000D-06 5.282625128000D+03
    6.480000000000D+04 2.600000000000D-07 8.700000000000D-01-2.200000000000D-07
    9.580000000000D-01 1.450000000000D+02-4.200000000000D-01-6.200000000000D-09
   -1.900000000000D-10 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.100000000000D-09 2.200000000000D-09
    6.470000000000D+04 3.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let dataset = parse_rinex_navigation_dataset(data).expect("parse BeiDou navigation");

        assert!(dataset.gps.is_empty());
        assert!(dataset.galileo.is_empty());
        assert_eq!(dataset.beidou.len(), 1);
        let beidou = &dataset.beidou[0];
        assert_eq!(beidou.sat, SatId { constellation: Constellation::Beidou, prn: 11 });
        assert_eq!(beidou.bdt, BeidouSystemTime { week: 0, sow_s: 64_700 });
        assert_eq!(beidou.urai, 2);
        assert!(beidou.signal_health.autonomous_satellite_good);
        assert_eq!(beidou.clock.aodc, 3);
        assert!((beidou.clock.toc_s - 64_800.0).abs() < 1.0e-9);
        assert!((beidou.clock.tgd1_s + 1.1e-9).abs() < 1.0e-20);
        assert_eq!(beidou.ephemeris.aode, 3);
        assert!((beidou.ephemeris.sqrt_a - 5_282.625_128).abs() < 1.0e-12);
        assert!((beidou.ephemeris.toe_s - 64_800.0).abs() < 1.0e-9);
    }

    #[test]
    fn parse_rinex_nav_reads_gps_epoch_on_inserted_leap_second() {
        let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
G01 2016 12 31 23 59 60-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 1.930000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let ephs = parse_rinex_nav(data).expect("RINEX NAV leap second parse");

        assert_eq!(ephs.len(), 1);
        assert_eq!(ephs[0].week, 1930);
        assert!((ephs[0].toc_s - 17.0).abs() < 1.0e-9);
    }

    #[test]
    fn parse_rinex_nav_reads_gps_rinex_2_ephemeris() {
        let data = "\
     2.11           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
 1 22  5 13 20  0  0-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00
";

        let ephs = parse_rinex_nav(data).expect("RINEX 2 NAV parse");

        assert_eq!(ephs.len(), 1);
        assert_eq!(ephs[0].week, 2209);
        assert_eq!(ephs[0].sat.prn, 1);
    }

    #[test]
    fn parse_rinex_broadcast_navigation_returns_ephemerides_and_klobuchar() {
        let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPSA  1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07       IONOSPHERIC CORR
GPSB  1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06       IONOSPHERIC CORR
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

        let navigation = parse_rinex_broadcast_navigation(data).expect("broadcast navigation");

        assert_eq!(navigation.ephemerides.len(), 1);
        assert_eq!(
            navigation.klobuchar,
            Some(KlobucharCoefficients::new(
                [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
                [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
            ))
        );
    }

    #[test]
    fn write_rinex_nav_round_trips_through_parser() {
        let eph = sample_ephemeris();
        let path = std::env::temp_dir().join(format!(
            "bijux-rinex-nav-roundtrip-{}-{}.rnx",
            std::process::id(),
            eph.sat.prn
        ));

        write_rinex_nav(&path, std::slice::from_ref(&eph), true).expect("write nav");
        let data = std::fs::read_to_string(&path).expect("read nav");
        let parsed = parse_rinex_nav(&data).expect("parse written nav");
        std::fs::remove_file(&path).expect("remove nav fixture");

        assert_eq!(parsed.len(), 1);
        let parsed = &parsed[0];
        assert_eq!(parsed.sat, eph.sat);
        assert_eq!(parsed.week, eph.week);
        assert_eq!(parsed.iode, eph.iode);
        assert_eq!(parsed.iodc, eph.iodc);
        assert!((parsed.toe_s - eph.toe_s).abs() < 1.0e-9);
        assert!((parsed.toc_s - eph.toc_s).abs() < 1.0e-6);
        assert!((parsed.af0 - eph.af0).abs() < 1.0e-16);
        assert!((parsed.af1 - eph.af1).abs() < 1.0e-24);
        assert!((parsed.tgd - eph.tgd).abs() < 1.0e-18);
    }

    #[test]
    fn write_rinex_broadcast_navigation_round_trips_klobuchar() {
        let klobuchar = KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        );
        let navigation = GpsBroadcastNavigationData {
            ephemerides: vec![sample_ephemeris()],
            klobuchar: Some(klobuchar),
        };
        let path = std::env::temp_dir().join(format!(
            "bijux-rinex-broadcast-navigation-roundtrip-{}-{}.rnx",
            std::process::id(),
            navigation.ephemerides[0].sat.prn
        ));

        write_rinex_broadcast_navigation(&path, &navigation, true)
            .expect("write broadcast navigation");
        let data = std::fs::read_to_string(&path).expect("read broadcast navigation");
        let parsed = parse_rinex_broadcast_navigation(&data).expect("parse written navigation");
        std::fs::remove_file(&path).expect("remove nav fixture");

        assert_eq!(parsed.ephemerides.len(), 1);
        assert_eq!(parsed.ephemerides[0].sat, navigation.ephemerides[0].sat);
        assert_eq!(parsed.klobuchar, Some(klobuchar));
    }
}
