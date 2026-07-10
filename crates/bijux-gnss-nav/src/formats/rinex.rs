#![allow(missing_docs)]

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::OnceLock;

use bijux_gnss_core::api::{Constellation, IoError, ParseError, SatId};
use regex::Regex;
use time::{Date, Month, PrimitiveDateTime, Time};

use bijux_gnss_core::api::ObsEpoch;

use crate::orbits::gps::GpsEphemeris;

fn write_header_line(writer: &mut BufWriter<File>, line: &str) -> Result<(), IoError> {
    let mut out = line.to_string();
    if out.len() > 80 {
        out.truncate(80);
    }
    writeln!(writer, "{out}").map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct RinexNavHeader {
    version: f64,
    is_mixed: bool,
}

fn rinex_nav_float_regex() -> &'static Regex {
    static REGEX: OnceLock<Regex> = OnceLock::new();
    REGEX.get_or_init(|| {
        Regex::new(r"[+-]?\d+(?:\.\d*)?(?:[DdEe][+-]?\d+)?")
            .expect("valid RINEX NAV float regex")
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
    rinex_nav_float_regex()
        .find_iter(line)
        .map(|field| parse_rinex_float(field.as_str()))
        .collect()
}

fn parse_rinex_nav_header(data: &str) -> Result<(RinexNavHeader, usize), ParseError> {
    let mut has_type = false;
    let mut has_end = false;
    let mut header = None;
    let mut header_line_count = 0usize;

    for line in data.lines() {
        header_line_count += 1;
        if header.is_none() && line.contains("RINEX VERSION / TYPE") {
            let version = line
                .get(..9)
                .unwrap_or_default()
                .trim()
                .parse::<f64>()
                .map_err(|err| ParseError {
                    message: format!("invalid RINEX NAV version '{}': {err}", &line[..9.min(line.len())]),
                })?;
            has_type = line.contains("NAVIGATION DATA");
            header = Some(RinexNavHeader { version, is_mixed: line.contains("M (MIXED)") });
        }
        if line.contains("END OF HEADER") {
            has_end = true;
            break;
        }
    }

    match (header, has_type, has_end) {
        (Some(header), true, true) => Ok((header, header_line_count)),
        _ => Err(ParseError { message: "invalid or incomplete RINEX NAV header".to_string() }),
    }
}

fn parse_rinex_epoch_utc(
    year: i32,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: f64,
) -> Result<bijux_gnss_core::api::UtcTime, ParseError> {
    let whole_seconds = second.floor();
    let nanos = ((second - whole_seconds) * 1_000_000_000.0).round();
    let date = Date::from_calendar_date(
        year,
        Month::try_from(month).map_err(|_| ParseError {
            message: format!("invalid RINEX NAV month {month}"),
        })?,
        day,
    )
    .map_err(|err| ParseError { message: format!("invalid RINEX NAV date: {err}") })?;
    let time = Time::from_hms_nano(
        hour,
        minute,
        whole_seconds as u8,
        nanos as u32,
    )
    .map_err(|err| ParseError { message: format!("invalid RINEX NAV time: {err}") })?;
    let utc = PrimitiveDateTime::new(date, time).assume_utc();
    Ok(bijux_gnss_core::api::UtcTime {
        unix_s: utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0,
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
    header: RinexNavHeader,
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
                message: format!("unsupported RINEX NAV satellite identifier '{}'", &first_line[..3.min(first_line.len())]),
            })?
            .parse::<u8>()
            .map_err(|err| ParseError {
                message: format!("invalid GPS satellite identifier '{}': {err}", &first_line[..3.min(first_line.len())]),
            })?;
        (sat, first_line.get(3..).unwrap_or_default())
    } else {
        let sat = first_line
            .get(..2)
            .unwrap_or_default()
            .trim()
            .parse::<u8>()
            .map_err(|err| ParseError {
                message: format!("invalid GPS satellite identifier '{}': {err}", &first_line[..2.min(first_line.len())]),
            })?;
        (sat, first_line.get(2..).unwrap_or_default())
    };

    let line1 = parse_rinex_numeric_fields(first_line_payload)?;
    if line1.len() < 9 {
        return Err(ParseError {
            message: format!("GPS RINEX NAV first line requires 9 numeric fields, found {}", line1.len()),
        });
    }
    let line2 = parse_rinex_numeric_fields(lines[1])?;
    let line3 = parse_rinex_numeric_fields(lines[2])?;
    let line4 = parse_rinex_numeric_fields(lines[3])?;
    let line5 = parse_rinex_numeric_fields(lines[4])?;
    let line6 = parse_rinex_numeric_fields(lines[5])?;
    let line7 = parse_rinex_numeric_fields(lines[6])?;

    for (line_index, fields) in [(2, &line2), (3, &line3), (4, &line4), (5, &line5), (6, &line6), (7, &line7)] {
        if fields.len() < 4 {
            return Err(ParseError {
                message: format!("GPS RINEX NAV line {line_index} requires 4 numeric fields, found {}", fields.len()),
            });
        }
    }

    let year = parse_rinex_nav_year(line1[0] as i32, header.version);
    let utc = parse_rinex_epoch_utc(
        year,
        line1[1] as u8,
        line1[2] as u8,
        line1[3] as u8,
        line1[4] as u8,
        line1[5],
    )?;
    let toc = crate::time::gps_time_from_utc(utc);
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

pub fn parse_rinex_nav(data: &str) -> Result<Vec<GpsEphemeris>, ParseError> {
    let (header, header_line_count) = parse_rinex_nav_header(data)?;
    let lines = data.lines().skip(header_line_count).filter(|line| !line.trim().is_empty()).collect::<Vec<_>>();
    let mut ephemerides = Vec::new();
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
        if header.version >= 3.0 && !line.starts_with('G') {
            index += line_count;
            continue;
        }
        let record = &lines[index..index + line_count];
        ephemerides.push(parse_gps_rinex_nav_record(header, record)?);
        index += line_count;
    }

    Ok(ephemerides)
}

pub fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], _strict: bool) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError { message: e.to_string() })?;
    let mut writer = BufWriter::new(file);
    write_header_line(
        &mut writer,
        "     3.04           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE",
    )?;
    write_header_line(&mut writer, "bijux-gnss                              PGM / RUN BY / DATE")?;
    write_header_line(
        &mut writer,
        "                                                            END OF HEADER",
    )?;

    for epoch in epochs {
        let sat_count = epoch.sats.len();
        let line = format!(
            "> {:>4} {:>2} {:>2} {:>2} {:>2} {:>11.7}  {:>2} {:>3}",
            1980, 1, 6, 0, 0, epoch.t_rx_s.0, 0, sat_count
        );
        write_header_line(&mut writer, &line)?;
    }

    writer.flush().map_err(|e| IoError { message: e.to_string() })?;
    Ok(())
}

pub fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], _strict: bool) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError { message: e.to_string() })?;
    let mut writer = BufWriter::new(file);
    write_header_line(
        &mut writer,
        "     3.04           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE",
    )?;
    write_header_line(&mut writer, "bijux-gnss                              PGM / RUN BY / DATE")?;
    write_header_line(
        &mut writer,
        "                                                            END OF HEADER",
    )?;

    for eph in ephs {
        let line = format!("G{:02} 0 0 0 0 0 0 0 0 0 0 0 0 0 0", eph.sat.prn);
        write_header_line(&mut writer, &line)?;
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
        parse_rinex_epoch_utc, parse_rinex_float, parse_rinex_nav, parse_rinex_nav_header,
        parse_rinex_numeric_fields,
    };
    use bijux_gnss_core::api::{Constellation, SatId};

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
        assert_eq!(line_count, 3);
    }

    #[test]
    fn rinex_nav_float_parses_fortran_exponents() {
        let value = parse_rinex_float("-1.981128007174D-04").expect("D exponent float");

        assert!((value + 1.981_128_007_174e-4).abs() < 1.0e-16);
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
}
