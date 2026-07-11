#![allow(missing_docs)]

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::OnceLock;

use bijux_gnss_core::api::{gps_to_utc, Constellation, GpsTime, IoError, LeapSeconds, ParseError, SatId};
use regex::Regex;
use time::{Date, Month, PrimitiveDateTime, Time};

use bijux_gnss_core::api::ObsEpoch;

use crate::models::atmosphere::KlobucharCoefficients;
use crate::orbits::gps::GpsBroadcastNavigationData;
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
    klobuchar: Option<KlobucharCoefficients>,
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
    let mut klobuchar_alpha = None;
    let mut klobuchar_beta = None;

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
            header = Some(RinexNavHeader {
                version,
                is_mixed: line.contains("M (MIXED)"),
                klobuchar: None,
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

fn parse_rinex_klobuchar_header_fields(
    line: &str,
    label: &str,
) -> Result<[f64; 4], ParseError> {
    let fields = parse_rinex_numeric_fields(line)?;
    if fields.len() < 4 {
        return Err(ParseError {
            message: format!(
                "{label} requires 4 numeric fields, found {}",
                fields.len()
            ),
        });
    }
    Ok([fields[0], fields[1], fields[2], fields[3]])
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

fn format_rinex_nav_float(value: f64) -> String {
    format!("{value:>19.12E}").replace('E', "D")
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
    Ok(parse_rinex_broadcast_navigation(data)?.ephemerides)
}

pub fn parse_rinex_broadcast_navigation(
    data: &str,
) -> Result<GpsBroadcastNavigationData, ParseError> {
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

    Ok(GpsBroadcastNavigationData { ephemerides, klobuchar: header.klobuchar })
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
        "     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE",
    )?;
    write_header_line(&mut writer, "bijux-gnss                              PGM / RUN BY / DATE")?;
    write_header_line(
        &mut writer,
        "                                                            END OF HEADER",
    )?;

    for eph in ephs {
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
        parse_rinex_float, parse_rinex_nav, parse_rinex_nav_header, parse_rinex_numeric_fields,
        write_rinex_nav,
    };
    use crate::models::atmosphere::KlobucharCoefficients;
    use bijux_gnss_core::api::{Constellation, SatId};
    use crate::orbits::gps::GpsEphemeris;

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
}
