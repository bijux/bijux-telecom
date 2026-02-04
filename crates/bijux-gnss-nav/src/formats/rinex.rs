#![allow(missing_docs)]

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use bijux_gnss_core::api::{IoError, ParseError};

use bijux_gnss_core::api::ObsEpoch;

use crate::orbits::gps::GpsEphemeris;

fn write_header_line(writer: &mut BufWriter<File>, line: &str) -> Result<(), IoError> {
    let mut out = line.to_string();
    if out.len() > 80 {
        out.truncate(80);
    }
    writeln!(writer, "{out}").map_err(|e| IoError {
        message: e.to_string(),
    })?;
    Ok(())
}

pub fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], _strict: bool) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError {
        message: e.to_string(),
    })?;
    let mut writer = BufWriter::new(file);
    write_header_line(
        &mut writer,
        "     3.04           OBSERVATION DATA    M (MIXED)           RINEX VERSION / TYPE",
    )?;
    write_header_line(
        &mut writer,
        "bijux-gnss                              PGM / RUN BY / DATE",
    )?;
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

    writer.flush().map_err(|e| IoError {
        message: e.to_string(),
    })?;
    Ok(())
}

pub fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], _strict: bool) -> Result<(), IoError> {
    let file = File::create(path).map_err(|e| IoError {
        message: e.to_string(),
    })?;
    let mut writer = BufWriter::new(file);
    write_header_line(
        &mut writer,
        "     3.04           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE",
    )?;
    write_header_line(
        &mut writer,
        "bijux-gnss                              PGM / RUN BY / DATE",
    )?;
    write_header_line(
        &mut writer,
        "                                                            END OF HEADER",
    )?;

    for eph in ephs {
        let line = format!("G{:02} 0 0 0 0 0 0 0 0 0 0 0 0 0 0", eph.sat.prn);
        write_header_line(&mut writer, &line)?;
    }

    writer.flush().map_err(|e| IoError {
        message: e.to_string(),
    })?;
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
        Err(ParseError {
            message: "invalid or incomplete RINEX header".to_string(),
        })
    }
}
