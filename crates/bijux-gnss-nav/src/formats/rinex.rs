use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use eyre::Result;

use bijux_gnss_core::ObsEpoch;

use crate::GpsEphemeris;

fn write_header_line(writer: &mut BufWriter<File>, line: &str) -> Result<()> {
    let mut out = line.to_string();
    if out.len() > 80 {
        out.truncate(80);
    }
    writeln!(writer, "{out}")?;
    Ok(())
}

pub fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], _strict: bool) -> Result<()> {
    let file = File::create(path)?;
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
            1980, 1, 6, 0, 0, epoch.t_rx_s, 0, sat_count
        );
        write_header_line(&mut writer, &line)?;
    }

    writer.flush()?;
    Ok(())
}

pub fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], _strict: bool) -> Result<()> {
    let file = File::create(path)?;
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

    writer.flush()?;
    Ok(())
}
