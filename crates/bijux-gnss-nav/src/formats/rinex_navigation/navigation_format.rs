use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use bijux_gnss_core::api::{gps_to_utc, GpsTime, IoError, LeapSeconds};

use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::{GpsBroadcastNavigationData, GpsEphemeris};
use crate::time::{
    beidou_to_gps_with_offset, galileo_to_gps_with_offset, BeidouTime, GalileoGpsTimeOffset,
    GalileoTime,
};

use super::{RinexBroadcastNavigationDataset, RinexNavigationTimeSystemCorrection};

fn push_rinex_line(output: &mut String, line: &str) {
    if line.len() > 80 {
        output.push_str(&line[..80]);
    } else {
        output.push_str(line);
    }
    output.push('\n');
}

pub(super) fn format_rinex_nav_float(value: f64) -> String {
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
