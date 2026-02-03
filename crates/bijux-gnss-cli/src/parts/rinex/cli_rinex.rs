fn write_rinex_obs(path: &Path, epochs: &[ObsEpoch], strict: bool) -> Result<()> {
    let mut out = String::new();
    push_rinex_line(
        &mut out,
        "     3.03           OBSERVATION DATA    G                   RINEX VERSION / TYPE",
    );
    push_rinex_line(
        &mut out,
        "BIJUX GNSS                               MARKER NAME",
    );
    push_rinex_line(
        &mut out,
        "G    4 C1C L1C D1C S1C                                      SYS / # / OBS TYPES",
    );
    push_rinex_line(
        &mut out,
        "                                                            END OF HEADER",
    );
    for epoch in epochs {
        let (year, month, day, hour, min, sec) = gps_epoch_to_ymdhms(epoch);
        push_rinex_line(
            &mut out,
            &format!(
                "> {:04} {:02} {:02} {:02} {:02} {:011.7}  0 {:2}",
                year,
                month,
                day,
                hour,
                min,
                sec,
                epoch.sats.len()
            ),
        );
        for sat in &epoch.sats {
            let line = format!(
                "G{:02}{}{}{}{}",
                sat.signal_id.sat.prn,
                format_f14_3(sat.pseudorange_m),
                format_f14_6(sat.carrier_phase_cycles),
                format_f14_3(sat.doppler_hz),
                format_f14_3(sat.cn0_dbhz)
            );
            push_rinex_line(&mut out, &line);
        }
    }
    fs::write(path, out)?;
    if strict {
        validate_rinex_file(path)?;
    }
    Ok(())
}

fn write_rinex_nav(path: &Path, ephs: &[GpsEphemeris], strict: bool) -> Result<()> {
    let mut out = String::new();
    push_rinex_line(
        &mut out,
        "     3.03           NAVIGATION DATA     G                   RINEX VERSION / TYPE",
    );
    push_rinex_line(
        &mut out,
        "                                                            END OF HEADER",
    );
    for eph in ephs {
        let (year, month, day, hour, min, sec) = gps_week_tow_to_ymdhms(eph.week, eph.toc_s);
        let line1 = format!(
            "G{:02} {:04} {:02} {:02} {:02} {:02} {:02}{}{}{}",
            eph.sat.prn,
            year,
            month,
            day,
            hour,
            min,
            sec,
            format_e19_12(eph.af0),
            format_e19_12(eph.af1),
            format_e19_12(eph.af2)
        );
        push_rinex_line(&mut out, &line1);
        let line2 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.iode as f64),
            format_e19_12(eph.crs),
            format_e19_12(eph.delta_n),
            format_e19_12(eph.m0)
        );
        push_rinex_line(&mut out, &line2);
        let line3 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.cuc),
            format_e19_12(eph.e),
            format_e19_12(eph.cus),
            format_e19_12(eph.sqrt_a)
        );
        push_rinex_line(&mut out, &line3);
        let line4 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.toe_s),
            format_e19_12(eph.cic),
            format_e19_12(eph.omega0),
            format_e19_12(eph.cis)
        );
        push_rinex_line(&mut out, &line4);
        let line5 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.i0),
            format_e19_12(eph.crc),
            format_e19_12(eph.w),
            format_e19_12(eph.omegadot)
        );
        push_rinex_line(&mut out, &line5);
        let line6 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.idot),
            format_e19_12(0.0_f64),
            format_e19_12(eph.week as f64),
            format_e19_12(0.0_f64)
        );
        push_rinex_line(&mut out, &line6);
        let line7 = format!(
            "    {}{}{}{}",
            format_e19_12(eph.tgd),
            format_e19_12(eph.iodc as f64),
            format_e19_12(0.0_f64),
            format_e19_12(0.0_f64)
        );
        push_rinex_line(&mut out, &line7);
        push_rinex_line(
            &mut out,
            "    0.000000000000E+00 0.000000000000E+00 0.000000000000E+00 0.000000000000E+00",
        );
    }
    fs::write(path, out)?;
    if strict {
        validate_rinex_file(path)?;
    }
    Ok(())
}

fn push_rinex_line(out: &mut String, line: &str) {
    if line.len() >= 80 {
        out.push_str(&line[..80]);
    } else {
        out.push_str(&format!("{:<80}", line));
    }
    out.push('\n');
}

fn format_e19_12(value: f64) -> String {
    format!("{:>19.12E}", value)
}

fn format_f14_3(value: f64) -> String {
    format!("{:>14.3}", value)
}

fn format_f14_6(value: f64) -> String {
    format!("{:>14.6}", value)
}

fn validate_rinex_file(path: &Path) -> Result<()> {
    let data = fs::read_to_string(path)?;
    for (idx, line) in data.lines().enumerate() {
        if line.len() != 80 {
            bail!("RINEX line length != 80 at {}:{}", path.display(), idx + 1);
        }
    }
    Ok(())
}

fn gps_epoch_to_ymdhms(epoch: &ObsEpoch) -> (i32, u32, u32, u32, u32, f64) {
    let week = epoch.gps_week.unwrap_or(0);
    let tow = epoch.tow_s.unwrap_or(epoch.t_rx_s);
    gps_week_tow_to_ymdhms(week, tow)
}

fn gps_week_tow_to_ymdhms(week: u16, tow_s: f64) -> (i32, u32, u32, u32, u32, f64) {
    let gps_epoch_days = 365 * 10 + 2 + 5; // 1980-01-06 offset in days from 1970-01-01
    let total_days = gps_epoch_days as f64 + week as f64 * 7.0 + (tow_s / 86400.0);
    let (year, month, day) = days_to_ymd(total_days.floor() as i64);
    let sec_of_day = tow_s % 86400.0;
    let hour = (sec_of_day / 3600.0).floor() as u32;
    let min = ((sec_of_day - hour as f64 * 3600.0) / 60.0).floor() as u32;
    let sec = sec_of_day - hour as f64 * 3600.0 - min as f64 * 60.0;
    (year, month, day, hour, min, sec)
}

fn days_to_ymd(days_since_1970: i64) -> (i32, u32, u32) {
    let mut days = days_since_1970;
    let mut year = 1970;
    loop {
        let leap = is_leap(year);
        let year_days = if leap { 366 } else { 365 };
        if days >= year_days {
            days -= year_days;
            year += 1;
        } else {
            break;
        }
    }
    let month_days = [
        31,
        if is_leap(year) { 29 } else { 28 },
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
    let mut month = 1;
    for md in month_days {
        if days >= md {
            days -= md;
            month += 1;
        } else {
            break;
        }
    }
    (year, month as u32, (days + 1) as u32)
}

fn is_leap(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}
