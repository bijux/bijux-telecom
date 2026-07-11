#![allow(dead_code, missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_core::api::{Constellation, GpsTime, SatId};

#[derive(Debug, Clone, PartialEq)]
pub struct RtkLibReferenceEpoch {
    pub gps_time: GpsTime,
    pub quality_flag: u8,
    pub ecef_m: (f64, f64, f64),
    pub clock_bias_ns: f64,
    pub residuals: Vec<RtkLibReferenceResidual>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RtkLibReferenceResidual {
    pub sat: SatId,
    pub azimuth_deg: f64,
    pub elevation_deg: f64,
    pub residual_m: f64,
    pub snr_dbhz: f64,
}

pub fn ab43_rtklib_single_reference() -> &'static Vec<RtkLibReferenceEpoch> {
    static REFERENCE: OnceLock<Vec<RtkLibReferenceEpoch>> = OnceLock::new();
    REFERENCE.get_or_init(|| {
        parse_rtklib_single_reference(&fixture("ab43_20180114_rtklib_single.pos.stat"))
            .expect("parse AB43 RTKLIB single-point residual reference")
    })
}

fn parse_rtklib_single_reference(data: &str) -> Result<Vec<RtkLibReferenceEpoch>, String> {
    let mut epochs = Vec::new();
    let mut current_epoch = None;

    for (index, line) in data.lines().enumerate() {
        let line_number = index + 1;
        let fields = line.split(',').collect::<Vec<_>>();
        if fields.is_empty() {
            continue;
        }
        match fields[0] {
            "$POS" => {
                if let Some(epoch) = current_epoch.take() {
                    epochs.push(epoch);
                }
                current_epoch = Some(parse_pos_line(line_number, &fields)?);
            }
            "$VELACC" => {}
            "$CLK" => {
                let epoch = current_epoch.as_mut().ok_or_else(|| {
                    format!("RTKLIB reference line {line_number} declares clock before position")
                })?;
                let (gps_time, quality_flag, clock_bias_ns) = parse_clk_line(line_number, &fields)?;
                ensure_epoch_alignment(line_number, epoch, gps_time, quality_flag)?;
                epoch.clock_bias_ns = clock_bias_ns;
            }
            "$SAT" => {
                let epoch = current_epoch.as_mut().ok_or_else(|| {
                    format!("RTKLIB reference line {line_number} declares residual before position")
                })?;
                let (gps_time, residual) = parse_sat_line(line_number, &fields)?;
                ensure_epoch_time_alignment(line_number, epoch, gps_time)?;
                epoch.residuals.push(residual);
            }
            other => {
                return Err(format!(
                    "unsupported RTKLIB reference record '{other}' on line {line_number}"
                ));
            }
        }
    }

    if let Some(epoch) = current_epoch.take() {
        epochs.push(epoch);
    }
    if epochs.is_empty() {
        return Err("RTKLIB reference fixture is empty".to_string());
    }

    Ok(epochs)
}

fn parse_pos_line(line_number: usize, fields: &[&str]) -> Result<RtkLibReferenceEpoch, String> {
    if fields.len() < 7 {
        return Err(format!(
            "RTKLIB $POS line {line_number} expected at least 7 fields, found {}",
            fields.len()
        ));
    }

    Ok(RtkLibReferenceEpoch {
        gps_time: GpsTime {
            week: parse_u32_field(line_number, "$POS week", fields[1])?,
            tow_s: parse_f64_field(line_number, "$POS tow_s", fields[2])?,
        },
        quality_flag: parse_u8_field(line_number, "$POS quality_flag", fields[3])?,
        ecef_m: (
            parse_f64_field(line_number, "$POS x_m", fields[4])?,
            parse_f64_field(line_number, "$POS y_m", fields[5])?,
            parse_f64_field(line_number, "$POS z_m", fields[6])?,
        ),
        clock_bias_ns: 0.0,
        residuals: Vec::new(),
    })
}

fn parse_clk_line(line_number: usize, fields: &[&str]) -> Result<(GpsTime, u8, f64), String> {
    if fields.len() < 6 {
        return Err(format!(
            "RTKLIB $CLK line {line_number} expected at least 6 fields, found {}",
            fields.len()
        ));
    }

    Ok((
        GpsTime {
            week: parse_u32_field(line_number, "$CLK week", fields[1])?,
            tow_s: parse_f64_field(line_number, "$CLK tow_s", fields[2])?,
        },
        parse_u8_field(line_number, "$CLK quality_flag", fields[3])?,
        parse_f64_field(line_number, "$CLK clock_bias_ns", fields[5])?,
    ))
}

fn parse_sat_line(
    line_number: usize,
    fields: &[&str],
) -> Result<(GpsTime, RtkLibReferenceResidual), String> {
    if fields.len() < 11 {
        return Err(format!(
            "RTKLIB $SAT line {line_number} expected at least 11 fields, found {}",
            fields.len()
        ));
    }

    Ok((
        GpsTime {
            week: parse_u32_field(line_number, "$SAT week", fields[1])?,
            tow_s: parse_f64_field(line_number, "$SAT tow_s", fields[2])?,
        },
        RtkLibReferenceResidual {
            sat: parse_sat_token(line_number, fields[3])?,
            azimuth_deg: parse_f64_field(line_number, "$SAT azimuth_deg", fields[5])?,
            elevation_deg: parse_f64_field(line_number, "$SAT elevation_deg", fields[6])?,
            residual_m: parse_f64_field(line_number, "$SAT residual_m", fields[7])?,
            snr_dbhz: parse_f64_field(line_number, "$SAT snr_dbhz", fields[10])?,
        },
    ))
}

fn ensure_epoch_alignment(
    line_number: usize,
    epoch: &RtkLibReferenceEpoch,
    gps_time: GpsTime,
    quality_flag: u8,
) -> Result<(), String> {
    if epoch.gps_time != gps_time {
        return Err(format!(
            "RTKLIB reference line {line_number} time {:?} does not match active epoch {:?}",
            gps_time, epoch.gps_time
        ));
    }
    if epoch.quality_flag != quality_flag {
        return Err(format!(
            "RTKLIB reference line {line_number} quality flag {quality_flag} does not match active epoch {}",
            epoch.quality_flag
        ));
    }
    Ok(())
}

fn ensure_epoch_time_alignment(
    line_number: usize,
    epoch: &RtkLibReferenceEpoch,
    gps_time: GpsTime,
) -> Result<(), String> {
    if epoch.gps_time != gps_time {
        return Err(format!(
            "RTKLIB reference line {line_number} time {:?} does not match active epoch {:?}",
            gps_time, epoch.gps_time
        ));
    }
    Ok(())
}

fn parse_sat_token(line_number: usize, token: &str) -> Result<SatId, String> {
    let trimmed = token.trim();
    if trimmed.len() != 3 {
        return Err(format!(
            "RTKLIB satellite token on line {line_number} must have 3 characters, found '{trimmed}'"
        ));
    }
    let Some(prn_text) = trimmed.get(1..) else {
        return Err(format!("RTKLIB satellite token on line {line_number} is truncated"));
    };
    let constellation = match trimmed.chars().next().expect("non-empty token") {
        'G' => Constellation::Gps,
        other => {
            return Err(format!(
                "RTKLIB satellite token on line {line_number} uses unsupported constellation '{other}'"
            ));
        }
    };
    Ok(SatId {
        constellation,
        prn: prn_text.parse::<u8>().map_err(|err| {
            format!("invalid RTKLIB PRN '{prn_text}' on line {line_number}: {err}")
        })?,
    })
}

fn parse_u8_field(line_number: usize, field_name: &str, value: &str) -> Result<u8, String> {
    value
        .trim()
        .parse::<u8>()
        .map_err(|err| format!("invalid {field_name} on line {line_number}: {err}"))
}

fn parse_u32_field(line_number: usize, field_name: &str, value: &str) -> Result<u32, String> {
    value
        .trim()
        .parse::<u32>()
        .map_err(|err| format!("invalid {field_name} on line {line_number}: {err}"))
}

fn parse_f64_field(line_number: usize, field_name: &str, value: &str) -> Result<f64, String> {
    value
        .trim()
        .parse::<f64>()
        .map_err(|err| format!("invalid {field_name} on line {line_number}: {err}"))
}

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("read RTKLIB reference fixture {}", path.display()))
}
