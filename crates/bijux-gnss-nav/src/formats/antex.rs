#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, ParseError, SatId};
use time::{Date, Month, PrimitiveDateTime, Time};

use crate::models::antenna::{
    satellite_band_from_antex_frequency, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
    SatellitePhaseCenterOffset,
};

pub fn parse_antex_satellite_calibrations(
    data: &str,
) -> Result<SatelliteAntennaCalibrations, ParseError> {
    let mut entries = Vec::new();
    let mut current: Option<PendingSatelliteAntennaCalibration> = None;
    let mut current_frequency: Option<String> = None;

    for line in data.lines() {
        let label = antex_label(line);
        match label {
            "START OF ANTENNA" => {
                current = Some(PendingSatelliteAntennaCalibration::default());
                current_frequency = None;
            }
            "END OF ANTENNA" => {
                if let Some(pending) = current.take() {
                    if let Some(entry) = pending.finish() {
                        entries.push(entry);
                    }
                }
                current_frequency = None;
            }
            "TYPE / SERIAL NO" => {
                if let Some(pending) = current.as_mut() {
                    pending.antenna_type = field(line, 0, 20).trim().to_string();
                    pending.sat = parse_antex_satellite_id(&field(line, 0, 40))?;
                }
            }
            "VALID FROM" => {
                if let Some(pending) = current.as_mut() {
                    pending.valid_from_unix_s = Some(parse_antex_datetime_unix_s(line)?);
                }
            }
            "VALID UNTIL" => {
                if let Some(pending) = current.as_mut() {
                    pending.valid_until_unix_s = Some(parse_antex_datetime_unix_s(line)?);
                }
            }
            "START OF FREQUENCY" => {
                current_frequency = Some(field(line, 0, 20).trim().to_string());
            }
            "END OF FREQUENCY" => {
                current_frequency = None;
            }
            "NORTH / EAST / UP" => {
                let Some(pending) = current.as_mut() else {
                    continue;
                };
                let Some(frequency_code) = current_frequency.as_deref() else {
                    continue;
                };
                let Some(sat) = pending.sat else {
                    continue;
                };
                let Some(band) =
                    satellite_band_from_antex_frequency(sat.constellation, frequency_code)
                else {
                    continue;
                };
                let offsets = parse_antex_offsets_m(line)?;
                pending.offsets_by_band.insert(band, offsets);
            }
            _ => {}
        }
    }

    Ok(SatelliteAntennaCalibrations { entries })
}

#[derive(Default)]
struct PendingSatelliteAntennaCalibration {
    sat: Option<SatId>,
    antenna_type: String,
    valid_from_unix_s: Option<f64>,
    valid_until_unix_s: Option<f64>,
    offsets_by_band: BTreeMap<bijux_gnss_core::api::SignalBand, SatellitePhaseCenterOffset>,
}

impl PendingSatelliteAntennaCalibration {
    fn finish(self) -> Option<SatelliteAntennaCalibration> {
        let sat = self.sat?;
        if self.offsets_by_band.is_empty() {
            return None;
        }
        Some(SatelliteAntennaCalibration {
            sat,
            antenna_type: self.antenna_type,
            valid_from_unix_s: self.valid_from_unix_s,
            valid_until_unix_s: self.valid_until_unix_s,
            offsets_by_band: self.offsets_by_band,
        })
    }
}

fn parse_antex_satellite_id(value: &str) -> Result<Option<SatId>, ParseError> {
    for token in value.split_whitespace() {
        if token.len() == 3 && token[1..].chars().all(|ch| ch.is_ascii_digit()) {
            let constellation = match &token[..1] {
                "G" => Constellation::Gps,
                "E" => Constellation::Galileo,
                "C" => Constellation::Beidou,
                _ => continue,
            };
            let prn = token[1..].parse::<u8>().map_err(|_| ParseError {
                message: format!("invalid ANTEX satellite id: {token}"),
            })?;
            return Ok(Some(SatId { constellation, prn }));
        }
    }
    Ok(None)
}

fn parse_antex_datetime_unix_s(line: &str) -> Result<f64, ParseError> {
    let fields = field(line, 0, 60)
        .split_whitespace()
        .take(6)
        .map(|value| {
            value.parse::<i32>().map_err(|_| ParseError {
                message: format!("invalid ANTEX datetime field: {value}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    if fields.len() != 6 {
        return Err(ParseError { message: "ANTEX datetime must contain six fields".to_string() });
    }
    let month = Month::try_from(fields[1] as u8).map_err(|_| ParseError {
        message: format!("invalid ANTEX month: {}", fields[1]),
    })?;
    let date = Date::from_calendar_date(fields[0], month, fields[2] as u8).map_err(|_| {
        ParseError { message: "invalid ANTEX calendar date".to_string() }
    })?;
    let time = Time::from_hms(fields[3] as u8, fields[4] as u8, fields[5] as u8).map_err(|_| {
        ParseError { message: "invalid ANTEX clock time".to_string() }
    })?;
    Ok(PrimitiveDateTime::new(date, time).assume_utc().unix_timestamp() as f64)
}

fn parse_antex_offsets_m(line: &str) -> Result<SatellitePhaseCenterOffset, ParseError> {
    let values = field(line, 0, 60)
        .split_whitespace()
        .take(3)
        .map(|value| {
            value.parse::<f64>().map_err(|_| ParseError {
                message: format!("invalid ANTEX offset: {value}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 3 {
        return Err(ParseError { message: "ANTEX offsets must contain three fields".to_string() });
    }
    Ok(SatellitePhaseCenterOffset::new(
        values[0] / 1_000.0,
        values[1] / 1_000.0,
        values[2] / 1_000.0,
    ))
}

fn antex_label(line: &str) -> &str {
    line.get(60..).unwrap_or("").trim()
}

fn field(line: &str, start: usize, end: usize) -> &str {
    line.get(start..end).unwrap_or("")
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};

    use super::parse_antex_satellite_calibrations;

    #[test]
    fn parse_antex_satellite_calibrations_reads_band_offsets_and_validity() {
        let antex = [
            line("", "START OF ANTENNA"),
            line(&format!("{:<20}{:<20}", "GPS TEST", "G01"), "TYPE / SERIAL NO"),
            line("  2020  01  01  00  00  00", "VALID FROM"),
            line("  2021  01  01  00  00  00", "VALID UNTIL"),
            line("G01", "START OF FREQUENCY"),
            line("   100.0     0.0  1000.0", "NORTH / EAST / UP"),
            line("", "END OF FREQUENCY"),
            line("G02", "START OF FREQUENCY"),
            line("   200.0     0.0  1200.0", "NORTH / EAST / UP"),
            line("", "END OF FREQUENCY"),
            line("", "END OF ANTENNA"),
        ]
        .join("");

        let calibrations =
            parse_antex_satellite_calibrations(&antex).expect("parse ANTEX satellite offsets");

        assert_eq!(calibrations.entries.len(), 1);
        let entry = &calibrations.entries[0];
        assert_eq!(entry.sat, SatId { constellation: Constellation::Gps, prn: 1 });
        assert_eq!(
            entry.offsets_by_band.get(&SignalBand::L1).expect("L1 offset").body_z_m,
            1.0
        );
        assert_eq!(
            entry.offsets_by_band.get(&SignalBand::L2).expect("L2 offset").body_x_m,
            0.2
        );
        assert_eq!(
            calibrations
                .phase_center_offset(
                    entry.sat,
                    SignalBand::L2,
                    Some(GpsTime { week: 2138, tow_s: 259_218.0 })
                )
                .expect("valid L2 offset")
                .body_z_m,
            1.2
        );
    }

    fn line(value: &str, label: &str) -> String {
        format!("{value:<60}{label}\n")
    }
}
