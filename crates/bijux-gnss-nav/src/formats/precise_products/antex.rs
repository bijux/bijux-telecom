#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{Constellation, ParseError, SatId, SignalBand};
use time::{Date, Month, PrimitiveDateTime, Time};

use crate::models::antenna::{
    canonical_receiver_antenna_type, satellite_band_from_antex_frequency,
    AntennaAzimuthPhaseCenterVariation, AntennaPhaseCenterVariation, ReceiverAntennaCalibration,
    ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset, SatelliteAntennaCalibration,
    SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
};

pub fn parse_antex_satellite_calibrations(
    data: &str,
) -> Result<SatelliteAntennaCalibrations, ParseError> {
    let (satellite_calibrations, _) = parse_antex_calibrations(data)?;
    Ok(satellite_calibrations)
}

pub fn parse_antex_receiver_calibrations(
    data: &str,
) -> Result<ReceiverAntennaCalibrations, ParseError> {
    let (_, receiver_calibrations) = parse_antex_calibrations(data)?;
    Ok(receiver_calibrations)
}

fn parse_antex_calibrations(
    data: &str,
) -> Result<(SatelliteAntennaCalibrations, ReceiverAntennaCalibrations), ParseError> {
    let mut satellite_entries = Vec::new();
    let mut receiver_entries = Vec::new();
    let mut current: Option<PendingSatelliteAntennaCalibration> = None;
    let mut current_frequency: Option<PendingFrequencyCalibration> = None;

    for line in data.lines() {
        let label = antex_label(line);
        match label {
            "START OF ANTENNA" => {
                current = Some(PendingSatelliteAntennaCalibration::default());
                current_frequency = None;
            }
            "END OF ANTENNA" => {
                if let Some(frequency) = current_frequency.take() {
                    if let Some(pending) = current.as_mut() {
                        pending.store_frequency(frequency);
                    }
                }
                if let Some(pending) = current.take() {
                    let (satellite_entry, receiver_entry) = pending.finish();
                    if let Some(entry) = satellite_entry {
                        satellite_entries.push(entry);
                    }
                    if let Some(entry) = receiver_entry {
                        receiver_entries.push(entry);
                    }
                }
                current_frequency = None;
            }
            "TYPE / SERIAL NO" => {
                if let Some(pending) = current.as_mut() {
                    pending.antenna_type = field(line, 0, 20).trim().to_string();
                    pending.sat = parse_antex_satellite_id(field(line, 0, 40))?;
                    pending.receiver_antenna_type =
                        pending.sat.is_none().then(|| parse_antex_receiver_antenna_type(line));
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
                if let Some(frequency) = current_frequency.take() {
                    if let Some(pending) = current.as_mut() {
                        pending.store_frequency(frequency);
                    }
                }
                current_frequency =
                    Some(PendingFrequencyCalibration::new(field(line, 0, 20).trim().to_string()));
            }
            "END OF FREQUENCY" => {
                if let Some(frequency) = current_frequency.take() {
                    if let Some(pending) = current.as_mut() {
                        pending.store_frequency(frequency);
                    }
                }
            }
            "NORTH / EAST / UP" => {
                let Some(frequency) = current_frequency.as_mut() else {
                    continue;
                };
                frequency.satellite_offset = Some(parse_antex_offsets_m(line)?);
                frequency.receiver_offset = Some(parse_antex_receiver_offsets_m(line)?);
            }
            "ZEN1 / ZEN2 / DZEN" => {
                let Some(frequency) = current_frequency.as_mut() else {
                    continue;
                };
                let (zenith_start_deg, zenith_step_deg) = parse_antex_zenith_grid(line)?;
                frequency.zenith_start_deg = Some(zenith_start_deg);
                frequency.zenith_step_deg = Some(zenith_step_deg);
            }
            "NOAZI" => {
                let Some(frequency) = current_frequency.as_mut() else {
                    continue;
                };
                frequency.no_azimuth_values_m = parse_antex_phase_variation_values_m(line, true)?;
            }
            _ if label.is_empty() => {
                let Some(frequency) = current_frequency.as_mut() else {
                    continue;
                };
                if let Some(row) = parse_antex_azimuth_phase_variation_row_m(line)? {
                    frequency.azimuth_values_m.push(row);
                }
            }
            _ => {}
        }
    }

    Ok((
        SatelliteAntennaCalibrations { entries: satellite_entries },
        ReceiverAntennaCalibrations { entries: receiver_entries },
    ))
}

#[derive(Default)]
struct PendingSatelliteAntennaCalibration {
    sat: Option<SatId>,
    antenna_type: String,
    receiver_antenna_type: Option<String>,
    valid_from_unix_s: Option<f64>,
    valid_until_unix_s: Option<f64>,
    offsets_by_band: BTreeMap<SignalBand, SatellitePhaseCenterOffset>,
    variations_by_band: BTreeMap<SignalBand, AntennaPhaseCenterVariation>,
    receiver_offsets_by_band: BTreeMap<SignalBand, ReceiverPhaseCenterOffset>,
    receiver_variations_by_band: BTreeMap<SignalBand, AntennaPhaseCenterVariation>,
}

impl PendingSatelliteAntennaCalibration {
    fn store_frequency(&mut self, frequency: PendingFrequencyCalibration) {
        if let Some(sat) = self.sat {
            let Some(band) =
                satellite_band_from_antex_frequency(sat.constellation, &frequency.code)
            else {
                return;
            };
            if let Some(offset) = frequency.satellite_offset {
                self.offsets_by_band.insert(band, offset);
            }
            if let Some(variation) = frequency.phase_center_variation() {
                self.variations_by_band.insert(band, variation);
            }
            return;
        }

        if self.receiver_antenna_type.is_none() {
            return;
        }
        let Some(band) = parse_antex_receiver_band(&frequency.code) else {
            return;
        };
        if let Some(offset) = frequency.receiver_offset {
            self.receiver_offsets_by_band.insert(band, offset);
        }
        if let Some(variation) = frequency.phase_center_variation() {
            self.receiver_variations_by_band.insert(band, variation);
        }
    }

    fn finish(self) -> (Option<SatelliteAntennaCalibration>, Option<ReceiverAntennaCalibration>) {
        let satellite_entry = self.sat.and_then(|sat| {
            (!self.offsets_by_band.is_empty()).then_some(SatelliteAntennaCalibration {
                sat,
                antenna_type: self.antenna_type.clone(),
                valid_from_unix_s: self.valid_from_unix_s,
                valid_until_unix_s: self.valid_until_unix_s,
                offsets_by_band: self.offsets_by_band.clone(),
                variations_by_band: self.variations_by_band.clone(),
            })
        });
        let receiver_entry = self.receiver_antenna_type.and_then(|antenna_type| {
            (!self.receiver_offsets_by_band.is_empty()).then_some(ReceiverAntennaCalibration {
                antenna_type,
                valid_from_unix_s: self.valid_from_unix_s,
                valid_until_unix_s: self.valid_until_unix_s,
                offsets_by_band: self.receiver_offsets_by_band,
                variations_by_band: self.receiver_variations_by_band,
            })
        });
        (satellite_entry, receiver_entry)
    }
}

struct PendingFrequencyCalibration {
    code: String,
    satellite_offset: Option<SatellitePhaseCenterOffset>,
    receiver_offset: Option<ReceiverPhaseCenterOffset>,
    zenith_start_deg: Option<f64>,
    zenith_step_deg: Option<f64>,
    no_azimuth_values_m: Vec<f64>,
    azimuth_values_m: Vec<AntennaAzimuthPhaseCenterVariation>,
}

impl PendingFrequencyCalibration {
    fn new(code: String) -> Self {
        Self {
            code,
            satellite_offset: None,
            receiver_offset: None,
            zenith_start_deg: None,
            zenith_step_deg: None,
            no_azimuth_values_m: Vec::new(),
            azimuth_values_m: Vec::new(),
        }
    }

    fn phase_center_variation(&self) -> Option<AntennaPhaseCenterVariation> {
        if self.no_azimuth_values_m.is_empty() && self.azimuth_values_m.is_empty() {
            return None;
        }
        Some(AntennaPhaseCenterVariation {
            zenith_start_deg: self.zenith_start_deg?,
            zenith_step_deg: self.zenith_step_deg?,
            no_azimuth_values_m: self.no_azimuth_values_m.clone(),
            azimuth_values_m: self.azimuth_values_m.clone(),
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
    let month = Month::try_from(fields[1] as u8)
        .map_err(|_| ParseError { message: format!("invalid ANTEX month: {}", fields[1]) })?;
    let date = Date::from_calendar_date(fields[0], month, fields[2] as u8)
        .map_err(|_| ParseError { message: "invalid ANTEX calendar date".to_string() })?;
    let time = Time::from_hms(fields[3] as u8, fields[4] as u8, fields[5] as u8)
        .map_err(|_| ParseError { message: "invalid ANTEX clock time".to_string() })?;
    Ok(PrimitiveDateTime::new(date, time).assume_utc().unix_timestamp() as f64)
}

fn parse_antex_receiver_antenna_type(line: &str) -> String {
    let antenna_model = field(line, 0, 20).trim();
    let radome = field(line, 20, 40).trim();
    canonical_receiver_antenna_type(&format!("{antenna_model} {radome}"))
}

fn parse_antex_offsets_m(line: &str) -> Result<SatellitePhaseCenterOffset, ParseError> {
    let values = field(line, 0, 60)
        .split_whitespace()
        .take(3)
        .map(|value| {
            value
                .parse::<f64>()
                .map_err(|_| ParseError { message: format!("invalid ANTEX offset: {value}") })
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

fn parse_antex_receiver_offsets_m(line: &str) -> Result<ReceiverPhaseCenterOffset, ParseError> {
    let values = field(line, 0, 60)
        .split_whitespace()
        .take(3)
        .map(|value| {
            value.parse::<f64>().map_err(|_| ParseError {
                message: format!("invalid ANTEX receiver offset: {value}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    if values.len() != 3 {
        return Err(ParseError {
            message: "ANTEX receiver offsets must contain three fields".to_string(),
        });
    }
    Ok(ReceiverPhaseCenterOffset::new(
        values[0] / 1_000.0,
        values[1] / 1_000.0,
        values[2] / 1_000.0,
    ))
}

fn parse_antex_zenith_grid(line: &str) -> Result<(f64, f64), ParseError> {
    let values = parse_antex_numeric_values(line)?;
    if values.len() < 3 {
        return Err(ParseError {
            message: "ANTEX zenith grid must contain ZEN1, ZEN2, and DZEN".to_string(),
        });
    }
    Ok((values[0], values[2]))
}

fn parse_antex_phase_variation_values_m(
    line: &str,
    skip_record_name: bool,
) -> Result<Vec<f64>, ParseError> {
    let values = field(line, 0, 60)
        .split_whitespace()
        .skip(usize::from(skip_record_name))
        .map(|value| {
            value.parse::<f64>().map(|millimeters| millimeters / 1_000.0).map_err(|_| ParseError {
                message: format!("invalid ANTEX phase-center variation: {value}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(values)
}

fn parse_antex_azimuth_phase_variation_row_m(
    line: &str,
) -> Result<Option<AntennaAzimuthPhaseCenterVariation>, ParseError> {
    let tokens = field(line, 0, 60).split_whitespace().collect::<Vec<_>>();
    if tokens.len() < 2 {
        return Ok(None);
    }
    let azimuth_deg = match tokens[0].parse::<f64>() {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let values_m = tokens[1..]
        .iter()
        .map(|value| {
            value.parse::<f64>().map(|millimeters| millimeters / 1_000.0).map_err(|_| ParseError {
                message: format!("invalid ANTEX azimuth phase-center variation: {value}"),
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Some(AntennaAzimuthPhaseCenterVariation { azimuth_deg, values_m }))
}

fn parse_antex_numeric_values(line: &str) -> Result<Vec<f64>, ParseError> {
    field(line, 0, 60)
        .split_whitespace()
        .map(|value| {
            value.parse::<f64>().map_err(|_| ParseError {
                message: format!("invalid ANTEX numeric field: {value}"),
            })
        })
        .collect()
}

fn parse_antex_receiver_band(frequency_code: &str) -> Option<SignalBand> {
    let constellation = match frequency_code.trim().chars().next()? {
        'G' => Constellation::Gps,
        'E' => Constellation::Galileo,
        'C' => Constellation::Beidou,
        _ => return None,
    };
    satellite_band_from_antex_frequency(constellation, frequency_code)
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

    use super::{parse_antex_receiver_calibrations, parse_antex_satellite_calibrations};

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
        assert_eq!(entry.offsets_by_band.get(&SignalBand::L1).expect("L1 offset").body_z_m, 1.0);
        assert_eq!(entry.offsets_by_band.get(&SignalBand::L2).expect("L2 offset").body_x_m, 0.2);
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

    #[test]
    fn parse_antex_receiver_calibrations_reads_type_and_band_offsets() {
        let antex = [
            line("", "START OF ANTENNA"),
            line(&format!("{:<20}{:<20}{:<20}", "AOAD/M_T", "NONE", "12345"), "TYPE / SERIAL NO"),
            line("  2020  01  01  00  00  00", "VALID FROM"),
            line("G01", "START OF FREQUENCY"),
            line("   100.0   200.0  1200.0", "NORTH / EAST / UP"),
            line("", "END OF FREQUENCY"),
            line("G02", "START OF FREQUENCY"),
            line("   400.0   500.0  1600.0", "NORTH / EAST / UP"),
            line("", "END OF FREQUENCY"),
            line("", "END OF ANTENNA"),
        ]
        .join("");

        let calibrations =
            parse_antex_receiver_calibrations(&antex).expect("parse ANTEX receiver offsets");

        assert_eq!(calibrations.entries.len(), 1);
        let entry = &calibrations.entries[0];
        assert_eq!(entry.antenna_type, "AOAD/M_T NONE");
        assert_eq!(
            entry.offsets_by_band.get(&SignalBand::L1).expect("L1 receiver offset").up_m,
            1.2
        );
        assert_eq!(
            entry.offsets_by_band.get(&SignalBand::L2).expect("L2 receiver offset").east_m,
            0.5
        );
    }

    #[test]
    fn parse_antex_satellite_calibrations_reads_no_azimuth_variations() {
        let antex = [
            line("", "START OF ANTENNA"),
            line(&format!("{:<20}{:<20}", "GPS TEST", "G01"), "TYPE / SERIAL NO"),
            line("  2020  01  01  00  00  00", "VALID FROM"),
            line("G01", "START OF FREQUENCY"),
            line("   100.0     0.0  1000.0", "NORTH / EAST / UP"),
            line("     0.0    20.0    10.0", "ZEN1 / ZEN2 / DZEN"),
            line("NOAZI       0.0    10.0    20.0", "NOAZI"),
            line("", "END OF FREQUENCY"),
            line("", "END OF ANTENNA"),
        ]
        .join("");

        let calibrations =
            parse_antex_satellite_calibrations(&antex).expect("parse ANTEX satellite PCV");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        let variation_m = calibrations
            .phase_center_variation_m(
                sat,
                SignalBand::L1,
                Some(GpsTime { week: 2138, tow_s: 259_218.0 }),
                75.0,
                None,
            )
            .expect("satellite NOAZI PCV");

        assert!((variation_m - 0.015).abs() < 1.0e-12);
    }

    #[test]
    fn parse_antex_receiver_calibrations_reads_azimuth_variations() {
        let antex = [
            line("", "START OF ANTENNA"),
            line(&format!("{:<20}{:<20}{:<20}", "AOAD/M_T", "NONE", "12345"), "TYPE / SERIAL NO"),
            line("G01", "START OF FREQUENCY"),
            line("   100.0   200.0  1200.0", "NORTH / EAST / UP"),
            line("    0.0    20.0    10.0", "ZEN1 / ZEN2 / DZEN"),
            line("NOAZI       0.0    10.0    20.0", "NOAZI"),
            line("    0.0     0.0    20.0    40.0", ""),
            line("   90.0    10.0    30.0    50.0", ""),
            line("", "END OF FREQUENCY"),
            line("", "END OF ANTENNA"),
        ]
        .join("");

        let calibrations =
            parse_antex_receiver_calibrations(&antex).expect("parse ANTEX receiver PCV");

        let variation_m = calibrations
            .phase_center_variation_m("aoad/m_t none", SignalBand::L1, None, 75.0, Some(45.0))
            .expect("receiver azimuth PCV");

        assert!((variation_m - 0.035).abs() < 1.0e-12);
    }

    fn line(value: &str, label: &str) -> String {
        format!("{value:<60}{label}\n")
    }
}
