use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use bijux_gnss_core::api::{
    Constellation, GpsTime, IoError, ObsEpoch, ObsSatellite, ParseError, SatId, SignalBand,
    SignalCode,
};

use crate::formats::rinex_obs::{
    format_rinex_observation_dataset, RinexObservationDataset, RinexObservationEpoch,
    RinexObservationEpochTime, RinexObservationRecord, RinexObservationTimeSystem,
    RinexObservationValue, RinexSatelliteObservation,
};

const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;

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
