#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::gps::{GpsSatState, GpsSatelliteClockCorrection};

#[derive(Debug, Clone)]
pub struct Sp3Record {
    pub epoch_s: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
}

#[derive(Debug, Clone)]
pub struct Sp3Provider {
    records: BTreeMap<SatId, Vec<Sp3Record>>,
}

impl Sp3Provider {
    fn parse_internal(input: &str) -> Result<Self, String> {
        let mut records: BTreeMap<SatId, Vec<Sp3Record>> = BTreeMap::new();
        let mut current_epoch = None;
        for line in input.lines() {
            if line.starts_with('*') {
                current_epoch = Some(parse_sp3_epoch(line)?);
                continue;
            }
            if let Some(stripped) = line.strip_prefix('P') {
                let epoch_s = current_epoch.ok_or("SP3 record without epoch")?;
                let sat = parse_sat_id(line)?;
                let parts: Vec<&str> = stripped.split_whitespace().collect();
                if parts.len() < 4 {
                    continue;
                }
                let x_km: f64 = parts[1].parse().map_err(|_| "invalid x")?;
                let y_km: f64 = parts[2].parse().map_err(|_| "invalid y")?;
                let z_km: f64 = parts[3].parse().map_err(|_| "invalid z")?;
                records.entry(sat).or_default().push(Sp3Record {
                    epoch_s,
                    x_m: x_km * 1000.0,
                    y_m: y_km * 1000.0,
                    z_m: z_km * 1000.0,
                });
            }
        }
        normalize_records(&mut records);
        Ok(Self { records })
    }

    pub fn coverage_s(&self, sat: SatId) -> Option<(f64, f64)> {
        let list = self.records.get(&sat)?;
        let first = list.first()?.epoch_s;
        let last = list.last()?.epoch_s;
        Some((first, last))
    }

    pub fn sat_state(&self, sat: SatId, t_s: f64) -> Option<GpsSatState> {
        let list = self.records.get(&sat)?;
        if let Some(record) = list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return Some(record_state(record));
        }
        if list.len() == 1 {
            return Some(record_state(&list[0]));
        }
        let mut before = None;
        let mut after = None;
        for rec in list {
            if rec.epoch_s <= t_s {
                before = Some(rec);
            }
            if rec.epoch_s >= t_s {
                after = Some(rec);
                break;
            }
        }
        match (before, after) {
            (Some(b), Some(a)) if (a.epoch_s - b.epoch_s).abs() > 0.0 => {
                let w = (t_s - b.epoch_s) / (a.epoch_s - b.epoch_s);
                Some(GpsSatState {
                    x_m: b.x_m + w * (a.x_m - b.x_m),
                    y_m: b.y_m + w * (a.y_m - b.y_m),
                    z_m: b.z_m + w * (a.z_m - b.z_m),
                    clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
                })
            }
            (Some(b), Some(_)) => Some(record_state(b)),
            _ => None,
        }
    }
}

impl FromStr for Sp3Provider {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::parse_internal(s)
    }
}

fn parse_sp3_epoch(line: &str) -> Result<f64, String> {
    let parts: Vec<&str> = line[1..].split_whitespace().collect();
    if parts.len() < 6 {
        return Err("invalid SP3 epoch".to_string());
    }
    let year: i32 = parts[0].parse().map_err(|_| "invalid year")?;
    let month: u32 = parts[1].parse().map_err(|_| "invalid month")?;
    let day: u32 = parts[2].parse().map_err(|_| "invalid day")?;
    let hour: u32 = parts[3].parse().map_err(|_| "invalid hour")?;
    let min: u32 = parts[4].parse().map_err(|_| "invalid min")?;
    let sec: f64 = parts[5].parse().map_err(|_| "invalid sec")?;
    let days = days_from_civil(year, month, day);
    Ok(days as f64 * 86_400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec)
}

fn parse_sat_id(line: &str) -> Result<SatId, String> {
    let id = line.chars().skip(1).take(3).collect::<String>().trim().to_string();
    if id.len() < 2 {
        return Err("invalid sat id".to_string());
    }
    let sys = id.chars().next().unwrap_or('G');
    let prn: u8 = id[1..].trim().parse().map_err(|_| "invalid prn")?;
    let constellation = match sys {
        'G' => Constellation::Gps,
        'E' => Constellation::Galileo,
        'R' => Constellation::Glonass,
        'C' => Constellation::Beidou,
        _ => Constellation::Unknown,
    };
    Ok(SatId { constellation, prn })
}

fn normalize_records(records: &mut BTreeMap<SatId, Vec<Sp3Record>>) {
    let Some(epoch_origin_s) = records
        .values()
        .filter_map(|sat_records| sat_records.iter().map(|record| record.epoch_s).min_by(f64::total_cmp))
        .min_by(f64::total_cmp)
    else {
        return;
    };

    for sat_records in records.values_mut() {
        sat_records.sort_by(|left, right| left.epoch_s.total_cmp(&right.epoch_s));
        for record in sat_records.iter_mut() {
            record.epoch_s -= epoch_origin_s;
        }
        sat_records.dedup_by(|left, right| (left.epoch_s - right.epoch_s).abs() <= f64::EPSILON);
    }
}

fn record_state(record: &Sp3Record) -> GpsSatState {
    GpsSatState {
        x_m: record.x_m,
        y_m: record.y_m,
        z_m: record.z_m,
        clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
    }
}

fn days_from_civil(year: i32, month: u32, day: u32) -> i64 {
    let y = year - (month <= 2) as i32;
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let m = month as i32;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + day as i32 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    (era * 146_097 + doe - 719_468) as i64
}

#[cfg(test)]
mod tests {
    use super::Sp3Provider;
    use bijux_gnss_core::api::{Constellation, SatId};

    #[test]
    fn parse_internal_sorts_and_deduplicates_satellite_epochs() {
        let data = "\
* 2020 01 01 00 15 00.000000
PG01  10001.000000  20001.000000  30001.000000  0.000000
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10001.000000  20001.000000  30001.000000  0.000000
";
        let provider: Sp3Provider = data.parse().expect("parse SP3");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let records = provider.records.get(&sat).expect("satellite records");

        assert_eq!(records.len(), 2);
        assert_eq!(records[0].epoch_s, 0.0);
        assert_eq!(records[1].epoch_s, 900.0);
    }

    #[test]
    fn sat_state_returns_exact_epoch_record_without_interpolation() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10001.000000  20001.000000  30001.000000  0.000000
";
        let provider: Sp3Provider = data.parse().expect("parse SP3");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let state = provider.sat_state(sat, 900.0).expect("exact epoch state");

        assert_eq!(state.x_m, 10_001_000.0);
        assert_eq!(state.y_m, 20_001_000.0);
        assert_eq!(state.z_m, 30_001_000.0);
    }
}
