#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::gps::GpsSatState;

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
        let mut epoch0 = None;
        let mut current_epoch = None;
        for line in input.lines() {
            if line.starts_with('*') {
                let abs = parse_sp3_epoch(line)?;
                let base = *epoch0.get_or_insert(abs);
                current_epoch = Some(abs - base);
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
        if list.len() == 1 {
            let r = &list[0];
            return Some(GpsSatState {
                x_m: r.x_m,
                y_m: r.y_m,
                z_m: r.z_m,
                clock_bias_s: 0.0,
                clock_drift_s: 0.0,
                relativistic_s: 0.0,
            });
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
                    clock_bias_s: 0.0,
                    clock_drift_s: 0.0,
                    relativistic_s: 0.0,
                })
            }
            (Some(b), Some(_)) => Some(GpsSatState {
                x_m: b.x_m,
                y_m: b.y_m,
                z_m: b.z_m,
                clock_bias_s: 0.0,
                clock_drift_s: 0.0,
                relativistic_s: 0.0,
            }),
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

fn days_from_civil(year: i32, month: u32, day: u32) -> i64 {
    let y = year - (month <= 2) as i32;
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let m = month as i32;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + day as i32 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    (era * 146_097 + doe - 719_468) as i64
}
