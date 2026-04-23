#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

#[derive(Debug, Clone)]
pub struct ClkRecord {
    pub epoch_s: f64,
    pub bias_s: f64,
}

#[derive(Debug, Clone)]
pub struct ClkProvider {
    records: BTreeMap<SatId, Vec<ClkRecord>>,
}

impl ClkProvider {
    fn parse_internal(input: &str) -> Result<Self, String> {
        let mut records: BTreeMap<SatId, Vec<ClkRecord>> = BTreeMap::new();
        let mut epoch0 = None;
        for line in input.lines() {
            if !line.starts_with("AS") {
                continue;
            }
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 10 {
                continue;
            }
            let sat = parse_sat_id(parts[1])?;
            let year: i32 = parts[2].parse().map_err(|_| "invalid year")?;
            let month: u32 = parts[3].parse().map_err(|_| "invalid month")?;
            let day: u32 = parts[4].parse().map_err(|_| "invalid day")?;
            let hour: u32 = parts[5].parse().map_err(|_| "invalid hour")?;
            let min: u32 = parts[6].parse().map_err(|_| "invalid min")?;
            let sec: f64 = parts[7].parse().map_err(|_| "invalid sec")?;
            let bias_s: f64 =
                parts.last().ok_or("missing bias")?.parse().map_err(|_| "invalid bias")?;
            let days = days_from_civil(year, month, day);
            let epoch_abs = days as f64 * 86_400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec;
            let base = *epoch0.get_or_insert(epoch_abs);
            let epoch_s = epoch_abs - base;
            records.entry(sat).or_default().push(ClkRecord { epoch_s, bias_s });
        }
        Ok(Self { records })
    }

    pub fn coverage_s(&self, sat: SatId) -> Option<(f64, f64)> {
        let list = self.records.get(&sat)?;
        let first = list.first()?.epoch_s;
        let last = list.last()?.epoch_s;
        Some((first, last))
    }

    pub fn bias_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if list.len() == 1 {
            return Some(list[0].bias_s);
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
                Some(b.bias_s + w * (a.bias_s - b.bias_s))
            }
            (Some(b), Some(_)) => Some(b.bias_s),
            _ => None,
        }
    }
}

impl FromStr for ClkProvider {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::parse_internal(s)
    }
}

fn parse_sat_id(id: &str) -> Result<SatId, String> {
    let id = id.trim();
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
