#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

#[derive(Debug, Clone)]
pub struct ClkRecord {
    pub epoch_s: f64,
    pub bias_s: f64,
    pub sigma_s: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct ClkProvider {
    records: BTreeMap<SatId, Vec<ClkRecord>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClkInterpolationSummary {
    pub sample_count: usize,
    pub max_bias_error_s: f64,
    pub rms_bias_error_s: f64,
    pub max_sigma_error_s: Option<f64>,
    pub rms_sigma_error_s: Option<f64>,
}

impl ClkProvider {
    fn parse_internal(input: &str) -> Result<Self, String> {
        let mut records: BTreeMap<SatId, Vec<ClkRecord>> = BTreeMap::new();
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
            let value_count: usize = parts[8].parse().map_err(|_| "invalid value count")?;
            if value_count == 0 || parts.len() < 9 + value_count {
                return Err("missing clock values".to_string());
            }
            let bias_s: f64 = parts[9].parse().map_err(|_| "invalid bias")?;
            let sigma_s = if value_count >= 2 {
                Some(parts[10].parse().map_err(|_| "invalid sigma")?)
            } else {
                None
            };
            let days = days_from_civil(year, month, day);
            let epoch_s =
                days as f64 * 86_400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec;
            records.entry(sat).or_default().push(ClkRecord { epoch_s, bias_s, sigma_s });
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

    pub fn bias_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) = list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return Some(record.bias_s);
        }
        if list.len() == 1 {
            return Some(list[0].bias_s);
        }
        interpolate_bias_s(list, t_s)
    }

    pub fn sigma_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) = list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.sigma_s;
        }
        if list.len() == 1 {
            return list[0].sigma_s;
        }
        interpolate_sigma_s(list, t_s)
    }

    pub fn interpolation_summary(&self, sat: SatId) -> Option<ClkInterpolationSummary> {
        let records = self.records.get(&sat)?;
        summarize_interpolation_errors(records)
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

fn normalize_records(records: &mut BTreeMap<SatId, Vec<ClkRecord>>) {
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

fn interpolate_bias_s(records: &[ClkRecord], t_s: f64) -> Option<f64> {
    let first_epoch_s = records.first()?.epoch_s;
    let last_epoch_s = records.last()?.epoch_s;
    if t_s < first_epoch_s || t_s > last_epoch_s {
        return None;
    }

    let support = interpolation_support_records(records, t_s, None);
    if support.is_empty() {
        return None;
    }

    lagrange_scalar(&support, t_s, |record| Some(record.bias_s))
}

fn interpolate_sigma_s(records: &[ClkRecord], t_s: f64) -> Option<f64> {
    let first_epoch_s = records.first()?.epoch_s;
    let last_epoch_s = records.last()?.epoch_s;
    if t_s < first_epoch_s || t_s > last_epoch_s {
        return None;
    }

    let support = interpolation_support_records(records, t_s, None);
    if support.is_empty() {
        return None;
    }

    lagrange_scalar(&support, t_s, |record| record.sigma_s)
}

fn interpolation_support_records<'a>(
    records: &'a [ClkRecord],
    t_s: f64,
    skip_index: Option<usize>,
) -> Vec<&'a ClkRecord> {
    let mut support = records
        .iter()
        .enumerate()
        .filter(|(index, _)| skip_index != Some(*index))
        .collect::<Vec<_>>();
    support.sort_by(|(left_index, left), (right_index, right)| {
        let left_dt = (left.epoch_s - t_s).abs();
        let right_dt = (right.epoch_s - t_s).abs();
        left_dt
            .total_cmp(&right_dt)
            .then_with(|| left_index.cmp(right_index))
    });
    support.truncate(4);
    support.sort_by(|(_, left), (_, right)| left.epoch_s.total_cmp(&right.epoch_s));
    support.into_iter().map(|(_, record)| record).collect()
}

fn lagrange_scalar(
    support: &[&ClkRecord],
    t_s: f64,
    value: impl Fn(&ClkRecord) -> Option<f64>,
) -> Option<f64> {
    let mut result = 0.0;
    for (index, record) in support.iter().enumerate() {
        let value = value(record)?;
        let mut basis = 1.0;
        for (other_index, other) in support.iter().enumerate() {
            if index == other_index {
                continue;
            }
            let denominator = record.epoch_s - other.epoch_s;
            if denominator.abs() <= f64::EPSILON {
                return None;
            }
            basis *= (t_s - other.epoch_s) / denominator;
        }
        result += basis * value;
    }
    Some(result)
}

fn summarize_interpolation_errors(records: &[ClkRecord]) -> Option<ClkInterpolationSummary> {
    let mut bias_errors_s = Vec::new();
    let mut sigma_errors_s = Vec::new();

    for index in 0..records.len() {
        if let Some(error_s) = interpolate_bias_error_s(records, index) {
            bias_errors_s.push(error_s);
        }
        if let Some(error_s) = interpolate_sigma_error_s(records, index) {
            sigma_errors_s.push(error_s);
        }
    }

    if bias_errors_s.is_empty() {
        return None;
    }

    Some(ClkInterpolationSummary {
        sample_count: bias_errors_s.len(),
        max_bias_error_s: bias_errors_s.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        rms_bias_error_s: rms_error_s(&bias_errors_s),
        max_sigma_error_s: (!sigma_errors_s.is_empty())
            .then(|| sigma_errors_s.iter().copied().fold(f64::NEG_INFINITY, f64::max)),
        rms_sigma_error_s: (!sigma_errors_s.is_empty()).then(|| rms_error_s(&sigma_errors_s)),
    })
}

fn interpolate_bias_error_s(records: &[ClkRecord], index: usize) -> Option<f64> {
    let target = records.get(index)?;
    let support = interpolation_support_records(records, target.epoch_s, Some(index));
    let first_epoch_s = support.first()?.epoch_s;
    let last_epoch_s = support.last()?.epoch_s;
    if target.epoch_s <= first_epoch_s || target.epoch_s >= last_epoch_s {
        return None;
    }

    let bias_s = lagrange_scalar(&support, target.epoch_s, |record| Some(record.bias_s))?;
    Some((bias_s - target.bias_s).abs())
}

fn interpolate_sigma_error_s(records: &[ClkRecord], index: usize) -> Option<f64> {
    let target = records.get(index)?;
    let target_sigma_s = target.sigma_s?;
    let support = interpolation_support_records(records, target.epoch_s, Some(index));
    let first_epoch_s = support.first()?.epoch_s;
    let last_epoch_s = support.last()?.epoch_s;
    if target.epoch_s <= first_epoch_s || target.epoch_s >= last_epoch_s {
        return None;
    }

    let sigma_s = lagrange_scalar(&support, target.epoch_s, |record| record.sigma_s)?;
    Some((sigma_s - target_sigma_s).abs())
}

fn rms_error_s(errors_s: &[f64]) -> f64 {
    (errors_s.iter().map(|error_s| error_s.powi(2)).sum::<f64>() / errors_s.len() as f64).sqrt()
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
    use super::ClkProvider;
    use bijux_gnss_core::api::{Constellation, SatId};

    #[test]
    fn parse_internal_sorts_and_deduplicates_clock_epochs() {
        let data = "\
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000010
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let records = provider.records.get(&sat).expect("satellite clock records");

        assert_eq!(records.len(), 2);
        assert_eq!(records[0].epoch_s, 0.0);
        assert_eq!(records[1].epoch_s, 900.0);
    }

    #[test]
    fn exact_epoch_bias_and_sigma_do_not_interpolate() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000010
AS G01 2020 01 01 00 15 00.000000  2  0.000000003  0.000000020
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert_eq!(provider.bias_s(sat, 900.0), Some(3.0e-9));
        assert_eq!(provider.sigma_s(sat, 900.0), Some(2.0e-8));
    }

    #[test]
    fn interpolation_summary_measures_cubic_clock_consistency() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  2  0.000000001  0.000000001
AS G01 2020 01 01 00 15 00.000000  2  0.000000010  0.000000002
AS G01 2020 01 01 00 30 00.000000  2  0.000000049  0.000000009
AS G01 2020 01 01 00 45 00.000000  2  0.000000142  0.000000028
AS G01 2020 01 01 01 00 00.000000  2  0.000000313  0.000000065
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let summary = provider
            .interpolation_summary(sat)
            .expect("clock interpolation summary");

        assert_eq!(summary.sample_count, 3);
        assert!(summary.max_bias_error_s.abs() < 1e-18);
        assert!(summary.rms_bias_error_s.abs() < 1e-18);
        assert!(summary.max_sigma_error_s.expect("sigma summary").abs() < 1e-18);
        assert!(summary.rms_sigma_error_s.expect("sigma RMS summary").abs() < 1e-18);
    }
}
