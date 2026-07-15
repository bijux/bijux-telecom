#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::gps::GpsSatelliteClockCorrection;

const CLK_MISSING_ABS_THRESHOLD: f64 = 999_999.0;
const CLK_MAX_INTERPOLATED_BIAS_STEP_S: f64 = 1.0e-6;
const CLK_INTERPOLATION_SUPPORT_POINTS: usize = 4;
const CLK_INTERPOLATION_GAP_MULTIPLIER: f64 = 1.5;
const CLK_INTERPOLATION_GAP_MARGIN_S: f64 = 1.0;

pub const CLK_INTERPOLATION_POLICY: ClkInterpolationPolicy = ClkInterpolationPolicy {
    support_point_count: CLK_INTERPOLATION_SUPPORT_POINTS,
    polynomial_order: CLK_INTERPOLATION_SUPPORT_POINTS - 1,
    window_policy: ClkInterpolationWindowPolicy::NearestCentered,
    edge_policy: ClkInterpolationEdgePolicy::InsideCoverageOnly,
    max_gap_multiplier: CLK_INTERPOLATION_GAP_MULTIPLIER,
    max_gap_margin_s: CLK_INTERPOLATION_GAP_MARGIN_S,
    max_bias_step_s: CLK_MAX_INTERPOLATED_BIAS_STEP_S,
};

#[derive(Debug, Clone)]
pub struct ClkRecord {
    pub epoch_s: f64,
    pub bias_s: f64,
    pub sigma_s: Option<f64>,
    pub rate_s_per_s: Option<f64>,
    pub rate_sigma_s_per_s: Option<f64>,
    pub acceleration_s_per_s2: Option<f64>,
    pub acceleration_sigma_s_per_s2: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct ClkProvider {
    records: BTreeMap<SatId, Vec<ClkRecord>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClkInterpolationWindowPolicy {
    NearestCentered,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClkInterpolationEdgePolicy {
    InsideCoverageOnly,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClkInterpolationPolicy {
    pub support_point_count: usize,
    pub polynomial_order: usize,
    pub window_policy: ClkInterpolationWindowPolicy,
    pub edge_policy: ClkInterpolationEdgePolicy,
    pub max_gap_multiplier: f64,
    pub max_gap_margin_s: f64,
    pub max_bias_step_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClkInterpolationSummary {
    pub policy: ClkInterpolationPolicy,
    pub sample_count: usize,
    pub withheld_sample_count: usize,
    pub withheld_edge_sample_count: usize,
    pub max_bias_error_s: f64,
    pub rms_bias_error_s: f64,
    pub max_sigma_error_s: Option<f64>,
    pub rms_sigma_error_s: Option<f64>,
}

impl ClkProvider {
    fn parse_internal(input: &str) -> Result<Self, String> {
        let mut records: BTreeMap<SatId, Vec<ClkRecord>> = BTreeMap::new();
        let lines = input.lines().collect::<Vec<_>>();
        let mut line_index = 0usize;
        while line_index < lines.len() {
            let line = lines[line_index];
            line_index += 1;
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
            if value_count == 0 {
                return Err("missing clock values".to_string());
            }
            let mut value_fields = parts[9..].to_vec();
            while value_fields.len() < value_count && line_index < lines.len() {
                let continuation = lines[line_index];
                if continuation.starts_with("AS") {
                    break;
                }
                line_index += 1;
                value_fields.extend(continuation.split_whitespace());
            }
            if value_fields.len() < value_count {
                return Err("missing clock values".to_string());
            }
            let values = parse_clk_values(&value_fields[..value_count])?;
            let days = days_from_civil(year, month, day);
            let epoch_s = days as f64 * 86_400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec;
            records.entry(sat).or_default().push(ClkRecord {
                epoch_s,
                bias_s: values.bias_s,
                sigma_s: values.sigma_s,
                rate_s_per_s: values.rate_s_per_s,
                rate_sigma_s_per_s: values.rate_sigma_s_per_s,
                acceleration_s_per_s2: values.acceleration_s_per_s2,
                acceleration_sigma_s_per_s2: values.acceleration_sigma_s_per_s2,
            });
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
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return Some(record.bias_s);
        }
        if list.len() == 1 {
            return Some(list[0].bias_s);
        }
        interpolate_bias_s(list, t_s)
    }

    pub fn clock_correction(&self, sat: SatId, t_s: f64) -> Option<GpsSatelliteClockCorrection> {
        Some(GpsSatelliteClockCorrection {
            bias_s: self.bias_s(sat, t_s)?,
            drift_s_per_s: self.rate_s_per_s(sat, t_s).unwrap_or(0.0),
            drift_rate_s_per_s2: self.acceleration_s_per_s2(sat, t_s).unwrap_or(0.0),
            base_bias_s: self.bias_s(sat, t_s)?,
            relativistic_s: 0.0,
            group_delay_s: 0.0,
        })
    }

    pub fn sigma_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.sigma_s;
        }
        if list.len() == 1 {
            return list[0].sigma_s;
        }
        interpolate_sigma_s(list, t_s)
    }

    pub fn rate_s_per_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.rate_s_per_s;
        }
        if list.len() == 1 {
            return list[0].rate_s_per_s;
        }
        interpolate_optional_s(list, t_s, |record| record.rate_s_per_s)
    }

    pub fn rate_sigma_s_per_s(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.rate_sigma_s_per_s;
        }
        if list.len() == 1 {
            return list[0].rate_sigma_s_per_s;
        }
        interpolate_optional_s(list, t_s, |record| record.rate_sigma_s_per_s)
    }

    pub fn acceleration_s_per_s2(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.acceleration_s_per_s2;
        }
        if list.len() == 1 {
            return list[0].acceleration_s_per_s2;
        }
        interpolate_optional_s(list, t_s, |record| record.acceleration_s_per_s2)
    }

    pub fn acceleration_sigma_s_per_s2(&self, sat: SatId, t_s: f64) -> Option<f64> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return record.acceleration_sigma_s_per_s2;
        }
        if list.len() == 1 {
            return list[0].acceleration_sigma_s_per_s2;
        }
        interpolate_optional_s(list, t_s, |record| record.acceleration_sigma_s_per_s2)
    }

    pub fn interpolation_summary(&self, sat: SatId) -> Option<ClkInterpolationSummary> {
        let records = self.records.get(&sat)?;
        summarize_interpolation_errors(records)
    }
}

#[derive(Debug, Clone, Copy)]
struct ClkValues {
    bias_s: f64,
    sigma_s: Option<f64>,
    rate_s_per_s: Option<f64>,
    rate_sigma_s_per_s: Option<f64>,
    acceleration_s_per_s2: Option<f64>,
    acceleration_sigma_s_per_s2: Option<f64>,
}

fn parse_clk_values(fields: &[&str]) -> Result<ClkValues, String> {
    let bias_s = parse_clk_required_value(fields.first().copied(), "bias")?;
    Ok(ClkValues {
        bias_s,
        sigma_s: parse_clk_optional_value(fields.get(1).copied(), "sigma")?,
        rate_s_per_s: parse_clk_optional_value(fields.get(2).copied(), "clock rate")?,
        rate_sigma_s_per_s: parse_clk_optional_value(fields.get(3).copied(), "clock rate sigma")?,
        acceleration_s_per_s2: parse_clk_optional_value(
            fields.get(4).copied(),
            "clock acceleration",
        )?,
        acceleration_sigma_s_per_s2: parse_clk_optional_value(
            fields.get(5).copied(),
            "clock acceleration sigma",
        )?,
    })
}

fn parse_clk_required_value(field: Option<&str>, label: &str) -> Result<f64, String> {
    parse_clk_optional_value(field, label)?.ok_or_else(|| format!("missing clock {label}"))
}

fn parse_clk_optional_value(field: Option<&str>, label: &str) -> Result<Option<f64>, String> {
    let Some(field) = field else {
        return Ok(None);
    };
    let value = field.parse::<f64>().map_err(|_| format!("invalid clock {label}"))?;
    Ok((value.abs() < CLK_MISSING_ABS_THRESHOLD).then_some(value))
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
        .filter_map(|sat_records| {
            sat_records.iter().map(|record| record.epoch_s).min_by(f64::total_cmp)
        })
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

    let support = interpolation_support_records(records, t_s, None)?;
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

    let support = interpolation_support_records(records, t_s, None)?;
    if support.is_empty() {
        return None;
    }

    lagrange_scalar(&support, t_s, |record| record.sigma_s)
}

fn interpolate_optional_s(
    records: &[ClkRecord],
    t_s: f64,
    value: impl Fn(&ClkRecord) -> Option<f64>,
) -> Option<f64> {
    let first_epoch_s = records.first()?.epoch_s;
    let last_epoch_s = records.last()?.epoch_s;
    if t_s < first_epoch_s || t_s > last_epoch_s {
        return None;
    }

    let support = interpolation_support_records(records, t_s, None)?;
    lagrange_scalar(&support, t_s, value)
}

fn interpolation_support_records<'a>(
    records: &'a [ClkRecord],
    t_s: f64,
    skip_index: Option<usize>,
) -> Option<Vec<&'a ClkRecord>> {
    if skip_index.is_none() {
        let bracket_index = records.partition_point(|record| record.epoch_s < t_s);
        let previous = records.get(bracket_index.checked_sub(1)?)?;
        let next = records.get(bracket_index)?;
        if !is_valid_clock_interpolation_interval(records, previous, next) {
            return None;
        }
    }
    let mut support = records
        .iter()
        .enumerate()
        .filter(|(index, _)| skip_index != Some(*index))
        .collect::<Vec<_>>();
    support.sort_by(|(left_index, left), (right_index, right)| {
        let left_dt = (left.epoch_s - t_s).abs();
        let right_dt = (right.epoch_s - t_s).abs();
        left_dt.total_cmp(&right_dt).then_with(|| left_index.cmp(right_index))
    });
    support.truncate(CLK_INTERPOLATION_SUPPORT_POINTS);
    support.sort_by(|(_, left), (_, right)| left.epoch_s.total_cmp(&right.epoch_s));
    let support = support.into_iter().map(|(_, record)| record).collect::<Vec<_>>();
    support_records_are_valid(records, &support, skip_index).then_some(support)
}

fn is_valid_clock_interpolation_interval(
    records: &[ClkRecord],
    previous: &ClkRecord,
    next: &ClkRecord,
) -> bool {
    if next.epoch_s <= previous.epoch_s {
        return false;
    }
    let nominal_step_s = nominal_epoch_step_s(records).unwrap_or(next.epoch_s - previous.epoch_s);
    is_valid_clock_gap(nominal_step_s, previous.epoch_s, next.epoch_s)
        && is_valid_clock_step(previous.bias_s, next.bias_s)
}

fn support_records_are_valid(
    records: &[ClkRecord],
    support: &[&ClkRecord],
    skip_index: Option<usize>,
) -> bool {
    let Some(nominal_step_s) = nominal_epoch_step_s(records) else {
        return true;
    };
    let skipped_record = skip_index.and_then(|index| records.get(index));
    support
        .windows(2)
        .all(|window| valid_support_interval(nominal_step_s, window[0], window[1], skipped_record))
}

fn valid_support_interval(
    nominal_step_s: f64,
    left: &ClkRecord,
    right: &ClkRecord,
    skipped_record: Option<&ClkRecord>,
) -> bool {
    if is_valid_clock_gap(nominal_step_s, left.epoch_s, right.epoch_s)
        && is_valid_clock_step(left.bias_s, right.bias_s)
    {
        return true;
    }

    skipped_record.is_some_and(|skipped| {
        skipped.epoch_s > left.epoch_s
            && skipped.epoch_s < right.epoch_s
            && is_valid_clock_gap(nominal_step_s, left.epoch_s, skipped.epoch_s)
            && is_valid_clock_gap(nominal_step_s, skipped.epoch_s, right.epoch_s)
            && is_valid_clock_step(left.bias_s, skipped.bias_s)
            && is_valid_clock_step(skipped.bias_s, right.bias_s)
    })
}

fn is_valid_clock_gap(nominal_step_s: f64, left_epoch_s: f64, right_epoch_s: f64) -> bool {
    right_epoch_s > left_epoch_s
        && right_epoch_s - left_epoch_s
            <= nominal_step_s * CLK_INTERPOLATION_GAP_MULTIPLIER + CLK_INTERPOLATION_GAP_MARGIN_S
}

fn is_valid_clock_step(left_bias_s: f64, right_bias_s: f64) -> bool {
    (right_bias_s - left_bias_s).abs() <= CLK_MAX_INTERPOLATED_BIAS_STEP_S
}

fn nominal_epoch_step_s(records: &[ClkRecord]) -> Option<f64> {
    records
        .windows(2)
        .map(|window| window[1].epoch_s - window[0].epoch_s)
        .filter(|step_s| *step_s > f64::EPSILON)
        .min_by(f64::total_cmp)
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
        policy: CLK_INTERPOLATION_POLICY,
        sample_count: bias_errors_s.len(),
        withheld_sample_count: bias_errors_s.len(),
        withheld_edge_sample_count: records
            .iter()
            .enumerate()
            .filter(|(index, _)| is_withheld_edge_sample(records, *index))
            .count(),
        max_bias_error_s: bias_errors_s.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        rms_bias_error_s: rms_error_s(&bias_errors_s),
        max_sigma_error_s: (!sigma_errors_s.is_empty())
            .then(|| sigma_errors_s.iter().copied().fold(f64::NEG_INFINITY, f64::max)),
        rms_sigma_error_s: (!sigma_errors_s.is_empty()).then(|| rms_error_s(&sigma_errors_s)),
    })
}

fn interpolate_bias_error_s(records: &[ClkRecord], index: usize) -> Option<f64> {
    let target = records.get(index)?;
    let support = interpolation_support_records(records, target.epoch_s, Some(index))?;
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
    let support = interpolation_support_records(records, target.epoch_s, Some(index))?;
    let first_epoch_s = support.first()?.epoch_s;
    let last_epoch_s = support.last()?.epoch_s;
    if target.epoch_s <= first_epoch_s || target.epoch_s >= last_epoch_s {
        return None;
    }

    let sigma_s = lagrange_scalar(&support, target.epoch_s, |record| record.sigma_s)?;
    Some((sigma_s - target_sigma_s).abs())
}

fn is_withheld_edge_sample(records: &[ClkRecord], index: usize) -> bool {
    let Some(target) = records.get(index) else {
        return false;
    };
    let Some(support) = interpolation_support_records(records, target.epoch_s, Some(index)) else {
        return false;
    };
    let Some(first_epoch_s) = support.first().map(|record| record.epoch_s) else {
        return false;
    };
    let Some(last_epoch_s) = support.last().map(|record| record.epoch_s) else {
        return false;
    };

    target.epoch_s <= first_epoch_s || target.epoch_s >= last_epoch_s
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
    use super::{
        interpolation_support_records, ClkInterpolationEdgePolicy, ClkInterpolationWindowPolicy,
        ClkProvider,
    };
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
        let summary = provider.interpolation_summary(sat).expect("clock interpolation summary");

        assert_eq!(summary.policy.support_point_count, 4);
        assert_eq!(summary.policy.polynomial_order, 3);
        assert_eq!(summary.policy.window_policy, ClkInterpolationWindowPolicy::NearestCentered);
        assert_eq!(summary.policy.edge_policy, ClkInterpolationEdgePolicy::InsideCoverageOnly);
        assert_eq!(summary.policy.max_gap_multiplier, 1.5);
        assert_eq!(summary.policy.max_gap_margin_s, 1.0);
        assert_eq!(summary.policy.max_bias_step_s, 1.0e-6);
        assert_eq!(summary.sample_count, 3);
        assert_eq!(summary.withheld_sample_count, 3);
        assert_eq!(summary.withheld_edge_sample_count, 2);
        assert!(summary.max_bias_error_s.abs() < 1e-18);
        assert!(summary.rms_bias_error_s.abs() < 1e-18);
        assert!(summary.max_sigma_error_s.expect("sigma summary").abs() < 1e-18);
        assert!(summary.rms_sigma_error_s.expect("sigma RMS summary").abs() < 1e-18);
    }

    #[test]
    fn interpolation_uses_nearest_centered_clock_support() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000010
AS G01 2020 01 01 00 30 00.000000  1  0.000000049
AS G01 2020 01 01 00 45 00.000000  1  0.000000142
AS G01 2020 01 01 01 00 00.000000  1  0.000000313
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let records = provider.records.get(&sat).expect("clock records");
        let support =
            interpolation_support_records(records, 2_250.0, None).expect("centered support");

        assert_eq!(
            support.iter().map(|record| record.epoch_s as i64).collect::<Vec<_>>(),
            vec![900, 1_800, 2_700, 3_600]
        );
    }

    #[test]
    fn interpolation_summary_refuses_withheld_clock_samples_across_gaps() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000002
AS G01 2020 01 01 01 00 00.000000  1  0.000000003
AS G01 2020 01 01 01 15 00.000000  1  0.000000004
";
        let provider: ClkProvider = data.parse().expect("parse CLK with gap");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.interpolation_summary(sat).is_none());
    }

    #[test]
    fn interpolation_summary_refuses_withheld_clock_samples_across_jumps() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000002
AS G01 2020 01 01 00 30 00.000000  1  0.000003500
AS G01 2020 01 01 00 45 00.000000  1  0.000003501
";
        let provider: ClkProvider = data.parse().expect("parse CLK with jump");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.interpolation_summary(sat).is_none());
    }

    #[test]
    fn clk_preserves_clock_rate_acceleration_and_sigmas() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  6  0.000000001  0.000000002 -0.000000003  0.000000004  0.000000005  0.000000006
AS G01 2020 01 01 00 15 00.000000  6  0.000000010  0.000000011 -0.000000012  0.000000013  0.000000014  0.000000015
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let records = provider.records.get(&sat).expect("clock records");

        assert_eq!(records[0].rate_s_per_s, Some(-3.0e-9));
        assert_eq!(records[0].rate_sigma_s_per_s, Some(4.0e-9));
        assert_eq!(records[0].acceleration_s_per_s2, Some(5.0e-9));
        assert_eq!(records[0].acceleration_sigma_s_per_s2, Some(6.0e-9));
        let correction = provider.clock_correction(sat, 900.0).expect("clock correction");
        assert_eq!(correction.bias_s, 1.0e-8);
        assert_eq!(correction.drift_s_per_s, -12.0e-9);
        assert_eq!(correction.drift_rate_s_per_s2, 14.0e-9);
    }

    #[test]
    fn clk_reads_continuation_values_for_rate_and_acceleration() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  6  0.000000001  0.000000002
   -0.000000003  0.000000004  0.000000005  0.000000006
";
        let provider: ClkProvider = data.parse().expect("parse CLK continuation");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert_eq!(provider.rate_s_per_s(sat, 0.0), Some(-3.0e-9));
        assert_eq!(provider.rate_sigma_s_per_s(sat, 0.0), Some(4.0e-9));
        assert_eq!(provider.acceleration_s_per_s2(sat, 0.0), Some(5.0e-9));
        assert_eq!(provider.acceleration_sigma_s_per_s2(sat, 0.0), Some(6.0e-9));
    }

    #[test]
    fn clk_interpolation_refuses_large_product_gaps() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000002
AS G01 2020 01 01 01 00 00.000000  1  0.000000003
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.bias_s(sat, 2_700.0).is_none());
    }

    #[test]
    fn clk_interpolation_refuses_clock_jumps() {
        let data = "\
AS G01 2020 01 01 00 00 00.000000  1  0.000000001
AS G01 2020 01 01 00 15 00.000000  1  0.000000002
AS G01 2020 01 01 00 30 00.000000  1  0.000003500
";
        let provider: ClkProvider = data.parse().expect("parse CLK");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.bias_s(sat, 1_350.0).is_none());
        assert_eq!(provider.bias_s(sat, 900.0), Some(2.0e-9));
    }
}
