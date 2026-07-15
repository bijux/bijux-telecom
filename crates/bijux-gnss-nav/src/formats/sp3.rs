#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{Constellation, SatId};

use crate::orbits::gps::{GpsSatState, GpsSatelliteClockCorrection};

const SP3_MISSING_ABS_THRESHOLD: f64 = 999_999.0;

#[derive(Debug, Clone)]
pub struct Sp3Record {
    pub epoch_s: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_bias_s: Option<f64>,
    pub velocity_mps: Option<[f64; 3]>,
    pub clock_rate_s_per_s: Option<f64>,
    pub accuracy: Sp3RecordAccuracy,
    pub flags: Sp3RecordFlags,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sp3State {
    pub epoch_s: f64,
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_bias_s: Option<f64>,
    pub velocity_mps: Option<[f64; 3]>,
    pub clock_rate_s_per_s: Option<f64>,
    pub accuracy: Sp3RecordAccuracy,
    pub flags: Sp3RecordFlags,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Sp3RecordAccuracy {
    pub x_m: Option<f64>,
    pub y_m: Option<f64>,
    pub z_m: Option<f64>,
    pub clock_s: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Sp3RecordFlags {
    pub clock_event: bool,
    pub clock_prediction: bool,
    pub orbit_maneuver: bool,
    pub orbit_prediction: bool,
}

#[derive(Debug, Clone)]
pub struct Sp3Provider {
    records: BTreeMap<SatId, Vec<Sp3Record>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sp3InterpolationSummary {
    pub sample_count: usize,
    pub max_position_error_m: f64,
    pub rms_position_error_m: f64,
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
                let record = parse_sp3_position_record(line, stripped, epoch_s)?;
                records.entry(record.0).or_default().push(record.1);
            } else if let Some(stripped) = line.strip_prefix('V') {
                let epoch_s = current_epoch.ok_or("SP3 velocity record without epoch")?;
                let (sat, velocity_mps, clock_rate_s_per_s) =
                    parse_sp3_velocity_record(line, stripped)?;
                let sat_records = records
                    .get_mut(&sat)
                    .ok_or_else(|| format!("SP3 velocity record for {sat:?} without position"))?;
                let record = sat_records
                    .iter_mut()
                    .rev()
                    .find(|record| (record.epoch_s - epoch_s).abs() <= f64::EPSILON)
                    .ok_or_else(|| {
                        format!("SP3 velocity record for {sat:?} without matching position epoch")
                    })?;
                record.velocity_mps = Some(velocity_mps);
                record.clock_rate_s_per_s = clock_rate_s_per_s;
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
        let state = self.precise_state(sat, t_s)?;
        Some(GpsSatState {
            x_m: state.x_m,
            y_m: state.y_m,
            z_m: state.z_m,
            clock_correction: GpsSatelliteClockCorrection {
                bias_s: state.clock_bias_s.unwrap_or(0.0),
                drift_s_per_s: state.clock_rate_s_per_s.unwrap_or(0.0),
                drift_rate_s_per_s2: 0.0,
                base_bias_s: state.clock_bias_s.unwrap_or(0.0),
                relativistic_s: 0.0,
                group_delay_s: 0.0,
            },
        })
    }

    pub fn precise_state(&self, sat: SatId, t_s: f64) -> Option<Sp3State> {
        let list = self.records.get(&sat)?;
        if let Some(record) =
            list.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
        {
            return Some(record_state(record));
        }
        if list.len() == 1 {
            return Some(record_state(&list[0]));
        }
        interpolate_record_state(list, t_s)
    }

    pub fn interpolation_summary(&self, sat: SatId) -> Option<Sp3InterpolationSummary> {
        let records = self.records.get(&sat)?;
        summarize_interpolation_errors(records)
    }

    pub fn record_accuracy(&self, sat: SatId, t_s: f64) -> Option<Sp3RecordAccuracy> {
        self.record_at_epoch(sat, t_s).map(|record| record.accuracy)
    }

    pub fn record_flags(&self, sat: SatId, t_s: f64) -> Option<Sp3RecordFlags> {
        self.record_at_epoch(sat, t_s).map(|record| record.flags)
    }

    fn record_at_epoch(&self, sat: SatId, t_s: f64) -> Option<&Sp3Record> {
        self.records.get(&sat)?.iter().find(|record| (record.epoch_s - t_s).abs() <= f64::EPSILON)
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

fn parse_sp3_position_record(
    line: &str,
    stripped: &str,
    epoch_s: f64,
) -> Result<(SatId, Sp3Record), String> {
    let sat = parse_sat_id(line)?;
    let parts: Vec<&str> = stripped.split_whitespace().collect();
    if parts.len() < 4 {
        return Err("invalid SP3 position record".to_string());
    }
    let x_km = parse_sp3_coordinate_km(parts[1])?;
    let y_km = parse_sp3_coordinate_km(parts[2])?;
    let z_km = parse_sp3_coordinate_km(parts[3])?;
    let Some((x_m, y_m, z_m)) = x_km
        .zip(y_km)
        .zip(z_km)
        .map(|((x_km, y_km), z_km)| (x_km * 1000.0, y_km * 1000.0, z_km * 1000.0))
    else {
        return Err(format!("SP3 position record for {sat:?} contains missing coordinates"));
    };
    let clock_bias_s =
        parts.get(4).map(|field| parse_sp3_clock_bias_s(field)).transpose()?.flatten();
    let accuracy = parse_sp3_record_accuracy(&parts);
    let flags = parse_sp3_record_flags(line, &parts);
    Ok((
        sat,
        Sp3Record {
            epoch_s,
            x_m,
            y_m,
            z_m,
            clock_bias_s,
            velocity_mps: None,
            clock_rate_s_per_s: None,
            accuracy,
            flags,
        },
    ))
}

fn parse_sp3_velocity_record(
    line: &str,
    stripped: &str,
) -> Result<(SatId, [f64; 3], Option<f64>), String> {
    let sat = parse_sat_id(line)?;
    let parts: Vec<&str> = stripped.split_whitespace().collect();
    if parts.len() < 4 {
        return Err("invalid SP3 velocity record".to_string());
    }
    let vx_dmps = parse_sp3_coordinate_km(parts[1])?;
    let vy_dmps = parse_sp3_coordinate_km(parts[2])?;
    let vz_dmps = parse_sp3_coordinate_km(parts[3])?;
    let Some((vx_mps, vy_mps, vz_mps)) = vx_dmps
        .zip(vy_dmps)
        .zip(vz_dmps)
        .map(|((vx_dmps, vy_dmps), vz_dmps)| (vx_dmps * 0.1, vy_dmps * 0.1, vz_dmps * 0.1))
    else {
        return Err(format!("SP3 velocity record for {sat:?} contains missing components"));
    };
    let clock_rate_s_per_s =
        parts.get(4).map(|field| parse_sp3_clock_rate_s_per_s(field)).transpose()?.flatten();
    Ok((sat, [vx_mps, vy_mps, vz_mps], clock_rate_s_per_s))
}

fn parse_sp3_coordinate_km(field: &str) -> Result<Option<f64>, String> {
    let value = field.parse::<f64>().map_err(|_| format!("invalid SP3 coordinate '{field}'"))?;
    Ok((value.abs() < SP3_MISSING_ABS_THRESHOLD).then_some(value))
}

fn parse_sp3_clock_bias_s(field: &str) -> Result<Option<f64>, String> {
    let value = field.parse::<f64>().map_err(|_| format!("invalid SP3 clock '{field}'"))?;
    Ok((value.abs() < SP3_MISSING_ABS_THRESHOLD).then_some(value * 1.0e-6))
}

fn parse_sp3_clock_rate_s_per_s(field: &str) -> Result<Option<f64>, String> {
    let value = field.parse::<f64>().map_err(|_| format!("invalid SP3 clock rate '{field}'"))?;
    Ok((value.abs() < SP3_MISSING_ABS_THRESHOLD).then_some(value * 1.0e-10))
}

fn parse_sp3_record_accuracy(parts: &[&str]) -> Sp3RecordAccuracy {
    Sp3RecordAccuracy {
        x_m: parts.get(5).and_then(|field| sp3_orbit_accuracy_m(field)),
        y_m: parts.get(6).and_then(|field| sp3_orbit_accuracy_m(field)),
        z_m: parts.get(7).and_then(|field| sp3_orbit_accuracy_m(field)),
        clock_s: parts.get(8).and_then(|field| sp3_clock_accuracy_s(field)),
    }
}

fn sp3_orbit_accuracy_m(field: &str) -> Option<f64> {
    let exponent = field.parse::<i32>().ok()?;
    (exponent > 0).then(|| 2.0_f64.powi(exponent) * 1.0e-3)
}

fn sp3_clock_accuracy_s(field: &str) -> Option<f64> {
    let exponent = field.parse::<i32>().ok()?;
    (exponent > 0).then(|| 2.0_f64.powi(exponent) * 1.0e-12)
}

fn parse_sp3_record_flags(line: &str, parts: &[&str]) -> Sp3RecordFlags {
    let mut flags = Sp3RecordFlags {
        clock_event: fixed_char_is_flagged(line, 74),
        clock_prediction: fixed_char_is_flagged(line, 75),
        orbit_maneuver: fixed_char_is_flagged(line, 78),
        orbit_prediction: fixed_char_is_flagged(line, 79),
    };
    for token in parts.iter().skip(5) {
        flags.clock_event |= token.contains('E');
        flags.clock_prediction |= token.contains('P');
        flags.orbit_maneuver |= token.contains('M');
        flags.orbit_prediction |= token.contains('O');
    }
    flags
}

fn fixed_char_is_flagged(line: &str, zero_based_column: usize) -> bool {
    line.as_bytes()
        .get(zero_based_column)
        .is_some_and(|byte| !byte.is_ascii_whitespace() && *byte != b'0')
}

fn normalize_records(records: &mut BTreeMap<SatId, Vec<Sp3Record>>) {
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

fn interpolate_record_state(records: &[Sp3Record], t_s: f64) -> Option<Sp3State> {
    let first_epoch_s = records.first()?.epoch_s;
    let last_epoch_s = records.last()?.epoch_s;
    if t_s < first_epoch_s || t_s > last_epoch_s {
        return None;
    }

    let support = interpolation_support_records(records, t_s, None)?;
    if support.is_empty() {
        return None;
    }

    let clock_bias_s = optional_lagrange(&support, t_s, |record| record.clock_bias_s);
    let velocity_mps = support
        .iter()
        .all(|record| record.velocity_mps.is_some())
        .then(|| {
            Some([
                lagrange_coordinate(&support, t_s, |record| record.velocity_mps.unwrap()[0])?,
                lagrange_coordinate(&support, t_s, |record| record.velocity_mps.unwrap()[1])?,
                lagrange_coordinate(&support, t_s, |record| record.velocity_mps.unwrap()[2])?,
            ])
        })
        .flatten();
    let clock_rate_s_per_s = optional_lagrange(&support, t_s, |record| record.clock_rate_s_per_s);

    Some(Sp3State {
        epoch_s: t_s,
        x_m: lagrange_coordinate(&support, t_s, |record| record.x_m)?,
        y_m: lagrange_coordinate(&support, t_s, |record| record.y_m)?,
        z_m: lagrange_coordinate(&support, t_s, |record| record.z_m)?,
        clock_bias_s,
        velocity_mps,
        clock_rate_s_per_s,
        accuracy: merged_accuracy(&support),
        flags: merged_flags(&support),
    })
}

fn interpolation_support_records<'a>(
    records: &'a [Sp3Record],
    t_s: f64,
    skip_index: Option<usize>,
) -> Option<Vec<&'a Sp3Record>> {
    if skip_index.is_none() {
        let bracket_index = records.partition_point(|record| record.epoch_s < t_s);
        let previous = records.get(bracket_index.checked_sub(1)?)?;
        let next = records.get(bracket_index)?;
        if !is_valid_interpolation_interval(records, previous.epoch_s, next.epoch_s) {
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
    support.truncate(4);
    support.sort_by(|(_, left), (_, right)| left.epoch_s.total_cmp(&right.epoch_s));
    let support = support.into_iter().map(|(_, record)| record).collect::<Vec<_>>();
    (!support.iter().any(|record| record.flags.invalid_for_interpolation())).then_some(support)
}

fn is_valid_interpolation_interval(
    records: &[Sp3Record],
    left_epoch_s: f64,
    right_epoch_s: f64,
) -> bool {
    if right_epoch_s <= left_epoch_s {
        return false;
    }
    let nominal_step_s = nominal_epoch_step_s(records).unwrap_or(right_epoch_s - left_epoch_s);
    right_epoch_s - left_epoch_s <= nominal_step_s * 1.5 + 1.0
}

fn nominal_epoch_step_s(records: &[Sp3Record]) -> Option<f64> {
    records
        .windows(2)
        .map(|window| window[1].epoch_s - window[0].epoch_s)
        .filter(|step_s| *step_s > f64::EPSILON)
        .min_by(f64::total_cmp)
}

impl Sp3RecordFlags {
    fn invalid_for_interpolation(self) -> bool {
        self.clock_event || self.clock_prediction || self.orbit_maneuver || self.orbit_prediction
    }
}

fn lagrange_coordinate(
    support: &[&Sp3Record],
    t_s: f64,
    coordinate: impl Fn(&Sp3Record) -> f64,
) -> Option<f64> {
    let mut value = 0.0;
    for (index, record) in support.iter().enumerate() {
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
        value += basis * coordinate(record);
    }
    Some(value)
}

fn optional_lagrange(
    support: &[&Sp3Record],
    t_s: f64,
    value: impl Fn(&Sp3Record) -> Option<f64>,
) -> Option<f64> {
    support
        .iter()
        .all(|record| value(record).is_some())
        .then(|| lagrange_coordinate(support, t_s, |record| value(record).unwrap()))
        .flatten()
}

fn summarize_interpolation_errors(records: &[Sp3Record]) -> Option<Sp3InterpolationSummary> {
    let errors = records
        .iter()
        .enumerate()
        .filter_map(|(index, _)| interpolate_position_error_m(records, index))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        return None;
    }

    let sample_count = errors.len();
    let max_position_error_m = errors.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let rms_position_error_m =
        (errors.iter().map(|error| error.powi(2)).sum::<f64>() / sample_count as f64).sqrt();
    Some(Sp3InterpolationSummary { sample_count, max_position_error_m, rms_position_error_m })
}

fn interpolate_position_error_m(records: &[Sp3Record], index: usize) -> Option<f64> {
    let target = records.get(index)?;
    let support = interpolation_support_records(records, target.epoch_s, Some(index))?;
    let first_epoch_s = support.first()?.epoch_s;
    let last_epoch_s = support.last()?.epoch_s;
    if target.epoch_s <= first_epoch_s || target.epoch_s >= last_epoch_s {
        return None;
    }

    let x_m = lagrange_coordinate(&support, target.epoch_s, |record| record.x_m)?;
    let y_m = lagrange_coordinate(&support, target.epoch_s, |record| record.y_m)?;
    let z_m = lagrange_coordinate(&support, target.epoch_s, |record| record.z_m)?;
    let dx_m = x_m - target.x_m;
    let dy_m = y_m - target.y_m;
    let dz_m = z_m - target.z_m;

    Some((dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt())
}

fn record_state(record: &Sp3Record) -> Sp3State {
    Sp3State {
        epoch_s: record.epoch_s,
        x_m: record.x_m,
        y_m: record.y_m,
        z_m: record.z_m,
        clock_bias_s: record.clock_bias_s,
        velocity_mps: record.velocity_mps,
        clock_rate_s_per_s: record.clock_rate_s_per_s,
        accuracy: record.accuracy,
        flags: record.flags,
    }
}

fn merged_accuracy(support: &[&Sp3Record]) -> Sp3RecordAccuracy {
    Sp3RecordAccuracy {
        x_m: max_optional_accuracy(support.iter().filter_map(|record| record.accuracy.x_m)),
        y_m: max_optional_accuracy(support.iter().filter_map(|record| record.accuracy.y_m)),
        z_m: max_optional_accuracy(support.iter().filter_map(|record| record.accuracy.z_m)),
        clock_s: max_optional_accuracy(support.iter().filter_map(|record| record.accuracy.clock_s)),
    }
}

fn max_optional_accuracy(values: impl Iterator<Item = f64>) -> Option<f64> {
    values.reduce(f64::max)
}

fn merged_flags(support: &[&Sp3Record]) -> Sp3RecordFlags {
    support.iter().fold(Sp3RecordFlags::default(), |mut flags, record| {
        flags.clock_event |= record.flags.clock_event;
        flags.clock_prediction |= record.flags.clock_prediction;
        flags.orbit_maneuver |= record.flags.orbit_maneuver;
        flags.orbit_prediction |= record.flags.orbit_prediction;
        flags
    })
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

    #[test]
    fn position_records_preserve_clock_accuracy_and_flags() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000    12.500000  4  5  6  7            EPM
";
        let provider: Sp3Provider = data.parse().expect("parse SP3");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let record = &provider.records.get(&sat).expect("satellite records")[0];

        assert!((record.clock_bias_s.expect("clock bias") - 12.5e-6).abs() < 1.0e-18);
        assert_eq!(record.accuracy.x_m, Some(0.016));
        assert_eq!(record.accuracy.y_m, Some(0.032));
        assert_eq!(record.accuracy.z_m, Some(0.064));
        assert_eq!(record.accuracy.clock_s, Some(128.0e-12));
        assert!(record.flags.clock_event);
        assert!(record.flags.clock_prediction);
        assert!(record.flags.orbit_maneuver);
        assert!(!record.flags.orbit_prediction);
        assert_eq!(provider.record_accuracy(sat, 0.0), Some(record.accuracy));
        assert_eq!(provider.record_flags(sat, 0.0), Some(record.flags));

        let state = provider.sat_state(sat, 0.0).expect("state with precise clock");
        assert!((state.clock_correction.bias_s - 12.5e-6).abs() < 1.0e-18);
    }

    #[test]
    fn missing_position_records_are_rejected() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  999999.999999  20000.000000  30000.000000  0.000000
";

        assert!(data.parse::<Sp3Provider>().is_err());
    }

    #[test]
    fn velocity_records_preserve_velocity_and_clock_rate() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  10000.000000  20000.000000  30000.000000    12.500000
VG01     10.000000     -20.000000      30.000000    -4.000000
";
        let provider: Sp3Provider = data.parse().expect("parse SP3 with velocity");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let precise_state = provider.precise_state(sat, 0.0).expect("precise state");
        let state = provider.sat_state(sat, 0.0).expect("GPS state");

        assert_eq!(precise_state.velocity_mps, Some([1.0, -2.0, 3.0]));
        assert_eq!(precise_state.clock_rate_s_per_s, Some(-4.0e-10));
        assert_eq!(state.clock_correction.drift_s_per_s, -4.0e-10);
    }

    #[test]
    fn interpolation_refuses_large_product_gaps() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  0.000000  0.000000  0.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  9.000000  1.000000  1.000000  0.000000
* 2020 01 01 01 00 00.000000
PG01  36.000000  4.000000  4.000000  0.000000
";
        let provider: Sp3Provider = data.parse().expect("parse SP3 with gap");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.sat_state(sat, 2_700.0).is_none());
    }

    #[test]
    fn interpolation_refuses_flagged_support_records() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  0.000000  0.000000  0.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  9.000000  1.000000  1.000000  0.000000                    M
* 2020 01 01 00 30 00.000000
PG01  36.000000  4.000000  4.000000  0.000000
";
        let provider: Sp3Provider = data.parse().expect("parse flagged SP3");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };

        assert!(provider.sat_state(sat, 1_350.0).is_none());
        assert!(provider.sat_state(sat, 900.0).is_some());
    }

    #[test]
    fn interpolation_summary_measures_cubic_self_consistency() {
        let data = "\
* 2020 01 01 00 00 00.000000
PG01  1.000000  0.000000  5.000000  0.000000
* 2020 01 01 00 15 00.000000
PG01  10.000000  1.000000  6.000000  0.000000
* 2020 01 01 00 30 00.000000
PG01  49.000000  8.000000  9.000000  0.000000
* 2020 01 01 00 45 00.000000
PG01  142.000000  27.000000  14.000000  0.000000
* 2020 01 01 01 00 00.000000
PG01  313.000000  64.000000  21.000000  0.000000
";
        let provider: Sp3Provider = data.parse().expect("parse SP3");
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let summary =
            provider.interpolation_summary(sat).expect("interpolation summary for cubic orbit");

        assert_eq!(summary.sample_count, 3);
        assert!(summary.max_position_error_m.abs() < 1e-6);
        assert!(summary.rms_position_error_m.abs() < 1e-6);
    }
}
