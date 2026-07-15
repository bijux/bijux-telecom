use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RolloverResolutionError {
    InvalidCycle,
    TruncatedValueOutOfRange,
    AmbiguousReference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WeekRolloverResolution {
    pub truncated_week: u16,
    pub reference_week: u32,
    pub cycle_weeks: u32,
    pub week: u32,
    pub distance_weeks: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassDayResolution {
    pub day_number: u16,
    pub reference_day_index: i64,
    pub cycle_days: i64,
    pub day_index: i64,
    pub distance_days: i64,
}

pub fn normalize_tow(tow_s: f64) -> f64 {
    if tow_s.is_finite() {
        tow_s.rem_euclid(604_800.0)
    } else {
        0.0
    }
}

pub fn gps_week_rollover(week: u16, reference_week: u32) -> u32 {
    resolve_gps_week_rollover(week, reference_week)
        .map(|resolution| resolution.week)
        .unwrap_or_else(|_| legacy_nearest_week_rollover(week, reference_week, 1024))
}

pub fn resolve_gps_week_rollover(
    truncated_week: u16,
    reference_week: u32,
) -> Result<WeekRolloverResolution, RolloverResolutionError> {
    resolve_truncated_week(truncated_week, reference_week, 1024)
}

pub fn resolve_galileo_week_rollover(
    truncated_week: u16,
    reference_week: u32,
) -> Result<WeekRolloverResolution, RolloverResolutionError> {
    resolve_truncated_week(truncated_week, reference_week, 4096)
}

pub fn resolve_beidou_week_rollover(
    truncated_week: u16,
    reference_week: u32,
) -> Result<WeekRolloverResolution, RolloverResolutionError> {
    resolve_truncated_week(truncated_week, reference_week, 8192)
}

pub fn resolve_truncated_week(
    truncated_week: u16,
    reference_week: u32,
    cycle_weeks: u32,
) -> Result<WeekRolloverResolution, RolloverResolutionError> {
    if cycle_weeks == 0 {
        return Err(RolloverResolutionError::InvalidCycle);
    }
    if u32::from(truncated_week) >= cycle_weeks {
        return Err(RolloverResolutionError::TruncatedValueOutOfRange);
    }

    let reference_week = i64::from(reference_week);
    let cycle = i64::from(cycle_weeks);
    let base_cycle = reference_week.div_euclid(cycle) * cycle;
    let base_candidate = base_cycle + i64::from(truncated_week);
    let candidates = [base_candidate - cycle, base_candidate, base_candidate + cycle];
    let mut best: Option<(i64, i64)> = None;

    for candidate in candidates {
        if candidate < 0 {
            continue;
        }
        let distance = candidate - reference_week;
        let abs_distance = distance.abs();
        match best {
            None => best = Some((candidate, distance)),
            Some((_, best_distance)) if abs_distance < best_distance.abs() => {
                best = Some((candidate, distance));
            }
            Some((_, best_distance)) if abs_distance == best_distance.abs() => {
                return Err(RolloverResolutionError::AmbiguousReference);
            }
            _ => {}
        }
    }

    let (week, distance_weeks) = best.ok_or(RolloverResolutionError::InvalidCycle)?;
    Ok(WeekRolloverResolution {
        truncated_week,
        reference_week: reference_week as u32,
        cycle_weeks,
        week: week as u32,
        distance_weeks: distance_weeks as i32,
    })
}

pub fn resolve_glonass_day_number(
    day_number: u16,
    reference_day_index: i64,
) -> Result<GlonassDayResolution, RolloverResolutionError> {
    const GLONASS_FOUR_YEAR_CYCLE_DAYS: i64 = 1_461;

    if !(1..=GLONASS_FOUR_YEAR_CYCLE_DAYS as u16).contains(&day_number) {
        return Err(RolloverResolutionError::TruncatedValueOutOfRange);
    }

    let day_modulo = i64::from(day_number - 1);
    let base_cycle =
        reference_day_index.div_euclid(GLONASS_FOUR_YEAR_CYCLE_DAYS) * GLONASS_FOUR_YEAR_CYCLE_DAYS;
    let base_candidate = base_cycle + day_modulo;
    let candidates = [
        base_candidate - GLONASS_FOUR_YEAR_CYCLE_DAYS,
        base_candidate,
        base_candidate + GLONASS_FOUR_YEAR_CYCLE_DAYS,
    ];
    let mut best: Option<(i64, i64)> = None;

    for candidate in candidates {
        let distance = candidate - reference_day_index;
        let abs_distance = distance.abs();
        match best {
            None => best = Some((candidate, distance)),
            Some((_, best_distance)) if abs_distance < best_distance.abs() => {
                best = Some((candidate, distance));
            }
            Some((_, best_distance)) if abs_distance == best_distance.abs() => {
                return Err(RolloverResolutionError::AmbiguousReference);
            }
            _ => {}
        }
    }

    let (day_index, distance_days) = best.ok_or(RolloverResolutionError::InvalidCycle)?;
    Ok(GlonassDayResolution {
        day_number,
        reference_day_index,
        cycle_days: GLONASS_FOUR_YEAR_CYCLE_DAYS,
        day_index,
        distance_days,
    })
}

fn legacy_nearest_week_rollover(week: u16, reference_week: u32, cycle_weeks: u32) -> u32 {
    let cycle = i64::from(cycle_weeks);
    let mut resolved_week = i64::from(reference_week / cycle_weeks) * cycle + i64::from(week);
    let reference_week = i64::from(reference_week);

    if resolved_week - reference_week > cycle / 2 {
        resolved_week -= cycle;
    } else if reference_week - resolved_week > cycle / 2 {
        resolved_week += cycle;
    }

    resolved_week as u32
}
