#![allow(missing_docs)]

use bijux_gnss_core::api::{utc_to_gps, GpsTime, LeapSeconds, UtcTime};

pub fn normalize_tow(tow_s: f64) -> f64 {
    if tow_s.is_finite() {
        tow_s.rem_euclid(604_800.0)
    } else {
        0.0
    }
}

pub fn gps_week_rollover(week: u16, reference_week: u32) -> u32 {
    const GPS_WEEK_ROLLOVER_CYCLE: i64 = 1024;
    const GPS_WEEK_ROLLOVER_HALF_CYCLE: i64 = GPS_WEEK_ROLLOVER_CYCLE / 2;

    let cycle = GPS_WEEK_ROLLOVER_CYCLE;
    let mut resolved_week = (reference_week / cycle as u32) as i64 * cycle + week as i64;
    let reference_week = reference_week as i64;

    if resolved_week - reference_week > GPS_WEEK_ROLLOVER_HALF_CYCLE {
        resolved_week -= cycle;
    } else if reference_week - resolved_week > GPS_WEEK_ROLLOVER_HALF_CYCLE {
        resolved_week += cycle;
    }

    resolved_week as u32
}

pub fn gps_time_from_utc(utc: UtcTime) -> GpsTime {
    utc_to_gps(utc, &LeapSeconds::default_table())
}

#[cfg(test)]
mod tests {
    use super::gps_week_rollover;

    #[test]
    fn gps_week_rollover_keeps_same_cycle_when_reference_matches() {
        assert_eq!(gps_week_rollover(161, 2209), 2209);
    }

    #[test]
    fn gps_week_rollover_uses_previous_cycle_near_lower_boundary() {
        assert_eq!(gps_week_rollover(1023, 1024), 1023);
    }

    #[test]
    fn gps_week_rollover_uses_next_cycle_near_upper_boundary() {
        assert_eq!(gps_week_rollover(0, 1023), 1024);
    }

    #[test]
    fn gps_week_rollover_prefers_nearest_week_across_multiple_cycles() {
        assert_eq!(gps_week_rollover(7, 3070), 3079);
    }
}
