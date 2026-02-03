use bijux_gnss_core::{GpsTime, UtcTime};

pub fn normalize_tow(tow_s: f64) -> f64 {
    if tow_s.is_finite() {
        tow_s.rem_euclid(604_800.0)
    } else {
        0.0
    }
}

pub fn gps_week_rollover(week: u16) -> u32 {
    (week as u32) % 1024
}

pub fn gps_time_from_utc(utc: UtcTime) -> GpsTime {
    GpsTime::from_seconds(utc.unix_s)
}
