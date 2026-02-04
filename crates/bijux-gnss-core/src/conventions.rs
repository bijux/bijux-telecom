#![allow(missing_docs)]

use crate::obs::SolutionStatus;
use crate::{DiagnosticEvent, DiagnosticSeverity, NavSolutionEpoch, ObsEpoch};

/// Compile-time unit type assertions.
#[allow(dead_code)]
pub fn compile_time_unit_checks() {
    fn _assert_unit<T: UnitType>() {}
    _assert_unit::<crate::Meters>();
    _assert_unit::<crate::Seconds>();
    _assert_unit::<crate::Hertz>();
    _assert_unit::<crate::Chips>();
    _assert_unit::<crate::Cycles>();
}

/// Marker trait for unit newtypes.
#[allow(dead_code)]
pub trait UnitType {}

impl UnitType for crate::Meters {}
impl UnitType for crate::Seconds {}
impl UnitType for crate::Hertz {}
impl UnitType for crate::Chips {}
impl UnitType for crate::Cycles {}

#[derive(Debug, Clone, Copy)]
pub struct ConventionsConfig {
    pub max_abs_doppler_hz: f64,
    pub min_cn0_dbhz: f64,
    pub max_cn0_dbhz: f64,
    pub min_pseudorange_m: f64,
    pub max_pseudorange_m: f64,
}

impl Default for ConventionsConfig {
    fn default() -> Self {
        Self {
            max_abs_doppler_hz: 20_000.0,
            min_cn0_dbhz: 0.0,
            max_cn0_dbhz: 80.0,
            min_pseudorange_m: 1.0e5,
            max_pseudorange_m: 5.0e7,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct NavSanityConfig {
    pub max_position_jump_m: f64,
    pub max_clock_jump_s: f64,
    pub max_rms_m: f64,
}

impl Default for NavSanityConfig {
    fn default() -> Self {
        Self {
            max_position_jump_m: 10_000.0,
            max_clock_jump_s: 0.1,
            max_rms_m: 10_000.0,
        }
    }
}

pub fn check_obs_epoch_sanity(epoch: &ObsEpoch) -> Vec<DiagnosticEvent> {
    let cfg = ConventionsConfig::default();
    let mut events = Vec::new();
    for sat in &epoch.sats {
        if sat.cn0_dbhz < cfg.min_cn0_dbhz || sat.cn0_dbhz > cfg.max_cn0_dbhz {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_OBS_CN0_INVALID",
                "obs cn0 out of bounds",
            ));
        }
        if sat.doppler_hz.0.abs() > cfg.max_abs_doppler_hz {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_OBS_DOPPLER_INVALID",
                "obs doppler out of bounds",
            ));
        }
        if sat.pseudorange_m.0 < cfg.min_pseudorange_m
            || sat.pseudorange_m.0 > cfg.max_pseudorange_m
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_OBS_PSEUDORANGE_INVALID",
                "obs pseudorange out of bounds",
            ));
        }
    }
    events
}

pub fn check_nav_solution_sanity(
    prev: Option<&NavSolutionEpoch>,
    current: &NavSolutionEpoch,
) -> Vec<DiagnosticEvent> {
    let cfg = NavSanityConfig::default();
    let mut events = Vec::new();
    if current.rms_m.0 > cfg.max_rms_m {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "NAV_SOLUTION_RMS_INVALID",
            "nav RMS exceeds sanity threshold",
        ));
    }
    if let Some(prev) = prev {
        let dx = current.ecef_x_m.0 - prev.ecef_x_m.0;
        let dy = current.ecef_y_m.0 - prev.ecef_y_m.0;
        let dz = current.ecef_z_m.0 - prev.ecef_z_m.0;
        let jump = (dx * dx + dy * dy + dz * dz).sqrt();
        if jump > cfg.max_position_jump_m {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "NAV_POSITION_JUMP",
                "nav position jump exceeds threshold",
            ));
        }
        let clock_jump = (current.clock_bias_s.0 - prev.clock_bias_s.0).abs();
        if clock_jump > cfg.max_clock_jump_s {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "NAV_CLOCK_JUMP",
                "nav clock jump exceeds threshold",
            ));
        }
    }
    events
}

pub fn is_solution_valid(status: SolutionStatus) -> bool {
    status.is_valid()
}

#[cfg(test)]
mod tests {
    use super::compile_time_unit_checks;

    #[test]
    fn unit_type_compile_checks() {
        compile_time_unit_checks();
    }
}
