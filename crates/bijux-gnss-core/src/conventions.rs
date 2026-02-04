#![allow(missing_docs)]

use crate::api::{
    Chips, Cycles, DiagnosticEvent, DiagnosticSeverity, Hertz, Meters, NavSolutionEpoch, ObsEpoch,
    Seconds, SolutionStatus,
};

/// Compile-time unit type assertions.
#[allow(dead_code)]
pub(crate) fn compile_time_unit_checks() {
    fn _assert_unit<T: UnitType>() {}
    _assert_unit::<Meters>();
    _assert_unit::<Seconds>();
    _assert_unit::<Hertz>();
    _assert_unit::<Chips>();
    _assert_unit::<Cycles>();
}

/// Marker trait for unit newtypes.
#[allow(dead_code)]
pub trait UnitType {}

impl UnitType for Meters {}
impl UnitType for Seconds {}
impl UnitType for Hertz {}
impl UnitType for Chips {}
impl UnitType for Cycles {}

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

/// Compute carrier phase increment (cycles) from Doppler and time step.
pub fn carrier_phase_increment(
    doppler_hz: crate::api::Hertz,
    dt_s: crate::api::Seconds,
) -> crate::api::Cycles {
    crate::api::Cycles(doppler_hz.0 * dt_s.0)
}

/// Compute Doppler (Hz) from carrier phase increment and time step.
pub fn doppler_from_phase_increment(
    delta_cycles: crate::api::Cycles,
    dt_s: crate::api::Seconds,
) -> crate::api::Hertz {
    if dt_s.0 == 0.0 {
        return crate::api::Hertz(0.0);
    }
    crate::api::Hertz(delta_cycles.0 / dt_s.0)
}

#[cfg(test)]
mod tests {
    use super::{carrier_phase_increment, compile_time_unit_checks, doppler_from_phase_increment};
    use crate::api::{Hertz, Seconds};

    #[test]
    fn unit_type_compile_checks() {
        compile_time_unit_checks();
    }

    #[test]
    fn doppler_phase_sign_convention() {
        let dt = Seconds(0.02);
        let doppler = Hertz(500.0);
        let delta = carrier_phase_increment(doppler, dt);
        assert!(delta.0 > 0.0);
        let recovered = doppler_from_phase_increment(delta, dt);
        assert!((recovered.0 - doppler.0).abs() < 1e-9);

        let doppler_neg = Hertz(-250.0);
        let delta_neg = carrier_phase_increment(doppler_neg, dt);
        assert!(delta_neg.0 < 0.0);
        let recovered_neg = doppler_from_phase_increment(delta_neg, dt);
        assert!((recovered_neg.0 - doppler_neg.0).abs() < 1e-9);
    }
}
