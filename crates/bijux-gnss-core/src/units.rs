#![allow(dead_code)]
#![allow(missing_docs)]
use serde::{Deserialize, Serialize};

/// Meters.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Meters(pub f64);

/// Seconds.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Seconds(pub f64);

/// Hertz.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Hertz(pub f64);

/// Chips (code).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Chips(pub f64);

/// Cycles (carrier phase).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Cycles(pub f64);

/// Meters per second.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct MetersPerSecond(pub f64);

impl From<f64> for Meters {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f64> for Seconds {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f64> for Hertz {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f64> for Chips {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f64> for Cycles {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl From<f64> for MetersPerSecond {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

/// Convert code chips to seconds given code rate in Hz.
///
/// ```
/// use bijux_gnss_core::api::{chips_to_seconds, Chips, Hertz, Seconds};
/// assert!((chips_to_seconds(Chips(1023.0), Hertz(1_023_000.0)).0 - 0.001).abs() < 1e-9);
/// ```
pub fn chips_to_seconds(chips: Chips, code_rate: Hertz) -> Seconds {
    Seconds(chips.0 / code_rate.0)
}

/// Convert carrier cycles to meters given carrier wavelength.
///
/// ```
/// use bijux_gnss_core::api::{cycles_to_meters, Cycles, Meters};
/// assert!((cycles_to_meters(Cycles(1.0), Meters(0.19)).0 - 0.19).abs() < 1e-9);
/// ```
pub fn cycles_to_meters(cycles: Cycles, wavelength_m: Meters) -> Meters {
    Meters(cycles.0 * wavelength_m.0)
}

/// Convert Hertz to radians per second.
///
/// ```
/// use bijux_gnss_core::api::{hz_to_rad_per_sec, Hertz};
/// assert!((hz_to_rad_per_sec(Hertz(1.0)) - std::f64::consts::TAU).abs() < 1e-9);
/// ```
pub fn hz_to_rad_per_sec(freq: Hertz) -> f64 {
    freq.0 * std::f64::consts::TAU
}
