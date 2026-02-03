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
