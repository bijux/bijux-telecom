//! DSP primitives for GNSS signal processing.
//!
//! Module map:
//! - `codes`: spreading code generators (GPS C/A, etc.)
//! - `dsp`: filters, NCOs, and basic DSP helpers
//! - `samples`: sample container utilities
//! - `dsp::signal`: signal-level helpers (samples per code, etc.)
//! - `dsp::math`: numeric helpers for DSP
//! - `dsp::nco`: numerically controlled oscillator
//! - `codes::ca_code`: GPS L1 C/A code implementation

#![deny(clippy::unwrap_used)]

pub mod codes;
pub mod dsp;
pub mod samples;

pub use codes::*;
pub use dsp::*;
pub use samples::*;
