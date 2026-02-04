//! Core GNSS pipeline contracts and error taxonomy.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `ids`: constellation, satellite, and signal identity types
//! - `time`: GPS/UTC/receiver time definitions and leap seconds
//! - `obs`: samples, acquisition, tracking, and observation contracts
//! - Start here: `ObsEpoch` and `SamplesFrame`
#![deny(clippy::unwrap_used)]
#![deny(missing_docs)]
#![forbid(unsafe_code)]

mod artifact;
mod config;
mod conventions;
mod diagnostic;
mod error;
mod geo;
mod ids;
mod obs;
mod reference_validation;
mod stats;
mod time;
mod units;

/// Public API surface for this crate.
pub mod api;
