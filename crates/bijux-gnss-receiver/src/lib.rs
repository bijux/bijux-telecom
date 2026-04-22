//! GNSS receiver pipeline orchestration.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `pipeline`: acquisition, tracking, observations, navigation
//! - `io`: sample sources and dataset readers
//! - `engine`: logging and runtime configuration
//! - `rtk`: differencing and baseline solution helpers
//! - `sim`: synthetic signal generation for tests
//! - `Receiver`: high-level pipeline entrypoint
//! - `ReceiverConfig`: on-disk configuration schema (serialize/deserialize)
//! - `ReceiverPipelineConfig`: derived pipeline configuration for engines
//! - `ReceiverRuntimeConfig`: runtime options for side-effectful output
//! - `ReceiverError`: error taxonomy for receiver stages

#![deny(clippy::unwrap_used)]
#![deny(missing_docs)]
#![forbid(unsafe_code)]

mod engine;
mod io;
mod pipeline;
mod ports;
#[cfg(feature = "nav")]
mod rtk;
#[cfg(feature = "nav")]
mod sim;
mod validation_helpers;
#[cfg(feature = "nav")]
mod validation_report;

/// Public API surface for this crate.
pub mod api;
