//! GNSS receiver pipeline orchestration.
//! See docs/GLOSSARY.md for acronym definitions.
//!
//! Module map:
//! - `stages`: acquisition, tracking, observations, navigation
//! - `io`: sample sources and dataset readers
//! - `runtime`: logging and runtime configuration
//! - `rtk`: differencing and baseline solution helpers
//! - `sim`: synthetic signal generation for tests
//! - `Receiver`: high-level pipeline entrypoint
//! - `ReceiverConfig`: runtime configuration
//! - `ReceiverProfile`: config file schema
//! - `ReceiverError`: error taxonomy for receiver stages

#![deny(clippy::unwrap_used)]
#![deny(missing_docs)]

mod io;
#[cfg(feature = "nav")]
#[path = "rtk/mod.rs"]
mod rtk_internal;
mod runtime;
#[cfg(feature = "nav")]
#[path = "sim/mod.rs"]
mod sim_internal;
mod stages;
#[cfg(feature = "nav")]
mod validation_report;

/// Public API surface for this crate.
pub mod api;
