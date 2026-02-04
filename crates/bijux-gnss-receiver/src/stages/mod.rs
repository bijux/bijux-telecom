#![allow(missing_docs)]

//! Receiver pipeline stages.
//!
//! This module is a small facade so call sites can import stage modules from
//! a single namespace without deep paths.

pub mod acquisition;
pub mod navigation;
pub mod observations;
pub mod tracking;
pub(crate) mod tracking_math;

#[allow(dead_code)]
pub fn stage_names() -> [&'static str; 4] {
    ["acquisition", "tracking", "observations", "navigation"]
}
