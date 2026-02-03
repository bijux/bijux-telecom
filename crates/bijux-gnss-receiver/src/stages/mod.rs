//! Receiver pipeline stages.
//!
//! This module is a small facade so call sites can import stage modules from
//! a single namespace without deep paths.

pub mod acquisition;
pub mod navigation;
pub mod observations;
pub mod tracking;

#[allow(dead_code)]
pub(crate) fn stage_names() -> [&'static str; 4] {
    ["acquisition", "tracking", "observations", "navigation"]
}
