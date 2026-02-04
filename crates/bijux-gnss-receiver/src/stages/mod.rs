#![allow(missing_docs)]

//! Receiver pipeline stages.
//!
//! This module is a small facade so call sites can import stage modules from
//! a single namespace without deep paths.

pub mod acquisition;
#[cfg(feature = "nav")]
pub mod navigation;
pub mod observations;
pub mod tracking;

#[allow(dead_code)]
#[cfg(feature = "nav")]
pub fn stage_names() -> [&'static str; 4] {
    ["acquisition", "tracking", "observations", "navigation"]
}

#[allow(dead_code)]
#[cfg(not(feature = "nav"))]
pub fn stage_names() -> [&'static str; 3] {
    ["acquisition", "tracking", "observations"]
}
