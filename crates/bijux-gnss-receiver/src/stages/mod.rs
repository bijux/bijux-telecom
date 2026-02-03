#![allow(missing_docs)]

//! Receiver pipeline stages.
//!
//! This module is a small facade so call sites can import stage modules from
//! a single namespace without deep paths.

pub(crate) mod acquisition;
pub(crate) mod navigation;
pub(crate) mod observations;
pub(crate) mod tracking;

#[allow(dead_code)]
pub(crate) fn stage_names() -> [&'static str; 4] {
    ["acquisition", "tracking", "observations", "navigation"]
}
