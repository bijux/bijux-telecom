//! Shared guardrail checks across crates.
//! See docs/GLOSSARY.md for acronym definitions.

#![deny(missing_docs)]
#![forbid(unsafe_code)]

mod internal;

/// Public API surface for this crate.
pub mod api;
