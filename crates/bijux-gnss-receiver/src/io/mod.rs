#![allow(missing_docs)]

//! Receiver I/O helpers.

pub(crate) mod data;

#[allow(dead_code)]
pub(crate) fn io_modules() -> [&'static str; 1] {
    ["data"]
}
