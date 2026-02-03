//! Receiver I/O helpers.

pub mod data;

#[allow(dead_code)]
pub(crate) fn io_modules() -> [&'static str; 1] {
    ["data"]
}
