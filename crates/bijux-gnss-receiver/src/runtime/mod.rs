#![allow(missing_docs)]

//! Receiver runtime utilities.

pub mod alloc;
pub mod diagnostics;
pub mod logging;
pub mod metrics;
pub mod receiver_config;
pub mod receiver_profile_defaults;
pub mod receiver_profile_validation;

#[allow(dead_code)]
pub fn runtime_modules() -> [&'static str; 7] {
    [
        "logging",
        "alloc",
        "diagnostics",
        "metrics",
        "receiver_config",
        "receiver_profile_defaults",
        "receiver_profile_validation",
    ]
}
