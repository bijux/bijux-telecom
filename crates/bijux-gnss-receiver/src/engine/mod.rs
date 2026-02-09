#![allow(missing_docs)]

//! Receiver runtime utilities.

pub mod alloc;
pub mod diagnostics;
pub mod engine;
pub mod logging;
pub mod metrics;
pub mod runtime;
pub mod receiver_config;
pub mod receiver_config_defaults;
pub mod receiver_config_validation;
pub mod receiver;

#[allow(dead_code)]
pub fn runtime_modules() -> [&'static str; 8] {
    [
        "logging",
        "alloc",
        "diagnostics",
        "metrics",
        "runtime",
        "receiver_config",
        "receiver_config_defaults",
        "receiver_config_validation",
    ]
}
