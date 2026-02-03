#![allow(missing_docs)]

//! Receiver runtime utilities.

pub mod logging;
pub mod receiver_config;
pub mod receiver_profile_defaults;
pub mod receiver_profile_validation;

#[allow(dead_code)]
pub fn runtime_modules() -> [&'static str; 4] {
    [
        "logging",
        "receiver_config",
        "receiver_profile_defaults",
        "receiver_profile_validation",
    ]
}
