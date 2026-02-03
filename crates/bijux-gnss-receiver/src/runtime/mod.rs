#![allow(missing_docs)]

//! Receiver runtime utilities.

pub(crate) mod logging;
pub(crate) mod receiver_config;
pub(crate) mod receiver_profile_defaults;
pub(crate) mod receiver_profile_validation;

#[allow(dead_code)]
pub(crate) fn runtime_modules() -> [&'static str; 4] {
    [
        "logging",
        "receiver_config",
        "receiver_profile_defaults",
        "receiver_profile_validation",
    ]
}
