//! Receiver runtime utilities.

pub mod logging;
pub mod receiver_config;

#[allow(dead_code)]
pub(crate) fn runtime_modules() -> [&'static str; 2] {
    ["logging", "receiver_config"]
}
