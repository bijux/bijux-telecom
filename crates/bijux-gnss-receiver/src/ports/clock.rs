//! Clock traits for wall-clock access.
#![allow(missing_docs)]

use bijux_gnss_core::api::Seconds;

pub trait Clock {
    fn now_s(&self) -> Seconds;
}

pub struct SystemClock;

impl Clock for SystemClock {
    fn now_s(&self) -> Seconds {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        Seconds(now)
    }
}
