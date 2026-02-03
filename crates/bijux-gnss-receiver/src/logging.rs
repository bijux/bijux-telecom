use crate::tracking::ChannelState;

#[cfg(feature = "tracing")]
use tracing::{debug, info};

pub fn acquisition_hit(prn: u8, carrier_hz: f64, code_phase: usize, metric: f32, ratio: f32) {
    #[cfg(feature = "tracing")]
    info!(
        prn,
        carrier_hz, code_phase, metric, ratio, "acquisition hit"
    );
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (prn, carrier_hz, code_phase, metric, ratio);
    }
}

pub fn channel_state_change(channel: u8, from: ChannelState, to: ChannelState) {
    #[cfg(feature = "tracing")]
    info!(channel, from = ?from, to = ?to, "channel state change");
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (channel, from, to);
    }
}

pub fn lock_status(channel: u8, locked: bool) {
    #[cfg(feature = "tracing")]
    debug!(channel, locked, "lock status change");
    #[cfg(not(feature = "tracing"))]
    {
        let _ = (channel, locked);
    }
}
