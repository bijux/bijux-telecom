use bijux_gnss_core::api::{Meters, SignalSpec};

use crate::engine::receiver_config::ReceiverPipelineConfig;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone)]
pub(super) struct ObservationReceiverClock {
    pub(super) bias_s: f64,
    pub(super) frequency_bias_hz: f64,
    pub(super) bias_sigma_s: f64,
    pub(super) source: String,
}

pub(super) fn observation_receiver_clock(
    config: &ReceiverPipelineConfig,
) -> ObservationReceiverClock {
    ObservationReceiverClock {
        bias_s: finite_receiver_clock_value(config.receiver_clock_bias_s),
        frequency_bias_hz: finite_receiver_clock_value(config.receiver_clock_frequency_bias_hz),
        bias_sigma_s: finite_receiver_clock_sigma(config.receiver_clock_bias_sigma_s),
        source: if config.receiver_clock_source.trim().is_empty() {
            "config".to_string()
        } else {
            config.receiver_clock_source.clone()
        },
    }
}

fn finite_receiver_clock_value(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

fn finite_receiver_clock_sigma(value: f64) -> f64 {
    if value.is_finite() && value >= 0.0 {
        value
    } else {
        0.0
    }
}

pub(super) fn receiver_clock_error_m(receiver_clock: &ObservationReceiverClock) -> Meters {
    Meters(receiver_clock.bias_sigma_s * SPEED_OF_LIGHT_MPS)
}

pub(super) fn receiver_clock_carrier_phase_cycles(
    receiver_clock: &ObservationReceiverClock,
    signal: SignalSpec,
) -> f64 {
    receiver_clock.bias_s * signal.carrier_hz.value()
}
