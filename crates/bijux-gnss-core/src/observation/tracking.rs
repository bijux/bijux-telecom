use super::{default_signal_band, default_signal_code, SignalDelayAlignment};
use crate::api::SignalCode;
use crate::api::{
    Chips, Constellation, Cycles, Epoch, GlonassFrequencyChannel, Hertz, ReceiverSampleTrace,
    SatId, SignalBand,
};
use serde::{Deserialize, Serialize};

fn default_cycles_zero() -> Cycles {
    Cycles(0.0)
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum TrackingLifecycleState {
    Init,
    PullIn,
    Lock,
    Degraded,
    Lost,
    Inactive,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackingAssumptions {
    pub integration_ms: u32,
    #[serde(default)]
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub discriminator_family: String,
    pub aiding_mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TrackingUncertainty {
    pub code_phase_samples: f64,
    pub carrier_phase_cycles: f64,
    pub doppler_hz: f64,
    pub cn0_dbhz: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackTransition {
    pub sat: SatId,
    pub channel_id: u8,
    pub epoch_idx: u64,
    pub sample_index: u64,
    pub from_state: String,
    pub to_state: String,
    pub reason: String,
    pub lock_quality: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TrackingTransmitTime {
    pub transmit_gps_time: crate::api::GpsTime,
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackEpoch {
    pub epoch: Epoch,
    pub sample_index: u64,
    #[serde(default)]
    pub source_time: ReceiverSampleTrace,
    pub sat: SatId,
    #[serde(default = "default_signal_band")]
    pub signal_band: SignalBand,
    #[serde(default = "default_signal_code")]
    pub signal_code: SignalCode,
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    pub prompt_i: f32,
    pub prompt_q: f32,
    #[serde(default)]
    pub early_i: f32,
    #[serde(default)]
    pub early_q: f32,
    #[serde(default)]
    pub late_i: f32,
    #[serde(default)]
    pub late_q: f32,
    pub carrier_hz: Hertz,
    #[serde(default = "default_cycles_zero")]
    pub carrier_phase_cycles: Cycles,
    pub code_rate_hz: Hertz,
    pub code_phase_samples: Chips,
    pub lock: bool,
    pub cn0_dbhz: f64,
    pub pll_lock: bool,
    pub dll_lock: bool,
    pub fll_lock: bool,
    pub cycle_slip: bool,
    pub nav_bit_lock: bool,
    #[serde(default)]
    pub navigation_bit_sign: Option<i8>,
    pub dll_err: f32,
    pub pll_err: f32,
    pub fll_err: f32,
    #[serde(default)]
    pub anti_false_lock: bool,
    #[serde(default)]
    pub cycle_slip_reason: Option<String>,
    #[serde(default)]
    pub lock_state: String,
    #[serde(default)]
    pub lock_state_reason: Option<String>,
    #[serde(default)]
    pub channel_id: Option<u8>,
    #[serde(default)]
    pub channel_uid: String,
    #[serde(default)]
    pub tracking_provenance: String,
    #[serde(default)]
    pub tracking_assumptions: Option<TrackingAssumptions>,
    #[serde(default)]
    pub signal_delay_alignment: Option<SignalDelayAlignment>,
    #[serde(default)]
    pub transmit_time: Option<TrackingTransmitTime>,
    #[serde(default)]
    pub tracking_uncertainty: Option<TrackingUncertainty>,
    #[serde(default)]
    pub processing_ms: Option<f64>,
}

impl Default for TrackEpoch {
    fn default() -> Self {
        Self {
            epoch: Epoch { index: 0 },
            sample_index: 0,
            source_time: ReceiverSampleTrace::default(),
            sat: SatId { constellation: Constellation::Unknown, prn: 0 },
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            prompt_i: 0.0,
            prompt_q: 0.0,
            early_i: 0.0,
            early_q: 0.0,
            late_i: 0.0,
            late_q: 0.0,
            carrier_hz: Hertz(0.0),
            carrier_phase_cycles: Cycles(0.0),
            code_rate_hz: Hertz(0.0),
            code_phase_samples: Chips(0.0),
            lock: false,
            cn0_dbhz: 0.0,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cycle_slip: false,
            nav_bit_lock: false,
            navigation_bit_sign: None,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: "inactive".to_string(),
            lock_state_reason: None,
            channel_id: None,
            channel_uid: String::new(),
            tracking_provenance: String::new(),
            tracking_assumptions: None,
            signal_delay_alignment: None,
            transmit_time: None,
            tracking_uncertainty: None,
            processing_ms: None,
        }
    }
}

impl TrackEpoch {
    pub fn lifecycle_state(&self) -> TrackingLifecycleState {
        match self.lock_state.as_str() {
            "tracking" => TrackingLifecycleState::Lock,
            "acquired" => TrackingLifecycleState::Init,
            "pull_in" => TrackingLifecycleState::PullIn,
            "lost" => TrackingLifecycleState::Lost,
            "degraded" => TrackingLifecycleState::Degraded,
            _ => TrackingLifecycleState::Inactive,
        }
    }
}
