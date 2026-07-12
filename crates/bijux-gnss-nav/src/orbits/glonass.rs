#![allow(missing_docs)]

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    glonass_slot_sat, GlonassFrequencyChannel, GlonassSlot, SatId,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassFrameTime {
    pub hour: u8,
    pub minute: u8,
    pub half_minute: bool,
}

impl GlonassFrameTime {
    pub fn seconds_of_day(self) -> u32 {
        u32::from(self.hour) * 3_600 + u32::from(self.minute) * 60 + if self.half_minute { 30 } else { 0 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GlonassSatelliteType {
    Glonass,
    GlonassM,
    Reserved(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassImmediateHealth {
    pub line_unhealthy: bool,
    pub status_code: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassStateVector {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub vz_mps: f64,
    pub ax_mps2: f64,
    pub ay_mps2: f64,
    pub az_mps2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassSystemTime {
    pub day_number: u16,
    pub four_year_interval: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassAlmanacTimeData {
    pub system_time: GlonassSystemTime,
    pub utc_offset_s: f64,
    pub gps_minus_glonass_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassImmediateNavigationData {
    pub sat: SatId,
    pub frame_time: GlonassFrameTime,
    pub ephemeris_reference_time_s: u32,
    pub tb_update_interval_min: u8,
    pub tb_is_odd: Option<bool>,
    pub state_vector: GlonassStateVector,
    pub relative_frequency_bias: f64,
    pub clock_bias_s: f64,
    pub l2_l1_delay_s: Option<f64>,
    pub health: GlonassImmediateHealth,
    pub immediate_data_age_days: u8,
    pub satellite_type: GlonassSatelliteType,
    pub reported_slot: Option<GlonassSlot>,
    pub system_time: Option<GlonassSystemTime>,
    pub accuracy_code: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassAlmanacEntry {
    pub sat: SatId,
    pub frequency_channel: GlonassFrequencyChannel,
    pub health_operational: bool,
    pub longitude_of_ascending_node_rad: f64,
    pub ascending_node_time_s: f64,
    pub inclination_delta_rad: f64,
    pub draconian_period_correction_s: f64,
    pub draconian_period_rate_s_per_orbit: f64,
    pub eccentricity: f64,
    pub argument_of_perigee_rad: f64,
    pub clock_bias_s: f64,
    pub satellite_type: GlonassSatelliteType,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GlonassBroadcastNavigationFrame {
    pub sat: SatId,
    pub immediate: GlonassImmediateNavigationData,
    pub system_time: Option<GlonassAlmanacTimeData>,
    pub almanac_entries: Vec<GlonassAlmanacEntry>,
}

impl GlonassBroadcastNavigationFrame {
    pub fn new(slot: GlonassSlot, immediate: GlonassImmediateNavigationData) -> Self {
        Self {
            sat: glonass_slot_sat(slot),
            immediate,
            system_time: None,
            almanac_entries: Vec::new(),
        }
    }
}

pub fn glonass_satellite_type_from_word(word: u8) -> GlonassSatelliteType {
    match word {
        0 => GlonassSatelliteType::Glonass,
        1 => GlonassSatelliteType::GlonassM,
        value => GlonassSatelliteType::Reserved(value),
    }
}

pub fn semicircles_to_radians(value: f64) -> f64 {
    value * PI
}
