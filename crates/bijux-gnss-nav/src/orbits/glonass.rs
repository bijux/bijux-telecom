#![allow(missing_docs)]

use std::cmp::Ordering;
use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    glonass_slot_sat, GlonassFrequencyChannel, GlonassSlot, ObsSignalTiming, SatId,
};

use super::glonass_orbit::{
    propagate_glonass_orbit_refined, GlonassOrbitPropagation, GlonassPropagationConfig,
    GLONASS_EARTH_ROTATION_RATE_RAD_S,
};

const GLONASS_DAY_S: f64 = 86_400.0;
const GLONASS_DAY_SECONDS: u32 = 86_400;
const GLONASS_STRING_DURATION_S: u32 = 2;
const GLONASS_MAX_NAVIGATION_AGE_S: f64 = 900.0;
const PZ90_02_TO_ITRF2000_X_M: f64 = -0.36;
const PZ90_02_TO_ITRF2000_Y_M: f64 = 0.08;
const PZ90_02_TO_ITRF2000_Z_M: f64 = 0.18;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassFrameTime {
    pub hour: u8,
    pub minute: u8,
    pub half_minute: bool,
}

impl GlonassFrameTime {
    pub fn seconds_of_day(self) -> u32 {
        u32::from(self.hour) * 3_600
            + u32::from(self.minute) * 60
            + if self.half_minute { 30 } else { 0 }
    }

    pub fn string_timing(self, string_number: u8) -> Option<GlonassStringTiming> {
        if !(1..=15).contains(&string_number) {
            return None;
        }
        let frame_start_s = self.seconds_of_day();
        let string_offset_s = u32::from(string_number - 1) * GLONASS_STRING_DURATION_S;
        let string_start_s = frame_start_s + string_offset_s;
        let string_end_s = string_start_s + GLONASS_STRING_DURATION_S;
        Some(GlonassStringTiming {
            string_number,
            start_day_offset: (string_start_s / GLONASS_DAY_SECONDS) as i8,
            start_seconds_of_day: string_start_s % GLONASS_DAY_SECONDS,
            end_day_offset: (string_end_s / GLONASS_DAY_SECONDS) as i8,
            end_seconds_of_day: string_end_s % GLONASS_DAY_SECONDS,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GlonassStringTiming {
    pub string_number: u8,
    pub start_day_offset: i8,
    pub start_seconds_of_day: u32,
    pub end_day_offset: i8,
    pub end_seconds_of_day: u32,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GlonassLeapSecondAnnouncement {
    NoCorrection,
    PositiveCorrection,
    DecisionPending,
    NegativeCorrection,
}

impl GlonassLeapSecondAnnouncement {
    pub fn from_kp_word(word: u8) -> Option<Self> {
        match word {
            0 => Some(Self::NoCorrection),
            1 => Some(Self::PositiveCorrection),
            2 => Some(Self::DecisionPending),
            3 => Some(Self::NegativeCorrection),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassUt1Correction {
    pub utc_su_minus_ut1_at_day_start_s: f64,
    pub utc_su_minus_ut1_daily_change_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassSuperframeTimeData {
    pub ut1_correction: GlonassUt1Correction,
    pub leap_second_announcement: GlonassLeapSecondAnnouncement,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassUtcRelation {
    pub system_time: GlonassSystemTime,
    pub utc_su_plus_three_hours_minus_glonass_s: f64,
    pub gps_minus_glonass_fractional_s: f64,
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
    pub resolved_day_index: Option<i64>,
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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassSlotChannelAssociation {
    pub sat: SatId,
    pub expected_slot: Option<GlonassSlot>,
    pub reported_slot: Option<GlonassSlot>,
    pub almanac_frequency_channel: Option<GlonassFrequencyChannel>,
    pub almanac_health_operational: Option<bool>,
    pub is_slot_consistent: bool,
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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassSatelliteClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub base_bias_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_correction: GlonassSatelliteClockCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassNumericalOrbitPropagation {
    pub sat: SatId,
    pub transmit_gps_tow_s: f64,
    pub delta_time_s: f64,
    pub pz90_position_m: [f64; 3],
    pub pz90_velocity_mps: [f64; 3],
    pub base_step_s: f64,
    pub accepted_step_s: f64,
    pub refinements: usize,
    pub position_refinement_m: f64,
    pub position_tolerance_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassNavigationAge {
    pub transmit_gps_tow_s: f64,
    pub transmit_glonass_time_s: f64,
    pub transmit_day_index: Option<i64>,
    pub ephemeris_reference_time_s: f64,
    pub ephemeris_day_index: Option<i64>,
    pub delta_time_s: f64,
    pub age_s: f64,
    pub max_age_s: f64,
}

impl GlonassNavigationAge {
    pub fn is_valid(&self) -> bool {
        self.age_s <= self.max_age_s
    }

    pub fn is_stale(&self) -> bool {
        !self.is_valid()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GlonassTransmitTime {
    pub transmit_gps_tow_s: f64,
    pub gps_minus_glonass_s: f64,
    pub transmit_seconds_of_day: f64,
    pub transmit_day_index: Option<i64>,
    pub frame_seconds_of_day: f64,
    pub frame_day_index: Option<i64>,
    pub ephemeris_reference_time_s: f64,
    pub ephemeris_day_index: Option<i64>,
    pub delta_time_s: f64,
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

pub fn glonass_gps_minus_glonass_s(navigation: &GlonassBroadcastNavigationFrame) -> Option<f64> {
    navigation.system_time.map(|system_time| system_time.gps_minus_glonass_s)
}

pub fn glonass_utc_relation(
    navigation: &GlonassBroadcastNavigationFrame,
) -> Option<GlonassUtcRelation> {
    navigation.system_time.map(|system_time| GlonassUtcRelation {
        system_time: system_time.system_time,
        utc_su_plus_three_hours_minus_glonass_s: system_time.utc_offset_s,
        gps_minus_glonass_fractional_s: system_time.gps_minus_glonass_s,
    })
}

pub fn glonass_slot_channel_association(
    navigation: &GlonassBroadcastNavigationFrame,
) -> GlonassSlotChannelAssociation {
    let expected_slot = GlonassSlot::new(navigation.sat.prn);
    let almanac_entry = navigation.almanac_entries.iter().find(|entry| entry.sat == navigation.sat);
    let reported_slot_matches = match navigation.immediate.reported_slot {
        Some(reported_slot) => Some(reported_slot) == expected_slot,
        None => true,
    };

    GlonassSlotChannelAssociation {
        sat: navigation.sat,
        expected_slot,
        reported_slot: navigation.immediate.reported_slot,
        almanac_frequency_channel: almanac_entry.map(|entry| entry.frequency_channel),
        almanac_health_operational: almanac_entry.map(|entry| entry.health_operational),
        is_slot_consistent: reported_slot_matches,
    }
}

pub fn glonass_navigation_age(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassNavigationAge> {
    let transmit_time = glonass_transmit_time(navigation, transmit_gps_tow_s)?;
    Some(GlonassNavigationAge {
        transmit_gps_tow_s,
        transmit_glonass_time_s: transmit_time.transmit_seconds_of_day,
        transmit_day_index: transmit_time.transmit_day_index,
        ephemeris_reference_time_s: transmit_time.ephemeris_reference_time_s,
        ephemeris_day_index: transmit_time.ephemeris_day_index,
        delta_time_s: transmit_time.delta_time_s,
        age_s: transmit_time.delta_time_s.abs(),
        max_age_s: GLONASS_MAX_NAVIGATION_AGE_S,
    })
}

pub fn glonass_transmit_time(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassTransmitTime> {
    let gps_minus_glonass_s = glonass_gps_minus_glonass_s(navigation)?;
    let transmit_seconds_of_day =
        glonass_time_of_day_from_gps_tow_s(transmit_gps_tow_s, gps_minus_glonass_s);
    let frame_seconds_of_day = f64::from(navigation.immediate.frame_time.seconds_of_day());
    let ephemeris_reference_time_s = f64::from(navigation.immediate.ephemeris_reference_time_s);
    let frame_day_index = navigation.immediate.resolved_day_index;
    let transmit_day_index = frame_day_index.map(|day_index| {
        glonass_day_index_near_time(day_index, frame_seconds_of_day, transmit_seconds_of_day)
    });
    let ephemeris_day_index = frame_day_index.map(|day_index| {
        glonass_day_index_near_time(day_index, frame_seconds_of_day, ephemeris_reference_time_s)
    });
    let delta_time_s = match (transmit_day_index, ephemeris_day_index) {
        (Some(transmit_day_index), Some(ephemeris_day_index)) => {
            (transmit_day_index - ephemeris_day_index) as f64 * GLONASS_DAY_S
                + transmit_seconds_of_day
                - ephemeris_reference_time_s
        }
        _ => wrap_glonass_time_delta_s(transmit_seconds_of_day - ephemeris_reference_time_s),
    };

    Some(GlonassTransmitTime {
        transmit_gps_tow_s,
        gps_minus_glonass_s,
        transmit_seconds_of_day,
        transmit_day_index,
        frame_seconds_of_day,
        frame_day_index,
        ephemeris_reference_time_s,
        ephemeris_day_index,
        delta_time_s,
    })
}

pub fn is_glonass_navigation_valid(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> bool {
    glonass_navigation_age(navigation, transmit_gps_tow_s).is_some_and(|age| age.is_valid())
}

pub fn select_best_glonass_navigation<'a>(
    navigation_frames: &'a [GlonassBroadcastNavigationFrame],
    sat: SatId,
    transmit_gps_tow_s: f64,
) -> Option<&'a GlonassBroadcastNavigationFrame> {
    navigation_frames
        .iter()
        .filter(|navigation| navigation.sat == sat)
        .filter(|navigation| is_glonass_navigation_valid(navigation, transmit_gps_tow_s))
        .min_by(|left, right| {
            let left_age = glonass_navigation_age(left, transmit_gps_tow_s)
                .expect("filtered valid GLONASS age");
            let right_age = glonass_navigation_age(right, transmit_gps_tow_s)
                .expect("filtered valid GLONASS age");
            left_age.age_s.partial_cmp(&right_age.age_s).unwrap_or(Ordering::Equal).then_with(
                || {
                    left_age
                        .transmit_glonass_time_s
                        .partial_cmp(&right_age.transmit_glonass_time_s)
                        .unwrap_or(Ordering::Equal)
                },
            )
        })
}

pub fn glonass_satellite_clock_correction(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassSatelliteClockCorrection> {
    let delta_t_s = glonass_broadcast_delta_time_s(navigation, transmit_gps_tow_s)?;
    let drift_s_per_s = navigation.immediate.relative_frequency_bias;
    let base_bias_s = -navigation.immediate.clock_bias_s;
    Some(GlonassSatelliteClockCorrection {
        bias_s: base_bias_s + drift_s_per_s * delta_t_s,
        drift_s_per_s,
        base_bias_s,
    })
}

pub fn glonass_earth_rotation_correction(
    x_m: f64,
    y_m: f64,
    signal_travel_time_s: f64,
) -> GlonassEarthRotationCorrection {
    let rotation_rad = GLONASS_EARTH_ROTATION_RATE_RAD_S * signal_travel_time_s;
    let (x_rotated_m, y_rotated_m) = if rotation_rad.abs() > 0.0 {
        let cos_rot = rotation_rad.cos();
        let sin_rot = rotation_rad.sin();
        (cos_rot * x_m + sin_rot * y_m, -sin_rot * x_m + cos_rot * y_m)
    } else {
        (x_m, y_m)
    };
    GlonassEarthRotationCorrection {
        signal_travel_time_s,
        rotation_rad,
        delta_x_m: x_rotated_m - x_m,
        delta_y_m: y_rotated_m - y_m,
    }
}

pub fn sat_state_glonass_l1_at_receive_time(
    navigation: &GlonassBroadcastNavigationFrame,
    receive_gps_tow_s: f64,
    signal_travel_time_s: f64,
) -> Option<GlonassSatState> {
    sat_state_glonass_l1(navigation, receive_gps_tow_s - signal_travel_time_s, signal_travel_time_s)
}

pub fn sat_state_glonass_l1_from_observation(
    navigation: &GlonassBroadcastNavigationFrame,
    receive_gps_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> Option<GlonassSatState> {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_gps_tow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_gps_tow_s - signal_travel_time_s);
    sat_state_glonass_l1(navigation, transmit_gps_tow_s, signal_travel_time_s)
}

pub fn sat_state_glonass_l1(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
    signal_travel_time_s: f64,
) -> Option<GlonassSatState> {
    let propagated_state = glonass_refined_orbit_propagation(navigation, transmit_gps_tow_s)?.state;
    let clock_correction = glonass_satellite_clock_correction(navigation, transmit_gps_tow_s)?;

    let mut x_m = propagated_state.position_m[0] + PZ90_02_TO_ITRF2000_X_M;
    let mut y_m = propagated_state.position_m[1] + PZ90_02_TO_ITRF2000_Y_M;
    let z_m = propagated_state.position_m[2] + PZ90_02_TO_ITRF2000_Z_M;

    let earth_rotation = glonass_earth_rotation_correction(x_m, y_m, signal_travel_time_s);
    x_m += earth_rotation.delta_x_m;
    y_m += earth_rotation.delta_y_m;

    Some(GlonassSatState { x_m, y_m, z_m, clock_correction })
}

pub fn glonass_numerical_orbit_propagation(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassNumericalOrbitPropagation> {
    let delta_time_s = glonass_broadcast_delta_time_s(navigation, transmit_gps_tow_s)?;
    let propagation = glonass_refined_orbit_propagation(navigation, transmit_gps_tow_s)?;
    Some(GlonassNumericalOrbitPropagation {
        sat: navigation.sat,
        transmit_gps_tow_s,
        delta_time_s,
        pz90_position_m: propagation.state.position_m,
        pz90_velocity_mps: propagation.state.velocity_mps,
        base_step_s: propagation.convergence.base_step_s,
        accepted_step_s: propagation.convergence.accepted_step_s,
        refinements: propagation.convergence.refinements,
        position_refinement_m: propagation.convergence.position_refinement_m,
        position_tolerance_m: propagation.convergence.position_tolerance_m,
    })
}

fn glonass_broadcast_delta_time_s(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<f64> {
    let age = glonass_navigation_age(navigation, transmit_gps_tow_s)?;
    if age.is_stale() {
        return None;
    }
    Some(age.delta_time_s)
}

fn glonass_refined_orbit_propagation(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassOrbitPropagation> {
    let delta_t_s = glonass_broadcast_delta_time_s(navigation, transmit_gps_tow_s)?;
    propagate_glonass_orbit_refined(
        navigation.immediate.state_vector,
        delta_t_s,
        GlonassPropagationConfig::default(),
    )
}

fn glonass_time_of_day_from_gps_tow_s(transmit_gps_tow_s: f64, gps_minus_glonass_s: f64) -> f64 {
    (transmit_gps_tow_s - gps_minus_glonass_s).rem_euclid(GLONASS_DAY_S)
}

fn glonass_day_index_near_time(
    reference_day_index: i64,
    reference_seconds_of_day: f64,
    target_seconds_of_day: f64,
) -> i64 {
    let delta_t_s = target_seconds_of_day - reference_seconds_of_day;
    if delta_t_s > GLONASS_DAY_S / 2.0 {
        reference_day_index - 1
    } else if delta_t_s < -GLONASS_DAY_S / 2.0 {
        reference_day_index + 1
    } else {
        reference_day_index
    }
}

fn wrap_glonass_time_delta_s(delta_t_s: f64) -> f64 {
    let mut wrapped = delta_t_s;
    let half_day_s = GLONASS_DAY_S / 2.0;
    while wrapped > half_day_s {
        wrapped -= GLONASS_DAY_S;
    }
    while wrapped < -half_day_s {
        wrapped += GLONASS_DAY_S;
    }
    wrapped
}

#[cfg(test)]
mod tests {
    use super::{
        glonass_earth_rotation_correction, glonass_navigation_age,
        glonass_numerical_orbit_propagation, glonass_satellite_clock_correction,
        glonass_slot_channel_association, glonass_transmit_time, glonass_utc_relation,
        sat_state_glonass_l1, GlonassAlmanacEntry, GlonassAlmanacTimeData,
        GlonassBroadcastNavigationFrame, GlonassFrameTime, GlonassImmediateHealth,
        GlonassImmediateNavigationData, GlonassLeapSecondAnnouncement, GlonassSatelliteType,
        GlonassStateVector, GlonassSystemTime, PZ90_02_TO_ITRF2000_X_M, PZ90_02_TO_ITRF2000_Y_M,
        PZ90_02_TO_ITRF2000_Z_M,
    };
    use bijux_gnss_core::api::{Constellation, GlonassFrequencyChannel, GlonassSlot, SatId};

    fn sample_navigation() -> GlonassBroadcastNavigationFrame {
        let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        GlonassBroadcastNavigationFrame {
            sat,
            immediate: GlonassImmediateNavigationData {
                sat,
                frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
                ephemeris_reference_time_s: 83_700,
                tb_update_interval_min: 30,
                tb_is_odd: Some(true),
                state_vector: GlonassStateVector {
                    x_m: -7_557_760.253_906_25,
                    y_m: -23_962_225.585_937_5,
                    z_m: -4_337_567.871_093_75,
                    vx_mps: 101.318_359_375,
                    vy_mps: 602.112_770_080_566_4,
                    vz_mps: -3_495.733_261_108_398_4,
                    ax_mps2: -3.725_290_298_461_914e-6,
                    ay_mps2: 0.0,
                    az_mps2: 1.862_645_149_230_957e-6,
                },
                relative_frequency_bias: 0.0,
                clock_bias_s: -2.572_406_083_345_413_2e-5,
                l2_l1_delay_s: Some(5.587_935_448e-9),
                health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
                immediate_data_age_days: 28,
                satellite_type: GlonassSatelliteType::GlonassM,
                reported_slot: None,
                system_time: Some(GlonassSystemTime {
                    day_number: 864,
                    four_year_interval: Some(8),
                }),
                resolved_day_index: None,
                accuracy_code: Some(2),
            },
            system_time: Some(GlonassAlmanacTimeData {
                system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
                utc_offset_s: 0.0,
                gps_minus_glonass_s: -10_782.0,
            }),
            almanac_entries: Vec::new(),
        }
    }

    #[test]
    fn glonass_navigation_age_uses_gps_to_glonass_offset() {
        let navigation = sample_navigation();

        let age = glonass_navigation_age(&navigation, 504_918.0).expect("GLONASS navigation age");

        assert_eq!(age.transmit_glonass_time_s, 83_700.0);
        assert_eq!(age.transmit_day_index, None);
        assert_eq!(age.ephemeris_reference_time_s, 83_700.0);
        assert_eq!(age.ephemeris_day_index, None);
        assert_eq!(age.delta_time_s, 0.0);
        assert_eq!(age.age_s, 0.0);
        assert!(age.is_valid());
    }

    #[test]
    fn glonass_frame_time_tracks_string_boundaries() {
        let frame_time = GlonassFrameTime { hour: 12, minute: 0, half_minute: true };

        let first = frame_time.string_timing(1).expect("first string timing");
        let last = frame_time.string_timing(15).expect("last string timing");

        assert_eq!(first.string_number, 1);
        assert_eq!(first.start_day_offset, 0);
        assert_eq!(first.start_seconds_of_day, 43_230);
        assert_eq!(first.end_seconds_of_day, 43_232);
        assert_eq!(last.start_seconds_of_day, 43_258);
        assert_eq!(last.end_seconds_of_day, 43_260);
        assert!(frame_time.string_timing(0).is_none());
        assert!(frame_time.string_timing(16).is_none());
    }

    #[test]
    fn glonass_string_timing_records_midnight_rollover() {
        let frame_time = GlonassFrameTime { hour: 23, minute: 59, half_minute: true };

        let last = frame_time.string_timing(15).expect("last string timing");

        assert_eq!(last.start_day_offset, 0);
        assert_eq!(last.start_seconds_of_day, 86_398);
        assert_eq!(last.end_day_offset, 1);
        assert_eq!(last.end_seconds_of_day, 0);
    }

    #[test]
    fn glonass_utc_relation_preserves_broadcast_offsets() {
        let navigation = sample_navigation();

        let relation = glonass_utc_relation(&navigation).expect("UTC relation");

        assert_eq!(relation.system_time.day_number, 864);
        assert_eq!(relation.system_time.four_year_interval, Some(8));
        assert_eq!(relation.utc_su_plus_three_hours_minus_glonass_s, 0.0);
        assert_eq!(relation.gps_minus_glonass_fractional_s, -10_782.0);
    }

    #[test]
    fn glonass_leap_second_announcement_decodes_kp_word() {
        assert_eq!(
            GlonassLeapSecondAnnouncement::from_kp_word(0),
            Some(GlonassLeapSecondAnnouncement::NoCorrection)
        );
        assert_eq!(
            GlonassLeapSecondAnnouncement::from_kp_word(1),
            Some(GlonassLeapSecondAnnouncement::PositiveCorrection)
        );
        assert_eq!(
            GlonassLeapSecondAnnouncement::from_kp_word(2),
            Some(GlonassLeapSecondAnnouncement::DecisionPending)
        );
        assert_eq!(
            GlonassLeapSecondAnnouncement::from_kp_word(3),
            Some(GlonassLeapSecondAnnouncement::NegativeCorrection)
        );
        assert_eq!(GlonassLeapSecondAnnouncement::from_kp_word(4), None);
    }

    #[test]
    fn glonass_slot_channel_association_reports_almanac_channel() {
        let mut navigation = sample_navigation();
        let slot = GlonassSlot::new(14).expect("slot");
        let frequency_channel = GlonassFrequencyChannel::new(-4).expect("frequency channel");
        navigation.immediate.reported_slot = Some(slot);
        navigation.almanac_entries.push(GlonassAlmanacEntry {
            sat: navigation.sat,
            frequency_channel,
            health_operational: true,
            longitude_of_ascending_node_rad: 0.0,
            ascending_node_time_s: 0.0,
            inclination_delta_rad: 0.0,
            draconian_period_correction_s: 0.0,
            draconian_period_rate_s_per_orbit: 0.0,
            eccentricity: 0.0,
            argument_of_perigee_rad: 0.0,
            clock_bias_s: 0.0,
            satellite_type: GlonassSatelliteType::GlonassM,
        });

        let association = glonass_slot_channel_association(&navigation);

        assert_eq!(association.expected_slot, Some(slot));
        assert_eq!(association.reported_slot, Some(slot));
        assert_eq!(association.almanac_frequency_channel, Some(frequency_channel));
        assert_eq!(association.almanac_health_operational, Some(true));
        assert!(association.is_slot_consistent);
    }

    #[test]
    fn glonass_slot_channel_association_exposes_inconsistent_reported_slot() {
        let mut navigation = sample_navigation();
        navigation.immediate.reported_slot = GlonassSlot::new(13);

        let association = glonass_slot_channel_association(&navigation);

        assert_eq!(association.expected_slot, GlonassSlot::new(14));
        assert_eq!(association.reported_slot, GlonassSlot::new(13));
        assert!(!association.is_slot_consistent);
    }

    #[test]
    fn glonass_transmit_time_resolves_midnight_ephemeris_day() {
        let mut navigation = sample_navigation();
        navigation.immediate.frame_time =
            GlonassFrameTime { hour: 23, minute: 59, half_minute: true };
        navigation.immediate.ephemeris_reference_time_s = 0;
        navigation.immediate.resolved_day_index = Some(864);
        navigation.system_time.as_mut().expect("system time").gps_minus_glonass_s = 0.0;

        let transmit_time = glonass_transmit_time(&navigation, 5.0).expect("transmit time");

        assert_eq!(transmit_time.frame_day_index, Some(864));
        assert_eq!(transmit_time.transmit_day_index, Some(865));
        assert_eq!(transmit_time.ephemeris_day_index, Some(865));
        assert_eq!(transmit_time.transmit_seconds_of_day, 5.0);
        assert_eq!(transmit_time.ephemeris_reference_time_s, 0.0);
        assert_eq!(transmit_time.delta_time_s, 5.0);
    }

    #[test]
    fn glonass_transmit_time_falls_back_to_wrapped_time_without_day_reference() {
        let navigation = sample_navigation();

        let transmit_time = glonass_transmit_time(&navigation, 504_918.0).expect("transmit time");

        assert_eq!(transmit_time.frame_day_index, None);
        assert_eq!(transmit_time.transmit_day_index, None);
        assert_eq!(transmit_time.ephemeris_day_index, None);
        assert_eq!(transmit_time.transmit_seconds_of_day, 83_700.0);
        assert_eq!(transmit_time.delta_time_s, 0.0);
    }

    #[test]
    fn glonass_navigation_age_preserves_midnight_day_indices() {
        let mut navigation = sample_navigation();
        navigation.immediate.frame_time =
            GlonassFrameTime { hour: 23, minute: 59, half_minute: true };
        navigation.immediate.ephemeris_reference_time_s = 0;
        navigation.immediate.resolved_day_index = Some(864);
        navigation.system_time.as_mut().expect("system time").gps_minus_glonass_s = 0.0;

        let age = glonass_navigation_age(&navigation, 5.0).expect("navigation age");

        assert_eq!(age.transmit_day_index, Some(865));
        assert_eq!(age.ephemeris_day_index, Some(865));
        assert_eq!(age.delta_time_s, 5.0);
        assert_eq!(age.age_s, 5.0);
        assert!(age.is_valid());
    }

    #[test]
    fn glonass_satellite_clock_correction_tracks_broadcast_drift() {
        let navigation = sample_navigation();

        let clock =
            glonass_satellite_clock_correction(&navigation, 505_818.0).expect("clock correction");

        let expected_bias_s = 2.572_406_083_345_413_2e-5;
        assert!((clock.base_bias_s - 2.572_406_083_345_413_2e-5).abs() < 1.0e-18);
        assert!((clock.bias_s - expected_bias_s).abs() < 1.0e-18);
        assert_eq!(clock.drift_s_per_s, 0.0);
    }

    #[test]
    fn glonass_satellite_clock_correction_uses_resolved_midnight_delta() {
        let mut navigation = sample_navigation();
        navigation.immediate.frame_time =
            GlonassFrameTime { hour: 23, minute: 59, half_minute: true };
        navigation.immediate.ephemeris_reference_time_s = 0;
        navigation.immediate.resolved_day_index = Some(864);
        navigation.immediate.clock_bias_s = 0.0;
        navigation.immediate.relative_frequency_bias = 1.0e-10;
        navigation.system_time.as_mut().expect("system time").gps_minus_glonass_s = 0.0;

        let clock = glonass_satellite_clock_correction(&navigation, 5.0).expect("clock correction");

        assert!((clock.bias_s - 5.0e-10).abs() < 1.0e-18);
        assert_eq!(clock.drift_s_per_s, 1.0e-10);
        assert_eq!(clock.base_bias_s, 0.0);
    }

    #[test]
    fn glonass_state_at_reference_time_matches_broadcast_state_vector() {
        let navigation = sample_navigation();

        let state = sat_state_glonass_l1(&navigation, 504_918.0, 0.0).expect("satellite state");

        assert!(
            (state.x_m - (navigation.immediate.state_vector.x_m + PZ90_02_TO_ITRF2000_X_M)).abs()
                < 1.0e-9
        );
        assert!(
            (state.y_m - (navigation.immediate.state_vector.y_m + PZ90_02_TO_ITRF2000_Y_M)).abs()
                < 1.0e-9
        );
        assert!(
            (state.z_m - (navigation.immediate.state_vector.z_m + PZ90_02_TO_ITRF2000_Z_M)).abs()
                < 1.0e-9
        );
        assert!((state.clock_correction.bias_s - 2.572_406_083_345_413_2e-5).abs() < 1.0e-18);
    }

    #[test]
    fn glonass_numerical_orbit_propagation_exposes_convergence() {
        let navigation = sample_navigation();

        let propagation = glonass_numerical_orbit_propagation(&navigation, 505_518.0)
            .expect("GLONASS numerical propagation");

        assert_eq!(propagation.sat, navigation.sat);
        assert_eq!(propagation.transmit_gps_tow_s, 505_518.0);
        assert_eq!(propagation.delta_time_s, 600.0);
        assert_eq!(propagation.base_step_s, 60.0);
        assert!(propagation.accepted_step_s > 0.0);
        assert!(propagation.accepted_step_s <= 30.0);
        assert!(propagation.refinements > 0);
        assert!(propagation.position_refinement_m <= propagation.position_tolerance_m);
        assert!(propagation.pz90_position_m.iter().all(|component| component.is_finite()));
        assert!(propagation.pz90_velocity_mps.iter().all(|component| component.is_finite()));
    }

    #[test]
    fn glonass_satellite_state_uses_numerical_orbit_propagation() {
        let navigation = sample_navigation();
        let transmit_gps_tow_s = 505_518.0;
        let signal_travel_time_s = 0.073;

        let propagation = glonass_numerical_orbit_propagation(&navigation, transmit_gps_tow_s)
            .expect("GLONASS numerical propagation");
        let state = sat_state_glonass_l1(&navigation, transmit_gps_tow_s, signal_travel_time_s)
            .expect("GLONASS satellite state");

        let pz90_shifted_x_m = propagation.pz90_position_m[0] + PZ90_02_TO_ITRF2000_X_M;
        let pz90_shifted_y_m = propagation.pz90_position_m[1] + PZ90_02_TO_ITRF2000_Y_M;
        let earth_rotation = glonass_earth_rotation_correction(
            pz90_shifted_x_m,
            pz90_shifted_y_m,
            signal_travel_time_s,
        );

        assert!((state.x_m - (pz90_shifted_x_m + earth_rotation.delta_x_m)).abs() < 1.0e-9);
        assert!((state.y_m - (pz90_shifted_y_m + earth_rotation.delta_y_m)).abs() < 1.0e-9);
        assert!(
            (state.z_m - (propagation.pz90_position_m[2] + PZ90_02_TO_ITRF2000_Z_M)).abs() < 1.0e-9
        );
    }

    #[test]
    fn glonass_numerical_orbit_propagation_refuses_nonfinite_state_vector() {
        let mut navigation = sample_navigation();
        navigation.immediate.state_vector.x_m = f64::NAN;

        let propagation = glonass_numerical_orbit_propagation(&navigation, 505_518.0);
        let state = sat_state_glonass_l1(&navigation, 505_518.0, 0.073);

        assert!(propagation.is_none());
        assert!(state.is_none());
    }

    #[test]
    fn glonass_state_rejects_stale_navigation() {
        let navigation = sample_navigation();

        let state = sat_state_glonass_l1(&navigation, 506_000.0, 0.0);

        assert!(state.is_none());
    }
}
