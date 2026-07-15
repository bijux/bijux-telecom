#![allow(missing_docs)]

use std::cmp::Ordering;
use std::f64::consts::PI;

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{
    glonass_slot_sat, GlonassFrequencyChannel, GlonassSlot, ObsSignalTiming, SatId,
};

const GLONASS_MU_M3PS2: f64 = 398_600.44e9;
const GLONASS_EARTH_RADIUS_M: f64 = 6_378_136.0;
const GLONASS_C20: f64 = -1_082.63e-6;
const GLONASS_OMEGA_E_DOT: f64 = 7.292_115e-5;
const GLONASS_DAY_S: f64 = 86_400.0;
const GLONASS_DAY_SECONDS: u32 = 86_400;
const GLONASS_STRING_DURATION_S: u32 = 2;
const GLONASS_MAX_NAVIGATION_AGE_S: f64 = 900.0;
const GLONASS_INTEGRATION_STEP_S: f64 = 60.0;
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
    pub ephemeris_reference_time_s: f64,
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

pub fn glonass_navigation_age(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<GlonassNavigationAge> {
    let transmit_glonass_time_s = glonass_time_of_day_from_gps_tow_s(
        transmit_gps_tow_s,
        glonass_gps_minus_glonass_s(navigation)?,
    );
    let ephemeris_reference_time_s = f64::from(navigation.immediate.ephemeris_reference_time_s);
    Some(GlonassNavigationAge {
        transmit_gps_tow_s,
        transmit_glonass_time_s,
        ephemeris_reference_time_s,
        age_s: wrap_glonass_time_delta_s(transmit_glonass_time_s - ephemeris_reference_time_s)
            .abs(),
        max_age_s: GLONASS_MAX_NAVIGATION_AGE_S,
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
    let rotation_rad = GLONASS_OMEGA_E_DOT * signal_travel_time_s;
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
    let delta_t_s = glonass_broadcast_delta_time_s(navigation, transmit_gps_tow_s)?;
    let propagated_state =
        propagate_glonass_state_vector(navigation.immediate.state_vector, delta_t_s);
    let clock_correction = glonass_satellite_clock_correction(navigation, transmit_gps_tow_s)?;

    let mut x_m = propagated_state[0] + PZ90_02_TO_ITRF2000_X_M;
    let mut y_m = propagated_state[1] + PZ90_02_TO_ITRF2000_Y_M;
    let z_m = propagated_state[2] + PZ90_02_TO_ITRF2000_Z_M;

    let earth_rotation = glonass_earth_rotation_correction(x_m, y_m, signal_travel_time_s);
    x_m += earth_rotation.delta_x_m;
    y_m += earth_rotation.delta_y_m;

    Some(GlonassSatState { x_m, y_m, z_m, clock_correction })
}

fn glonass_broadcast_delta_time_s(
    navigation: &GlonassBroadcastNavigationFrame,
    transmit_gps_tow_s: f64,
) -> Option<f64> {
    let age = glonass_navigation_age(navigation, transmit_gps_tow_s)?;
    if age.is_stale() {
        return None;
    }
    Some(wrap_glonass_time_delta_s(age.transmit_glonass_time_s - age.ephemeris_reference_time_s))
}

fn glonass_time_of_day_from_gps_tow_s(transmit_gps_tow_s: f64, gps_minus_glonass_s: f64) -> f64 {
    (transmit_gps_tow_s - gps_minus_glonass_s).rem_euclid(GLONASS_DAY_S)
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

fn propagate_glonass_state_vector(state_vector: GlonassStateVector, delta_t_s: f64) -> [f64; 6] {
    let mut state = glonass_inertial_initial_state(state_vector);
    if delta_t_s.abs() <= f64::EPSILON {
        return glonass_ecef_state_from_inertial(state, delta_t_s);
    }

    let luni_solar_acceleration_mps2 =
        [state_vector.ax_mps2, state_vector.ay_mps2, state_vector.az_mps2];
    let mut remaining_s = delta_t_s;
    while remaining_s.abs() > f64::EPSILON {
        let step_s = remaining_s.signum() * remaining_s.abs().min(GLONASS_INTEGRATION_STEP_S);
        state = rk4_step(state, step_s, luni_solar_acceleration_mps2, glonass_inertial_dynamics);
        remaining_s -= step_s;
    }
    glonass_ecef_state_from_inertial(state, delta_t_s)
}

fn rk4_step(
    state: [f64; 6],
    step_s: f64,
    luni_solar_acceleration_mps2: [f64; 3],
    dynamics: fn([f64; 6], [f64; 3]) -> [f64; 6],
) -> [f64; 6] {
    let k1 = dynamics(state, luni_solar_acceleration_mps2);
    let k2 = dynamics(add_scaled_state(state, k1, step_s * 0.5), luni_solar_acceleration_mps2);
    let k3 = dynamics(add_scaled_state(state, k2, step_s * 0.5), luni_solar_acceleration_mps2);
    let k4 = dynamics(add_scaled_state(state, k3, step_s), luni_solar_acceleration_mps2);

    let mut next_state = state;
    for index in 0..next_state.len() {
        next_state[index] +=
            step_s / 6.0 * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index]);
    }
    next_state
}

fn add_scaled_state(state: [f64; 6], derivative: [f64; 6], scale: f64) -> [f64; 6] {
    let mut next = state;
    for index in 0..next.len() {
        next[index] += derivative[index] * scale;
    }
    next
}

fn glonass_inertial_initial_state(state_vector: GlonassStateVector) -> [f64; 6] {
    [
        state_vector.x_m,
        state_vector.y_m,
        state_vector.z_m,
        state_vector.vx_mps - GLONASS_OMEGA_E_DOT * state_vector.y_m,
        state_vector.vy_mps + GLONASS_OMEGA_E_DOT * state_vector.x_m,
        state_vector.vz_mps,
    ]
}

fn glonass_ecef_state_from_inertial(inertial_state: [f64; 6], delta_t_s: f64) -> [f64; 6] {
    let rotation_rad = GLONASS_OMEGA_E_DOT * delta_t_s;
    let cos_rotation = rotation_rad.cos();
    let sin_rotation = rotation_rad.sin();
    [
        inertial_state[0] * cos_rotation + inertial_state[1] * sin_rotation,
        -inertial_state[0] * sin_rotation + inertial_state[1] * cos_rotation,
        inertial_state[2],
        0.0,
        0.0,
        0.0,
    ]
}

fn glonass_inertial_dynamics(state: [f64; 6], luni_solar_acceleration_mps2: [f64; 3]) -> [f64; 6] {
    let x_m = state[0];
    let y_m = state[1];
    let z_m = state[2];
    let vx_mps = state[3];
    let vy_mps = state[4];
    let vz_mps = state[5];

    let radius_squared_m2 = x_m * x_m + y_m * y_m + z_m * z_m;
    let radius_m = radius_squared_m2.sqrt();
    let radius_cubed_m3 = radius_squared_m2 * radius_m;
    let radius_fifth_m5 = radius_cubed_m3 * radius_squared_m2;
    let z_ratio_squared = if radius_squared_m2 > 0.0 { z_m * z_m / radius_squared_m2 } else { 0.0 };
    let central_acceleration = -GLONASS_MU_M3PS2 / radius_cubed_m3;
    let j2_acceleration =
        1.5 * GLONASS_C20 * GLONASS_MU_M3PS2 * GLONASS_EARTH_RADIUS_M.powi(2) / radius_fifth_m5;

    let ax_mps2 = central_acceleration * x_m
        + j2_acceleration * x_m * (1.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[0];
    let ay_mps2 = central_acceleration * y_m
        + j2_acceleration * y_m * (1.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[1];
    let az_mps2 = central_acceleration * z_m
        + j2_acceleration * z_m * (3.0 - 5.0 * z_ratio_squared)
        + luni_solar_acceleration_mps2[2];

    [vx_mps, vy_mps, vz_mps, ax_mps2, ay_mps2, az_mps2]
}

#[cfg(test)]
mod tests {
    use super::{
        glonass_navigation_age, glonass_satellite_clock_correction, sat_state_glonass_l1,
        GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame, GlonassFrameTime,
        GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassSatelliteType,
        GlonassStateVector, GlonassSystemTime, PZ90_02_TO_ITRF2000_X_M, PZ90_02_TO_ITRF2000_Y_M,
        PZ90_02_TO_ITRF2000_Z_M,
    };
    use bijux_gnss_core::api::{Constellation, SatId};

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
        assert_eq!(age.ephemeris_reference_time_s, 83_700.0);
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
    fn glonass_state_rejects_stale_navigation() {
        let navigation = sample_navigation();

        let state = sat_state_glonass_l1(&navigation, 506_000.0, 0.0);

        assert!(state.is_none());
    }
}
