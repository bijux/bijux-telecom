#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{gps_to_utc, Constellation, GpsTime, LeapSeconds, SatId, SignalBand};
use serde::{Deserialize, Serialize};

use crate::models::celestial::approximate_sun_position_ecef_m;

const WGS84_SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
const WGS84_FIRST_ECCENTRICITY_SQUARED: f64 = 6.694_379_990_14e-3;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SatellitePhaseCenterOffset {
    pub body_x_m: f64,
    pub body_y_m: f64,
    pub body_z_m: f64,
}

impl SatellitePhaseCenterOffset {
    pub fn new(body_x_m: f64, body_y_m: f64, body_z_m: f64) -> Self {
        Self { body_x_m, body_y_m, body_z_m }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SatelliteAntennaFrame {
    pub x_axis_ecef: [f64; 3],
    pub y_axis_ecef: [f64; 3],
    pub z_axis_ecef: [f64; 3],
}

impl SatelliteAntennaFrame {
    pub fn phase_center_offset_ecef_m(self, offset: SatellitePhaseCenterOffset) -> [f64; 3] {
        [
            self.x_axis_ecef[0] * offset.body_x_m
                + self.y_axis_ecef[0] * offset.body_y_m
                + self.z_axis_ecef[0] * offset.body_z_m,
            self.x_axis_ecef[1] * offset.body_x_m
                + self.y_axis_ecef[1] * offset.body_y_m
                + self.z_axis_ecef[1] * offset.body_z_m,
            self.x_axis_ecef[2] * offset.body_x_m
                + self.y_axis_ecef[2] * offset.body_y_m
                + self.z_axis_ecef[2] * offset.body_z_m,
        ]
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AntennaAzimuthPhaseCenterVariation {
    pub azimuth_deg: f64,
    pub values_m: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AntennaPhaseCenterVariation {
    pub zenith_start_deg: f64,
    pub zenith_step_deg: f64,
    pub no_azimuth_values_m: Vec<f64>,
    pub azimuth_values_m: Vec<AntennaAzimuthPhaseCenterVariation>,
}

impl AntennaPhaseCenterVariation {
    pub fn no_azimuth(
        zenith_start_deg: f64,
        zenith_step_deg: f64,
        no_azimuth_values_m: Vec<f64>,
    ) -> Self {
        Self {
            zenith_start_deg,
            zenith_step_deg,
            no_azimuth_values_m,
            azimuth_values_m: Vec::new(),
        }
    }

    pub fn phase_variation_m(&self, elevation_deg: f64, azimuth_deg: Option<f64>) -> Option<f64> {
        if !elevation_deg.is_finite() {
            return None;
        }
        let zenith_deg = 90.0 - elevation_deg;
        if let Some(azimuth_deg) = azimuth_deg {
            if let Some(value) = self.azimuth_phase_variation_m(zenith_deg, azimuth_deg) {
                return Some(value);
            }
        }
        interpolate_zenith_values_m(
            self.zenith_start_deg,
            self.zenith_step_deg,
            &self.no_azimuth_values_m,
            zenith_deg,
        )
    }

    fn azimuth_phase_variation_m(&self, zenith_deg: f64, azimuth_deg: f64) -> Option<f64> {
        if self.azimuth_values_m.is_empty() || !azimuth_deg.is_finite() {
            return None;
        }
        let azimuth_deg = wrap_azimuth_deg(azimuth_deg);
        let rows = self
            .azimuth_values_m
            .iter()
            .filter_map(|row| {
                interpolate_zenith_values_m(
                    self.zenith_start_deg,
                    self.zenith_step_deg,
                    &row.values_m,
                    zenith_deg,
                )
                .map(|value| (wrap_azimuth_deg(row.azimuth_deg), value))
            })
            .collect::<Vec<_>>();
        if rows.is_empty() {
            return None;
        }
        if rows.len() == 1 {
            return Some(rows[0].1);
        }
        let mut rows = rows;
        rows.sort_by(|left, right| left.0.total_cmp(&right.0));
        for pair in rows.windows(2) {
            let left = pair[0];
            let right = pair[1];
            if azimuth_deg >= left.0 && azimuth_deg <= right.0 {
                return Some(linear_interpolate(left.0, left.1, right.0, right.1, azimuth_deg));
            }
        }
        let last = *rows.last()?;
        let first = rows[0];
        let wrapped_right = first.0 + 360.0;
        let wrapped_azimuth = if azimuth_deg < first.0 { azimuth_deg + 360.0 } else { azimuth_deg };
        Some(linear_interpolate(last.0, last.1, wrapped_right, first.1, wrapped_azimuth))
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SatelliteAntennaCalibration {
    pub sat: SatId,
    pub antenna_type: String,
    pub valid_from_unix_s: Option<f64>,
    pub valid_until_unix_s: Option<f64>,
    pub offsets_by_band: BTreeMap<SignalBand, SatellitePhaseCenterOffset>,
    pub variations_by_band: BTreeMap<SignalBand, AntennaPhaseCenterVariation>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SatelliteAntennaCalibrations {
    pub entries: Vec<SatelliteAntennaCalibration>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ReceiverPhaseCenterOffset {
    pub north_m: f64,
    pub east_m: f64,
    pub up_m: f64,
}

impl ReceiverPhaseCenterOffset {
    pub fn new(north_m: f64, east_m: f64, up_m: f64) -> Self {
        Self { north_m, east_m, up_m }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ReceiverAntennaFrame {
    pub north_axis_ecef: [f64; 3],
    pub east_axis_ecef: [f64; 3],
    pub up_axis_ecef: [f64; 3],
}

impl ReceiverAntennaFrame {
    pub fn phase_center_offset_ecef_m(self, offset: ReceiverPhaseCenterOffset) -> [f64; 3] {
        [
            self.north_axis_ecef[0] * offset.north_m
                + self.east_axis_ecef[0] * offset.east_m
                + self.up_axis_ecef[0] * offset.up_m,
            self.north_axis_ecef[1] * offset.north_m
                + self.east_axis_ecef[1] * offset.east_m
                + self.up_axis_ecef[1] * offset.up_m,
            self.north_axis_ecef[2] * offset.north_m
                + self.east_axis_ecef[2] * offset.east_m
                + self.up_axis_ecef[2] * offset.up_m,
        ]
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReceiverAntennaCalibration {
    pub antenna_type: String,
    pub valid_from_unix_s: Option<f64>,
    pub valid_until_unix_s: Option<f64>,
    pub offsets_by_band: BTreeMap<SignalBand, ReceiverPhaseCenterOffset>,
    pub variations_by_band: BTreeMap<SignalBand, AntennaPhaseCenterVariation>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ReceiverAntennaCalibrations {
    pub entries: Vec<ReceiverAntennaCalibration>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AntennaRangeGeometry {
    pub gps_time: Option<GpsTime>,
    pub receiver_pos_m: [f64; 3],
    pub sat_pos_m: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AntennaPhaseGeometry {
    pub range: AntennaRangeGeometry,
    pub elevation_deg: f64,
    pub azimuth_deg: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DualFrequencySignal {
    pub primary_band: SignalBand,
    pub primary_frequency_hz: f64,
    pub secondary_band: SignalBand,
    pub secondary_frequency_hz: f64,
}

impl SatelliteAntennaCalibrations {
    pub fn phase_center_offset(
        &self,
        sat: SatId,
        band: SignalBand,
        gps_time: Option<GpsTime>,
    ) -> Option<SatellitePhaseCenterOffset> {
        let unix_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
        self.entries
            .iter()
            .filter(|entry| entry.sat == sat)
            .filter(|entry| {
                calibration_window_matches_time(
                    entry.valid_from_unix_s,
                    entry.valid_until_unix_s,
                    unix_s,
                )
            })
            .filter_map(|entry| {
                entry
                    .offsets_by_band
                    .get(&band)
                    .copied()
                    .map(|offset| (entry.valid_from_unix_s.unwrap_or(f64::NEG_INFINITY), offset))
            })
            .max_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, offset)| offset)
    }

    pub fn phase_center_variation_m(
        &self,
        sat: SatId,
        band: SignalBand,
        gps_time: Option<GpsTime>,
        elevation_deg: f64,
        azimuth_deg: Option<f64>,
    ) -> Option<f64> {
        let unix_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
        self.entries
            .iter()
            .filter(|entry| entry.sat == sat)
            .filter(|entry| {
                calibration_window_matches_time(
                    entry.valid_from_unix_s,
                    entry.valid_until_unix_s,
                    unix_s,
                )
            })
            .filter_map(|entry| {
                entry.variations_by_band.get(&band).and_then(|variation| {
                    variation
                        .phase_variation_m(elevation_deg, azimuth_deg)
                        .map(|value| (entry.valid_from_unix_s.unwrap_or(f64::NEG_INFINITY), value))
                })
            })
            .max_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, value)| value)
    }

    pub fn range_correction_m(
        &self,
        sat: SatId,
        band: SignalBand,
        geometry: AntennaRangeGeometry,
    ) -> Option<f64> {
        let offset = self.phase_center_offset(sat, band, geometry.gps_time)?;
        let gps_time = geometry.gps_time?;
        let sun_pos_m = approximate_sun_position_ecef_m(gps_time);
        Some(satellite_antenna_range_correction_m(
            geometry.sat_pos_m,
            geometry.receiver_pos_m,
            sun_pos_m,
            offset,
        ))
    }

    pub fn range_correction_with_phase_variation_m(
        &self,
        sat: SatId,
        band: SignalBand,
        geometry: AntennaPhaseGeometry,
    ) -> Option<f64> {
        let offset_correction_m = self.range_correction_m(sat, band, geometry.range)?;
        let phase_variation_m = self
            .phase_center_variation_m(
                sat,
                band,
                geometry.range.gps_time,
                geometry.elevation_deg,
                geometry.azimuth_deg,
            )
            .unwrap_or(0.0);
        Some(offset_correction_m + phase_variation_m)
    }

    pub fn iono_free_range_correction_m(
        &self,
        sat: SatId,
        signal: DualFrequencySignal,
        geometry: AntennaRangeGeometry,
    ) -> Option<f64> {
        let correction_1 = self.range_correction_m(sat, signal.primary_band, geometry)?;
        let correction_2 = self.range_correction_m(sat, signal.secondary_band, geometry)?;
        iono_free_linear_combination_m(
            correction_1,
            signal.primary_frequency_hz,
            correction_2,
            signal.secondary_frequency_hz,
        )
    }

    pub fn iono_free_range_correction_with_phase_variation_m(
        &self,
        sat: SatId,
        signal: DualFrequencySignal,
        geometry: AntennaPhaseGeometry,
    ) -> Option<f64> {
        let correction_1 =
            self.range_correction_with_phase_variation_m(sat, signal.primary_band, geometry)?;
        let correction_2 =
            self.range_correction_with_phase_variation_m(sat, signal.secondary_band, geometry)?;
        iono_free_linear_combination_m(
            correction_1,
            signal.primary_frequency_hz,
            correction_2,
            signal.secondary_frequency_hz,
        )
    }
}

impl ReceiverAntennaCalibrations {
    pub fn phase_center_offset(
        &self,
        antenna_type: &str,
        band: SignalBand,
        gps_time: Option<GpsTime>,
    ) -> Option<ReceiverPhaseCenterOffset> {
        let unix_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
        let canonical_antenna_type = canonical_receiver_antenna_type(antenna_type);
        self.entries
            .iter()
            .filter(|entry| {
                canonical_receiver_antenna_type(&entry.antenna_type) == canonical_antenna_type
            })
            .filter(|entry| {
                calibration_window_matches_time(
                    entry.valid_from_unix_s,
                    entry.valid_until_unix_s,
                    unix_s,
                )
            })
            .filter_map(|entry| {
                entry
                    .offsets_by_band
                    .get(&band)
                    .copied()
                    .map(|offset| (entry.valid_from_unix_s.unwrap_or(f64::NEG_INFINITY), offset))
            })
            .max_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, offset)| offset)
    }

    pub fn phase_center_variation_m(
        &self,
        antenna_type: &str,
        band: SignalBand,
        gps_time: Option<GpsTime>,
        elevation_deg: f64,
        azimuth_deg: Option<f64>,
    ) -> Option<f64> {
        let unix_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
        let canonical_antenna_type = canonical_receiver_antenna_type(antenna_type);
        self.entries
            .iter()
            .filter(|entry| {
                canonical_receiver_antenna_type(&entry.antenna_type) == canonical_antenna_type
            })
            .filter(|entry| {
                calibration_window_matches_time(
                    entry.valid_from_unix_s,
                    entry.valid_until_unix_s,
                    unix_s,
                )
            })
            .filter_map(|entry| {
                entry.variations_by_band.get(&band).and_then(|variation| {
                    variation
                        .phase_variation_m(elevation_deg, azimuth_deg)
                        .map(|value| (entry.valid_from_unix_s.unwrap_or(f64::NEG_INFINITY), value))
                })
            })
            .max_by(|left, right| left.0.total_cmp(&right.0))
            .map(|(_, value)| value)
    }

    pub fn range_correction_m(
        &self,
        antenna_type: &str,
        band: SignalBand,
        geometry: AntennaRangeGeometry,
    ) -> Option<f64> {
        let offset = self.phase_center_offset(antenna_type, band, geometry.gps_time)?;
        Some(receiver_antenna_range_correction_m(
            geometry.receiver_pos_m,
            geometry.sat_pos_m,
            offset,
        ))
    }

    pub fn range_correction_with_phase_variation_m(
        &self,
        antenna_type: &str,
        band: SignalBand,
        geometry: AntennaPhaseGeometry,
    ) -> Option<f64> {
        let offset_correction_m = self.range_correction_m(antenna_type, band, geometry.range)?;
        let phase_variation_m = self
            .phase_center_variation_m(
                antenna_type,
                band,
                geometry.range.gps_time,
                geometry.elevation_deg,
                geometry.azimuth_deg,
            )
            .unwrap_or(0.0);
        Some(offset_correction_m + phase_variation_m)
    }

    pub fn iono_free_range_correction_m(
        &self,
        antenna_type: &str,
        signal: DualFrequencySignal,
        geometry: AntennaRangeGeometry,
    ) -> Option<f64> {
        let correction_1 = self.range_correction_m(antenna_type, signal.primary_band, geometry)?;
        let correction_2 =
            self.range_correction_m(antenna_type, signal.secondary_band, geometry)?;
        iono_free_linear_combination_m(
            correction_1,
            signal.primary_frequency_hz,
            correction_2,
            signal.secondary_frequency_hz,
        )
    }

    pub fn iono_free_range_correction_with_phase_variation_m(
        &self,
        antenna_type: &str,
        signal: DualFrequencySignal,
        geometry: AntennaPhaseGeometry,
    ) -> Option<f64> {
        let correction_1 = self.range_correction_with_phase_variation_m(
            antenna_type,
            signal.primary_band,
            geometry,
        )?;
        let correction_2 = self.range_correction_with_phase_variation_m(
            antenna_type,
            signal.secondary_band,
            geometry,
        )?;
        iono_free_linear_combination_m(
            correction_1,
            signal.primary_frequency_hz,
            correction_2,
            signal.secondary_frequency_hz,
        )
    }
}

fn iono_free_linear_combination_m(
    correction_1_m: f64,
    f1_hz: f64,
    correction_2_m: f64,
    f2_hz: f64,
) -> Option<f64> {
    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return None;
    }
    let weight_1 = f1_2 / denom;
    let weight_2 = -f2_2 / denom;
    Some(weight_1 * correction_1_m + weight_2 * correction_2_m)
}

pub fn satellite_antenna_range_correction_m(
    sat_pos_m: [f64; 3],
    receiver_pos_m: [f64; 3],
    sun_pos_m: [f64; 3],
    offset: SatellitePhaseCenterOffset,
) -> f64 {
    let Some(body_frame) = satellite_antenna_frame_ecef(sat_pos_m, sun_pos_m) else {
        return 0.0;
    };
    let phase_center_offset_ecef_m = body_frame.phase_center_offset_ecef_m(offset);
    let corrected_sat_pos_m = [
        sat_pos_m[0] + phase_center_offset_ecef_m[0],
        sat_pos_m[1] + phase_center_offset_ecef_m[1],
        sat_pos_m[2] + phase_center_offset_ecef_m[2],
    ];
    norm3([
        receiver_pos_m[0] - corrected_sat_pos_m[0],
        receiver_pos_m[1] - corrected_sat_pos_m[1],
        receiver_pos_m[2] - corrected_sat_pos_m[2],
    ]) - norm3([
        receiver_pos_m[0] - sat_pos_m[0],
        receiver_pos_m[1] - sat_pos_m[1],
        receiver_pos_m[2] - sat_pos_m[2],
    ])
}

pub fn receiver_antenna_range_correction_m(
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    offset: ReceiverPhaseCenterOffset,
) -> f64 {
    let Some(local_frame) = receiver_antenna_frame_ecef(receiver_pos_m) else {
        return 0.0;
    };
    let phase_center_offset_ecef_m = local_frame.phase_center_offset_ecef_m(offset);
    let corrected_receiver_pos_m = [
        receiver_pos_m[0] + phase_center_offset_ecef_m[0],
        receiver_pos_m[1] + phase_center_offset_ecef_m[1],
        receiver_pos_m[2] + phase_center_offset_ecef_m[2],
    ];
    norm3([
        corrected_receiver_pos_m[0] - sat_pos_m[0],
        corrected_receiver_pos_m[1] - sat_pos_m[1],
        corrected_receiver_pos_m[2] - sat_pos_m[2],
    ]) - norm3([
        receiver_pos_m[0] - sat_pos_m[0],
        receiver_pos_m[1] - sat_pos_m[1],
        receiver_pos_m[2] - sat_pos_m[2],
    ])
}

pub fn canonical_receiver_antenna_type(value: &str) -> String {
    value.split_whitespace().collect::<Vec<_>>().join(" ").to_ascii_uppercase()
}

pub fn satellite_band_from_antex_frequency(
    constellation: Constellation,
    frequency_code: &str,
) -> Option<SignalBand> {
    match (constellation, frequency_code.trim()) {
        (Constellation::Gps, "G01") => Some(SignalBand::L1),
        (Constellation::Gps, "G02") => Some(SignalBand::L2),
        (Constellation::Gps, "G05") => Some(SignalBand::L5),
        (Constellation::Galileo, "E01") => Some(SignalBand::E1),
        (Constellation::Galileo, "E05")
        | (Constellation::Galileo, "E07")
        | (Constellation::Galileo, "E08") => Some(SignalBand::E5),
        (Constellation::Beidou, "C02") => Some(SignalBand::B1),
        (Constellation::Beidou, "C07") => Some(SignalBand::B2),
        _ => None,
    }
}

pub fn satellite_antenna_frame_ecef(
    sat_pos_m: [f64; 3],
    sun_pos_m: [f64; 3],
) -> Option<SatelliteAntennaFrame> {
    let z_axis_ecef = try_normalize3([-sat_pos_m[0], -sat_pos_m[1], -sat_pos_m[2]])?;
    let sat_to_sun = try_normalize3([
        sun_pos_m[0] - sat_pos_m[0],
        sun_pos_m[1] - sat_pos_m[1],
        sun_pos_m[2] - sat_pos_m[2],
    ])?;
    let y_axis_ecef = try_normalize3(cross3(z_axis_ecef, sat_to_sun))?;
    let x_axis_ecef = try_normalize3(cross3(y_axis_ecef, z_axis_ecef))?;
    Some(SatelliteAntennaFrame { x_axis_ecef, y_axis_ecef, z_axis_ecef })
}

pub fn receiver_antenna_frame_ecef(receiver_pos_m: [f64; 3]) -> Option<ReceiverAntennaFrame> {
    if !receiver_pos_m.iter().all(|value| value.is_finite())
        || norm3(receiver_pos_m) <= f64::EPSILON
    {
        return None;
    }
    let (lat_rad, lon_rad) = receiver_lat_lon_rad(receiver_pos_m);
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    let east_axis_ecef = [-sin_lon, cos_lon, 0.0];
    let north_axis_ecef = [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat];
    let up_axis_ecef = [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat];
    Some(ReceiverAntennaFrame { north_axis_ecef, east_axis_ecef, up_axis_ecef })
}

fn receiver_lat_lon_rad(receiver_pos_m: [f64; 3]) -> (f64, f64) {
    let x = receiver_pos_m[0];
    let y = receiver_pos_m[1];
    let z = receiver_pos_m[2];
    let lon_rad = y.atan2(x);
    let p = (x * x + y * y).sqrt().max(f64::EPSILON);
    let mut lat_rad = z.atan2(p * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED));
    for _ in 0..5 {
        let sin_lat = lat_rad.sin();
        let prime_vertical_radius_m = WGS84_SEMI_MAJOR_AXIS_M
            / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat).sqrt();
        lat_rad =
            (z + WGS84_FIRST_ECCENTRICITY_SQUARED * prime_vertical_radius_m * sin_lat).atan2(p);
    }
    (lat_rad, lon_rad)
}

fn calibration_window_matches_time(
    valid_from_unix_s: Option<f64>,
    valid_until_unix_s: Option<f64>,
    unix_s: Option<f64>,
) -> bool {
    let Some(unix_s) = unix_s else {
        return true;
    };
    if let Some(valid_from_unix_s) = valid_from_unix_s {
        if unix_s < valid_from_unix_s {
            return false;
        }
    }
    if let Some(valid_until_unix_s) = valid_until_unix_s {
        if unix_s > valid_until_unix_s {
            return false;
        }
    }
    true
}

fn interpolate_zenith_values_m(
    zenith_start_deg: f64,
    zenith_step_deg: f64,
    values_m: &[f64],
    zenith_deg: f64,
) -> Option<f64> {
    if values_m.is_empty()
        || !zenith_start_deg.is_finite()
        || !zenith_step_deg.is_finite()
        || zenith_step_deg <= 0.0
        || !zenith_deg.is_finite()
    {
        return None;
    }
    if values_m.len() == 1 {
        return Some(values_m[0]);
    }
    let max_index = values_m.len() - 1;
    let position = ((zenith_deg - zenith_start_deg) / zenith_step_deg).clamp(0.0, max_index as f64);
    let lower_index = position.floor() as usize;
    let upper_index = position.ceil() as usize;
    if lower_index == upper_index {
        return values_m.get(lower_index).copied();
    }
    let lower_zenith_deg = zenith_start_deg + lower_index as f64 * zenith_step_deg;
    let upper_zenith_deg = zenith_start_deg + upper_index as f64 * zenith_step_deg;
    Some(linear_interpolate(
        lower_zenith_deg,
        values_m[lower_index],
        upper_zenith_deg,
        values_m[upper_index],
        zenith_deg.clamp(lower_zenith_deg, upper_zenith_deg),
    ))
}

fn linear_interpolate(x0: f64, y0: f64, x1: f64, y1: f64, x: f64) -> f64 {
    let span = x1 - x0;
    if !span.is_finite() || span.abs() <= f64::EPSILON {
        return y0;
    }
    let fraction = ((x - x0) / span).clamp(0.0, 1.0);
    y0 + fraction * (y1 - y0)
}

fn wrap_azimuth_deg(azimuth_deg: f64) -> f64 {
    azimuth_deg.rem_euclid(360.0)
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn norm3(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn try_normalize3(vector: [f64; 3]) -> Option<[f64; 3]> {
    if !vector.iter().all(|value| value.is_finite()) {
        return None;
    }
    let norm = norm3(vector);
    if !norm.is_finite() || norm <= f64::EPSILON {
        return None;
    }
    Some([vector[0] / norm, vector[1] / norm, vector[2] / norm])
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};

    use super::{
        canonical_receiver_antenna_type, receiver_antenna_frame_ecef,
        receiver_antenna_range_correction_m, satellite_antenna_frame_ecef,
        satellite_antenna_range_correction_m, satellite_band_from_antex_frequency,
        AntennaAzimuthPhaseCenterVariation, AntennaPhaseCenterVariation,
        ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverAntennaFrame,
        ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
        SatelliteAntennaFrame, SatellitePhaseCenterOffset,
    };

    #[test]
    fn satellite_antenna_frame_uses_nadir_and_sun_oriented_axes() {
        let sat_pos_m = [20_200_000.0, 0.0, 0.0];
        let sun_pos_m = [0.0, 149_597_870_700.0, 0.0];

        let frame = satellite_antenna_frame_ecef(sat_pos_m, sun_pos_m).expect("satellite frame");

        assert_eq!(
            frame,
            SatelliteAntennaFrame {
                x_axis_ecef: [0.0, 1.0, 0.0],
                y_axis_ecef: [0.0, 0.0, -1.0],
                z_axis_ecef: [-1.0, 0.0, 0.0],
            }
        );
    }

    #[test]
    fn satellite_phase_center_range_correction_projects_body_axes_into_ecef() {
        let sat_pos_m = [20_200_000.0, 0.0, 0.0];
        let receiver_pos_m = [0.0, 0.0, 0.0];
        let sun_pos_m = [0.0, 149_597_870_700.0, 0.0];
        let offset = SatellitePhaseCenterOffset::new(0.25, 0.0, 1.0);

        let correction_m =
            satellite_antenna_range_correction_m(sat_pos_m, receiver_pos_m, sun_pos_m, offset);

        let corrected_sat_pos_m = [20_199_999.0, 0.25, 0.0];
        let expected = ((receiver_pos_m[0] - corrected_sat_pos_m[0]).powi(2)
            + (receiver_pos_m[1] - corrected_sat_pos_m[1]).powi(2)
            + (receiver_pos_m[2] - corrected_sat_pos_m[2]).powi(2))
        .sqrt()
            - 20_200_000.0;

        assert!((correction_m - expected).abs() < 1.0e-9);
        assert!(correction_m < 0.0);
    }

    #[test]
    fn receiver_antenna_frame_uses_north_east_up_axes() {
        let receiver_pos_m = [6_378_137.0, 0.0, 0.0];

        let frame = receiver_antenna_frame_ecef(receiver_pos_m).expect("receiver frame");

        assert_eq!(
            frame,
            ReceiverAntennaFrame {
                north_axis_ecef: [0.0, 0.0, 1.0],
                east_axis_ecef: [0.0, 1.0, 0.0],
                up_axis_ecef: [1.0, 0.0, 0.0],
            }
        );
    }

    #[test]
    fn satellite_antex_frequency_map_covers_supported_bands() {
        assert_eq!(
            satellite_band_from_antex_frequency(Constellation::Gps, "G01"),
            Some(SignalBand::L1)
        );
        assert_eq!(
            satellite_band_from_antex_frequency(Constellation::Galileo, "E05"),
            Some(SignalBand::E5)
        );
        assert_eq!(
            satellite_band_from_antex_frequency(Constellation::Beidou, "C07"),
            Some(SignalBand::B2)
        );
    }

    #[test]
    fn satellite_antenna_calibrations_select_valid_entry_by_time() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let mut offsets_by_band = BTreeMap::new();
        offsets_by_band.insert(SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.1));
        let old = SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS-A".to_string(),
            valid_from_unix_s: Some(1_577_836_800.0),
            valid_until_unix_s: Some(1_609_459_199.0),
            offsets_by_band: offsets_by_band.clone(),
            variations_by_band: BTreeMap::new(),
        };
        let mut offsets_by_band_new = BTreeMap::new();
        offsets_by_band_new.insert(SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.3));
        let new = SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS-B".to_string(),
            valid_from_unix_s: Some(1_609_459_200.0),
            valid_until_unix_s: None,
            offsets_by_band: offsets_by_band_new,
            variations_by_band: BTreeMap::new(),
        };
        let calibrations = SatelliteAntennaCalibrations { entries: vec![old, new] };

        let selected = calibrations
            .phase_center_offset(sat, SignalBand::L1, Some(GpsTime { week: 2200, tow_s: 0.0 }))
            .expect("valid L1 offset");

        assert_eq!(selected, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.3));
    }

    #[test]
    fn receiver_phase_center_range_correction_projects_local_neu_axes_into_ecef() {
        let receiver_pos_m = [6_378_137.0, 0.0, 0.0];
        let sat_pos_m = [20_200_000.0, 10_000.0, 5_000.0];
        let offset = ReceiverPhaseCenterOffset::new(0.2, 0.5, 1.0);

        let correction_m = receiver_antenna_range_correction_m(receiver_pos_m, sat_pos_m, offset);

        let corrected_receiver_pos_m = [6_378_138.0, 0.5, 0.2];
        let expected = ((corrected_receiver_pos_m[0] - sat_pos_m[0]).powi(2)
            + (corrected_receiver_pos_m[1] - sat_pos_m[1]).powi(2)
            + (corrected_receiver_pos_m[2] - sat_pos_m[2]).powi(2))
        .sqrt()
            - ((receiver_pos_m[0] - sat_pos_m[0]).powi(2)
                + (receiver_pos_m[1] - sat_pos_m[1]).powi(2)
                + (receiver_pos_m[2] - sat_pos_m[2]).powi(2))
            .sqrt();

        assert!((correction_m - expected).abs() < 1.0e-9);
    }

    #[test]
    fn receiver_antenna_calibrations_select_type_and_valid_entry() {
        let mut offsets_by_band = BTreeMap::new();
        offsets_by_band.insert(SignalBand::L1, ReceiverPhaseCenterOffset::new(0.1, 0.2, 0.3));
        let old = ReceiverAntennaCalibration {
            antenna_type: "AOAD/M_T NONE".to_string(),
            valid_from_unix_s: Some(1_577_836_800.0),
            valid_until_unix_s: Some(1_609_459_199.0),
            offsets_by_band: offsets_by_band.clone(),
            variations_by_band: BTreeMap::new(),
        };
        let mut offsets_by_band_new = BTreeMap::new();
        offsets_by_band_new.insert(SignalBand::L1, ReceiverPhaseCenterOffset::new(0.4, 0.5, 0.6));
        let new = ReceiverAntennaCalibration {
            antenna_type: "AOAD/M_T NONE".to_string(),
            valid_from_unix_s: Some(1_609_459_200.0),
            valid_until_unix_s: None,
            offsets_by_band: offsets_by_band_new,
            variations_by_band: BTreeMap::new(),
        };
        let ignored = ReceiverAntennaCalibration {
            antenna_type: "TRM57971.00 NONE".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(1.0, 1.0, 1.0),
            )]),
            variations_by_band: BTreeMap::new(),
        };
        let calibrations = ReceiverAntennaCalibrations { entries: vec![old, new, ignored] };

        let selected = calibrations
            .phase_center_offset(
                "  aoad/m_t   none ",
                SignalBand::L1,
                Some(GpsTime { week: 2200, tow_s: 0.0 }),
            )
            .expect("valid receiver L1 offset");

        assert_eq!(selected, ReceiverPhaseCenterOffset::new(0.4, 0.5, 0.6));
        assert_eq!(canonical_receiver_antenna_type("  aoad/m_t   none "), "AOAD/M_T NONE");
    }

    #[test]
    fn antenna_phase_center_variation_interpolates_zenith_and_azimuth() {
        let variation = AntennaPhaseCenterVariation {
            zenith_start_deg: 0.0,
            zenith_step_deg: 10.0,
            no_azimuth_values_m: vec![0.0, 0.010, 0.020],
            azimuth_values_m: vec![
                AntennaAzimuthPhaseCenterVariation {
                    azimuth_deg: 0.0,
                    values_m: vec![0.0, 0.020, 0.040],
                },
                AntennaAzimuthPhaseCenterVariation {
                    azimuth_deg: 90.0,
                    values_m: vec![0.010, 0.030, 0.050],
                },
            ],
        };

        let no_azimuth =
            variation.phase_variation_m(75.0, None).expect("zenith-only interpolation");
        let azimuth_dependent =
            variation.phase_variation_m(75.0, Some(45.0)).expect("azimuth-dependent interpolation");

        assert!((no_azimuth - 0.015).abs() < 1.0e-12);
        assert!((azimuth_dependent - 0.035).abs() < 1.0e-12);
    }

    #[test]
    fn satellite_and_receiver_calibrations_query_variation_by_time_signal_and_type() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let old_variation = AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.01]);
        let new_variation = AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.03]);
        let satellite_calibrations = SatelliteAntennaCalibrations {
            entries: vec![
                SatelliteAntennaCalibration {
                    sat,
                    antenna_type: "GPS-A".to_string(),
                    valid_from_unix_s: Some(1_577_836_800.0),
                    valid_until_unix_s: Some(1_609_459_199.0),
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        SatellitePhaseCenterOffset::new(0.0, 0.0, 0.1),
                    )]),
                    variations_by_band: BTreeMap::from([(SignalBand::L1, old_variation)]),
                },
                SatelliteAntennaCalibration {
                    sat,
                    antenna_type: "GPS-B".to_string(),
                    valid_from_unix_s: Some(1_609_459_200.0),
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        SatellitePhaseCenterOffset::new(0.0, 0.0, 0.1),
                    )]),
                    variations_by_band: BTreeMap::from([(SignalBand::L1, new_variation)]),
                },
            ],
        };
        let receiver_calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.1),
                )]),
                variations_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.02]),
                )]),
            }],
        };

        let satellite_variation = satellite_calibrations
            .phase_center_variation_m(
                sat,
                SignalBand::L1,
                Some(GpsTime { week: 2200, tow_s: 0.0 }),
                85.0,
                None,
            )
            .expect("satellite L1 variation");
        let receiver_variation = receiver_calibrations
            .phase_center_variation_m(
                "aoad/m_t none",
                SignalBand::L1,
                Some(GpsTime { week: 2200, tow_s: 0.0 }),
                85.0,
                None,
            )
            .expect("receiver L1 variation");

        assert!((satellite_variation - 0.015).abs() < 1.0e-12);
        assert!((receiver_variation - 0.010).abs() < 1.0e-12);
    }

    #[test]
    fn antenna_calibrations_apply_phase_variation_to_iono_free_correction() {
        let sat = SatId { constellation: Constellation::Gps, prn: 1 };
        let gps_time = Some(GpsTime { week: 2200, tow_s: 0.0 });
        let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let satellite_calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "GPS-A".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
                    (SignalBand::L2, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
                ]),
                variations_by_band: BTreeMap::from([
                    (
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                    ),
                    (
                        SignalBand::L2,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.02]),
                    ),
                ]),
            }],
        };
        let receiver_calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([
                    (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                    (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                ]),
                variations_by_band: BTreeMap::from([
                    (
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                    ),
                    (
                        SignalBand::L2,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.02]),
                    ),
                ]),
            }],
        };

        let satellite_correction = satellite_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                sat,
                DualFrequencySignal {
                    primary_band: SignalBand::L1,
                    primary_frequency_hz: 10.0,
                    secondary_band: SignalBand::L2,
                    secondary_frequency_hz: 5.0,
                },
                AntennaPhaseGeometry {
                    range: AntennaRangeGeometry { gps_time, receiver_pos_m, sat_pos_m },
                    elevation_deg: 85.0,
                    azimuth_deg: None,
                },
            )
            .expect("satellite iono-free phase variation correction");
        let receiver_correction = receiver_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                "aoad/m_t none",
                DualFrequencySignal {
                    primary_band: SignalBand::L1,
                    primary_frequency_hz: 10.0,
                    secondary_band: SignalBand::L2,
                    secondary_frequency_hz: 5.0,
                },
                AntennaPhaseGeometry {
                    range: AntennaRangeGeometry { gps_time, receiver_pos_m, sat_pos_m },
                    elevation_deg: 85.0,
                    azimuth_deg: None,
                },
            )
            .expect("receiver iono-free phase variation correction");

        assert!((satellite_correction - 0.063_333_333_333_333_34).abs() < 1.0e-12);
        assert!((receiver_correction - 0.063_333_333_333_333_34).abs() < 1.0e-12);
    }
}
