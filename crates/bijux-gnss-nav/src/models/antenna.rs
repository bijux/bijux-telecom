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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SatelliteAntennaCalibration {
    pub sat: SatId,
    pub antenna_type: String,
    pub valid_from_unix_s: Option<f64>,
    pub valid_until_unix_s: Option<f64>,
    pub offsets_by_band: BTreeMap<SignalBand, SatellitePhaseCenterOffset>,
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReceiverAntennaCalibration {
    pub antenna_type: String,
    pub valid_from_unix_s: Option<f64>,
    pub valid_until_unix_s: Option<f64>,
    pub offsets_by_band: BTreeMap<SignalBand, ReceiverPhaseCenterOffset>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ReceiverAntennaCalibrations {
    pub entries: Vec<ReceiverAntennaCalibration>,
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

    pub fn range_correction_m(
        &self,
        sat: SatId,
        band: SignalBand,
        gps_time: Option<GpsTime>,
        sat_pos_m: [f64; 3],
        receiver_pos_m: [f64; 3],
    ) -> Option<f64> {
        let offset = self.phase_center_offset(sat, band, gps_time)?;
        let gps_time = gps_time?;
        let sun_pos_m = approximate_sun_position_ecef_m(gps_time);
        Some(satellite_antenna_range_correction_m(sat_pos_m, receiver_pos_m, sun_pos_m, offset))
    }

    pub fn iono_free_range_correction_m(
        &self,
        sat: SatId,
        band_1: SignalBand,
        f1_hz: f64,
        band_2: SignalBand,
        f2_hz: f64,
        gps_time: Option<GpsTime>,
        sat_pos_m: [f64; 3],
        receiver_pos_m: [f64; 3],
    ) -> Option<f64> {
        let correction_1 =
            self.range_correction_m(sat, band_1, gps_time, sat_pos_m, receiver_pos_m)?;
        let correction_2 =
            self.range_correction_m(sat, band_2, gps_time, sat_pos_m, receiver_pos_m)?;
        let f1_2 = f1_hz * f1_hz;
        let f2_2 = f2_hz * f2_hz;
        let denom = f1_2 - f2_2;
        if !denom.is_finite() || denom.abs() <= f64::EPSILON {
            return None;
        }
        let weight_1 = f1_2 / denom;
        let weight_2 = -f2_2 / denom;
        Some(weight_1 * correction_1 + weight_2 * correction_2)
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

    pub fn range_correction_m(
        &self,
        antenna_type: &str,
        band: SignalBand,
        gps_time: Option<GpsTime>,
        receiver_pos_m: [f64; 3],
        sat_pos_m: [f64; 3],
    ) -> Option<f64> {
        let offset = self.phase_center_offset(antenna_type, band, gps_time)?;
        Some(receiver_antenna_range_correction_m(receiver_pos_m, sat_pos_m, offset))
    }

    pub fn iono_free_range_correction_m(
        &self,
        antenna_type: &str,
        band_1: SignalBand,
        f1_hz: f64,
        band_2: SignalBand,
        f2_hz: f64,
        gps_time: Option<GpsTime>,
        receiver_pos_m: [f64; 3],
        sat_pos_m: [f64; 3],
    ) -> Option<f64> {
        let correction_1 =
            self.range_correction_m(antenna_type, band_1, gps_time, receiver_pos_m, sat_pos_m)?;
        let correction_2 =
            self.range_correction_m(antenna_type, band_2, gps_time, receiver_pos_m, sat_pos_m)?;
        let f1_2 = f1_hz * f1_hz;
        let f2_2 = f2_hz * f2_hz;
        let denom = f1_2 - f2_2;
        if !denom.is_finite() || denom.abs() <= f64::EPSILON {
            return None;
        }
        let weight_1 = f1_2 / denom;
        let weight_2 = -f2_2 / denom;
        Some(weight_1 * correction_1 + weight_2 * correction_2)
    }
}

pub fn satellite_antenna_range_correction_m(
    sat_pos_m: [f64; 3],
    receiver_pos_m: [f64; 3],
    sun_pos_m: [f64; 3],
    offset: SatellitePhaseCenterOffset,
) -> f64 {
    let body_frame = satellite_body_frame_axes(sat_pos_m, sun_pos_m);
    let phase_center_offset_ecef_m = [
        body_frame.x_axis[0] * offset.body_x_m
            + body_frame.y_axis[0] * offset.body_y_m
            + body_frame.z_axis[0] * offset.body_z_m,
        body_frame.x_axis[1] * offset.body_x_m
            + body_frame.y_axis[1] * offset.body_y_m
            + body_frame.z_axis[1] * offset.body_z_m,
        body_frame.x_axis[2] * offset.body_x_m
            + body_frame.y_axis[2] * offset.body_y_m
            + body_frame.z_axis[2] * offset.body_z_m,
    ];
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
    let local_frame = receiver_local_frame_axes(receiver_pos_m);
    let phase_center_offset_ecef_m = [
        local_frame.north_axis[0] * offset.north_m
            + local_frame.east_axis[0] * offset.east_m
            + local_frame.up_axis[0] * offset.up_m,
        local_frame.north_axis[1] * offset.north_m
            + local_frame.east_axis[1] * offset.east_m
            + local_frame.up_axis[1] * offset.up_m,
        local_frame.north_axis[2] * offset.north_m
            + local_frame.east_axis[2] * offset.east_m
            + local_frame.up_axis[2] * offset.up_m,
    ];
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

#[derive(Debug, Clone, Copy)]
struct SatelliteBodyFrame {
    x_axis: [f64; 3],
    y_axis: [f64; 3],
    z_axis: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct ReceiverLocalFrame {
    north_axis: [f64; 3],
    east_axis: [f64; 3],
    up_axis: [f64; 3],
}

fn satellite_body_frame_axes(sat_pos_m: [f64; 3], sun_pos_m: [f64; 3]) -> SatelliteBodyFrame {
    let z_axis = normalize3([-sat_pos_m[0], -sat_pos_m[1], -sat_pos_m[2]]);
    let sat_to_sun = normalize3([
        sun_pos_m[0] - sat_pos_m[0],
        sun_pos_m[1] - sat_pos_m[1],
        sun_pos_m[2] - sat_pos_m[2],
    ]);
    let y_axis = normalize3(cross3(z_axis, sat_to_sun));
    let x_axis = normalize3(cross3(y_axis, z_axis));
    SatelliteBodyFrame { x_axis, y_axis, z_axis }
}

fn receiver_local_frame_axes(receiver_pos_m: [f64; 3]) -> ReceiverLocalFrame {
    let (lat_rad, lon_rad) = receiver_lat_lon_rad(receiver_pos_m);
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    let east_axis = [-sin_lon, cos_lon, 0.0];
    let north_axis = [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat];
    let up_axis = [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat];
    ReceiverLocalFrame { north_axis, east_axis, up_axis }
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

fn normalize3(vector: [f64; 3]) -> [f64; 3] {
    let norm = norm3(vector).max(f64::EPSILON);
    [vector[0] / norm, vector[1] / norm, vector[2] / norm]
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};

    use super::{
        canonical_receiver_antenna_type, receiver_antenna_range_correction_m,
        satellite_antenna_range_correction_m, satellite_band_from_antex_frequency,
        ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
        SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
    };

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
        };
        let mut offsets_by_band_new = BTreeMap::new();
        offsets_by_band_new.insert(SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.3));
        let new = SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS-B".to_string(),
            valid_from_unix_s: Some(1_609_459_200.0),
            valid_until_unix_s: None,
            offsets_by_band: offsets_by_band_new,
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
        };
        let mut offsets_by_band_new = BTreeMap::new();
        offsets_by_band_new.insert(SignalBand::L1, ReceiverPhaseCenterOffset::new(0.4, 0.5, 0.6));
        let new = ReceiverAntennaCalibration {
            antenna_type: "AOAD/M_T NONE".to_string(),
            valid_from_unix_s: Some(1_609_459_200.0),
            valid_until_unix_s: None,
            offsets_by_band: offsets_by_band_new,
        };
        let ignored = ReceiverAntennaCalibration {
            antenna_type: "TRM57971.00 NONE".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(1.0, 1.0, 1.0),
            )]),
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
}
