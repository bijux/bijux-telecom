#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{gps_to_utc, Constellation, GpsTime, LeapSeconds, SatId, SignalBand};
use serde::{Deserialize, Serialize};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

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
            .filter(|entry| calibration_matches_time(entry, unix_s))
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
        Some(satellite_antenna_range_correction_m(
            sat_pos_m,
            receiver_pos_m,
            sun_pos_m,
            offset,
        ))
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

pub fn satellite_band_from_antex_frequency(
    constellation: Constellation,
    frequency_code: &str,
) -> Option<SignalBand> {
    match (constellation, frequency_code.trim()) {
        (Constellation::Gps, "G01") => Some(SignalBand::L1),
        (Constellation::Gps, "G02") => Some(SignalBand::L2),
        (Constellation::Gps, "G05") => Some(SignalBand::L5),
        (Constellation::Galileo, "E01") => Some(SignalBand::E1),
        (Constellation::Galileo, "E05") | (Constellation::Galileo, "E07") | (Constellation::Galileo, "E08") => {
            Some(SignalBand::E5)
        }
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

fn approximate_sun_position_ecef_m(gps_time: GpsTime) -> [f64; 3] {
    let utc = gps_to_utc(gps_time, &LeapSeconds::default_table());
    let julian_day = utc.unix_s / 86_400.0 + 2_440_587.5;
    let centuries = (julian_day - 2_451_545.0) / 36_525.0;
    let mean_longitude_deg = wrap_degrees(280.460 + 36_000.770 * centuries);
    let mean_anomaly_deg = wrap_degrees(357.528 + 35_999.050 * centuries);
    let mean_anomaly_rad = mean_anomaly_deg.to_radians();
    let ecliptic_longitude_deg = mean_longitude_deg
        + 1.915 * mean_anomaly_rad.sin()
        + 0.020 * (2.0 * mean_anomaly_rad).sin();
    let ecliptic_longitude_rad = ecliptic_longitude_deg.to_radians();
    let obliquity_rad = (23.4393 - 0.0130 * centuries).to_radians();

    let sun_eci_m = [
        ASTRONOMICAL_UNIT_M * ecliptic_longitude_rad.cos(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.cos() * ecliptic_longitude_rad.sin(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.sin() * ecliptic_longitude_rad.sin(),
    ];

    let gmst_deg =
        wrap_degrees(280.460_618_37 + 360.985_647_366_29 * (julian_day - 2_451_545.0));
    let gmst_rad = gmst_deg.to_radians();
    let cos_theta = gmst_rad.cos();
    let sin_theta = gmst_rad.sin();

    [
        cos_theta * sun_eci_m[0] + sin_theta * sun_eci_m[1],
        -sin_theta * sun_eci_m[0] + cos_theta * sun_eci_m[1],
        sun_eci_m[2],
    ]
}

fn calibration_matches_time(entry: &SatelliteAntennaCalibration, unix_s: Option<f64>) -> bool {
    let Some(unix_s) = unix_s else {
        return true;
    };
    if let Some(valid_from_unix_s) = entry.valid_from_unix_s {
        if unix_s < valid_from_unix_s {
            return false;
        }
    }
    if let Some(valid_until_unix_s) = entry.valid_until_unix_s {
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

fn wrap_degrees(degrees: f64) -> f64 {
    degrees.rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};

    use super::{
        satellite_antenna_range_correction_m, satellite_band_from_antex_frequency,
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
        offsets_by_band_new
            .insert(SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.3));
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
}
