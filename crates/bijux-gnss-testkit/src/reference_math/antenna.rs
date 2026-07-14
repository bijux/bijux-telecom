//! Independent antenna reference modeling for test scenarios.

use bijux_gnss_core::api::{gps_to_utc, GpsTime, LeapSeconds, SatId, SignalBand};
use bijux_gnss_nav::api::{
    ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset, SatelliteAntennaCalibrations,
    SatellitePhaseCenterOffset,
};

use crate::reference_math::coordinates::{
    cross3, dot3, ecef_to_geodetic_point, normalize3, subtract3,
};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

pub(crate) fn receiver_range_correction_m(
    calibrations: &ReceiverAntennaCalibrations,
    antenna_type: &str,
    band: SignalBand,
    gps_time: Option<GpsTime>,
    receiver_ecef_m: [f64; 3],
    satellite_ecef_m: [f64; 3],
) -> Option<f64> {
    let offset = receiver_phase_center_offset(calibrations, antenna_type, band, gps_time)?;
    Some(receiver_range_correction_from_offset_m(receiver_ecef_m, satellite_ecef_m, offset))
}

pub(crate) fn satellite_range_correction_m(
    calibrations: &SatelliteAntennaCalibrations,
    sat: SatId,
    band: SignalBand,
    gps_time: Option<GpsTime>,
    satellite_ecef_m: [f64; 3],
    receiver_ecef_m: [f64; 3],
) -> Option<f64> {
    let offset = satellite_phase_center_offset(calibrations, sat, band, gps_time)?;
    let sun_ecef_m = approximate_sun_position_ecef_m(gps_time?);
    Some(satellite_range_correction_from_offset_m(
        satellite_ecef_m,
        receiver_ecef_m,
        sun_ecef_m,
        offset,
    ))
}

fn receiver_phase_center_offset(
    calibrations: &ReceiverAntennaCalibrations,
    antenna_type: &str,
    band: SignalBand,
    gps_time: Option<GpsTime>,
) -> Option<ReceiverPhaseCenterOffset> {
    let unix_time_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
    let canonical = canonical_receiver_antenna_type(antenna_type);
    calibrations
        .entries
        .iter()
        .filter(|entry| canonical_receiver_antenna_type(&entry.antenna_type) == canonical)
        .filter(|entry| {
            calibration_window_matches(
                entry.valid_from_unix_s,
                entry.valid_until_unix_s,
                unix_time_s,
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

fn satellite_phase_center_offset(
    calibrations: &SatelliteAntennaCalibrations,
    sat: SatId,
    band: SignalBand,
    gps_time: Option<GpsTime>,
) -> Option<SatellitePhaseCenterOffset> {
    let unix_time_s = gps_time.map(|time| gps_to_utc(time, &LeapSeconds::default_table()).unix_s);
    calibrations
        .entries
        .iter()
        .filter(|entry| entry.sat == sat)
        .filter(|entry| {
            calibration_window_matches(
                entry.valid_from_unix_s,
                entry.valid_until_unix_s,
                unix_time_s,
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

fn receiver_range_correction_from_offset_m(
    receiver_ecef_m: [f64; 3],
    satellite_ecef_m: [f64; 3],
    offset: ReceiverPhaseCenterOffset,
) -> f64 {
    let geodetic = ecef_to_geodetic_point(receiver_ecef_m);
    let latitude_rad = geodetic.lat_deg.to_radians();
    let longitude_rad = geodetic.lon_deg.to_radians();
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let sin_longitude = longitude_rad.sin();
    let cos_longitude = longitude_rad.cos();
    let north_axis = [-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude];
    let east_axis = [-sin_longitude, cos_longitude, 0.0];
    let up_axis = [cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude];
    let offset_ecef_m = [
        north_axis[0] * offset.north_m + east_axis[0] * offset.east_m + up_axis[0] * offset.up_m,
        north_axis[1] * offset.north_m + east_axis[1] * offset.east_m + up_axis[1] * offset.up_m,
        north_axis[2] * offset.north_m + east_axis[2] * offset.east_m + up_axis[2] * offset.up_m,
    ];
    let corrected_receiver_ecef_m = [
        receiver_ecef_m[0] + offset_ecef_m[0],
        receiver_ecef_m[1] + offset_ecef_m[1],
        receiver_ecef_m[2] + offset_ecef_m[2],
    ];
    geometric_range_m(corrected_receiver_ecef_m, satellite_ecef_m)
        - geometric_range_m(receiver_ecef_m, satellite_ecef_m)
}

fn satellite_range_correction_from_offset_m(
    satellite_ecef_m: [f64; 3],
    receiver_ecef_m: [f64; 3],
    sun_ecef_m: [f64; 3],
    offset: SatellitePhaseCenterOffset,
) -> f64 {
    let body_z_axis =
        normalize3([-satellite_ecef_m[0], -satellite_ecef_m[1], -satellite_ecef_m[2]]);
    let satellite_to_sun = normalize3(subtract3(sun_ecef_m, satellite_ecef_m));
    let body_y_axis = normalize3(cross3(body_z_axis, satellite_to_sun));
    let body_x_axis = normalize3(cross3(body_y_axis, body_z_axis));
    let offset_ecef_m = [
        body_x_axis[0] * offset.body_x_m
            + body_y_axis[0] * offset.body_y_m
            + body_z_axis[0] * offset.body_z_m,
        body_x_axis[1] * offset.body_x_m
            + body_y_axis[1] * offset.body_y_m
            + body_z_axis[1] * offset.body_z_m,
        body_x_axis[2] * offset.body_x_m
            + body_y_axis[2] * offset.body_y_m
            + body_z_axis[2] * offset.body_z_m,
    ];
    let corrected_satellite_ecef_m = [
        satellite_ecef_m[0] + offset_ecef_m[0],
        satellite_ecef_m[1] + offset_ecef_m[1],
        satellite_ecef_m[2] + offset_ecef_m[2],
    ];
    geometric_range_m(receiver_ecef_m, corrected_satellite_ecef_m)
        - geometric_range_m(receiver_ecef_m, satellite_ecef_m)
}

fn approximate_sun_position_ecef_m(gps_time: GpsTime) -> [f64; 3] {
    let utc_unix_s = gps_to_utc(gps_time, &LeapSeconds::default_table()).unix_s;
    let julian_day = utc_unix_s / 86_400.0 + 2_440_587.5;
    let centuries_since_j2000 = (julian_day - 2_451_545.0) / 36_525.0;
    let mean_longitude_deg = wrap_degrees(280.460 + 36_000.770 * centuries_since_j2000);
    let mean_anomaly_deg = wrap_degrees(357.528 + 35_999.050 * centuries_since_j2000);
    let mean_anomaly_rad = mean_anomaly_deg.to_radians();
    let ecliptic_longitude_rad = (mean_longitude_deg
        + 1.915 * mean_anomaly_rad.sin()
        + 0.020 * (2.0 * mean_anomaly_rad).sin())
    .to_radians();
    let obliquity_rad = (23.4393 - 0.0130 * centuries_since_j2000).to_radians();

    let eci_m = [
        ASTRONOMICAL_UNIT_M * ecliptic_longitude_rad.cos(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.cos() * ecliptic_longitude_rad.sin(),
        ASTRONOMICAL_UNIT_M * obliquity_rad.sin() * ecliptic_longitude_rad.sin(),
    ];
    let sidereal_deg =
        wrap_degrees(280.460_618_37 + 360.985_647_366_29 * (julian_day - 2_451_545.0));
    let sidereal_rad = sidereal_deg.to_radians();
    let cos_sidereal = sidereal_rad.cos();
    let sin_sidereal = sidereal_rad.sin();

    [
        cos_sidereal * eci_m[0] + sin_sidereal * eci_m[1],
        -sin_sidereal * eci_m[0] + cos_sidereal * eci_m[1],
        eci_m[2],
    ]
}

fn canonical_receiver_antenna_type(value: &str) -> String {
    value.split_whitespace().collect::<Vec<_>>().join(" ").to_ascii_uppercase()
}

fn calibration_window_matches(
    valid_from_unix_s: Option<f64>,
    valid_until_unix_s: Option<f64>,
    unix_time_s: Option<f64>,
) -> bool {
    let Some(unix_time_s) = unix_time_s else {
        return true;
    };
    if let Some(valid_from_unix_s) = valid_from_unix_s {
        if unix_time_s < valid_from_unix_s {
            return false;
        }
    }
    if let Some(valid_until_unix_s) = valid_until_unix_s {
        if unix_time_s > valid_until_unix_s {
            return false;
        }
    }
    true
}

fn wrap_degrees(angle_deg: f64) -> f64 {
    angle_deg.rem_euclid(360.0)
}

fn geometric_range_m(left_ecef_m: [f64; 3], right_ecef_m: [f64; 3]) -> f64 {
    dot3(subtract3(left_ecef_m, right_ecef_m), subtract3(left_ecef_m, right_ecef_m)).sqrt()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};
    use bijux_gnss_nav::api::{
        ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
        SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
    };

    use super::{receiver_range_correction_m, satellite_range_correction_m};

    #[test]
    fn receiver_range_correction_projects_neu_offset() {
        let calibrations = ReceiverAntennaCalibrations {
            entries: vec![ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.2, 0.5, 1.0),
                )]),
            }],
        };
        let correction_m = receiver_range_correction_m(
            &calibrations,
            "AOAD/M_T NONE",
            SignalBand::L1,
            None,
            [6_378_137.0, 0.0, 0.0],
            [20_200_000.0, 10_000.0, 5_000.0],
        )
        .expect("receiver correction");
        let corrected_receiver_ecef_m = [6_378_138.0, 0.5, 0.2];
        let satellite_ecef_m = [20_200_000.0, 10_000.0, 5_000.0];
        let corrected_delta_x_m = corrected_receiver_ecef_m[0] - satellite_ecef_m[0];
        let corrected_delta_y_m = corrected_receiver_ecef_m[1] - satellite_ecef_m[1];
        let corrected_delta_z_m = corrected_receiver_ecef_m[2] - satellite_ecef_m[2];
        let uncorrected_delta_x_m = 6_378_137.0_f64 - satellite_ecef_m[0];
        let uncorrected_delta_y_m = -satellite_ecef_m[1];
        let uncorrected_delta_z_m = -satellite_ecef_m[2];
        let expected_m = (corrected_delta_x_m * corrected_delta_x_m
            + corrected_delta_y_m * corrected_delta_y_m
            + corrected_delta_z_m * corrected_delta_z_m)
            .sqrt()
            - (uncorrected_delta_x_m * uncorrected_delta_x_m
                + uncorrected_delta_y_m * uncorrected_delta_y_m
                + uncorrected_delta_z_m * uncorrected_delta_z_m)
                .sqrt();

        assert!(correction_m.is_finite());
        assert!((correction_m - expected_m).abs() < 1.0e-9);
    }

    #[test]
    fn satellite_range_correction_projects_body_offset() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let calibrations = SatelliteAntennaCalibrations {
            entries: vec![SatelliteAntennaCalibration {
                sat,
                antenna_type: "BLOCK IIR".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    SatellitePhaseCenterOffset::new(0.25, 0.0, 1.0),
                )]),
            }],
        };
        let correction_m = satellite_range_correction_m(
            &calibrations,
            sat,
            SignalBand::L1,
            Some(GpsTime { week: 2200, tow_s: 43_200.0 }),
            [20_200_000.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        .expect("satellite correction");

        assert!(correction_m.is_finite());
        assert!(correction_m < 0.0);
    }
}
