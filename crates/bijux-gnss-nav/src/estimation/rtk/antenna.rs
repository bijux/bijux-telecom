use bijux_gnss_core::api::{GpsTime, SatId, SignalBand};

use crate::estimation::position::solver::elevation_azimuth_deg;
use crate::models::antenna::{ReceiverAntennaCalibrations, SatelliteAntennaCalibrations};

/// Antenna calibrations used to evaluate RTK residuals against antenna-aware geometry.
#[derive(Debug, Clone, Default)]
pub struct RtkAntennaCorrectionConfig {
    /// ANTEX receiver antenna type at the base station.
    pub base_antenna_type: Option<String>,
    /// ANTEX receiver antenna type at the rover.
    pub rover_antenna_type: Option<String>,
    /// Receiver antenna calibrations available to both stations.
    pub receiver_calibrations: Option<ReceiverAntennaCalibrations>,
    /// Satellite antenna calibrations available to the modeled satellites.
    pub satellite_calibrations: Option<SatelliteAntennaCalibrations>,
}

impl RtkAntennaCorrectionConfig {
    /// Return whether no antenna correction source is enabled.
    pub fn is_disabled(&self) -> bool {
        self.base_antenna_type.is_none()
            && self.rover_antenna_type.is_none()
            && self.receiver_calibrations.is_none()
            && self.satellite_calibrations.is_none()
    }
}

pub(crate) fn modeled_pseudorange_with_antenna_corrections_m(
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    sat_clock_bias_s: f64,
    sat: SatId,
    band: SignalBand,
    gps_time: Option<GpsTime>,
    receiver_antenna_type: Option<&str>,
    corrections: Option<&RtkAntennaCorrectionConfig>,
) -> f64 {
    modeled_pseudorange_m(receiver_ecef_m, sat_ecef_m, sat_clock_bias_s)
        + antenna_range_correction_m(
            sat,
            band,
            gps_time,
            receiver_ecef_m,
            sat_ecef_m,
            receiver_antenna_type,
            corrections,
        )
}

fn antenna_range_correction_m(
    sat: SatId,
    band: SignalBand,
    gps_time: Option<GpsTime>,
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    receiver_antenna_type: Option<&str>,
    corrections: Option<&RtkAntennaCorrectionConfig>,
) -> f64 {
    let Some(corrections) = corrections else {
        return 0.0;
    };
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m[0],
        receiver_ecef_m[1],
        receiver_ecef_m[2],
        sat_ecef_m[0],
        sat_ecef_m[1],
        sat_ecef_m[2],
    );

    let satellite_correction_m = corrections
        .satellite_calibrations
        .as_ref()
        .and_then(|calibrations| {
            calibrations.range_correction_with_phase_variation_m(
                sat,
                band,
                gps_time,
                sat_ecef_m,
                receiver_ecef_m,
                elevation_deg,
                Some(azimuth_deg),
            )
        })
        .unwrap_or(0.0);
    let receiver_correction_m = receiver_antenna_type
        .and_then(|antenna_type| {
            corrections.receiver_calibrations.as_ref().and_then(|calibrations| {
                calibrations.range_correction_with_phase_variation_m(
                    antenna_type,
                    band,
                    gps_time,
                    receiver_ecef_m,
                    sat_ecef_m,
                    elevation_deg,
                    Some(azimuth_deg),
                )
            })
        })
        .unwrap_or(0.0);

    satellite_correction_m + receiver_correction_m
}

pub(crate) fn modeled_pseudorange_m(
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    sat_clock_bias_s: f64,
) -> f64 {
    geometric_range_m(receiver_ecef_m, sat_ecef_m) - sat_clock_bias_s * 299_792_458.0
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, SatId, SignalBand};

    use super::{modeled_pseudorange_m, modeled_pseudorange_with_antenna_corrections_m};
    use crate::models::antenna::{
        AntennaPhaseCenterVariation, ReceiverAntennaCalibration, ReceiverAntennaCalibrations,
        ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
        SatellitePhaseCenterOffset,
    };

    #[test]
    fn rtk_antenna_config_changes_modeled_pseudorange() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_ecef_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_ecef_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let config = super::RtkAntennaCorrectionConfig {
            base_antenna_type: None,
            rover_antenna_type: Some("AOAD/M_T NONE".to_string()),
            receiver_calibrations: Some(ReceiverAntennaCalibrations {
                entries: vec![ReceiverAntennaCalibration {
                    antenna_type: "AOAD/M_T NONE".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        ReceiverPhaseCenterOffset::new(0.12, -0.04, 0.95),
                    )]),
                    variations_by_band: BTreeMap::new(),
                }],
            }),
            satellite_calibrations: Some(SatelliteAntennaCalibrations {
                entries: vec![SatelliteAntennaCalibration {
                    sat,
                    antenna_type: "BLOCK IIR".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        SatellitePhaseCenterOffset::new(0.05, -0.02, 0.18),
                    )]),
                    variations_by_band: BTreeMap::new(),
                }],
            }),
        };

        let uncorrected = modeled_pseudorange_m(receiver_ecef_m, sat_ecef_m, 0.0);
        let corrected = modeled_pseudorange_with_antenna_corrections_m(
            receiver_ecef_m,
            sat_ecef_m,
            0.0,
            sat,
            SignalBand::L1,
            gps_time,
            config.rover_antenna_type.as_deref(),
            Some(&config),
        );

        assert!((corrected - uncorrected).abs() > 1.0e-6);
    }

    #[test]
    fn rtk_antenna_config_applies_phase_center_variation() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let gps_time = Some(GpsTime { week: 2200, tow_s: 86_400.0 });
        let sat_ecef_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
        let receiver_ecef_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
        let config = super::RtkAntennaCorrectionConfig {
            base_antenna_type: None,
            rover_antenna_type: Some("AOAD/M_T NONE".to_string()),
            receiver_calibrations: Some(ReceiverAntennaCalibrations {
                entries: vec![ReceiverAntennaCalibration {
                    antenna_type: "AOAD/M_T NONE".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0),
                    )]),
                    variations_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                    )]),
                }],
            }),
            satellite_calibrations: Some(SatelliteAntennaCalibrations {
                entries: vec![SatelliteAntennaCalibration {
                    sat,
                    antenna_type: "BLOCK IIR".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0),
                    )]),
                    variations_by_band: BTreeMap::from([(
                        SignalBand::L1,
                        AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                    )]),
                }],
            }),
        };

        let uncorrected = modeled_pseudorange_m(receiver_ecef_m, sat_ecef_m, 0.0);
        let corrected = modeled_pseudorange_with_antenna_corrections_m(
            receiver_ecef_m,
            sat_ecef_m,
            0.0,
            sat,
            SignalBand::L1,
            gps_time,
            config.rover_antenna_type.as_deref(),
            Some(&config),
        );

        assert!((corrected - uncorrected).abs() > 1.0e-6);
    }
}
