//! Deterministic ANTEX validation scenarios for PPP and RTK residual checks.

use bijux_gnss_core::api::{GpsTime, ObsEpoch, ReceiverRole};
use bijux_gnss_nav::api::{
    GpsEphemeris, ReceiverAntennaCalibrations, SatelliteAntennaCalibrations,
};

use super::synthesis::{make_obs_epoch, test_calibrations, test_ephemerides};
use crate::reference_models::coordinates::{ecef_to_enu_m, geodetic_to_ecef_m, GeodeticPoint};

/// Deterministic GPS L1 PPP scenario with antenna biases baked into the observations.
#[derive(Debug, Clone)]
pub struct GpsL1PppAntennaEffectCase {
    /// Receiver position in ECEF meters.
    pub receiver_ecef_m: [f64; 3],
    /// Receive time used to synthesize the observables.
    pub receive_gps_time: GpsTime,
    /// Broadcast ephemerides used for the scenario.
    pub ephemerides: Vec<GpsEphemeris>,
    /// PPP observation epoch with antenna-biased code and carrier values.
    pub epoch: ObsEpoch,
    /// ANTEX receiver antenna type for the PPP receiver.
    pub receiver_antenna_type: String,
    /// Receiver calibrations matching the scenario antenna types.
    pub receiver_calibrations: ReceiverAntennaCalibrations,
    /// Satellite calibrations matching the scenario ephemerides.
    pub satellite_calibrations: SatelliteAntennaCalibrations,
}

/// Deterministic GPS L1 RTK scenario with antenna biases baked into both stations.
#[derive(Debug, Clone)]
pub struct GpsL1RtkAntennaEffectCase {
    /// Base-station position in ECEF meters.
    pub base_ecef_m: [f64; 3],
    /// Rover position in ECEF meters.
    pub rover_ecef_m: [f64; 3],
    /// Rover-minus-base truth in the base-station ENU frame.
    pub rover_enu_m: [f64; 3],
    /// Receive time used to synthesize the observables.
    pub receive_gps_time: GpsTime,
    /// Broadcast ephemerides used for the scenario.
    pub ephemerides: Vec<GpsEphemeris>,
    /// Base-station observation epoch with antenna-biased code and carrier values.
    pub base_epoch: ObsEpoch,
    /// Rover observation epoch with antenna-biased code and carrier values.
    pub rover_epoch: ObsEpoch,
    /// ANTEX receiver antenna type at the base station.
    pub base_antenna_type: String,
    /// ANTEX receiver antenna type at the rover.
    pub rover_antenna_type: String,
    /// Receiver calibrations matching the scenario antenna types.
    pub receiver_calibrations: ReceiverAntennaCalibrations,
    /// Satellite calibrations matching the scenario ephemerides.
    pub satellite_calibrations: SatelliteAntennaCalibrations,
}

/// Build a deterministic GPS L1 PPP antenna-effect scenario.
pub fn gps_l1_ppp_antenna_effect_case() -> GpsL1PppAntennaEffectCase {
    let receiver_ecef_m =
        geodetic_to_ecef_m(GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 15.0 });
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
    let ephemerides = test_ephemerides(receive_gps_time);
    let (receiver_calibrations, satellite_calibrations) = test_calibrations(&ephemerides);
    let receiver_antenna_type = "TRM57971.00 NONE".to_string();
    let epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        receiver_ecef_m,
        &ephemerides,
        Some(receiver_antenna_type.as_str()),
        &receiver_calibrations,
        &satellite_calibrations,
    );

    GpsL1PppAntennaEffectCase {
        receiver_ecef_m,
        receive_gps_time,
        ephemerides,
        epoch,
        receiver_antenna_type,
        receiver_calibrations,
        satellite_calibrations,
    }
}

/// Build a deterministic GPS L1 RTK antenna-effect scenario.
pub fn gps_l1_rtk_antenna_effect_case() -> GpsL1RtkAntennaEffectCase {
    let base_geodetic = GeodeticPoint { lat_deg: 37.0, lon_deg: -122.0, alt_m: 10.0 };
    let base_ecef_m = geodetic_to_ecef_m(base_geodetic);
    let rover_ecef_m =
        geodetic_to_ecef_m(GeodeticPoint { lat_deg: 37.0001, lon_deg: -121.9999, alt_m: 12.0 });
    let rover_enu_m = {
        let enu = ecef_to_enu_m(rover_ecef_m, base_geodetic);
        [enu.east_m, enu.north_m, enu.up_m]
    };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
    let ephemerides = test_ephemerides(receive_gps_time);
    let (receiver_calibrations, satellite_calibrations) = test_calibrations(&ephemerides);
    let base_antenna_type = "AOAD/M_T NONE".to_string();
    let rover_antenna_type = "TRM57971.00 NONE".to_string();
    let base_epoch = make_obs_epoch(
        ReceiverRole::Base,
        receive_gps_time,
        base_ecef_m,
        &ephemerides,
        Some(base_antenna_type.as_str()),
        &receiver_calibrations,
        &satellite_calibrations,
    );
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        rover_ecef_m,
        &ephemerides,
        Some(rover_antenna_type.as_str()),
        &receiver_calibrations,
        &satellite_calibrations,
    );

    GpsL1RtkAntennaEffectCase {
        base_ecef_m,
        rover_ecef_m,
        rover_enu_m,
        receive_gps_time,
        ephemerides,
        base_epoch,
        rover_epoch,
        base_antenna_type,
        rover_antenna_type,
        receiver_calibrations,
        satellite_calibrations,
    }
}

#[cfg(test)]
mod tests {
    use super::{gps_l1_ppp_antenna_effect_case, gps_l1_rtk_antenna_effect_case};

    #[test]
    fn ppp_antenna_effect_case_emits_four_or_more_satellites() {
        let case = gps_l1_ppp_antenna_effect_case();
        assert!(case.epoch.sats.len() >= 4);
    }

    #[test]
    fn rtk_antenna_effect_case_emits_matching_satellite_sets() {
        let case = gps_l1_rtk_antenna_effect_case();
        assert_eq!(case.base_epoch.sats.len(), case.rover_epoch.sats.len());
        assert!(case.base_epoch.sats.len() >= 4);
    }
}
