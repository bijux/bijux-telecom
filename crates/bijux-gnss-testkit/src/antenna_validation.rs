//! Deterministic ANTEX validation scenarios for PPP and RTK residual checks.

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
    SignalCode, SignalSpec, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_nav::api::{
    GpsEphemeris, ReceiverAntennaCalibration, ReceiverAntennaCalibrations,
    ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
    SatellitePhaseCenterOffset,
};

use crate::reference_math::antenna::{receiver_range_correction_m, satellite_range_correction_m};
use crate::reference_math::coordinates::{ecef_to_enu_m, geodetic_to_ecef_m, GeodeticPoint};
use crate::reference_math::gps_broadcast::{
    pseudorange_from_truth, solve_transmit_state_for_receiver,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

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

fn test_ephemerides(receive_gps_time: GpsTime) -> Vec<GpsEphemeris> {
    vec![
        make_eph(receive_gps_time, 3, 0.0, 0.0),
        make_eph(receive_gps_time, 7, 0.8, 0.9),
        make_eph(receive_gps_time, 11, 1.6, 1.8),
        make_eph(receive_gps_time, 14, 2.4, 2.7),
        make_eph(receive_gps_time, 19, 3.2, 3.6),
    ]
}

fn test_calibrations(
    ephemerides: &[GpsEphemeris],
) -> (ReceiverAntennaCalibrations, SatelliteAntennaCalibrations) {
    let receiver_calibrations = ReceiverAntennaCalibrations {
        entries: vec![
            ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.03, 0.01, 0.82),
                )]),
                variations_by_band: BTreeMap::new(),
            },
            ReceiverAntennaCalibration {
                antenna_type: "TRM57971.00 NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.15, -0.06, 1.23),
                )]),
                variations_by_band: BTreeMap::new(),
            },
        ],
    };
    let satellite_calibrations = SatelliteAntennaCalibrations {
        entries: ephemerides
            .iter()
            .enumerate()
            .map(|(index, ephemeris)| SatelliteAntennaCalibration {
                sat: ephemeris.sat,
                antenna_type: "BLOCK IIR".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    SatellitePhaseCenterOffset::new(
                        0.04 - index as f64 * 0.01,
                        -0.02 + index as f64 * 0.008,
                        0.18 - index as f64 * 0.01,
                    ),
                )]),
                variations_by_band: BTreeMap::new(),
            })
            .collect(),
    };
    (receiver_calibrations, satellite_calibrations)
}

fn make_eph(receive_gps_time: GpsTime, prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: receive_gps_time.week,
        sv_health: 0,
        toe_s: receive_gps_time.tow_s - 900.0,
        toc_s: receive_gps_time.tow_s - 900.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn make_obs_epoch(
    role: ReceiverRole,
    receive_gps_time: GpsTime,
    receiver_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receiver_antenna_type: Option<&str>,
    receiver_calibrations: &ReceiverAntennaCalibrations,
    satellite_calibrations: &SatelliteAntennaCalibrations,
) -> ObsEpoch {
    let wavelength_m = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ.value();
    let sats = ephemerides
        .iter()
        .map(|ephemeris| {
            let solved = solve_transmit_state_for_receiver(
                ephemeris,
                receive_gps_time.tow_s,
                receiver_ecef_m,
            );
            let sat = solved.transmit_state;
            let sat_ecef_m = [sat.x_m, sat.y_m, sat.z_m];
            let unbiased_pseudorange_m =
                pseudorange_from_truth(ephemeris, receiver_ecef_m, receive_gps_time.tow_s, 0.0);
            let gps_time = Some(receive_gps_time);
            let satellite_correction_m = satellite_range_correction_m(
                satellite_calibrations,
                ephemeris.sat,
                SignalBand::L1,
                gps_time,
                sat_ecef_m,
                receiver_ecef_m,
            )
            .expect("satellite antenna correction");
            let receiver_correction_m = receiver_antenna_type
                .and_then(|antenna_type| {
                    receiver_range_correction_m(
                        receiver_calibrations,
                        antenna_type,
                        SignalBand::L1,
                        gps_time,
                        receiver_ecef_m,
                        sat_ecef_m,
                    )
                })
                .unwrap_or(0.0);
            let antenna_correction_m = satellite_correction_m + receiver_correction_m;
            let timing = ObsSignalTiming {
                signal_travel_time_s: Seconds(solved.signal_travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-solved.signal_travel_time_s),
            };
            ObsSatellite {
                signal_id: SigId { sat: ephemeris.sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: bijux_gnss_core::api::Meters(
                    unbiased_pseudorange_m + antenna_correction_m,
                ),
                pseudorange_var_m2: 4.0e-4,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(
                    (unbiased_pseudorange_m + antenna_correction_m) / wavelength_m,
                ),
                carrier_phase_var_cycles2: 1.0e-4,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
                cn0_dbhz: 48.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: Some(timing),
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "synthetic_antex".to_string(),
                    integration_ms: 1,
                    lock_quality: 48.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: SignalSpec {
                        constellation: Constellation::Gps,
                        band: SignalBand::L1,
                        code: SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: GPS_L1_CA_CARRIER_HZ,
                    },
                    ..ObsMetadata::default()
                },
            }
        })
        .collect();

    ObsEpoch {
        t_rx_s: Seconds(receive_gps_time.tow_s),
        source_time: ReceiverSampleTrace::from_sample_index(
            (receive_gps_time.tow_s * 1000.0) as u64,
            1_000.0,
        ),
        gps_week: Some(receive_gps_time.week),
        tow_s: Some(Seconds(receive_gps_time.tow_s)),
        epoch_idx: (receive_gps_time.tow_s * 1000.0) as u64,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role,
        sats,
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
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
