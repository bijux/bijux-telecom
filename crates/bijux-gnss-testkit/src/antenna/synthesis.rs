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

use crate::reference_models::antenna::{receiver_range_correction_m, satellite_range_correction_m};
use crate::reference_models::gps_broadcast::{
    pseudorange_from_truth, solve_transmit_state_for_receiver,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

pub(super) fn test_ephemerides(receive_gps_time: GpsTime) -> Vec<GpsEphemeris> {
    vec![
        make_eph(receive_gps_time, 3, 0.0, 0.0),
        make_eph(receive_gps_time, 7, 0.8, 0.9),
        make_eph(receive_gps_time, 11, 1.6, 1.8),
        make_eph(receive_gps_time, 14, 2.4, 2.7),
        make_eph(receive_gps_time, 19, 3.2, 3.6),
    ]
}

pub(super) fn test_calibrations(
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

pub(super) fn make_obs_epoch(
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

fn make_eph(receive_gps_time: GpsTime, prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: receive_gps_time.week,
        sv_health: 0,
        sv_accuracy: Some(2),
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
