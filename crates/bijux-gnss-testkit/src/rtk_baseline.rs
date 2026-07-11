//! Synthetic RTK short-baseline fixtures with deterministic carrier ambiguity truth.

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
    SignalSpec,
};
use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signal, ecef_to_geodetic,
    rtk_double_differences_from_single_differences, rtk_single_differences_from_obs_epochs,
    sat_state_gps_l1ca_at_receive_time, GpsEphemeris, RtkDoubleDifferenceObservation,
    RtkSingleDifferenceObservation,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

/// Deterministic clean GPS L1 RTK scenario for short-baseline float and accuracy tests.
#[derive(Debug, Clone)]
pub struct CleanGpsL1RtkBaselineCase {
    /// Base station position in ECEF meters.
    pub base_ecef_m: [f64; 3],
    /// Rover position in ECEF meters.
    pub rover_ecef_m: [f64; 3],
    /// Rover-minus-base truth in the base-station ENU frame.
    pub truth_enu_m: [f64; 3],
    /// Receive time used to synthesize the observables.
    pub receive_gps_time: GpsTime,
    /// Broadcast ephemerides used for the scenario.
    pub ephemerides: Vec<GpsEphemeris>,
    /// Base-station observation epoch.
    pub base_epoch: ObsEpoch,
    /// Rover observation epoch.
    pub rover_epoch: ObsEpoch,
    /// Selected reference signal for double differences.
    pub reference_sig: SigId,
    /// Single-difference observables generated from the scenario.
    pub single_differences: Vec<RtkSingleDifferenceObservation>,
    /// Double-difference observables generated from the scenario.
    pub double_differences: Vec<RtkDoubleDifferenceObservation>,
    /// Rover carrier ambiguities in cycles keyed by satellite.
    pub rover_ambiguities_cycles: BTreeMap<SatId, f64>,
}

/// Axis and norm errors between an estimated and truth RTK baseline.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkBaselineAccuracy {
    /// Signed east error in meters.
    pub east_error_m: f64,
    /// Signed north error in meters.
    pub north_error_m: f64,
    /// Signed up error in meters.
    pub up_error_m: f64,
    /// Horizontal norm of the east/north error in meters.
    pub horizontal_error_m: f64,
    /// Three-dimensional norm of the ENU error in meters.
    pub three_dimensional_error_m: f64,
    /// Largest absolute axis error in meters.
    pub max_axis_error_m: f64,
}

/// Accuracy budget for clean short-baseline RTK truth validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkBaselineAccuracyBudget {
    /// Maximum absolute east error in meters.
    pub max_east_error_m: f64,
    /// Maximum absolute north error in meters.
    pub max_north_error_m: f64,
    /// Maximum absolute up error in meters.
    pub max_up_error_m: f64,
    /// Maximum horizontal error norm in meters.
    pub max_horizontal_error_m: f64,
    /// Maximum three-dimensional error norm in meters.
    pub max_three_dimensional_error_m: f64,
}

impl RtkBaselineAccuracy {
    /// Return whether the measured baseline error fits within the provided budget.
    pub fn satisfies(self, budget: RtkBaselineAccuracyBudget) -> bool {
        self.east_error_m.abs() <= budget.max_east_error_m
            && self.north_error_m.abs() <= budget.max_north_error_m
            && self.up_error_m.abs() <= budget.max_up_error_m
            && self.horizontal_error_m <= budget.max_horizontal_error_m
            && self.three_dimensional_error_m <= budget.max_three_dimensional_error_m
    }
}

/// Build a deterministic clean short-baseline GPS L1 RTK scenario.
pub fn clean_gps_l1_short_baseline_case() -> CleanGpsL1RtkBaselineCase {
    let base = bijux_gnss_nav::api::geodetic_to_ecef(37.0, -122.0, 10.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let truth_enu_m = [8.5, -4.25, 1.75];
    let rover_ecef_m = enu_to_ecef(base_ecef_m, truth_enu_m);
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
    let ephemerides = vec![
        make_eph(3, 0.0, 0.0, receive_gps_time.tow_s - 900.0),
        make_eph(7, 0.8, 0.9, receive_gps_time.tow_s - 900.0),
        make_eph(11, 1.6, 1.8, receive_gps_time.tow_s - 900.0),
        make_eph(14, 2.4, 2.7, receive_gps_time.tow_s - 900.0),
        make_eph(19, 3.2, 3.6, receive_gps_time.tow_s - 900.0),
    ];
    let base_ambiguities_cycles = BTreeMap::new();
    let rover_ambiguities_cycles = BTreeMap::from([
        (ephemerides[0].sat, 17.25),
        (ephemerides[1].sat, 24.75),
        (ephemerides[2].sat, 31.5),
        (ephemerides[3].sat, 42.125),
        (ephemerides[4].sat, 53.625),
    ]);
    let base_epoch = make_obs_epoch(
        ReceiverRole::Base,
        receive_gps_time,
        base_ecef_m,
        &ephemerides,
        &base_ambiguities_cycles,
    );
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        rover_ecef_m,
        &ephemerides,
        &rover_ambiguities_cycles,
    );
    let single_differences = rtk_single_differences_from_obs_epochs(&base_epoch, &rover_epoch);
    let reference_sig =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference_sig);

    CleanGpsL1RtkBaselineCase {
        base_ecef_m,
        rover_ecef_m,
        truth_enu_m,
        receive_gps_time,
        ephemerides,
        base_epoch,
        rover_epoch,
        reference_sig,
        single_differences,
        double_differences,
        rover_ambiguities_cycles,
    }
}

/// Compare an estimated ENU baseline against RTK truth.
pub fn rtk_baseline_accuracy(estimated_enu_m: [f64; 3], truth_enu_m: [f64; 3]) -> RtkBaselineAccuracy {
    let east_error_m = estimated_enu_m[0] - truth_enu_m[0];
    let north_error_m = estimated_enu_m[1] - truth_enu_m[1];
    let up_error_m = estimated_enu_m[2] - truth_enu_m[2];
    let horizontal_error_m = (east_error_m * east_error_m + north_error_m * north_error_m).sqrt();
    let three_dimensional_error_m =
        (horizontal_error_m * horizontal_error_m + up_error_m * up_error_m).sqrt();
    let max_axis_error_m = east_error_m
        .abs()
        .max(north_error_m.abs())
        .max(up_error_m.abs());
    RtkBaselineAccuracy {
        east_error_m,
        north_error_m,
        up_error_m,
        horizontal_error_m,
        three_dimensional_error_m,
        max_axis_error_m,
    }
}

/// Return the clean short-baseline validation budget for centimeter-level RTK accuracy.
pub fn centimeter_level_rtk_baseline_budget() -> RtkBaselineAccuracyBudget {
    RtkBaselineAccuracyBudget {
        max_east_error_m: 0.01,
        max_north_error_m: 0.01,
        max_up_error_m: 0.02,
        max_horizontal_error_m: 0.015,
        max_three_dimensional_error_m: 0.02,
    }
}

fn make_eph(prn: u8, omega0: f64, m0: f64, toe_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 2200,
        sv_health: 0,
        toe_s,
        toc_s: toe_s,
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
    ambiguities_cycles: &BTreeMap<SatId, f64>,
) -> ObsEpoch {
    let wavelength_m = SPEED_OF_LIGHT_MPS / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value();
    let sats = ephemerides
        .iter()
        .map(|ephemeris| {
            let mut travel_time_s = 0.07;
            let sat = loop {
                let state = sat_state_gps_l1ca_at_receive_time(
                    ephemeris,
                    receive_gps_time.tow_s,
                    travel_time_s,
                );
                let range_m = geometric_range_m(receiver_ecef_m, [state.x_m, state.y_m, state.z_m]);
                let next_travel_time_s = range_m / SPEED_OF_LIGHT_MPS;
                if (next_travel_time_s - travel_time_s).abs() < 1.0e-12 {
                    break state;
                }
                travel_time_s = next_travel_time_s;
            };
            let range_m = geometric_range_m(receiver_ecef_m, [sat.x_m, sat.y_m, sat.z_m]);
            let pseudorange_m = range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
            let ambiguity_cycles = ambiguities_cycles.get(&ephemeris.sat).copied().unwrap_or(0.0);
            let timing = ObsSignalTiming {
                signal_travel_time_s: Seconds(travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-travel_time_s),
            };
            ObsSatellite {
                signal_id: SigId {
                    sat: ephemeris.sat,
                    band: SignalBand::L1,
                    code: bijux_gnss_core::api::SignalCode::Ca,
                },
                pseudorange_m: bijux_gnss_core::api::Meters(pseudorange_m),
                pseudorange_var_m2: 4.0e-4,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(
                    range_m / wavelength_m + ambiguity_cycles,
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
                    tracking_mode: "synthetic".to_string(),
                    integration_ms: 1,
                    lock_quality: 48.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: SignalSpec {
                        constellation: Constellation::Gps,
                        band: SignalBand::L1,
                        code: bijux_gnss_core::api::SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
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

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) =
        ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::{
        centimeter_level_rtk_baseline_budget, clean_gps_l1_short_baseline_case,
        rtk_baseline_accuracy,
    };

    #[test]
    fn clean_gps_l1_short_baseline_case_builds_consistent_differences() {
        let scenario = clean_gps_l1_short_baseline_case();

        assert_eq!(scenario.single_differences.len(), 5);
        assert_eq!(scenario.double_differences.len(), 4);
        assert!(scenario
            .double_differences
            .iter()
            .all(|observation| observation.ref_sig == scenario.reference_sig));
        assert!(scenario
            .double_differences
            .iter()
            .all(|observation| scenario.rover_ambiguities_cycles.contains_key(&observation.sig.sat)));
    }

    #[test]
    fn rtk_baseline_accuracy_reports_axis_and_norm_errors() {
        let accuracy = rtk_baseline_accuracy([1.01, 1.98, 3.005], [1.0, 2.0, 3.0]);

        assert!((accuracy.east_error_m - 0.01).abs() < 1.0e-12);
        assert!((accuracy.north_error_m + 0.02).abs() < 1.0e-12);
        assert!((accuracy.up_error_m - 0.005).abs() < 1.0e-12);
        assert!((accuracy.horizontal_error_m - (0.01_f64 * 0.01 + 0.02 * 0.02).sqrt()).abs() < 1.0e-12);
        assert!((accuracy.max_axis_error_m - 0.02).abs() < 1.0e-12);
    }

    #[test]
    fn centimeter_level_budget_accepts_small_clean_errors() {
        let budget = centimeter_level_rtk_baseline_budget();
        let accuracy = rtk_baseline_accuracy([8.505, -4.243, 1.764], [8.5, -4.25, 1.75]);

        assert!(accuracy.satisfies(budget), "{accuracy:?}");
    }
}
