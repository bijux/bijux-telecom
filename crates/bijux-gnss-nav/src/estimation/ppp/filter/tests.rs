use std::collections::{BTreeMap, BTreeSet};

use super::{
    iono_free_antenna_range_correction_m, iono_free_satellite_representatives,
    phase_windup_cycles_for_satellite, ppp_code_sigma_m, ppp_common_range_sigma_m,
    ppp_indices_from_state_identities, ppp_measurement_observations, ppp_phase_sigma_cycles,
    ppp_state_label, ppp_stochastic_evidence_from_config, resolved_code_bias_m,
    resolved_iono_free_code_bias_m, single_frequency_antenna_range_correction_m, PppFilter,
    PppPhaseBreaks, SPEED_OF_LIGHT_MPS,
};
use crate::api::{
    ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, BroadcastProductsProvider,
    GpsEphemeris, GpsSatState, GpsSatelliteClockCorrection, OceanTideConstituent,
    OceanTideLoadingConstituent, OceanTideLoadingModel, PppConfig, ProductDiagnostics,
    ProductsProvider, SatelliteClockUncertaintySource, SatelliteHealthSource,
    SatelliteHealthStatus, SatelliteOrbitUncertaintySource, SatelliteStateUncertainty,
    SolidEarthTideModel,
};
use crate::corrections::biases::{
    CodeBias, CodeBiasProvider, PhaseBias, SignalCodeBiases, SignalPhaseBiases, ZeroBiases,
};
use crate::corrections::phase_windup::PhaseWindupState;
use crate::estimation::ppp::config::{
    PppArMode, PppIntegerAmbiguityKind, PppLifecycleEventKind, PppMeasurementNoise,
    PppPreciseProductAction, PppProductSupport, PppStateIdentity, WlAmbiguity,
};
use crate::estimation::ppp::measurements::iono_free_code_observation_from_obs;
use crate::formats::precise_products::{
    PreciseProductDiscontinuity, PreciseProductDiscontinuityKind, PreciseProductSurface,
};
use crate::models::antenna::{
    AntennaPhaseCenterVariation, ReceiverAntennaCalibration, ReceiverAntennaCalibrations,
    ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
    SatellitePhaseCenterOffset,
};
use crate::models::atmosphere::SaastamoinenModel;
use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, Llh, LockFlags, Meters, ObsEpoch, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_wavelength_m};

#[path = "tests/ambiguity_resolution.rs"]
mod ambiguity_resolution;
#[path = "tests/precise_clock_selection.rs"]
mod precise_clock_selection;
#[path = "tests/precise_product_policy.rs"]
mod precise_product_policy;
#[path = "tests/state_lifecycle.rs"]
mod state_lifecycle;
#[path = "tests/static_tide_models.rs"]
mod static_tide_models;

fn make_eph(prn: u8) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.02,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.1,
        omegadot: 0.0,
        w: 0.2,
        m0: 0.3,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 1.0e-4,
        af1: -2.0e-12,
        af2: 3.0e-20,
        tgd: 8.0e-9,
    }
}

fn ppp_test_satellite(sat: SatId) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
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
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    }
}

fn ppp_test_signal_satellite(sat: SatId, band: SignalBand, code: SignalCode) -> ObsSatellite {
    let mut observation = ppp_test_satellite(sat);
    observation.signal_id = SigId { sat, band, code };
    observation.metadata.signal = match (band, code) {
        (SignalBand::L1, SignalCode::Ca) => signal_spec_gps_l1_ca(),
        (SignalBand::L2, SignalCode::Py) => signal_spec_gps_l2_py(),
        _ => observation.metadata.signal,
    };
    observation
}

fn ppp_test_epoch(sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(2200),
        tow_s: Some(Seconds(0.0)),
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn ppp_solution_epoch_exposes_ecef_position_covariance() {
    let mut filter = PppFilter::new(PppConfig::default());
    filter.seed_receiver_state([6_378_137.0, 10.0, 10.0], 4.0e-6);
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[0])] = 4.0;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[1])] = 9.0;
    filter.ekf.p[(filter.indices.pos[2], filter.indices.pos[2])] = 16.0;
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[1])] = 0.5;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[0])] = 0.5;

    let solution = filter.solution_epoch(
        7,
        7.0,
        Vec::new(),
        0,
        ppp_stochastic_evidence_from_config(&filter.config),
    );
    let covariance =
        solution.position_covariance_ecef_m2.expect("PPP solution should emit position covariance");

    assert_eq!(covariance[0][0], 4.0);
    assert_eq!(covariance[1][1], 9.0);
    assert_eq!(covariance[2][2], 16.0);
    assert_eq!(covariance[0][1], 0.5);
    assert_eq!(covariance[1][0], 0.5);
    assert!(solution.sigma_e_m.expect("east sigma").is_finite());
    assert!(solution.sigma_n_m.expect("north sigma").is_finite());
    assert!(solution.sigma_u_m.expect("up sigma").is_finite());
    assert!(solution
        .horizontal_error_ellipse_major_axis_m
        .expect("ellipse major axis")
        .is_finite());
    assert!(solution
        .horizontal_error_ellipse_minor_axis_m
        .expect("ellipse minor axis")
        .is_finite());
    assert!(solution.horizontal_error_ellipse_azimuth_deg.expect("ellipse azimuth").is_finite());
}

#[test]
fn iono_free_mode_keeps_one_representative_per_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
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
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let sats = vec![&l1, &l2];

    let representatives = iono_free_satellite_representatives(&sats);

    assert_eq!(representatives.len(), 1);
    assert_eq!(representatives[0].signal_id.band, SignalBand::L1);
}

#[test]
fn ppp_filter_resolves_iono_free_code_bias_from_both_signals() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_200_010.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
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
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        pseudorange_m: Meters(20_200_005.0),
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let obs = ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![l1, l2],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };
    let measurement = iono_free_code_observation_from_obs(&obs, sat).expect("iono-free code");
    let bias_table = SignalCodeBiases::from_biases([
        CodeBias { sig: measurement.signal_1, bias_m: 2.0 },
        CodeBias { sig: measurement.signal_2, bias_m: -0.5 },
    ]);

    let resolved_bias_m = resolved_iono_free_code_bias_m(&bias_table, measurement);

    assert!((resolved_bias_m - 2.0).abs() > 1.0e-3);
    assert!(resolved_bias_m.is_finite());
}

#[test]
fn ppp_filter_resolves_single_frequency_code_bias_at_epoch_time() {
    struct TimeAwareBiasProvider;

    impl CodeBiasProvider for TimeAwareBiasProvider {
        fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
            None
        }

        fn code_bias_m_at(
            &self,
            _sig: SigId,
            time: Option<bijux_gnss_core::api::GpsTime>,
        ) -> Option<f64> {
            Some(time?.tow_s)
        }
    }

    let signal = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };

    let resolved_bias_m = resolved_code_bias_m(
        &TimeAwareBiasProvider,
        signal,
        Some(bijux_gnss_core::api::GpsTime { week: 0, tow_s: 123.5 }),
    );

    assert!((resolved_bias_m - 123.5).abs() < f64::EPSILON);
}

#[test]
fn ppp_filter_tracks_phase_windup_by_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let (receiver_x_m, receiver_y_m, receiver_z_m) = geodetic_to_ecef(45.0, 12.0, 80.0);
    let receiver_pos_m = [receiver_x_m, receiver_y_m, receiver_z_m];
    let sat_pos_m = [15_600_000.0, -10_200_000.0, 21_700_000.0];
    let mut states = BTreeMap::new();

    let first = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_400.0 }),
    );
    let second = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_430.0 }),
    );

    assert!(first.is_finite());
    assert!(second.is_finite());
    assert!(states.contains_key(&sat));
    assert!((second - first).abs() < 0.5);
}

#[test]
fn ppp_filter_skips_phase_windup_without_epoch_time() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut states = BTreeMap::new();

    let windup_cycles = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        [1.0, 0.0, 0.0],
        [20_200_000.0, 14_000_000.0, 21_700_000.0],
        None,
    );

    assert_eq!(windup_cycles, 0.0);
    assert!(states.is_empty());
}

#[test]
fn ppp_filter_uses_satellite_antenna_calibration_for_single_frequency_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                SatellitePhaseCenterOffset::new(0.12, -0.04, 0.95),
            )]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .range_correction_m(sat, SignalBand::L1, gps_time, sat_pos_m, receiver_pos_m)
        .expect("single-frequency antenna correction");
    let actual = single_frequency_antenna_range_correction_m(
        Some(&calibrations),
        None,
        None,
        sat,
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_includes_phase_variation_for_single_frequency_antenna_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let satellite_calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
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
    };
    let receiver_calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0),
            )]),
            variations_by_band: BTreeMap::from([(
                SignalBand::L1,
                AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
            )]),
        }],
    };

    let actual = single_frequency_antenna_range_correction_m(
        Some(&satellite_calibrations),
        Some(receiver_antenna_type),
        Some(&receiver_calibrations),
        sat,
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!((actual - 0.08).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_satellite_antenna_calibration_for_iono_free_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, SatellitePhaseCenterOffset::new(0.08, 0.01, 0.91)),
                (SignalBand::L2, SatellitePhaseCenterOffset::new(0.14, -0.03, 1.12)),
            ]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .iono_free_range_correction_m(
            sat,
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
        )
        .expect("iono-free antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        Some(&calibrations),
        None,
        None,
        sat,
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_includes_phase_variation_for_iono_free_antenna_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let satellite_calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
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
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
            ]),
            variations_by_band: BTreeMap::from([
                (
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
                ),
                (
                    SignalBand::L2,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.01]),
                ),
            ]),
        }],
    };

    let expected = satellite_calibrations
        .iono_free_range_correction_with_phase_variation_m(
            sat,
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        )
        .expect("satellite antenna correction")
        + receiver_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                receiver_antenna_type,
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
                gps_time,
                receiver_pos_m,
                sat_pos_m,
                85.0,
                None,
            )
            .expect("receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        Some(&satellite_calibrations),
        Some(receiver_antenna_type),
        Some(&receiver_calibrations),
        sat,
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!(expected.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_receiver_antenna_calibration_for_single_frequency_range() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(0.12, -0.04, 0.95),
            )]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .range_correction_m(
            receiver_antenna_type,
            SignalBand::L1,
            gps_time,
            receiver_pos_m,
            sat_pos_m,
        )
        .expect("single-frequency receiver antenna correction");
    let actual = single_frequency_antenna_range_correction_m(
        None,
        Some(receiver_antenna_type),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_receiver_antenna_calibration_for_iono_free_range() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.08, 0.01, 0.91)),
                (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.14, -0.03, 1.12)),
            ]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .iono_free_range_correction_m(
            receiver_antenna_type,
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            receiver_pos_m,
            sat_pos_m,
        )
        .expect("iono-free receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        None,
        Some(receiver_antenna_type),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_receiver_antenna_type_selects_matching_calibration() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![
            ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.05, 0.01, 0.70),
                )]),
                variations_by_band: BTreeMap::new(),
            },
            ReceiverAntennaCalibration {
                antenna_type: "TRM57971.00 NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.25, 0.08, 1.30),
                )]),
                variations_by_band: BTreeMap::new(),
            },
        ],
    };

    let aoad = single_frequency_antenna_range_correction_m(
        None,
        Some("AOAD/M_T NONE"),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );
    let trimble = single_frequency_antenna_range_correction_m(
        None,
        Some("TRM57971.00 NONE"),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!((trimble - aoad).abs() > 1.0e-6);
}
