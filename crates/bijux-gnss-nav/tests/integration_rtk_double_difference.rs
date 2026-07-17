#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, FreqHz, GlonassFrequencyChannel, GpsTime, LockFlags,
    ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalSpec, GLONASS_L1_CARRIER_HZ,
    GLONASS_L1_CHANNEL_SPACING_HZ,
};
use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signal,
    choose_rtk_single_difference_reference_signals_by_constellation, ecef_to_enu, ecef_to_geodetic,
    geodetic_to_ecef, rtk_apply_glonass_inter_frequency_bias_calibrations,
    rtk_double_difference_code_covariance_matrix, rtk_double_difference_residual_metrics,
    rtk_double_differences_by_constellation, rtk_double_differences_from_single_differences,
    rtk_glonass_inter_frequency_bias_is_integer_compatible,
    rtk_single_differences_from_aligned_obs_epochs_with_covariance,
    rtk_single_differences_from_obs_epochs, rtk_switch_double_difference_reference,
    sat_state_gps_l1ca_at_receive_time, GpsEphemeris, RtkDifferencedCovarianceConfig,
    RtkEpochAlignmentEvidence, RtkGlonassInterFrequencyBiasCalibration,
    RtkGlonassInterFrequencyBiasStatus, RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn make_eph(prn: u8, omega0: f64, m0: f64, toe_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 2200,
        sv_health: 0,
        sv_accuracy: Some(2),
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

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn make_obs_epoch(
    role: ReceiverRole,
    receive_gps_time: GpsTime,
    receiver_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
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
                pseudorange_var_m2: 4.0,
                carrier_phase_cycles: bijux_gnss_core::api::Cycles(range_m / wavelength_m),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::api::Hertz(0.0),
                doppler_var_hz2: 4.0,
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
                timing: Some(timing),
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "synthetic".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
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

fn make_satellite(
    constellation: Constellation,
    prn: u8,
    pseudorange_m: f64,
    cn0_dbhz: f64,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId {
            sat: SatId { constellation, prn },
            band: SignalBand::L1,
            code: bijux_gnss_core::api::SignalCode::Ca,
        },
        pseudorange_m: bijux_gnss_core::api::Meters(pseudorange_m),
        pseudorange_var_m2: 0.0,
        carrier_phase_cycles: bijux_gnss_core::api::Cycles(1_000.0 + prn as f64),
        carrier_phase_var_cycles2: 0.0,
        doppler_hz: bijux_gnss_core::api::Hertz(-500.0),
        doppler_var_hz2: 4.0,
        cn0_dbhz,
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
            tracking_mode: "synthetic".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal: SignalSpec {
                constellation,
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
                code_rate_hz: 1_023_000.0,
                carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
            },
            ..ObsMetadata::default()
        },
    }
}

fn glonass_l1_carrier_hz(channel: GlonassFrequencyChannel) -> FreqHz {
    FreqHz::new(
        GLONASS_L1_CARRIER_HZ.value()
            + f64::from(channel.value()) * GLONASS_L1_CHANNEL_SPACING_HZ.value(),
    )
}

fn make_glonass_satellite(
    prn: u8,
    channel: GlonassFrequencyChannel,
    pseudorange_m: f64,
    cn0_dbhz: f64,
) -> ObsSatellite {
    let mut satellite = make_satellite(Constellation::Glonass, prn, pseudorange_m, cn0_dbhz);
    satellite.metadata.signal.carrier_hz = glonass_l1_carrier_hz(channel);
    satellite
}

fn make_epoch(role: ReceiverRole, sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
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

#[test]
fn rtk_double_difference_builder_uses_reference_satellite_deltas() {
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_000.0, 40.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_215.0, 40.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);

    assert_eq!(reference.sat.prn, 3);
    assert_eq!(double_differences.len(), 2);
    assert!(double_differences.iter().all(|observation| observation.ref_sig == reference));
    assert!(double_differences
        .iter()
        .any(|observation| observation.sig.sat.prn == 7
            && (observation.code_m - 50.0).abs() < 1.0e-9));
    assert!(double_differences.iter().all(|observation| observation.canceled.len() == 4));
}

#[test]
fn rtk_double_difference_builder_marks_same_channel_glonass_bias_handled() {
    let channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_glonass_satellite(3, channel, 20_000_000.0, 46.0),
            make_glonass_satellite(7, channel, 20_100_000.0, 42.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_glonass_satellite(3, channel, 20_000_120.0, 46.0),
            make_glonass_satellite(7, channel, 20_100_170.0, 42.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);

    assert_eq!(double_differences.len(), 1);
    assert_eq!(
        double_differences[0].glonass_inter_frequency_bias.status,
        RtkGlonassInterFrequencyBiasStatus::BiasHandled
    );
    assert!(rtk_glonass_inter_frequency_bias_is_integer_compatible(&double_differences[0]));
    assert!(double_differences[0].validate_payload().is_empty());
}

#[test]
fn rtk_double_difference_builder_requires_calibration_for_mixed_glonass_channels() {
    let reference_channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let signal_channel = GlonassFrequencyChannel::new(5).expect("valid GLONASS channel");
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_glonass_satellite(3, reference_channel, 20_000_000.0, 46.0),
            make_glonass_satellite(7, signal_channel, 20_100_000.0, 42.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_glonass_satellite(3, reference_channel, 20_000_120.0, 46.0),
            make_glonass_satellite(7, signal_channel, 20_100_170.0, 42.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);

    assert_eq!(double_differences.len(), 1);
    assert_eq!(
        double_differences[0].glonass_inter_frequency_bias.status,
        RtkGlonassInterFrequencyBiasStatus::CalibrationRequired
    );
    assert!(!rtk_glonass_inter_frequency_bias_is_integer_compatible(&double_differences[0]));
    let diagnostics = double_differences[0].validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_GLONASS_BIAS_UNHANDLED"));
}

#[test]
fn rtk_double_difference_glonass_calibration_corrects_code_and_phase_biases() {
    let reference_channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let signal_channel = GlonassFrequencyChannel::new(5).expect("valid GLONASS channel");
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_glonass_satellite(3, reference_channel, 20_000_000.0, 46.0),
            make_glonass_satellite(7, signal_channel, 20_100_000.0, 42.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_glonass_satellite(3, reference_channel, 20_000_120.0, 46.0),
            make_glonass_satellite(7, signal_channel, 20_100_170.0, 42.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);
    let uncorrected = &double_differences[0];

    let corrected = rtk_apply_glonass_inter_frequency_bias_calibrations(
        &double_differences,
        &[RtkGlonassInterFrequencyBiasCalibration {
            signal_channel,
            reference_channel,
            code_bias_m: 2.5,
            phase_bias_cycles: -0.25,
        }],
    );

    assert_eq!(corrected.len(), 1);
    assert!((corrected[0].code_m - (uncorrected.code_m - 2.5)).abs() < 1.0e-12);
    assert!((corrected[0].phase_cycles - (uncorrected.phase_cycles + 0.25)).abs() < 1.0e-12);
    assert_eq!(
        corrected[0].glonass_inter_frequency_bias.status,
        RtkGlonassInterFrequencyBiasStatus::BiasHandled
    );
    assert_eq!(corrected[0].glonass_inter_frequency_bias.code_bias_m, 2.5);
    assert_eq!(corrected[0].glonass_inter_frequency_bias.phase_bias_cycles, -0.25);
    assert!(rtk_glonass_inter_frequency_bias_is_integer_compatible(&corrected[0]));
    assert!(corrected[0].validate_payload().is_empty());
}

#[test]
fn rtk_double_difference_builder_rejects_glonass_without_channel_evidence() {
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Glonass, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Glonass, 7, 20_100_000.0, 42.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Glonass, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Glonass, 7, 20_100_170.0, 42.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);

    assert_eq!(double_differences.len(), 1);
    assert_eq!(
        double_differences[0].glonass_inter_frequency_bias.status,
        RtkGlonassInterFrequencyBiasStatus::ChannelEvidenceMissing
    );
    assert!(!rtk_glonass_inter_frequency_bias_is_integer_compatible(&double_differences[0]));
    let diagnostics = double_differences[0].validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_GLONASS_BIAS_UNHANDLED"));
}

#[test]
fn rtk_double_difference_builder_refuses_mixed_epoch_alignment_evidence() {
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
        ],
    );

    let mut single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    for observation in &mut single_differences {
        if observation.sig != reference {
            observation.epoch_alignment = RtkEpochAlignmentEvidence {
                base_receive_time_s: 0.0,
                rover_receive_time_s: 0.0001,
                delta_s: 0.0001,
                tolerance_s: 0.0005,
            };
        }
    }

    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);

    assert!(double_differences.is_empty());
}

#[test]
fn rtk_double_difference_covariance_matrix_matches_direct_transformation() {
    let mut base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_000.0, 40.0),
        ],
    );
    let mut rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_215.0, 40.0),
        ],
    );
    for sat in &mut base.sats {
        sat.pseudorange_var_m2 = 9.0;
    }
    for sat in &mut rover.sats {
        sat.pseudorange_var_m2 = 4.0;
    }

    let single_differences = rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        &base,
        &rover,
        RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
        RtkDifferencedCovarianceConfig {
            rover_base_code_correlation: 0.5,
            shared_environment_code_m2: 5.25,
            ..RtkDifferencedCovarianceConfig::default()
        },
    );
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);
    let covariance =
        rtk_double_difference_code_covariance_matrix(&double_differences).expect("DD covariance");

    assert_eq!(double_differences.len(), 2);
    assert!((covariance[0][0] - 3.5).abs() < 1.0e-12);
    assert!((covariance[1][1] - 3.5).abs() < 1.0e-12);
    assert!((covariance[0][1] - 1.75).abs() < 1.0e-12);
    assert_eq!(covariance[0][1], covariance[1][0]);
}

#[test]
fn rtk_double_difference_covariance_matrix_refuses_invalid_evidence() {
    let mut base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_000.0, 40.0),
        ],
    );
    let mut rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_215.0, 40.0),
        ],
    );
    for sat in &mut base.sats {
        sat.pseudorange_var_m2 = 9.0;
    }
    for sat in &mut rover.sats {
        sat.pseudorange_var_m2 = 4.0;
    }

    let single_differences = rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        &base,
        &rover,
        RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
        RtkDifferencedCovarianceConfig::default(),
    );
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let mut double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);
    double_differences[0].covariance_evidence.signal.rover_base_code_covariance_m2 = 7.0;

    let diagnostics = double_differences[0].validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "RTK_DD_COVARIANCE_EVIDENCE_INVALID"));
    assert!(rtk_double_difference_code_covariance_matrix(&double_differences).is_none());
}

#[test]
fn rtk_double_difference_reference_switch_preserves_measurements_and_covariance() {
    let mut base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_000.0, 40.0),
        ],
    );
    let mut rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
            make_satellite(Constellation::Gps, 11, 20_200_215.0, 40.0),
        ],
    );
    for sat in &mut base.sats {
        sat.pseudorange_var_m2 = 9.0;
    }
    for sat in &mut rover.sats {
        sat.pseudorange_var_m2 = 4.0;
    }

    let single_differences = rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        &base,
        &rover,
        RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
        RtkDifferencedCovarianceConfig {
            rover_base_code_correlation: 0.5,
            shared_environment_code_m2: 5.25,
            ..RtkDifferencedCovarianceConfig::default()
        },
    );
    let old_reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let old_double_differences =
        rtk_double_differences_from_single_differences(&single_differences, old_reference);
    let new_reference = single_differences
        .iter()
        .find(|observation| observation.sig.sat.prn == 7)
        .expect("new reference")
        .sig;

    let switched = rtk_switch_double_difference_reference(&old_double_differences, new_reference)
        .expect("switched reference");
    let covariance = rtk_double_difference_code_covariance_matrix(&switched).expect("covariance");

    assert_eq!(switched.len(), 2);
    assert_eq!(switched[0].sig.sat.prn, 3);
    assert_eq!(switched[0].ref_sig, new_reference);
    assert!((switched[0].code_m + 50.0).abs() < 1.0e-12);
    assert_eq!(switched[1].sig.sat.prn, 11);
    assert!((switched[1].code_m - 45.0).abs() < 1.0e-12);
    assert!((covariance[0][0] - 3.5).abs() < 1.0e-12);
    assert!((covariance[1][1] - 3.5).abs() < 1.0e-12);
    assert!((covariance[0][1] - 1.75).abs() < 1.0e-12);
}

#[test]
fn rtk_double_difference_builder_scopes_references_by_constellation() {
    let base = make_epoch(
        ReceiverRole::Base,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_000.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_000.0, 42.0),
            make_satellite(Constellation::Galileo, 11, 24_000_000.0, 47.0),
            make_satellite(Constellation::Galileo, 19, 24_100_000.0, 41.0),
        ],
    );
    let rover = make_epoch(
        ReceiverRole::Rover,
        vec![
            make_satellite(Constellation::Gps, 3, 20_000_120.0, 46.0),
            make_satellite(Constellation::Gps, 7, 20_100_170.0, 42.0),
            make_satellite(Constellation::Galileo, 11, 24_000_090.0, 47.0),
            make_satellite(Constellation::Galileo, 19, 24_100_165.0, 41.0),
        ],
    );

    let single_differences = rtk_single_differences_from_obs_epochs(&base, &rover);
    let references =
        choose_rtk_single_difference_reference_signals_by_constellation(&single_differences);
    let double_differences =
        rtk_double_differences_by_constellation(&single_differences, &references);
    let refs_by_constellation: BTreeMap<Constellation, u8> =
        references.iter().map(|(constellation, sig)| (*constellation, sig.sat.prn)).collect();

    assert_eq!(refs_by_constellation.get(&Constellation::Gps), Some(&3));
    assert_eq!(refs_by_constellation.get(&Constellation::Galileo), Some(&11));
    assert_eq!(double_differences.len(), 2);
    assert!(double_differences
        .iter()
        .any(|observation| observation.sig.sat.constellation == Constellation::Gps
            && observation.ref_sig.sat.prn == 3));
    assert!(double_differences
        .iter()
        .any(|observation| observation.sig.sat.constellation == Constellation::Galileo
            && observation.ref_sig.sat.prn == 11));
}

#[test]
fn rtk_double_difference_residuals_match_synthetic_baseline_truth() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(base.0, base.1, base.2);
    let rover_enu_m = {
        let (east_m, north_m, up_m) =
            ecef_to_enu(rover.0, rover.1, rover.2, lat_deg, lon_deg, alt_m);
        [east_m, north_m, up_m]
    };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.08 };
    let ephemerides = vec![
        make_eph(1, 0.0, 0.0, 345_600.0),
        make_eph(2, 0.8, 0.9, 345_600.0),
        make_eph(3, 1.6, 1.8, 345_600.0),
        make_eph(4, 2.4, 2.7, 345_600.0),
        make_eph(5, 3.2, 3.6, 345_600.0),
    ];

    let base_epoch =
        make_obs_epoch(ReceiverRole::Base, receive_gps_time, base_ecef_m, &ephemerides);
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        [rover.0, rover.1, rover.2],
        &ephemerides,
    );
    let single_differences = rtk_single_differences_from_obs_epochs(&base_epoch, &rover_epoch);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let timed = rtk_double_differences_from_single_differences(&single_differences, reference);
    let timed_metrics = rtk_double_difference_residual_metrics(
        &timed,
        base_ecef_m,
        rover_enu_m,
        &ephemerides,
        receive_gps_time.tow_s,
    )
    .expect("timed double-difference residual metrics");

    assert!(!timed.is_empty());
    assert!(
        timed_metrics.residual_rms_m < 1.0e-3,
        "double-difference residual RMS {:.6} m should stay close to synthetic truth",
        timed_metrics.residual_rms_m
    );
    assert_eq!(timed_metrics.used_observations, timed.len());
}

#[test]
fn rtk_double_difference_residuals_use_aligned_base_and_rover_receive_times() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(base.0, base.1, base.2);
    let rover_enu_m = {
        let (east_m, north_m, up_m) =
            ecef_to_enu(rover.0, rover.1, rover.2, lat_deg, lon_deg, alt_m);
        [east_m, north_m, up_m]
    };
    let base_receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.08 };
    let rover_receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.080_2 };
    let ephemerides = vec![
        make_eph(1, 0.0, 0.0, 345_600.0),
        make_eph(2, 0.8, 0.9, 345_600.0),
        make_eph(3, 1.6, 1.8, 345_600.0),
        make_eph(4, 2.4, 2.7, 345_600.0),
        make_eph(5, 3.2, 3.6, 345_600.0),
    ];

    let base_epoch =
        make_obs_epoch(ReceiverRole::Base, base_receive_gps_time, base_ecef_m, &ephemerides);
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        rover_receive_gps_time,
        [rover.0, rover.1, rover.2],
        &ephemerides,
    );
    let single_differences = rtk_single_differences_from_obs_epochs(&base_epoch, &rover_epoch);
    assert!(!single_differences.is_empty());
    assert!(single_differences.iter().all(|observation| observation.epoch_alignment.delta_s > 0.0));
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let double_differences =
        rtk_double_differences_from_single_differences(&single_differences, reference);
    assert!(!double_differences.is_empty());

    let metrics = rtk_double_difference_residual_metrics(
        &double_differences,
        base_ecef_m,
        rover_enu_m,
        &ephemerides,
        base_receive_gps_time.tow_s,
    )
    .expect("double-difference residual metrics");

    assert!(
        metrics.residual_rms_m < 1.0e-3,
        "double-difference residual RMS {:.6} m should use aligned base/rover times",
        metrics.residual_rms_m
    );
    assert_eq!(metrics.used_observations, double_differences.len());
}

#[test]
fn rtk_double_difference_residuals_use_carried_satellite_timing() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(base.0, base.1, base.2);
    let rover_enu_m = {
        let (east_m, north_m, up_m) =
            ecef_to_enu(rover.0, rover.1, rover.2, lat_deg, lon_deg, alt_m);
        [east_m, north_m, up_m]
    };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.08 };
    let ephemerides = vec![
        make_eph(1, 0.0, 0.0, 345_600.0),
        make_eph(2, 0.8, 0.9, 345_600.0),
        make_eph(3, 1.6, 1.8, 345_600.0),
        make_eph(4, 2.4, 2.7, 345_600.0),
        make_eph(5, 3.2, 3.6, 345_600.0),
    ];

    let base_epoch =
        make_obs_epoch(ReceiverRole::Base, receive_gps_time, base_ecef_m, &ephemerides);
    let rover_epoch = make_obs_epoch(
        ReceiverRole::Rover,
        receive_gps_time,
        [rover.0, rover.1, rover.2],
        &ephemerides,
    );
    let single_differences = rtk_single_differences_from_obs_epochs(&base_epoch, &rover_epoch);
    let reference =
        choose_rtk_single_difference_reference_signal(&single_differences).expect("reference");
    let timed = rtk_double_differences_from_single_differences(&single_differences, reference);

    let timed_metrics = rtk_double_difference_residual_metrics(
        &timed,
        base_ecef_m,
        rover_enu_m,
        &ephemerides,
        receive_gps_time.tow_s,
    )
    .expect("timed double-difference residual metrics");

    let mut uncorrected = timed.clone();
    for observation in &mut uncorrected {
        observation.rover_signal_timing = None;
        observation.base_signal_timing = None;
        observation.rover_ref_signal_timing = None;
        observation.base_ref_signal_timing = None;
        observation.rover_signal_pseudorange_m = 1.0;
        observation.base_signal_pseudorange_m = 1.0;
        observation.rover_ref_pseudorange_m = 1.0;
        observation.base_ref_pseudorange_m = 1.0;
    }
    let uncorrected_metrics = rtk_double_difference_residual_metrics(
        &uncorrected,
        base_ecef_m,
        rover_enu_m,
        &ephemerides,
        receive_gps_time.tow_s,
    )
    .expect("uncorrected double-difference residual metrics");

    assert!(
        uncorrected_metrics.residual_rms_m > timed_metrics.residual_rms_m + 1.0e-4,
        "dropping carried timing should measurably worsen DD residuals (timed={:.6}, uncorrected={:.6})",
        timed_metrics.residual_rms_m,
        uncorrected_metrics.residual_rms_m
    );
}
