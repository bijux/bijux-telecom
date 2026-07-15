#![allow(missing_docs)]

use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, FreqHz, GlonassFrequencyChannel, GpsTime, LockFlags,
    MeasurementErrorModel, Meters, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand,
    SignalSpec, GLONASS_L1_CARRIER_HZ, GLONASS_L1_CHANNEL_SPACING_HZ,
};
use bijux_gnss_nav::api::{
    choose_rtk_single_difference_reference_signals_by_constellation, geodetic_to_ecef,
    rtk_single_difference_code_covariance_matrix, rtk_single_difference_residual_metrics,
    rtk_single_differences_from_aligned_obs_epochs,
    rtk_single_differences_from_aligned_obs_epochs_with_covariance,
    rtk_single_differences_from_obs_epochs, sat_state_gps_l1ca_at_receive_time, GpsEphemeris,
    RtkDifferencedCovarianceConfig, RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
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

fn make_simple_epoch(
    prn: u8,
    constellation: Constellation,
    pseudorange_m: f64,
    cn0_dbhz: f64,
) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
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
        }],
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn glonass_l1_carrier_hz(channel: GlonassFrequencyChannel) -> FreqHz {
    FreqHz::new(
        GLONASS_L1_CARRIER_HZ.value()
            + f64::from(channel.value()) * GLONASS_L1_CHANNEL_SPACING_HZ.value(),
    )
}

#[test]
fn rtk_single_difference_builder_propagates_measurement_delta_and_variance() {
    let base = make_simple_epoch(7, Constellation::Gps, 20_000_100.0, 42.0);
    let rover = make_simple_epoch(7, Constellation::Gps, 20_000_250.0, 42.0);

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);

    assert_eq!(observations.len(), 1);
    assert!((observations[0].code_m - 150.0).abs() < 1.0e-9);
    assert!(observations[0].code_variance_m2.is_finite() && observations[0].code_variance_m2 > 0.0);
    assert_eq!(observations[0].ambiguity_rover.signal, "L1");
}

#[test]
fn rtk_single_difference_builder_preserves_glonass_fdma_channel_evidence() {
    let channel = GlonassFrequencyChannel::new(-4).expect("valid GLONASS channel");
    let mut base = make_simple_epoch(8, Constellation::Glonass, 20_000_100.0, 42.0);
    let mut rover = make_simple_epoch(8, Constellation::Glonass, 20_000_250.0, 42.0);
    base.sats[0].metadata.signal.carrier_hz = glonass_l1_carrier_hz(channel);
    rover.sats[0].metadata.signal.carrier_hz = glonass_l1_carrier_hz(channel);

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].glonass_frequency_channel, Some(channel));
    assert!(observations[0].validate_payload().is_empty());
}

#[test]
fn rtk_single_difference_validation_rejects_glonass_without_fdma_channel_evidence() {
    let base = make_simple_epoch(8, Constellation::Glonass, 20_000_100.0, 42.0);
    let rover = make_simple_epoch(8, Constellation::Glonass, 20_000_250.0, 42.0);

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].glonass_frequency_channel, None);
    let diagnostics = observations[0].validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "RTK_SD_GLONASS_CHANNEL_MISSING"));
}

#[test]
fn rtk_single_difference_builder_skips_rejected_or_unlocked_inputs() {
    let mut base = make_simple_epoch(11, Constellation::Gps, 20_000_000.0, 45.0);
    let mut rover = make_simple_epoch(11, Constellation::Gps, 20_000_050.0, 45.0);
    base.sats[0].observation_status = ObservationStatus::Rejected;
    rover.sats[0].lock_flags.carrier_lock = false;

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);

    assert!(observations.is_empty());
}

#[test]
fn rtk_single_difference_builder_refuses_misaligned_epochs() {
    let base = make_simple_epoch(14, Constellation::Gps, 20_000_000.0, 45.0);
    let mut rover = make_simple_epoch(14, Constellation::Gps, 20_000_050.0, 45.0);
    rover.t_rx_s = Seconds(RTK_EPOCH_ALIGNMENT_TOLERANCE_S * 3.0);

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);

    assert!(observations.is_empty());
}

#[test]
fn rtk_single_difference_builder_records_bounded_epoch_alignment() {
    let base = make_simple_epoch(15, Constellation::Gps, 20_000_000.0, 45.0);
    let mut rover = make_simple_epoch(15, Constellation::Gps, 20_000_050.0, 45.0);
    rover.t_rx_s = Seconds(0.0002);

    let observations = rtk_single_differences_from_aligned_obs_epochs(&base, &rover, 0.00025);

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].epoch_alignment.base_receive_time_s, 0.0);
    assert_eq!(observations[0].epoch_alignment.rover_receive_time_s, 0.0002);
    assert!((observations[0].epoch_alignment.delta_s - 0.0002).abs() < 1.0e-12);
    assert_eq!(observations[0].epoch_alignment.tolerance_s, 0.00025);
}

#[test]
fn rtk_single_difference_covariance_matrix_matches_direct_transformation() {
    let mut base = make_simple_epoch(3, Constellation::Gps, 20_000_000.0, 46.0);
    base.role = ReceiverRole::Base;
    let mut extra_base = make_simple_epoch(7, Constellation::Gps, 20_100_000.0, 42.0);
    base.sats.push(extra_base.sats.remove(0));
    let mut rover = make_simple_epoch(3, Constellation::Gps, 20_000_120.0, 46.0);
    rover.role = ReceiverRole::Rover;
    let mut extra_rover = make_simple_epoch(7, Constellation::Gps, 20_100_170.0, 42.0);
    rover.sats.push(extra_rover.sats.remove(0));
    for sat in &mut base.sats {
        sat.pseudorange_var_m2 = 9.0;
        sat.error_model = Some(shared_clock_model(1.0));
    }
    for sat in &mut rover.sats {
        sat.pseudorange_var_m2 = 4.0;
        sat.error_model = Some(shared_clock_model(2.0));
    }

    let observations = rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        &base,
        &rover,
        RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
        RtkDifferencedCovarianceConfig {
            rover_base_code_correlation: 0.5,
            shared_environment_code_m2: 0.25,
            ..RtkDifferencedCovarianceConfig::default()
        },
    );
    let covariance =
        rtk_single_difference_code_covariance_matrix(&observations).expect("SD covariance");

    assert_eq!(observations.len(), 2);
    let rover_base_covariance_m2 = 0.5 * (4.0_f64 * 9.0).sqrt();
    let expected_variance_m2 = 4.0 + 9.0 - 2.0 * rover_base_covariance_m2;
    assert!((covariance[0][0] - expected_variance_m2).abs() < 1.0e-12);
    assert!((covariance[1][1] - expected_variance_m2).abs() < 1.0e-12);
    assert!((covariance[0][1] - 5.25).abs() < 1.0e-12);
    assert_eq!(covariance[0][1], covariance[1][0]);
}

#[test]
fn rtk_single_difference_covariance_matrix_refuses_invalid_evidence() {
    let mut base = make_simple_epoch(3, Constellation::Gps, 20_000_000.0, 46.0);
    base.role = ReceiverRole::Base;
    let mut rover = make_simple_epoch(3, Constellation::Gps, 20_000_120.0, 46.0);
    rover.role = ReceiverRole::Rover;
    base.sats[0].pseudorange_var_m2 = 9.0;
    rover.sats[0].pseudorange_var_m2 = 4.0;

    let mut observations = rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        &base,
        &rover,
        RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
        RtkDifferencedCovarianceConfig::default(),
    );
    observations[0].covariance_evidence.rover_base_code_covariance_m2 = 7.0;

    let diagnostics = observations[0].validate_payload();
    assert!(diagnostics.iter().any(|event| event.code == "RTK_SD_COVARIANCE_EVIDENCE_INVALID"));
    assert!(rtk_single_difference_code_covariance_matrix(&observations).is_none());
}

#[test]
fn rtk_single_difference_residuals_use_carried_satellite_timing() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 12.0);
    let base_ecef_m = [base.0, base.1, base.2];
    let rover_ecef_m = [rover.0, rover.1, rover.2];
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
    let rover_epoch =
        make_obs_epoch(ReceiverRole::Rover, receive_gps_time, rover_ecef_m, &ephemerides);
    let timed = rtk_single_differences_from_obs_epochs(&base_epoch, &rover_epoch);
    assert!(!timed.is_empty());
    assert!(timed.iter().all(|observation| observation.rover_signal_timing.is_some()
        && observation.base_signal_timing.is_some()));

    let timed_metrics = rtk_single_difference_residual_metrics(
        &timed,
        base_ecef_m,
        rover_ecef_m,
        &ephemerides,
        receive_gps_time.tow_s,
        receive_gps_time.tow_s,
    )
    .expect("timed single-difference residual metrics");

    let mut uncorrected = timed.clone();
    for observation in &mut uncorrected {
        observation.rover_signal_timing = None;
        observation.base_signal_timing = None;
        observation.rover_pseudorange_m = 1.0;
        observation.base_pseudorange_m = 1.0;
    }
    let uncorrected_metrics = rtk_single_difference_residual_metrics(
        &uncorrected,
        base_ecef_m,
        rover_ecef_m,
        &ephemerides,
        receive_gps_time.tow_s,
        receive_gps_time.tow_s,
    )
    .expect("uncorrected single-difference residual metrics");

    assert!(
        timed_metrics.residual_rms_m < 1.0e-3,
        "timed single-difference residual RMS {:.6} m should stay close to truth",
        timed_metrics.residual_rms_m
    );
    assert_eq!(timed_metrics.used_observations, timed.len());
    assert!(
        uncorrected_metrics.residual_rms_m > timed_metrics.residual_rms_m * 5.0
            && uncorrected_metrics.residual_rms_m > timed_metrics.residual_rms_m + 5.0e-5,
        "dropping carried timing should measurably worsen SD residuals (timed={:.6}, uncorrected={:.6})",
        timed_metrics.residual_rms_m,
        uncorrected_metrics.residual_rms_m
    );
}

fn shared_clock_model(clock_error_m: f64) -> MeasurementErrorModel {
    MeasurementErrorModel {
        thermal_noise_m: Meters(0.0),
        tracking_jitter_m: Meters(0.0),
        multipath_proxy_m: Meters(0.0),
        clock_error_m: Meters(clock_error_m),
    }
}

#[test]
fn rtk_single_difference_reference_selection_stays_per_constellation() {
    let base_gps = make_simple_epoch(3, Constellation::Gps, 20_000_000.0, 45.0);
    let rover_gps = make_simple_epoch(3, Constellation::Gps, 20_000_120.0, 45.0);
    let base_galileo = make_simple_epoch(11, Constellation::Galileo, 24_000_000.0, 35.0);
    let rover_galileo = make_simple_epoch(11, Constellation::Galileo, 24_000_150.0, 35.0);

    let mut base = base_gps.clone();
    base.sats.extend(base_galileo.sats.clone());
    let mut rover = rover_gps.clone();
    rover.sats.extend(rover_galileo.sats.clone());

    let observations = rtk_single_differences_from_obs_epochs(&base, &rover);
    let references = choose_rtk_single_difference_reference_signals_by_constellation(&observations);

    assert_eq!(references.len(), 2);
    assert_eq!(references.get(&Constellation::Gps).expect("gps reference").sat.prn, 3);
    assert_eq!(references.get(&Constellation::Galileo).expect("galileo reference").sat.prn, 11);
}
