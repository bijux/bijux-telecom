#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{
    first_order_ionosphere_code_delay_m, geodetic_to_ecef, signal_meters_to_cycles,
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation, Cycles, GpsTime, Hertz,
    LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObsSignalTiming, ObservationEpochDecision, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, gps_broadcast_ionosphere_residuals_from_obs_epochs,
    KlobucharCoefficients,
    parse_rinex_broadcast_navigation, parse_rinex_gps_observation_dataset,
    summarize_broadcast_ionosphere_residuals,
    sat_state_gps_l1ca_at_receive_time, GpsBroadcastNavigationData, GpsEphemeris,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|_| panic!("read fixture {}", path.display()))
}

fn sample_ephemeris() -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        iodc: 1,
        iode: 1,
        week: 2209,
        sv_health: 0,
        toe_s: 504_000.0,
        toc_s: 504_018.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.8,
        omegadot: 0.0,
        w: 0.0,
        m0: 0.9,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 1.0e-4,
        af1: 2.0e-11,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn sample_klobuchar() -> KlobucharCoefficients {
    KlobucharCoefficients::new(
        [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
        [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
    )
}

fn satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    pseudorange_m: f64,
    carrier_phase_cycles: f64,
    timing: ObsSignalTiming,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 0.0,
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
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal,
            ..ObsMetadata::default()
        },
    }
}

#[test]
fn gps_broadcast_ionosphere_residuals_match_synthetic_klobuchar_truth() {
    let receiver_llh = (37.0, -122.0, 15.0);
    let receiver_ecef =
        geodetic_to_ecef(receiver_llh.0, receiver_llh.1, receiver_llh.2);
    let receive_tow_s = 504_018.0;
    let signal_travel_time_s = 0.075;
    let transmit_gps_time = GpsTime { week: 2209, tow_s: receive_tow_s - signal_travel_time_s };
    let timing = ObsSignalTiming { signal_travel_time_s: Seconds(signal_travel_time_s), transmit_gps_time };
    let ephemeris = sample_ephemeris();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: vec![ephemeris.clone()],
        klobuchar: Some(sample_klobuchar()),
    };
    let state =
        sat_state_gps_l1ca_at_receive_time(&ephemeris, receive_tow_s, signal_travel_time_s);
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef.0,
        receiver_ecef.1,
        receiver_ecef.2,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    let geometric_range_m = ((state.x_m - receiver_ecef.0).powi(2)
        + (state.y_m - receiver_ecef.1).powi(2)
        + (state.z_m - receiver_ecef.2).powi(2))
    .sqrt();
    let l1_signal = signal_spec_gps_l1_ca();
    let l2_signal = signal_spec_gps_l2_py();
    let l1_delay_m = navigation
        .klobuchar_delay_l1_m(
            bijux_gnss_core::api::Llh {
                lat_deg: receiver_llh.0,
                lon_deg: receiver_llh.1,
                alt_m: receiver_llh.2,
            },
            azimuth_deg,
            elevation_deg,
            receive_tow_s,
        )
        .expect("broadcast L1 delay");
    let l2_delay_m =
        first_order_ionosphere_code_delay_m(Meters(l1_delay_m), l1_signal, l2_signal)
            .expect("broadcast L2 delay")
            .0;
    let phase_bias_m = 4.0;
    let geometry_free_phase_m = (l2_delay_m - l1_delay_m) + phase_bias_m;
    let reference_phase_m = 22_000_000.0;
    let epoch = ObsEpoch {
        t_rx_s: Seconds(receive_tow_s),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(2209),
        tow_s: Some(Seconds(receive_tow_s)),
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            satellite(
                ephemeris.sat,
                SignalBand::L1,
                SignalCode::Ca,
                l1_signal,
                geometric_range_m + l1_delay_m,
                signal_meters_to_cycles(
                    Meters(reference_phase_m + geometry_free_phase_m),
                    l1_signal,
                )
                .0,
                timing,
            ),
            satellite(
                ephemeris.sat,
                SignalBand::L2,
                SignalCode::Py,
                l2_signal,
                geometric_range_m + l2_delay_m,
                signal_meters_to_cycles(Meters(reference_phase_m), l2_signal).0,
                timing,
            ),
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };

    let residuals = gps_broadcast_ionosphere_residuals_from_obs_epochs(
        &[epoch],
        receiver_ecef,
        &navigation,
        SignalBand::L1,
        SignalBand::L2,
    );

    assert_eq!(residuals.len(), 1);
    assert_eq!(residuals[0].broadcast_model, "gps_klobuchar");
    assert_eq!(residuals[0].broadcast_status, "ok");
    assert_eq!(residuals[0].broadcast_reason, "ok");
    assert!(
        residuals[0].code_residual_band_1_m.expect("code L1 residual").abs() < 1.0e-6
    );
    assert!(
        residuals[0].code_residual_band_2_m.expect("code L2 residual").abs() < 1.0e-6
    );
    assert!(
        residuals[0].phase_residual_band_1_m.expect("phase L1 residual").abs() < 1.0e-6
    );
    assert!(
        residuals[0].phase_residual_band_2_m.expect("phase L2 residual").abs() < 1.0e-6
    );
}

#[test]
fn gps_broadcast_ionosphere_residuals_measure_real_public_dual_frequency_data() {
    let observations = parse_rinex_gps_observation_dataset(&fixture("unavco_ab43_20180114.obs"))
        .expect("parse public dual-frequency observations");
    let navigation = parse_rinex_broadcast_navigation(&fixture("noaa_brdc0140_20180114.nav"))
        .expect("parse public broadcast navigation");
    let receiver_ecef = observations
        .approx_position_ecef_m
        .expect("public RINEX header should provide approximate receiver coordinates");

    let residuals = gps_broadcast_ionosphere_residuals_from_obs_epochs(
        &observations.epochs,
        receiver_ecef,
        &navigation,
        SignalBand::L1,
        SignalBand::L2,
    );
    let summary = summarize_broadcast_ionosphere_residuals(&residuals);

    assert!(!residuals.is_empty(), "public dual-frequency dataset should emit residuals");
    assert!(
        summary.broadcast_valid_count > 50,
        "expected many valid broadcast comparisons, got {}",
        summary.broadcast_valid_count
    );

    let code_band_1 = summary.code_band_1.expect("L1 code residual statistics");
    let code_band_2 = summary.code_band_2.expect("L2 code residual statistics");
    let phase_band_1 = summary.phase_band_1.expect("L1 phase residual statistics");
    let phase_band_2 = summary.phase_band_2.expect("L2 phase residual statistics");

    assert!(code_band_1.sample_count > 50);
    assert!(code_band_2.sample_count > 50);
    assert!(phase_band_1.sample_count > 50);
    assert!(phase_band_2.sample_count > 50);

    assert!(code_band_1.mean_abs_residual_m.is_finite());
    assert!(code_band_2.mean_abs_residual_m.is_finite());
    assert!(phase_band_1.mean_abs_residual_m.is_finite());
    assert!(phase_band_2.mean_abs_residual_m.is_finite());

    assert!(code_band_1.mean_abs_residual_m > 0.1);
    assert!(phase_band_1.mean_abs_residual_m > 0.1);
    assert!(code_band_1.max_abs_residual_m > code_band_1.mean_abs_residual_m);
    assert!(phase_band_1.max_abs_residual_m > phase_band_1.mean_abs_residual_m);
}
