#![allow(missing_docs)]
use bijux_gnss_core::api::{
    Constellation, GpsTime, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ObsSignalTiming,
    ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalSpec,
};
use bijux_gnss_receiver::api::{build_dd, build_sd, choose_ref_sat_per_constellation};
use bijux_gnss_receiver::api::{double_difference, single_difference};

fn make_epoch(prn: u8, pseudo: f64, phase: f64, doppler: f64) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(0.0),
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
                sat: SatId { constellation: Constellation::Gps, prn },
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            pseudorange_m: bijux_gnss_core::api::Meters(pseudo),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(phase),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: bijux_gnss_core::api::Hertz(doppler),
            doppler_var_hz2: 4.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: bijux_gnss_core::api::ObservationStatus::Accepted,
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
                signal: SignalSpec {
                    constellation: Constellation::Gps,
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

fn clone_with_signal(
    sat: &ObsSatellite,
    band: SignalBand,
    pseudo: f64,
    phase: f64,
    doppler: f64,
) -> ObsSatellite {
    let mut cloned = sat.clone();
    cloned.signal_id.band = band;
    cloned.metadata.signal.band = band;
    cloned.metadata.signal.carrier_hz = match band {
        SignalBand::L1 => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
        SignalBand::L2 => bijux_gnss_core::api::GPS_L2_PY_CARRIER_HZ,
        SignalBand::L5 => bijux_gnss_core::api::GPS_L5_CARRIER_HZ,
        _ => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
    };
    cloned.pseudorange_m = bijux_gnss_core::api::Meters(pseudo);
    cloned.carrier_phase_cycles = bijux_gnss_core::api::Cycles(phase);
    cloned.doppler_hz = bijux_gnss_core::api::Hertz(doppler);
    cloned
}

#[test]
fn single_and_double_difference_basic() {
    let rover = make_epoch(1, 20_000_100.0, 1_000.5, -500.0);
    let base = make_epoch(1, 20_000_000.0, 1_000.0, -450.0);
    let sds = single_difference(&rover, &base);
    assert_eq!(sds.len(), 1);
    assert!((sds[0].code_m.0 - 100.0).abs() < 1e-6);

    let rover2 = make_epoch(2, 21_000_000.0, 1_500.0, -400.0);
    let base2 = make_epoch(2, 20_999_900.0, 1_499.8, -380.0);
    let mut rover_all = rover.clone();
    rover_all.sats.push(rover2.sats[0].clone());
    let mut base_all = base.clone();
    base_all.sats.push(base2.sats[0].clone());
    let sds = single_difference(&rover_all, &base_all);
    let ref_sig = sds[0].sig;
    let dds = double_difference(&sds, ref_sig);
    assert_eq!(dds.len(), 1);
    assert_eq!(dds[0].ref_sig.sat.prn, 1);
}

#[test]
fn rtk_sd_dd_builders_work() {
    let rover = make_epoch(1, 20_000_100.0, 1_000.5, -500.0);
    let base = make_epoch(1, 20_000_000.0, 1_000.0, -450.0);
    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 1);
    let refs = choose_ref_sat_per_constellation(&sd);
    let ref_sig = *refs.values().next().expect("ref");
    let dd = build_dd(&sd, ref_sig);
    assert!(dd.is_empty());
}

#[test]
fn single_differences_match_full_signal_identity() {
    let mut rover = make_epoch(7, 20_000_100.0, 1_000.5, -500.0);
    let mut base = make_epoch(7, 20_000_000.0, 1_000.0, -450.0);

    rover.sats.push(clone_with_signal(
        &rover.sats[0],
        SignalBand::L2,
        24_000_120.0,
        1_100.5,
        -520.0,
    ));
    base.sats.push(clone_with_signal(&base.sats[0], SignalBand::L2, 24_000_010.0, 1_100.0, -470.0));

    let sds = single_difference(&rover, &base);
    assert_eq!(sds.len(), 2);
    assert!(sds
        .iter()
        .any(|sd| sd.sig.band == SignalBand::L1 && (sd.code_m.0 - 100.0).abs() < 1e-6));
    assert!(sds
        .iter()
        .any(|sd| sd.sig.band == SignalBand::L2 && (sd.code_m.0 - 110.0).abs() < 1e-6));

    let rtksd = build_sd(&base, &rover);
    assert_eq!(rtksd.len(), 2);
    assert!(rtksd
        .iter()
        .any(|sd| sd.sig.band == SignalBand::L1 && (sd.code_m - 100.0).abs() < 1e-6));
    assert!(rtksd
        .iter()
        .any(|sd| sd.sig.band == SignalBand::L2 && (sd.code_m - 110.0).abs() < 1e-6));
}

#[test]
fn sd_builder_preserves_rover_and_base_measurements() {
    let mut rover = make_epoch(9, 20_000_250.0, 1_250.5, -410.0);
    let mut base = make_epoch(9, 20_000_100.0, 1_250.0, -405.0);
    let rover_timing = ObsSignalTiming {
        signal_travel_time_s: Seconds(0.067_001),
        transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_000 },
    };
    let base_timing = ObsSignalTiming {
        signal_travel_time_s: Seconds(0.066_998),
        transmit_gps_time: GpsTime { week: 2200, tow_s: 345_600.123_003 },
    };
    rover.sats[0].timing = Some(rover_timing);
    base.sats[0].timing = Some(base_timing);

    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 1);
    assert_eq!(sd[0].rover_pseudorange_m, 20_000_250.0);
    assert_eq!(sd[0].base_pseudorange_m, 20_000_100.0);
    assert_eq!(sd[0].rover_signal_timing, Some(rover_timing));
    assert_eq!(sd[0].base_signal_timing, Some(base_timing));
}

#[test]
fn rtk_differencing_preserves_quality_flags_and_prefers_clean_reference() {
    let mut rover = make_epoch(4, 20_000_180.0, 1_100.3, -410.0);
    let mut base = make_epoch(4, 20_000_060.0, 1_099.9, -408.0);
    let mut clean_rover = make_epoch(9, 20_000_220.0, 1_250.7, -415.0);
    let clean_base = make_epoch(9, 20_000_100.0, 1_250.1, -414.0);

    rover.sats[0].cn0_dbhz = 27.0;
    base.sats[0].cn0_dbhz = 29.0;
    rover.sats[0].multipath_suspect = true;
    base.sats[0].multipath_suspect = false;
    clean_rover.sats[0].cn0_dbhz = 46.0;

    rover.sats.push(clean_rover.sats.remove(0));
    base.sats.push(clean_base.sats[0].clone());

    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 2);
    let degraded = sd.iter().find(|observation| observation.sig.sat.prn == 4).expect("degraded");
    let clean = sd.iter().find(|observation| observation.sig.sat.prn == 9).expect("clean");
    assert_eq!(degraded.min_cn0_dbhz, 27.0);
    assert!(degraded.multipath_suspect);
    assert!(!clean.multipath_suspect);

    let reference = bijux_gnss_receiver::api::choose_ref_sat(&sd).expect("reference");
    assert_eq!(reference.sat.prn, 9);

    let dd = build_dd(&sd, reference);
    assert_eq!(dd.len(), 1);
    assert_eq!(dd[0].min_cn0_dbhz, 27.0);
    assert!(dd[0].multipath_suspect);
}
