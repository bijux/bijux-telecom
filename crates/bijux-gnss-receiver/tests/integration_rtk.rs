#![allow(missing_docs)]
use bijux_gnss_core::api::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, SatId, SigId,
    SignalBand, SignalSpec,
};
use bijux_gnss_nav::api::geodetic_to_ecef;
use bijux_gnss_receiver::api::baseline_from_ecef;
use bijux_gnss_receiver::api::{
    apply_fix_hold, build_dd, build_dd_per_constellation, build_sd, choose_ref_sat,
    choose_ref_sat_per_constellation, EpochAligner,
};

fn make_epoch(t_rx_s: f64, prn: u8) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(t_rx_s),
        gps_week: None,
        tow_s: None,
        epoch_idx: (t_rx_s * 1000.0) as u64,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
            signal_id: SigId {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn,
                },
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            pseudorange_m: bijux_gnss_core::api::Meters(20_000_000.0 + prn as f64),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(1000.0 + prn as f64),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: bijux_gnss_core::api::Hertz(-500.0),
            doppler_var_hz2: 4.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
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
            },
        }],
    }
}

fn make_epoch_with_constellation(t_rx_s: f64, prn: u8, constellation: Constellation) -> ObsEpoch {
    let mut epoch = make_epoch(t_rx_s, prn);
    epoch.sats[0].signal_id.sat.constellation = constellation;
    epoch.sats[0].metadata.signal.constellation = constellation;
    epoch
}

#[test]
fn aligner_handles_missing_epochs() {
    let base = vec![
        make_epoch(0.000, 1),
        make_epoch(0.001, 1),
        make_epoch(0.003, 1),
    ];
    let rover = vec![
        make_epoch(0.000, 1),
        make_epoch(0.002, 1),
        make_epoch(0.003, 1),
    ];
    let mut aligner = EpochAligner::new(0.0002);
    let aligned = aligner.align(&base, &rover);
    assert_eq!(aligned.len(), 2);
    assert!(aligner.dropped_base > 0 || aligner.dropped_rover > 0);
}

#[test]
fn dd_builder_skips_ref_drop() {
    let base = make_epoch(0.0, 1);
    let rover = make_epoch(0.0, 1);
    let sd = build_sd(&base, &rover);
    let ref_sig = choose_ref_sat(&sd).expect("ref");
    let dd = build_dd(&sd, ref_sig);
    assert!(dd.is_empty());
}

#[test]
fn baseline_output_is_reasonable() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0, -121.999, 10.0);
    let baseline = baseline_from_ecef([base.0, base.1, base.2], [rover.0, rover.1, rover.2]);
    assert!(baseline.enu_m[0].abs() > 1.0);
}

#[test]
fn fix_hold_reduces_covariance() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0, -121.999, 10.0);
    let baseline = baseline_from_ecef([base.0, base.1, base.2], [rover.0, rover.1, rover.2]);
    let fixed = apply_fix_hold(baseline, true);
    assert!(fixed.fixed);
    let cov = fixed.covariance_m2.expect("cov");
    assert!(cov[0][0] < 1.0);
}

#[test]
fn dd_variance_matches_expected_sum() {
    let mut base = make_epoch(0.0, 1);
    let mut rover = make_epoch(0.0, 1);
    base.sats[0].pseudorange_var_m2 = 4.0;
    rover.sats[0].pseudorange_var_m2 = 9.0;
    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 1);
    let ref_sig = choose_ref_sat(&sd).expect("ref");
    let dd = build_dd(&sd, ref_sig);
    assert!(dd.is_empty());
    assert!((sd[0].variance_code - 13.0).abs() < 1e-6);
}

#[test]
fn variance_model_decreases_with_cn0() {
    let mut low = make_epoch(0.0, 1);
    let mut high = make_epoch(0.0, 1);
    low.sats[0].cn0_dbhz = 30.0;
    low.sats[0].pseudorange_var_m2 = 0.0;
    high.sats[0].cn0_dbhz = 45.0;
    high.sats[0].pseudorange_var_m2 = 0.0;
    let sd_low = build_sd(&low, &low);
    let sd_high = build_sd(&high, &high);
    assert!(sd_high[0].variance_code < sd_low[0].variance_code);
}

#[test]
fn multi_constellation_ref_selection_is_per_system() {
    let base_gps = make_epoch_with_constellation(0.0, 1, Constellation::Gps);
    let rover_gps = make_epoch_with_constellation(0.0, 1, Constellation::Gps);
    let base_gal = make_epoch_with_constellation(0.0, 1, Constellation::Galileo);
    let rover_gal = make_epoch_with_constellation(0.0, 1, Constellation::Galileo);
    let mut base = base_gps.clone();
    base.sats.extend(base_gal.sats.clone());
    let mut rover = rover_gps.clone();
    rover.sats.extend(rover_gal.sats.clone());

    let sd = build_sd(&base, &rover);
    let refs = choose_ref_sat_per_constellation(&sd);
    assert_eq!(refs.len(), 2);
    let dd = build_dd_per_constellation(&sd, &refs);
    assert!(dd.is_empty());
}
