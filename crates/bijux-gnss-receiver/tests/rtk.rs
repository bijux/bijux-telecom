use bijux_gnss_core::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, SignalBand, SignalSpec,
};
use bijux_gnss_nav::geodetic_to_ecef;
use bijux_gnss_receiver::rtk::baseline_from_ecef;
use bijux_gnss_receiver::rtk::{apply_fix_hold, build_dd, build_sd, choose_ref_sat, EpochAligner};

fn make_epoch(t_rx_s: f64, prn: u8) -> ObsEpoch {
    ObsEpoch {
        t_rx_s,
        gps_week: None,
        tow_s: None,
        epoch_idx: (t_rx_s * 1000.0) as u64,
        discontinuity: false,
        sats: vec![ObsSatellite {
            prn,
            pseudorange_m: 20_000_000.0 + prn as f64,
            carrier_phase_cycles: 1000.0 + prn as f64,
            doppler_hz: -500.0,
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
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: 1_575_420_000.0,
                },
            },
        }],
    }
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
    let ref_prn = choose_ref_sat(&sd).expect("ref");
    let dd = build_dd(&sd, ref_prn);
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
