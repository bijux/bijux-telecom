#![allow(missing_docs)]
use bijux_gnss_core::{
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation, LockFlags, ObsEpoch, ObsMetadata,
    ObsSatellite, ReceiverRole, SatId, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::combinations_from_obs_epochs;

fn make_dual_freq_epoch(p1: f64, p2: f64, phi1: f64, phi2: f64) -> ObsEpoch {
    let sat = SatId {
        constellation: Constellation::Gps,
        prn: 1,
    };
    let mut s1 = signal_spec_gps_l1_ca();
    let mut s2 = signal_spec_gps_l2_py();
    s1.code_rate_hz = 1_023_000.0;
    s2.code_rate_hz = 1_023_000.0;
    ObsEpoch {
        t_rx_s: bijux_gnss_core::Seconds(0.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        role: ReceiverRole::Rover,
        sats: vec![
            ObsSatellite {
                signal_id: SigId {
                    sat,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                },
                pseudorange_m: bijux_gnss_core::Meters(p1),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::Cycles(phi1),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s1,
                },
            },
            ObsSatellite {
                signal_id: SigId {
                    sat,
                    band: SignalBand::L2,
                    code: SignalCode::Py,
                },
                pseudorange_m: bijux_gnss_core::Meters(p2),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: bijux_gnss_core::Cycles(phi2),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: bijux_gnss_core::Hertz(0.0),
                doppler_var_hz2: 1.0,
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
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: s2,
                },
            },
        ],
    }
}

#[test]
fn iono_free_reduces_iono_term() {
    let base_range = 20_200_000.0;
    let iono_l1 = 5.0;
    let iono_l2 = 8.0;
    let epoch = make_dual_freq_epoch(
        base_range + iono_l1,
        base_range + iono_l2,
        (base_range - iono_l1) / (bijux_gnss_core::GPS_L1_CA_CARRIER_HZ.value() / 299_792_458.0),
        (base_range - iono_l2) / (bijux_gnss_core::GPS_L2_PY_CARRIER_HZ.value() / 299_792_458.0),
    );
    let combos = combinations_from_obs_epochs(&[epoch], SignalBand::L1, SignalBand::L2);
    assert_eq!(combos.len(), 1);
    let combo = &combos[0];
    let if_code = combo.if_code_m.expect("if code");
    assert!((if_code - base_range).abs() < 10.0);
}

#[test]
fn melbourne_wubbena_detects_slip() {
    let epoch1 = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0);
    let epoch2 = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1100.0, 1001.0);
    let combos = combinations_from_obs_epochs(&[epoch1, epoch2], SignalBand::L1, SignalBand::L2);
    let mw1 = combos[0].melbourne_wubbena_m.unwrap();
    let mw2 = combos[1].melbourne_wubbena_m.unwrap();
    assert!((mw2 - mw1).abs() > 1.0);
}

#[test]
fn missing_frequency_marks_invalid() {
    let epoch = make_dual_freq_epoch(20_000_000.0, 20_000_002.0, 1000.0, 1001.0);
    let mut single = epoch.clone();
    single.sats.pop();
    let combos = combinations_from_obs_epochs(&[single], SignalBand::L1, SignalBand::L2);
    assert_eq!(combos.len(), 1);
    assert_eq!(combos[0].status, "invalid");
}
