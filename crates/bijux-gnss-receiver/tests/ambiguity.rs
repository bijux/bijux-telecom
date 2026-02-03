use bijux_gnss_core::{
    Constellation, LockFlags, ObsMetadata, ObsSatellite, SignalBand, SignalSpec,
};
use bijux_gnss_receiver::ambiguity::AmbiguityManager;
use bijux_gnss_receiver::ambiguity::{ratio_test, FixPolicy, FixState};

#[test]
fn ratio_test_requires_consecutive_accepts() {
    let policy = FixPolicy {
        ratio_threshold: 3.0,
        consecutive_required: 2,
    };
    let mut state = FixState::default();
    assert!(!ratio_test(2.0, &policy, &mut state));
    assert!(!ratio_test(3.1, &policy, &mut state));
    assert!(ratio_test(3.2, &policy, &mut state));
}

#[test]
fn ambiguity_resets_on_cycle_slip() {
    let mut manager = AmbiguityManager::new();
    let sat = ObsSatellite {
        prn: 1,
        pseudorange_m: 20_000_000.0,
        carrier_phase_cycles: 100.0,
        doppler_hz: 0.0,
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
    };
    manager.update_from_obs(1, std::slice::from_ref(&sat));
    let mut slip_sat = sat.clone();
    slip_sat.lock_flags.cycle_slip = true;
    manager.update_from_obs(2, std::slice::from_ref(&slip_sat));
    let state = manager.states.values().next().expect("state");
    assert_eq!(state.status, bijux_gnss_core::AmbiguityStatus::Unknown);
}
