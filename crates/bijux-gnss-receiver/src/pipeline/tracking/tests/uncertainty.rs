use super::*;

#[test]
fn tracking_uncertainty_rewards_longer_coherent_integration() {
    let state = empty_loop_state();
    let short = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            samples_per_chip: 4.0,
            dll_err: 0.0,
            pll_err_rad: 0.0,
            fll_err_hz: 0.0,
            cn0_dbhz: 45.0,
            cn0_reference_dbhz: 45.0,
            integration_ms: 1,
            channel_locked: true,
            dll_locked: true,
            anti_false_lock: false,
            cycle_slip: false,
            channel_state: ChannelState::Tracking,
        },
    );
    let long = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            integration_ms: 10,
            ..super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            }
        },
    );

    assert!(
            long.code_phase_samples < short.code_phase_samples,
            "longer coherent integration should tighten code-phase uncertainty: short={short:?} long={long:?}"
        );
}

#[test]
fn tracking_uncertainty_penalizes_dll_unlock() {
    let state = empty_loop_state();
    let locked = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            samples_per_chip: 4.0,
            dll_err: 0.0,
            pll_err_rad: 0.0,
            fll_err_hz: 0.0,
            cn0_dbhz: 45.0,
            cn0_reference_dbhz: 45.0,
            integration_ms: 1,
            channel_locked: true,
            dll_locked: true,
            anti_false_lock: false,
            cycle_slip: false,
            channel_state: ChannelState::Tracking,
        },
    );
    let unlocked = super::estimate_tracking_uncertainty(
        &state,
        super::TrackingUncertaintyInputs {
            dll_locked: false,
            ..super::TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 45.0,
                cn0_reference_dbhz: 45.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                channel_state: ChannelState::Tracking,
            }
        },
    );

    assert!(
            unlocked.code_phase_samples > locked.code_phase_samples,
            "loss of DLL lock should inflate code-phase uncertainty: locked={locked:?} unlocked={unlocked:?}"
        );
}
