use super::*;

#[test]
fn estimate_tracking_uncertainty_rewards_longer_coherent_integration() {
    let empty = VecDeque::<f64>::new();
    let short = estimate_tracking_uncertainty(
        &empty,
        &empty,
        &empty,
        &empty,
        TrackingUncertaintyInputs {
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
            quality_class: TrackingQualityClass::Tracking,
        },
    );
    let long = estimate_tracking_uncertainty(
        &empty,
        &empty,
        &empty,
        &empty,
        TrackingUncertaintyInputs {
            integration_ms: 10,
            ..TrackingUncertaintyInputs {
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
                quality_class: TrackingQualityClass::Tracking,
            }
        },
    );

    assert!(long.code_phase_samples < short.code_phase_samples, "short={short:?} long={long:?}");
}

#[test]
fn estimate_tracking_uncertainty_penalizes_weaker_cn0_for_same_observed_window() {
    let code_window = VecDeque::from([0.10, 0.10, 0.10, 0.10]);
    let carrier_window = VecDeque::from([0.01, 0.01, 0.01, 0.01]);
    let doppler_window = VecDeque::from([0.5, 0.5, 0.5, 0.5]);
    let cn0_window = VecDeque::from([58.0, 58.0, 58.0, 58.0]);
    let strong = estimate_tracking_uncertainty(
        &code_window,
        &carrier_window,
        &doppler_window,
        &cn0_window,
        TrackingUncertaintyInputs {
            samples_per_chip: 4.0,
            dll_err: 0.0,
            pll_err_rad: 0.0,
            fll_err_hz: 0.0,
            cn0_dbhz: 58.0,
            cn0_reference_dbhz: 58.0,
            integration_ms: 1,
            channel_locked: true,
            dll_locked: true,
            anti_false_lock: false,
            cycle_slip: false,
            quality_class: TrackingQualityClass::Tracking,
        },
    );
    let weak = estimate_tracking_uncertainty(
        &code_window,
        &carrier_window,
        &doppler_window,
        &cn0_window,
        TrackingUncertaintyInputs {
            cn0_dbhz: 35.0,
            cn0_reference_dbhz: 35.0,
            ..TrackingUncertaintyInputs {
                samples_per_chip: 4.0,
                dll_err: 0.0,
                pll_err_rad: 0.0,
                fll_err_hz: 0.0,
                cn0_dbhz: 58.0,
                cn0_reference_dbhz: 58.0,
                integration_ms: 1,
                channel_locked: true,
                dll_locked: true,
                anti_false_lock: false,
                cycle_slip: false,
                quality_class: TrackingQualityClass::Tracking,
            }
        },
    );

    assert!(weak.code_phase_samples > strong.code_phase_samples, "strong={strong:?} weak={weak:?}");
    assert!(
        weak.carrier_phase_cycles > strong.carrier_phase_cycles,
        "strong={strong:?} weak={weak:?}"
    );
    assert!(weak.doppler_hz > strong.doppler_hz, "strong={strong:?} weak={weak:?}");
}
