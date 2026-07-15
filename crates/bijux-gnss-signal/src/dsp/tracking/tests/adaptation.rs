use super::*;

#[test]
fn tracking_adaptation_prefers_dynamic_stress_for_large_frequency_ramps() {
    let decision = advance_tracking_adaptation(
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 5 },
        TrackingAdaptationState::default(),
        TrackingAdaptationInput {
            cn0_dbhz: 45.0,
            fll_error_hz: 30.0,
            carrier_rate_hz_per_s: 40.0,
            carrier_lock_ready: false,
            steady_state_lock: false,
            discriminator_stable: false,
        },
    );

    assert_eq!(decision.profile_kind, TrackingLoopProfileKind::DynamicStress);
    assert_eq!(decision.profile.integration_ms, DYNAMIC_STRESS_INTEGRATION_MS);
    assert!(decision.profile.pll_bw_hz > 15.0, "{decision:?}");
}

#[test]
fn tracking_adaptation_requires_persistent_low_cn0_before_weak_signal_switch() {
    let base_profile =
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 1 };
    let input = TrackingAdaptationInput {
        cn0_dbhz: 29.5,
        fll_error_hz: 0.5,
        carrier_rate_hz_per_s: 0.0,
        carrier_lock_ready: true,
        steady_state_lock: true,
        discriminator_stable: true,
    };
    let first =
        advance_tracking_adaptation(base_profile, TrackingAdaptationState::default(), input);
    let second = advance_tracking_adaptation(base_profile, first.state, input);

    assert_eq!(first.profile_kind, TrackingLoopProfileKind::Nominal);
    assert_eq!(second.profile_kind, TrackingLoopProfileKind::WeakSignal);
    assert_eq!(second.profile.integration_ms, WEAK_SIGNAL_INTEGRATION_MS);
    assert!(second.profile.pll_bw_hz < base_profile.pll_bw_hz, "{second:?}");
}

#[test]
fn tracking_adaptation_holds_weak_signal_until_cn0_recovers() {
    let base_profile =
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 1 };
    let low_cn0 = TrackingAdaptationInput {
        cn0_dbhz: 29.0,
        fll_error_hz: 0.0,
        carrier_rate_hz_per_s: 0.0,
        carrier_lock_ready: true,
        steady_state_lock: true,
        discriminator_stable: true,
    };
    let recovered_cn0 = TrackingAdaptationInput { cn0_dbhz: 36.0, ..low_cn0 };
    let weak = advance_tracking_adaptation(
        base_profile,
        advance_tracking_adaptation(base_profile, TrackingAdaptationState::default(), low_cn0)
            .state,
        low_cn0,
    );
    let first_recovery = advance_tracking_adaptation(base_profile, weak.state, recovered_cn0);
    let second_recovery =
        advance_tracking_adaptation(base_profile, first_recovery.state, recovered_cn0);
    let third_recovery =
        advance_tracking_adaptation(base_profile, second_recovery.state, recovered_cn0);
    let fourth_recovery =
        advance_tracking_adaptation(base_profile, third_recovery.state, recovered_cn0);

    assert_eq!(weak.profile_kind, TrackingLoopProfileKind::WeakSignal);
    assert_eq!(first_recovery.profile_kind, TrackingLoopProfileKind::WeakSignal);
    assert_eq!(third_recovery.profile_kind, TrackingLoopProfileKind::WeakSignal);
    assert_eq!(fourth_recovery.profile_kind, TrackingLoopProfileKind::Nominal);
}

#[test]
fn tracking_adaptation_requires_carrier_confidence_for_weak_signal_integration() {
    let base_profile =
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 1 };
    let low_cn0_without_lock = TrackingAdaptationInput {
        cn0_dbhz: 29.0,
        fll_error_hz: 0.0,
        carrier_rate_hz_per_s: 0.0,
        carrier_lock_ready: false,
        steady_state_lock: true,
        discriminator_stable: true,
    };
    let low_cn0_without_steady_state = TrackingAdaptationInput {
        carrier_lock_ready: true,
        steady_state_lock: false,
        ..low_cn0_without_lock
    };
    let low_cn0_with_unstable_discriminator = TrackingAdaptationInput {
        carrier_lock_ready: true,
        steady_state_lock: true,
        discriminator_stable: false,
        ..low_cn0_without_lock
    };

    for input in
        [low_cn0_without_lock, low_cn0_without_steady_state, low_cn0_with_unstable_discriminator]
    {
        let decision =
            advance_tracking_adaptation(base_profile, TrackingAdaptationState::default(), input);

        assert_eq!(decision.profile_kind, TrackingLoopProfileKind::Nominal);
        assert_eq!(decision.profile.integration_ms, base_profile.integration_ms);
    }
}

#[test]
fn tracking_adaptation_does_not_treat_weak_signal_fll_noise_as_dynamic_stress() {
    let base_profile =
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 1 };
    let noisy_weak_signal = TrackingAdaptationInput {
        cn0_dbhz: 30.5,
        fll_error_hz: 80.0,
        carrier_rate_hz_per_s: 40.0,
        carrier_lock_ready: true,
        steady_state_lock: true,
        discriminator_stable: true,
    };
    let first = advance_tracking_adaptation(
        base_profile,
        TrackingAdaptationState::default(),
        noisy_weak_signal,
    );
    let second = advance_tracking_adaptation(base_profile, first.state, noisy_weak_signal);

    assert_eq!(first.profile_kind, TrackingLoopProfileKind::Nominal);
    assert_eq!(second.profile_kind, TrackingLoopProfileKind::WeakSignal);
    assert!(second.profile.pll_bw_hz < base_profile.pll_bw_hz, "{second:?}");
}

#[test]
fn tracking_adaptation_caps_pending_hysteresis_memory() {
    let base_profile =
        TrackingLoopProfile { dll_bw_hz: 2.0, pll_bw_hz: 15.0, fll_bw_hz: 10.0, integration_ms: 1 };
    let mut state = TrackingAdaptationState {
        active_profile: TrackingLoopProfileKind::WeakSignal,
        pending_profile: Some(TrackingLoopProfileKind::Nominal),
        pending_epochs: MAX_TRACKING_ADAPTATION_PENDING_EPOCHS,
    };
    let recovery_input = TrackingAdaptationInput {
        cn0_dbhz: 33.5,
        fll_error_hz: 0.0,
        carrier_rate_hz_per_s: 0.0,
        carrier_lock_ready: true,
        steady_state_lock: true,
        discriminator_stable: true,
    };

    for _ in 0..3 {
        let decision = advance_tracking_adaptation(base_profile, state, recovery_input);
        state = decision.state;

        assert!(
            state.pending_epochs <= MAX_TRACKING_ADAPTATION_PENDING_EPOCHS,
            "decision={decision:?}"
        );
    }
}
