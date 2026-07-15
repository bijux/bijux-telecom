use super::{
    advance_tracking_adaptation, anti_false_lock_detected, apply_carrier_tracking_loop,
    apply_code_loop, calibrated_lock_detector_thresholds,
    carrier_frequency_error_hz_from_phase_delta, carrier_phase_offset_radians,
    coherent_integration_seconds, correlate_early_prompt_late, delay_lock_loop_coefficients,
    discriminators, dll_lock_threshold, estimate_cn0_dbhz, estimate_tracking_uncertainty,
    first_order_angular_loop_coefficients, first_order_loop_coefficients,
    lock_detector_distributions, lock_detector_probability_summary, phase_lock_loop_coefficients,
    predict_code_phase_samples, wrap_phase_cycles_signed, wrap_phase_radians_positive,
    wrapped_phase_delta_cycles, CarrierTrackingLoopInput, CodeLoopInput,
    LockDetectorCalibrationInput, LockDetectorProbabilityInput, TrackingAdaptationInput,
    TrackingAdaptationState, TrackingLoopProfile, TrackingLoopProfileKind, TrackingQualityClass,
    TrackingUncertaintyInputs, DYNAMIC_STRESS_INTEGRATION_MS,
    MAX_TRACKING_ADAPTATION_PENDING_EPOCHS, WEAK_SIGNAL_INTEGRATION_MS,
};
use std::collections::VecDeque;

use num_complex::Complex;

#[path = "tests/lock_detection.rs"]
mod lock_detection;
#[path = "tests/loop_filter_design.rs"]
mod loop_filter_design;
#[path = "tests/loop_response.rs"]
mod loop_response;
#[path = "tests/signal_measurements.rs"]
mod signal_measurements;

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

#[test]
fn anti_false_lock_detected_rejects_early_late_energy_near_prompt() {
    assert!(anti_false_lock_detected(
        Complex::new(8.0, 0.0),
        Complex::new(8.0, 0.0),
        Complex::new(8.0, 0.0),
    ));
    assert!(!anti_false_lock_detected(
        Complex::new(1.0, 0.0),
        Complex::new(8.0, 0.0),
        Complex::new(1.0, 0.0),
    ));
}

#[test]
fn normalize_dll_discriminator_preserves_reference_spacing_gain() {
    assert_eq!(super::normalize_dll_discriminator(0.25, 0.5), 0.25);
}

#[test]
fn normalize_dll_discriminator_scales_narrow_correlator_gain() {
    let raw = 0.10;
    let standard = super::normalize_dll_discriminator(raw, 0.5);
    let narrow = super::normalize_dll_discriminator(raw, 0.25);

    assert!(
        narrow > standard,
        "narrow spacing must compensate discriminator gain: standard={standard} narrow={narrow}",
    );
    assert!((narrow - 0.116_666_67).abs() <= f32::EPSILON);
}

#[test]
fn dll_discriminator_from_early_late_matches_tracking_discriminators() {
    let early = Complex::new(3.0, 4.0);
    let prompt = Complex::new(10.0, 0.0);
    let late = Complex::new(1.0, 0.0);

    let (dll, _, _, _) = super::discriminators(early, prompt, late, None);

    assert_eq!(dll, super::dll_discriminator_from_early_late(early, late));
}

#[test]
fn double_delta_dll_discriminator_suppresses_outer_lobe_bias() {
    let inner_early = Complex::new(8.0, 0.0);
    let inner_late = Complex::new(4.0, 0.0);
    let outer_early = Complex::new(6.0, 0.0);
    let outer_late = Complex::new(2.0, 0.0);

    let early_late = super::dll_discriminator_from_early_late(inner_early, inner_late);
    let double_delta =
        super::double_delta_dll_discriminator(inner_early, inner_late, outer_early, outer_late);

    assert!(double_delta.abs() < early_late.abs());
    assert!((double_delta - 0.166_666_67).abs() <= f32::EPSILON);
}

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
