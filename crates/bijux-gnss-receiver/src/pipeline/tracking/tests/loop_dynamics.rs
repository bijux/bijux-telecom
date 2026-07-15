use super::*;

#[test]
fn advance_code_phase_samples_wraps_nominal_epoch_step() {
    let next = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 137.5,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 4_092_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.0,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    })
    .code_phase_samples;
    assert!((next - 137.5).abs() < 1.0e-9, "next={next}");
}

#[test]
fn advance_code_phase_samples_applies_dll_correction() {
    let next = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 4_092_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.4,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    })
    .code_phase_samples;
    assert!(next > 250.0, "next={next}");
}

#[test]
fn tracking_epoch_samples_scale_with_configured_integration_ms() {
    let tracking_params = crate::engine::receiver_config::TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 7,
    };

    let epoch_samples =
        super::tracking_epoch_samples(5_000_000.0, 1_023_000.0, 1023, tracking_params);

    assert_eq!(epoch_samples, 35_000);
}

#[test]
fn tracking_params_for_state_uses_adapted_profile_when_enabled() {
    let tracking = Tracking::new(
        ReceiverPipelineConfig {
            adaptive_tracking_enabled: true,
            ..ReceiverPipelineConfig::default()
        },
        ReceiverRuntime::default(),
    );
    let base_tracking_params = TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 1,
    };
    let mut state = empty_loop_state();
    state.tracking_loop_profile = SignalTrackingLoopProfile {
        dll_bw_hz: 1.2,
        pll_bw_hz: 8.25,
        fll_bw_hz: 5.0,
        integration_ms: 5,
    };

    let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

    assert_eq!(effective.early_late_spacing_chips, base_tracking_params.early_late_spacing_chips);
    assert_eq!(effective.dll_bw_hz, 1.2);
    assert_eq!(effective.pll_bw_hz, 8.25);
    assert_eq!(effective.fll_bw_hz, 5.0);
    assert_eq!(effective.integration_ms, 5);
}

#[test]
fn tracking_params_for_state_ignores_adapted_profile_when_disabled() {
    let tracking = Tracking::new(
        ReceiverPipelineConfig {
            adaptive_tracking_enabled: false,
            ..ReceiverPipelineConfig::default()
        },
        ReceiverRuntime::default(),
    );
    let base_tracking_params = TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 1,
    };
    let mut state = empty_loop_state();
    state.tracking_loop_profile = SignalTrackingLoopProfile {
        dll_bw_hz: 1.2,
        pll_bw_hz: 8.25,
        fll_bw_hz: 5.0,
        integration_ms: 5,
    };

    let effective = tracking.tracking_params_for_state(base_tracking_params, &state);

    assert_eq!(effective.dll_bw_hz, base_tracking_params.dll_bw_hz);
    assert_eq!(effective.pll_bw_hz, base_tracking_params.pll_bw_hz);
    assert_eq!(effective.fll_bw_hz, base_tracking_params.fll_bw_hz);
    assert_eq!(effective.integration_ms, base_tracking_params.integration_ms);
}
