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
