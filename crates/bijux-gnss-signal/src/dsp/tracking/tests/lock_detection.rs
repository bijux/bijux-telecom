use super::*;

#[test]
fn lock_detector_distributions_tighten_with_stronger_cn0() {
    let weak = lock_detector_distributions(LockDetectorCalibrationInput {
        cn0_dbhz: 32.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });
    let strong = lock_detector_distributions(LockDetectorCalibrationInput {
        cn0_dbhz: 52.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });

    assert!(strong.coherent_snr_linear > weak.coherent_snr_linear);
    assert!(strong.dll_sigma < weak.dll_sigma, "weak={weak:?} strong={strong:?}");
    assert!(strong.pll_sigma_rad < weak.pll_sigma_rad, "weak={weak:?} strong={strong:?}");
    assert!(strong.fll_sigma_hz < weak.fll_sigma_hz, "weak={weak:?} strong={strong:?}");
}

#[test]
fn calibrated_lock_detector_thresholds_expand_for_weaker_signals() {
    let strong = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz: 52.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });
    let weak = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz: 32.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });

    assert!(weak.dll_lock > strong.dll_lock, "weak={weak:?} strong={strong:?}");
    assert!(weak.pll_lock_rad > strong.pll_lock_rad, "weak={weak:?} strong={strong:?}");
    assert!(weak.fll_lock_hz > strong.fll_lock_hz, "weak={weak:?} strong={strong:?}");
    assert!(strong.dll_hold > strong.dll_lock);
    assert!(strong.pll_hold_rad > strong.pll_lock_rad);
}

#[test]
fn calibrated_lock_detector_thresholds_include_dynamic_frequency_stress() {
    let calm = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz: 45.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });
    let dynamic = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        dynamic_stress_hz: 35.0,
        ..LockDetectorCalibrationInput {
            cn0_dbhz: 45.0,
            coherent_integration_s: 0.001,
            samples_per_chip: 4.0,
            early_late_spacing_chips: 0.5,
            dll_false_unlock_probability: 1.0e-6,
            pll_false_unlock_probability: 1.0e-6,
            fll_false_unlock_probability: 1.0e-6,
            dynamic_stress_hz: 0.0,
        }
    });

    assert!((dynamic.fll_lock_hz - calm.fll_lock_hz - 35.0).abs() <= 1.0e-9);
    assert_eq!(dynamic.dll_lock, calm.dll_lock);
    assert_eq!(dynamic.pll_lock_rad, calm.pll_lock_rad);
}

#[test]
fn lock_detector_probability_summary_reports_calibrated_false_unlock_rate() {
    let target = 1.0e-6;
    let thresholds = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz: 45.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: target,
        pll_false_unlock_probability: target,
        fll_false_unlock_probability: target,
        dynamic_stress_hz: 0.0,
    });

    let summary = lock_detector_probability_summary(LockDetectorProbabilityInput {
        thresholds,
        unlocked_dll_half_width: 1.0,
        unlocked_pll_half_width_rad: std::f64::consts::PI,
        unlocked_fll_half_width_hz: 500.0,
        missed_unlock_bias_sigma: 4.0,
    });

    assert!((summary.dll_false_unlock_probability - target).abs() < 2.0e-9, "{summary:?}");
    assert!((summary.pll_false_unlock_probability - target).abs() < 2.0e-9, "{summary:?}");
    assert!((summary.fll_false_unlock_probability - target).abs() < 2.0e-9, "{summary:?}");
    assert!(summary.false_unlock_probability > target);
    assert!(summary.false_unlock_probability < target * 4.0);
}

#[test]
fn lock_detector_probability_summary_quantifies_false_lock_and_missed_unlock_regions() {
    let thresholds = calibrated_lock_detector_thresholds(LockDetectorCalibrationInput {
        cn0_dbhz: 45.0,
        coherent_integration_s: 0.001,
        samples_per_chip: 4.0,
        early_late_spacing_chips: 0.5,
        dll_false_unlock_probability: 1.0e-6,
        pll_false_unlock_probability: 1.0e-6,
        fll_false_unlock_probability: 1.0e-6,
        dynamic_stress_hz: 0.0,
    });
    let wide_unlocked = lock_detector_probability_summary(LockDetectorProbabilityInput {
        thresholds,
        unlocked_dll_half_width: 1.0,
        unlocked_pll_half_width_rad: std::f64::consts::PI,
        unlocked_fll_half_width_hz: 1_000.0,
        missed_unlock_bias_sigma: 3.0,
    });
    let narrow_unlocked = lock_detector_probability_summary(LockDetectorProbabilityInput {
        thresholds,
        unlocked_dll_half_width: 0.5,
        unlocked_pll_half_width_rad: std::f64::consts::FRAC_PI_2,
        unlocked_fll_half_width_hz: 500.0,
        missed_unlock_bias_sigma: 3.0,
    });
    let severe_loss = lock_detector_probability_summary(LockDetectorProbabilityInput {
        thresholds,
        unlocked_dll_half_width: 1.0,
        unlocked_pll_half_width_rad: std::f64::consts::PI,
        unlocked_fll_half_width_hz: 1_000.0,
        missed_unlock_bias_sigma: 6.0,
    });

    assert!(narrow_unlocked.false_lock_probability > wide_unlocked.false_lock_probability);
    assert!(severe_loss.missed_unlock_probability < wide_unlocked.missed_unlock_probability);
    assert!(wide_unlocked.false_lock_probability.is_finite());
    assert!(wide_unlocked.missed_unlock_probability.is_finite());
}
