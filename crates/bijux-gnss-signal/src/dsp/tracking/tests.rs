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

#[test]
fn dll_discriminator_is_positive_when_early_energy_exceeds_late_energy() {
    let (dll, pll, fll, lock) = discriminators(
        Complex::new(8.0, 0.0),
        Complex::new(12.0, 0.0),
        Complex::new(4.0, 0.0),
        None,
    );

    assert!(dll > 0.0, "dll={dll}");
    assert_eq!(pll, 0.0);
    assert_eq!(fll, 0.0);
    assert!(lock);
}

#[test]
fn dll_discriminator_is_negative_when_late_energy_exceeds_early_energy() {
    let (dll, pll, fll, lock) = discriminators(
        Complex::new(4.0, 0.0),
        Complex::new(12.0, 0.0),
        Complex::new(8.0, 0.0),
        None,
    );

    assert!(dll < 0.0, "dll={dll}");
    assert_eq!(pll, 0.0);
    assert_eq!(fll, 0.0);
    assert!(lock);
}

#[test]
fn estimate_cn0_dbhz_recovers_known_cn0_from_prompt_and_noise_weighting() {
    let sample_rate_hz = 4_092_000.0_f64;
    let coherent_samples = 4092.0_f64;
    let noise_weight_energy = 8_184.0_f64;
    let expected_cn0_dbhz = 58.0;
    let expected_cn0_linear = 10.0_f64.powf(expected_cn0_dbhz / 10.0);
    let noise = Complex::new(1.0, 0.0);
    let noise_power_per_sample = noise.norm_sqr() as f64 / noise_weight_energy;
    let prompt_noise_power = coherent_samples * noise_power_per_sample;
    let coherent_signal_power =
        expected_cn0_linear * coherent_samples.powi(2) * noise_power_per_sample / sample_rate_hz;
    let prompt = Complex::new((coherent_signal_power + prompt_noise_power).sqrt() as f32, 0.0);

    let measured =
        estimate_cn0_dbhz(prompt, noise, sample_rate_hz, coherent_samples, noise_weight_energy);

    assert!((measured - expected_cn0_dbhz).abs() < 1e-6, "measured={measured}");
}

#[test]
fn estimate_cn0_dbhz_returns_zero_for_invalid_estimator_parameters() {
    let prompt = Complex::new(1.0, 0.0);
    let noise = Complex::new(1.0, 0.0);

    assert_eq!(estimate_cn0_dbhz(prompt, noise, 0.0, 4092.0, 8_184.0), 0.0);
    assert_eq!(estimate_cn0_dbhz(prompt, noise, 4_092_000.0, 0.0, 8_184.0), 0.0);
    assert_eq!(estimate_cn0_dbhz(prompt, noise, 4_092_000.0, 4092.0, 0.0), 0.0);
}

#[test]
fn carrier_frequency_error_hz_from_phase_delta_matches_quarter_cycle_over_one_ms() {
    let measured = carrier_frequency_error_hz_from_phase_delta(std::f64::consts::FRAC_PI_2, 0.001);

    assert!((measured - 250.0).abs() < 1.0e-9, "measured={measured}");
}

#[test]
fn carrier_frequency_error_hz_from_phase_delta_rejects_invalid_inputs() {
    assert_eq!(carrier_frequency_error_hz_from_phase_delta(f64::NAN, 0.001), 0.0);
    assert_eq!(carrier_frequency_error_hz_from_phase_delta(0.5, 0.0), 0.0);
}

#[test]
fn wrap_phase_cycles_signed_centers_large_offsets_around_zero() {
    assert!((wrap_phase_cycles_signed(0.75) + 0.25).abs() < 1.0e-9);
    assert!((wrap_phase_cycles_signed(-0.75) - 0.25).abs() < 1.0e-9);
}

#[test]
fn wrap_phase_radians_positive_keeps_results_in_positive_turn() {
    let wrapped = wrap_phase_radians_positive(-std::f64::consts::FRAC_PI_2);
    assert!((wrapped - (std::f64::consts::TAU - std::f64::consts::FRAC_PI_2)).abs() < 1.0e-9);
}

#[test]
fn carrier_phase_offset_radians_wraps_whole_cycle_offsets() {
    let wrapped = carrier_phase_offset_radians(1.25);
    assert!((wrapped - std::f64::consts::FRAC_PI_2).abs() < 1.0e-9);
}

#[test]
fn wrapped_phase_delta_cycles_chooses_shortest_signed_delta() {
    assert!((wrapped_phase_delta_cycles(0.05, 0.95) - 0.10).abs() < 1.0e-9);
    assert!((wrapped_phase_delta_cycles(0.95, 0.05) + 0.10).abs() < 1.0e-9);
}

#[test]
fn correlate_early_prompt_late_accumulates_constant_prompt_signal() {
    let samples = vec![Complex::new(1.0, 0.0); 8];

    let correlation =
        correlate_early_prompt_late(&samples, 4_000.0, 0.0, 0.0, 0.0, 0.25, 0.5, |_| 1.0);

    assert_eq!(correlation.early, Complex::new(8.0, 0.0));
    assert_eq!(correlation.prompt, Complex::new(8.0, 0.0));
    assert_eq!(correlation.late, Complex::new(8.0, 0.0));
    assert_eq!(correlation.early_late_noise_weight_energy, 0.0);
}

#[test]
fn correlate_early_prompt_late_wipes_off_known_carrier_rotation() {
    let sample_rate_hz = 4_000.0;
    let carrier_hz = 250.0;
    let samples = (0..8)
        .map(|sample_index| {
            let phase = std::f64::consts::TAU * carrier_hz * sample_index as f64 / sample_rate_hz;
            Complex::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect::<Vec<_>>();

    let correlation = correlate_early_prompt_late(
        &samples,
        sample_rate_hz,
        carrier_hz,
        0.0,
        0.0,
        0.25,
        0.5,
        |_| 1.0,
    );

    assert!((correlation.prompt.re - 8.0).abs() < 1.0e-5, "{correlation:?}");
    assert!(correlation.prompt.im.abs() < 1.0e-5, "{correlation:?}");
}

#[test]
fn first_order_loop_coefficients_scale_with_bandwidth_and_integration_time() {
    let narrow = first_order_loop_coefficients(2.0, 0.001);
    let wide = first_order_loop_coefficients(10.0, 0.001);
    let long = first_order_loop_coefficients(2.0, 0.020);

    assert!(wide.error_blend > narrow.error_blend, "{wide:?} {narrow:?}");
    assert!(wide.rate_gain_hz > narrow.rate_gain_hz, "{wide:?} {narrow:?}");
    assert!(long.error_blend > narrow.error_blend, "{long:?} {narrow:?}");
    assert!(long.rate_gain_hz < narrow.rate_gain_hz, "{long:?} {narrow:?}");
}

#[test]
fn first_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 12.5;
    let design = super::loop_observer_design(
        super::LoopFilterOrder::First,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
        super::LoopFilterOrder::First,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn first_order_loop_design_places_requested_closed_loop_pole() {
    let coherent_integration_s = 0.001;
    let design =
        super::loop_observer_design(super::LoopFilterOrder::First, 8.0, coherent_integration_s);
    let transition = super::corrected_loop_transition(
        super::LoopFilterOrder::First,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (transition[0][0] - design.closed_loop_pole).abs() <= 1.0e-12,
        "design={design:?} transition={transition:?}",
    );
}

fn loop_observer_response<F>(
    order: super::LoopFilterOrder,
    correction_gains: [f64; 3],
    coherent_integration_s: f64,
    epochs: usize,
    measurement_at_epoch: F,
) -> Vec<[f64; 3]>
where
    F: Fn(usize) -> f64,
{
    let dimension = order.as_usize();
    let transition =
        super::corrected_loop_transition(order, coherent_integration_s, correction_gains);
    let mut state = [0.0; 3];
    let mut response = Vec::with_capacity(epochs);
    for epoch in 0..epochs {
        let measurement = measurement_at_epoch(epoch);
        let mut next_state = [0.0; 3];
        for row in 0..dimension {
            next_state[row] = correction_gains[row] * measurement;
            for column in 0..dimension {
                next_state[row] += transition[row][column] * state[column];
            }
        }
        state = next_state;
        response.push(state);
    }
    response
}

#[test]
fn loop_filter_step_responses_converge_for_each_order() {
    let coherent_integration_s = 0.001;
    let cases = [
        (super::LoopFilterOrder::First, 8.0),
        (super::LoopFilterOrder::Second, 6.0),
        (super::LoopFilterOrder::Third, 18.0),
    ];

    for (order, bandwidth_hz) in cases {
        let design = super::loop_observer_design(order, bandwidth_hz, coherent_integration_s);
        let response = loop_observer_response(
            order,
            design.correction_gains,
            coherent_integration_s,
            6_000,
            |_| 1.0,
        );

        let final_output = response.last().expect("step response sample")[0];
        let peak_output = response.iter().map(|state| state[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (final_output - 1.0).abs() <= 1.0e-8,
            "order={order:?} design={design:?} final_output={final_output}",
        );
        assert!(peak_output <= 1.25, "order={order:?} design={design:?} peak_output={peak_output}",);
    }
}

#[test]
fn loop_filter_ramp_responses_match_order_capability() {
    let coherent_integration_s = 0.001;
    let ramp_rate_units_per_s = 0.75;
    let cases = [
        (super::LoopFilterOrder::First, 8.0),
        (super::LoopFilterOrder::Second, 6.0),
        (super::LoopFilterOrder::Third, 18.0),
    ];

    for (order, bandwidth_hz) in cases {
        let design = super::loop_observer_design(order, bandwidth_hz, coherent_integration_s);
        let response = loop_observer_response(
            order,
            design.correction_gains,
            coherent_integration_s,
            12_000,
            |epoch| epoch as f64 * coherent_integration_s * ramp_rate_units_per_s,
        );
        let final_state = response.last().expect("ramp response sample");
        let final_measurement =
            (response.len() - 1) as f64 * coherent_integration_s * ramp_rate_units_per_s;
        let final_error = final_measurement - final_state[0];

        match order {
            super::LoopFilterOrder::First => {
                let ramp_step_units = ramp_rate_units_per_s * coherent_integration_s;
                let expected_lag =
                    design.closed_loop_pole * ramp_step_units / design.correction_gains[0];
                assert!(
                    (final_error - expected_lag).abs() <= 1.0e-8,
                    "design={design:?} final_error={final_error} expected_lag={expected_lag}",
                );
            }
            super::LoopFilterOrder::Second | super::LoopFilterOrder::Third => {
                let next_epoch_measurement =
                    final_measurement + ramp_rate_units_per_s * coherent_integration_s;
                assert!(
                        (next_epoch_measurement - final_state[0]).abs() <= 1.0e-6,
                        "order={order:?} design={design:?} next_epoch_measurement={next_epoch_measurement} position_state={}",
                        final_state[0],
                    );
                assert!(
                    (final_state[1] - ramp_rate_units_per_s).abs() <= 1.0e-6,
                    "order={order:?} design={design:?} rate_state={}",
                    final_state[1],
                );
            }
        }
    }
}

#[test]
fn second_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 6.0;
    let design = super::loop_observer_design(
        super::LoopFilterOrder::Second,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
        super::LoopFilterOrder::Second,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn second_order_loop_design_repeats_the_requested_pole() {
    let coherent_integration_s = 0.001;
    let design =
        super::loop_observer_design(super::LoopFilterOrder::Second, 6.0, coherent_integration_s);
    let transition = super::corrected_loop_transition(
        super::LoopFilterOrder::Second,
        coherent_integration_s,
        design.correction_gains,
    );
    let trace = transition[0][0] + transition[1][1];
    let determinant = transition[0][0] * transition[1][1] - transition[0][1] * transition[1][0];

    assert!((trace - 2.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
    assert!(
        (determinant - design.closed_loop_pole * design.closed_loop_pole).abs() <= 1.0e-9,
        "{transition:?}",
    );
}

#[test]
fn third_order_loop_design_matches_requested_noise_bandwidth() {
    let coherent_integration_s = 0.001;
    let target_bandwidth_hz = 18.0;
    let design = super::loop_observer_design(
        super::LoopFilterOrder::Third,
        target_bandwidth_hz,
        coherent_integration_s,
    );
    let measured_bandwidth_hz = super::loop_noise_bandwidth_hz(
        super::LoopFilterOrder::Third,
        coherent_integration_s,
        design.correction_gains,
    );

    assert!(
        (measured_bandwidth_hz - target_bandwidth_hz).abs() <= 1.0e-6,
        "design={design:?} measured_bandwidth_hz={measured_bandwidth_hz}",
    );
}

#[test]
fn third_order_loop_design_repeats_the_requested_pole() {
    let coherent_integration_s = 0.001;
    let design =
        super::loop_observer_design(super::LoopFilterOrder::Third, 18.0, coherent_integration_s);
    let transition = super::corrected_loop_transition(
        super::LoopFilterOrder::Third,
        coherent_integration_s,
        design.correction_gains,
    );
    let trace = transition[0][0] + transition[1][1] + transition[2][2];
    let principal_minors = transition[0][0] * transition[1][1]
        + transition[0][0] * transition[2][2]
        + transition[1][1] * transition[2][2]
        - transition[0][1] * transition[1][0]
        - transition[0][2] * transition[2][0]
        - transition[1][2] * transition[2][1];
    let determinant = transition[0][0]
        * (transition[1][1] * transition[2][2] - transition[1][2] * transition[2][1])
        - transition[0][1]
            * (transition[1][0] * transition[2][2] - transition[1][2] * transition[2][0])
        + transition[0][2]
            * (transition[1][0] * transition[2][1] - transition[1][1] * transition[2][0]);

    assert!((trace - 3.0 * design.closed_loop_pole).abs() <= 1.0e-9, "{transition:?}");
    assert!(
        (principal_minors - 3.0 * design.closed_loop_pole * design.closed_loop_pole).abs()
            <= 1.0e-9,
        "{transition:?}",
    );
    assert!((determinant - design.closed_loop_pole.powi(3)).abs() <= 1.0e-9, "{transition:?}",);
}

#[test]
fn phase_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
    let narrow = phase_lock_loop_coefficients(5.0, 0.001);
    let wide = phase_lock_loop_coefficients(15.0, 0.001);

    assert!(narrow.phase_blend.is_finite());
    assert!(narrow.frequency_gain_hz_per_rad.is_finite());
    assert!(narrow.frequency_rate_gain_hz_per_s_per_rad.is_finite());
    assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
    assert!(
        wide.frequency_gain_hz_per_rad > narrow.frequency_gain_hz_per_rad,
        "{wide:?} {narrow:?}"
    );
    assert!(
        wide.frequency_rate_gain_hz_per_s_per_rad > narrow.frequency_rate_gain_hz_per_s_per_rad,
        "{wide:?} {narrow:?}"
    );
}

#[test]
fn angular_first_order_loop_coefficients_strengthen_frequency_response() {
    let linear = first_order_loop_coefficients(10.0, 0.001);
    let angular = first_order_angular_loop_coefficients(10.0, 0.001);

    assert_eq!(angular, linear);
}

#[test]
fn delay_lock_loop_coefficients_are_finite_and_stronger_for_wider_bandwidths() {
    let narrow = delay_lock_loop_coefficients(2.0, 0.001);
    let wide = delay_lock_loop_coefficients(8.0, 0.001);

    assert!(narrow.phase_blend.is_finite());
    assert!(narrow.rate_gain_hz_per_chip.is_finite());
    assert!(wide.phase_blend > narrow.phase_blend, "{wide:?} {narrow:?}");
    assert!(wide.rate_gain_hz_per_chip > narrow.rate_gain_hz_per_chip, "{wide:?} {narrow:?}");
}

#[test]
fn dll_lock_threshold_relaxes_for_subsample_early_late_spacing() {
    assert_eq!(dll_lock_threshold(1.0, 0.5), 0.6);
    assert_eq!(dll_lock_threshold(4.0, 0.5), 0.2);
}

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

#[test]
fn apply_code_loop_decreases_code_rate_for_positive_discriminator() {
    let coherent_integration_s = coherent_integration_seconds(5_000, 1_023_000.0);
    let update = apply_code_loop(CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    let expected = 1_023_000.0
        - delay_lock_loop_coefficients(2.0, coherent_integration_s).rate_gain_hz_per_chip * 0.25;
    assert!((update.code_rate_hz - expected).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn delay_lock_loop_step_response_reduces_prompt_misalignment() {
    let coherent_integration_s = 0.001;
    let input = CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 4_092,
        coherent_integration_s,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 4.0,
        dll_err: 0.25,
        samples_per_chip: 4.0,
        samples_per_code: 4_092,
    };
    let coefficients = delay_lock_loop_coefficients(input.dll_bw_hz, coherent_integration_s);
    let predicted = super::predict_code_phase_samples(
        input.current_code_phase_samples,
        input.epoch_len_samples,
        input.current_code_rate_hz,
        input.nominal_code_rate_hz,
        input.samples_per_code,
    );
    let corrected = apply_code_loop(input);

    assert!(
        corrected.code_phase_samples > predicted,
        "coefficients={coefficients:?} corrected={corrected:?} predicted={predicted}",
    );
}

#[test]
fn apply_code_loop_follows_reference_code_rate_step_without_dll_error() {
    let coherent_integration_s = coherent_integration_seconds(5_000, 1_023_000.0);
    let update = apply_code_loop(CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_450.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.0,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!((update.code_rate_hz - 1_023_450.0).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_code_loop_reduces_phase_error_when_reference_code_rate_changes_each_epoch() {
    let coherent_integration_s = 0.001;
    let nominal_code_rate_hz = 1_023_000.0;
    let epoch_len_samples = 4_092;
    let samples_per_chip = 4.0;
    let samples_per_code = 4_092;
    let reference_code_rates_hz = [
        nominal_code_rate_hz,
        nominal_code_rate_hz + 15.0,
        nominal_code_rate_hz + 35.0,
        nominal_code_rate_hz + 60.0,
        nominal_code_rate_hz + 90.0,
        nominal_code_rate_hz + 125.0,
    ];
    let mut aided_code_rate_hz = nominal_code_rate_hz;
    let mut aided_code_phase_samples = 250.0;
    let mut frozen_code_rate_hz = nominal_code_rate_hz;
    let mut frozen_code_phase_samples = 250.0;
    let mut expected_code_phase_samples = 250.0;
    let mut previous_reference_code_rate_hz = reference_code_rates_hz[0];

    for reference_code_rate_hz in reference_code_rates_hz.iter().copied().skip(1) {
        expected_code_phase_samples = predict_code_phase_samples(
            expected_code_phase_samples,
            epoch_len_samples,
            reference_code_rate_hz,
            nominal_code_rate_hz,
            samples_per_code,
        );

        let aided_update = apply_code_loop(CodeLoopInput {
            current_code_rate_hz: aided_code_rate_hz,
            previous_reference_code_rate_hz,
            reference_code_rate_hz,
            current_code_phase_samples: aided_code_phase_samples,
            epoch_len_samples,
            coherent_integration_s,
            nominal_code_rate_hz,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip,
            samples_per_code,
        });
        aided_code_rate_hz = aided_update.code_rate_hz;
        aided_code_phase_samples = aided_update.code_phase_samples;

        let frozen_update = apply_code_loop(CodeLoopInput {
            current_code_rate_hz: frozen_code_rate_hz,
            previous_reference_code_rate_hz: nominal_code_rate_hz,
            reference_code_rate_hz: nominal_code_rate_hz,
            current_code_phase_samples: frozen_code_phase_samples,
            epoch_len_samples,
            coherent_integration_s,
            nominal_code_rate_hz,
            dll_bw_hz: 2.0,
            dll_err: 0.0,
            samples_per_chip,
            samples_per_code,
        });
        frozen_code_rate_hz = frozen_update.code_rate_hz;
        frozen_code_phase_samples = frozen_update.code_phase_samples;
        previous_reference_code_rate_hz = reference_code_rate_hz;
    }

    let aided_phase_error_samples = (aided_code_phase_samples - expected_code_phase_samples).abs();
    let frozen_phase_error_samples =
        (frozen_code_phase_samples - expected_code_phase_samples).abs();

    assert!(aided_phase_error_samples <= 1.0e-9, "{aided_phase_error_samples}");
    assert!(
        frozen_phase_error_samples > 0.5,
        "aided={aided_phase_error_samples} frozen={frozen_phase_error_samples}"
    );
}

#[test]
fn apply_carrier_tracking_loop_advances_phase_and_frequency_from_pll_error() {
    let update = apply_carrier_tracking_loop(CarrierTrackingLoopInput {
        current_carrier_hz: 1_000.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    let pll_coefficients = phase_lock_loop_coefficients(8.0, 0.001);
    assert!(
        (update.carrier_hz - (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)).abs()
            < 1.0e-9,
        "{update:?}"
    );
    assert!(
        (update.carrier_rate_hz_per_s
            - pll_coefficients.frequency_rate_gain_hz_per_s_per_rad * 0.25)
            .abs()
            < 1.0e-9,
        "{update:?}"
    );
    let expected_phase_cycles = 12.0
        + (1_000.0 + (1_000.0 + pll_coefficients.frequency_gain_hz_per_rad * 0.25)) * 0.0005
        + pll_coefficients.phase_blend * 0.25 / std::f64::consts::TAU;
    assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn phase_lock_loop_ramp_response_tracks_constant_frequency_rate() {
    let coherent_integration_s = 0.001;
    let true_initial_frequency_hz = 500.0;
    let true_frequency_rate_hz_per_s = 40.0;
    let mut true_frequency_hz = true_initial_frequency_hz;
    let mut true_phase_cycles = 0.0;
    let mut estimated_frequency_hz = true_initial_frequency_hz;
    let mut estimated_phase_cycles = 0.0;
    let mut estimated_frequency_rate_hz_per_s = 0.0;

    for _ in 0..1_000 {
        let next_true_frequency_hz =
            true_frequency_hz + true_frequency_rate_hz_per_s * coherent_integration_s;
        true_phase_cycles +=
            (true_frequency_hz + next_true_frequency_hz) * coherent_integration_s * 0.5;
        true_frequency_hz = next_true_frequency_hz;
        let phase_error_rad = (true_phase_cycles - estimated_phase_cycles) * std::f64::consts::TAU;
        let update = apply_carrier_tracking_loop(CarrierTrackingLoopInput {
            current_carrier_hz: estimated_frequency_hz,
            current_carrier_phase_cycles: estimated_phase_cycles,
            current_carrier_rate_hz_per_s: estimated_frequency_rate_hz_per_s,
            epoch_len_samples: 4_092,
            sample_rate_hz: 4_092_000.0,
            coherent_integration_s,
            pll_bw_hz: 18.0,
            pll_err_rad: phase_error_rad,
            fll_bw_hz: 0.0,
            fll_err_hz: 0.0,
            apply_fll: false,
            apply_pll_frequency: true,
        });
        estimated_frequency_hz = update.carrier_hz;
        estimated_phase_cycles = update.carrier_phase_cycles;
        estimated_frequency_rate_hz_per_s = update.carrier_rate_hz_per_s;
    }

    assert!(
        (estimated_frequency_rate_hz_per_s - true_frequency_rate_hz_per_s).abs() < 2.0,
        "estimated_frequency_rate_hz_per_s={estimated_frequency_rate_hz_per_s}",
    );
    assert!(
        (true_frequency_hz - estimated_frequency_hz).abs() < 1.0,
        "true_frequency_hz={true_frequency_hz} estimated_frequency_hz={estimated_frequency_hz}",
    );
    assert!(
        (true_phase_cycles - estimated_phase_cycles).abs() < 1.0,
        "true_phase_cycles={true_phase_cycles} estimated_phase_cycles={estimated_phase_cycles}",
    );
}

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
