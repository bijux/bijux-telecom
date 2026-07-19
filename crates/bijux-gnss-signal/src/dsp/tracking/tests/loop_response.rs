use super::*;

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
        apply_pll_phase: true,
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
            apply_pll_phase: true,
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
