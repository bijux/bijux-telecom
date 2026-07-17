use super::*;

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

    let correlation = correlate_early_prompt_late(
        EarlyPromptLateCorrelatorInput {
            samples: &samples,
            sample_rate_hz: 4_000.0,
            carrier_hz: 0.0,
            carrier_phase_offset_radians: 0.0,
            base_chip_phase: 0.0,
            chips_per_sample: 0.25,
            early_late_spacing_chips: 0.5,
        },
        |_| 1.0,
    );

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
        EarlyPromptLateCorrelatorInput {
            samples: &samples,
            sample_rate_hz,
            carrier_hz,
            carrier_phase_offset_radians: 0.0,
            base_chip_phase: 0.0,
            chips_per_sample: 0.25,
            early_late_spacing_chips: 0.5,
        },
        |_| 1.0,
    );

    assert!((correlation.prompt.re - 8.0).abs() < 1.0e-5, "{correlation:?}");
    assert!(correlation.prompt.im.abs() < 1.0e-5, "{correlation:?}");
}
