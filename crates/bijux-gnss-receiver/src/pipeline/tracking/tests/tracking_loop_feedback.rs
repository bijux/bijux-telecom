use super::*;

#[test]
fn apply_dll_code_loop_decreases_code_rate_for_positive_discriminator() {
    let coherent_integration_s = 5_000.0 / 1_023_000.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
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
fn apply_dll_code_loop_follows_reference_code_rate_step_without_dll_error() {
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_450.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.0,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!((update.code_rate_hz - 1_023_450.0).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_dll_code_loop_uses_coherent_interval_to_scale_gain() {
    let short = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 4_092,
        coherent_integration_s: 0.001,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.0,
        samples_per_code: 4_092,
    });
    let long = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples: 250.0,
        epoch_len_samples: 40_920,
        coherent_integration_s: 0.010,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.25,
        samples_per_chip: 4.0,
        samples_per_code: 40_920,
    });

    assert!(
        (long.code_rate_hz - 1_023_000.0).abs() > (short.code_rate_hz - 1_023_000.0).abs(),
        "short={short:?} long={long:?}"
    );
}

#[test]
fn apply_dll_code_loop_pulls_positive_code_error_toward_prompt() {
    let current_code_phase_samples = 250.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: 0.4,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!(update.code_phase_samples > current_code_phase_samples, "update={update:?}");
}

#[test]
fn apply_dll_code_loop_pulls_negative_code_error_toward_prompt() {
    let current_code_phase_samples = 250.0;
    let update = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: 1_023_000.0,
        previous_reference_code_rate_hz: 1_023_000.0,
        reference_code_rate_hz: 1_023_000.0,
        current_code_phase_samples,
        epoch_len_samples: 5_000,
        coherent_integration_s: 5_000.0 / 1_023_000.0,
        nominal_code_rate_hz: 1_023_000.0,
        dll_bw_hz: 2.0,
        dll_err: -0.4,
        samples_per_chip: 4.887585532746823,
        samples_per_code: 5_000,
    });

    assert!(update.code_phase_samples < current_code_phase_samples, "update={update:?}");
}

#[test]
fn correlate_epoch_aligns_prompt_with_non_integer_rate_sampled_code() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_phase_chips = 245.25;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 3 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        code_phase_chips,
        sample_count,
    )
    .expect("valid sampled code");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let correlator = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples,
        early_late_spacing_chips: 0.5,
    });

    assert!(correlator.prompt.norm() > correlator.early.norm());
    assert!(correlator.prompt.norm() > correlator.late.norm());
}

#[test]
fn correlate_epoch_uses_tracked_code_rate_for_code_rate_offset_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let signal_code_rate_hz = config.code_freq_basis_hz + 1_200.0;
    let code_phase_chips = 87.375;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 9 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        signal_code_rate_hz,
        code_phase_chips,
        sample_count,
    )
    .expect("valid sampled code with signal code-rate offset");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let nominal = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples,
        early_late_spacing_chips: 0.5,
    });
    let matched = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: signal_code_rate_hz,
        code_phase_samples,
        early_late_spacing_chips: 0.5,
    });

    assert!(
        matched.prompt.norm() > nominal.prompt.norm(),
        "matched_prompt={} nominal_prompt={}",
        matched.prompt.norm(),
        nominal.prompt.norm(),
    );
    assert!(matched.prompt.norm() > matched.early.norm());
    assert!(matched.prompt.norm() > matched.late.norm());
}

#[test]
fn correlate_epoch_uses_tracked_carrier_phase_for_phase_offset_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
    let carrier_phase_rad = 0.35;
    let carrier_phase_cycles = carrier_phase_rad / std::f64::consts::TAU;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad,
            cn0_db_hz: 70.0,
            navigation_data: false.into(),
        },
        0xC0A5_1E,
        0.001,
    );

    let unmatched = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples: 0.0,
        early_late_spacing_chips: 0.5,
    });
    let matched = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples: 0.0,
        early_late_spacing_chips: 0.5,
    });

    assert!(
        matched.prompt.re > unmatched.prompt.re,
        "matched={:?} unmatched={:?}",
        matched.prompt,
        unmatched.prompt,
    );
    assert!(
        matched.prompt.im.abs() < unmatched.prompt.im.abs(),
        "matched={:?} unmatched={:?}",
        matched.prompt,
        unmatched.prompt,
    );
}

#[test]
fn correlate_epoch_keeps_carrier_phase_aligned_across_nonzero_epoch_starts() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 5 };
    let carrier_hz = 120.0;
    let epoch_len_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let second_epoch_start_s = epoch_len_samples as f64 / config.sampling_freq_hz;
    let second_epoch_start_phase_cycles = 0.18;
    let initial_phase_cycles = second_epoch_start_phase_cycles - carrier_hz * second_epoch_start_s;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: carrier_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: initial_phase_cycles * std::f64::consts::TAU,
            cn0_db_hz: 90.0,
            navigation_data: false.into(),
        },
        0xC0A5_1E,
        0.002,
    );
    let second_epoch = super::frame_slice(&frame, epoch_len_samples, epoch_len_samples * 2);

    let correlator = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &second_epoch,
        sat,
        carrier_hz,
        carrier_phase_cycles: second_epoch_start_phase_cycles,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples: 0.0,
        early_late_spacing_chips: 0.5,
    });

    assert!(
        correlator.prompt.re > correlator.prompt.im.abs() * 10.0,
        "prompt={:?}",
        correlator.prompt,
    );
}

#[test]
fn apply_carrier_loop_advances_phase_and_frequency_from_pll_error() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
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
    assert!((update.carrier_phase_cycles - expected_phase_cycles).abs() < 1.0e-9, "{update:?}",);
}

#[test]
fn apply_carrier_loop_uses_fll_correction_during_pull_in() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 80.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 10.0,
        fll_err_hz: 30.0,
        apply_fll: true,
        apply_pll_frequency: false,
    });

    let fll_coefficients = first_order_angular_loop_coefficients(10.0, 0.001);
    let expected_carrier_hz = 80.0 + (30.0 * fll_coefficients.error_blend).clamp(-40.0, 40.0);
    assert!((update.carrier_hz - expected_carrier_hz).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_carrier_loop_accumulates_unwrapped_phase_across_epochs() {
    let first = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_500.0,
        current_carrier_phase_cycles: 128.25,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });
    let second = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: first.carrier_hz,
        current_carrier_phase_cycles: first.carrier_phase_cycles,
        current_carrier_rate_hz_per_s: first.carrier_rate_hz_per_s,
        epoch_len_samples: 4_092,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.001,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!((first.carrier_phase_cycles - 129.75).abs() < 1.0e-9, "{first:?}");
    assert!((second.carrier_phase_cycles - 131.25).abs() < 1.0e-9, "{second:?}");
    assert!(
        (second.carrier_phase_cycles - first.carrier_phase_cycles - 1.5).abs() < 1.0e-9,
        "{first:?} {second:?}"
    );
}

#[test]
fn apply_carrier_loop_preserves_continuous_phase_for_negative_doppler() {
    let update = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: -850.0,
        current_carrier_phase_cycles: 512.875,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 8_184,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.002,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.0,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!((update.carrier_hz + 850.0).abs() < 1.0e-9, "{update:?}");
    assert!((update.carrier_phase_cycles - 511.175).abs() < 1.0e-9, "{update:?}");
}

#[test]
fn apply_carrier_loop_uses_coherent_interval_to_scale_frequency_gain() {
    let short = super::apply_carrier_loop(super::CarrierLoopInput {
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
    let long = super::apply_carrier_loop(super::CarrierLoopInput {
        current_carrier_hz: 1_000.0,
        current_carrier_phase_cycles: 12.0,
        current_carrier_rate_hz_per_s: 0.0,
        epoch_len_samples: 40_920,
        sample_rate_hz: 4_092_000.0,
        coherent_integration_s: 0.010,
        pll_bw_hz: 8.0,
        pll_err_rad: 0.25,
        fll_bw_hz: 0.0,
        fll_err_hz: 0.0,
        apply_fll: false,
        apply_pll_frequency: true,
    });

    assert!(
        (long.carrier_hz - 1_000.0).abs() > (short.carrier_hz - 1_000.0).abs(),
        "short={short:?} long={long:?}"
    );
}

#[test]
fn dll_pull_in_increases_code_rate_for_faster_signal() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let signal_code_rate_hz = config.code_freq_basis_hz + 300.0;
    let code_phase_chips = 211.625;
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 14 };
    let samples = sample_ca_code(
        Prn(sat.prn),
        config.sampling_freq_hz,
        signal_code_rate_hz,
        code_phase_chips,
        samples_per_epoch,
    )
    .expect("valid sampled code with faster signal code rate");
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        samples.into_iter().map(|value| Complex::new(value, 0.0)).collect(),
    );
    let code_phase_samples = crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
        &config,
        &frame,
        code_phase_chips,
    );

    let correlator = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 0.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: config.code_freq_basis_hz,
        code_phase_samples,
        early_late_spacing_chips: 0.5,
    });
    let (dll_err, _, _, _) =
        discriminators(correlator.early, correlator.prompt, correlator.late, None);
    let code_loop = super::apply_dll_code_loop(super::CodeLoopInput {
        current_code_rate_hz: config.code_freq_basis_hz,
        previous_reference_code_rate_hz: config.code_freq_basis_hz,
        reference_code_rate_hz: config.code_freq_basis_hz,
        current_code_phase_samples: code_phase_samples,
        epoch_len_samples: samples_per_epoch,
        coherent_integration_s: samples_per_epoch as f64 / config.sampling_freq_hz,
        nominal_code_rate_hz: config.code_freq_basis_hz,
        dll_bw_hz: 900.0,
        dll_err,
        samples_per_chip: samples_per_epoch as f64 / config.code_length as f64,
        samples_per_code: samples_per_epoch,
    });

    assert!(
        code_loop.code_rate_hz > config.code_freq_basis_hz,
        "dll_err={dll_err} code_loop={code_loop:?}",
    );
}

#[test]
fn detect_sample_rate_mismatch_flags_persistent_phase_drift() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 24.0, 48.0, 72.0, 96.0]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_000,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            dll_err: 0.35,
            cn0_dbhz: 46.0,
            lock: true,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: ChannelState::Tracking.to_string(),
            lock_state_reason: Some("locked".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    let diagnostic =
        super::detect_sample_rate_mismatch(&epochs, 5_000).expect("diagnostic must exist");
    assert_eq!(diagnostic.first_unstable_epoch_index, 1);
    assert!(diagnostic.max_abs_phase_step_samples >= 24.0);
}

#[test]
fn detect_sample_rate_mismatch_ignores_small_stable_phase_steps() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 1.5, 2.0, 1.0, 0.5]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_000,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            dll_err: 0.05,
            cn0_dbhz: 46.0,
            lock: true,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: ChannelState::Tracking.to_string(),
            lock_state_reason: Some("locked".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    assert!(super::detect_sample_rate_mismatch(&epochs, 5_000).is_none());
}

#[test]
fn detect_sample_rate_mismatch_flags_catastrophic_pull_in_phase_jump() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 11 };
    let epochs = [0.0, 2.5, 4.0, 4_452.0, 4_452.5]
        .into_iter()
        .enumerate()
        .map(|(index, code_phase_samples)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: (index as u64) * 5_050,
            sat,
            code_phase_samples: Chips(code_phase_samples),
            cn0_dbhz: 48.0,
            lock: true,
            fll_lock: index > 0,
            lock_state: ChannelState::PullIn.to_string(),
            lock_state_reason: Some("carrier_pull_in".to_string()),
            ..TrackEpoch::default()
        })
        .collect::<Vec<_>>();

    let diagnostic =
        super::detect_sample_rate_mismatch(&epochs, 5_050).expect("diagnostic must exist");
    assert_eq!(diagnostic.first_unstable_epoch_index, 3);
    assert!(diagnostic.max_abs_phase_step_samples >= 500.0);
}
