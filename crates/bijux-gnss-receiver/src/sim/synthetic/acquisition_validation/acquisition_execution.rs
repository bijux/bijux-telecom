fn measure_truth_guided_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    measure_truth_guided_acquisition_trial_with_expected_measured_doppler_hz(
        config,
        frame,
        signal,
        signal.doppler_hz,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
    )
}

fn measure_truth_guided_acquisition_trial_with_expected_measured_doppler_hz(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    expected_measured_doppler_hz: f64,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    acquisition_trial_measurement_from_result(
        config,
        frame,
        signal,
        result,
        expected_measured_doppler_hz,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
    )
}

fn acquisition_trial_measurement_from_result(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    result: crate::api::core::AcqResult,
    expected_measured_doppler_hz: f64,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples(config, frame, signal.code_phase_chips);
    let code_phase_error_samples = wrapped_code_phase_error_samples(
        result.code_phase_samples,
        expected_code_phase_samples,
        period_samples,
    );
    let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
        config.intermediate_freq_hz,
        signal.sat,
        signal.signal_band,
        signal.signal_code,
        signal.glonass_frequency_channel,
        result.carrier_hz.0,
    );
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1) as f64;
    let doppler_error_bins =
        (measured_doppler_hz - expected_measured_doppler_hz).abs() / doppler_step_hz;
    let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);
    let detected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected)
        && code_phase_error_samples <= code_phase_tolerance_samples
        && doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON;

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted,
        detected,
        code_phase_error_samples: Some(code_phase_error_samples),
        doppler_error_bins: Some(doppler_error_bins),
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

fn measure_target_absent_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    let non_rejected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected);

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted: non_rejected,
        detected: false,
        code_phase_error_samples: None,
        doppler_error_bins: None,
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

fn acquisition_result_for_target_signal(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqResult {
    let acquisition = crate::pipeline::acquisition::Acquisition::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    acquisition
        .run_fft_for_requests(frame, &[acquisition_request_for_signal(config, signal)])
        .remove(0)
}

fn acquisition_request_for_signal(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqRequest {
    crate::api::core::AcqRequest {
        sat: signal.sat,
        glonass_frequency_channel: signal.glonass_frequency_channel,
        signal_band: signal.signal_band,
        signal_code: signal.signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: Some(signal.doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }
}

fn scale_synthetic_capture_frame(frame: &SamplesFrame) -> SamplesFrame {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}

fn scaled_synthetic_acquisition_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    description: &str,
) -> SamplesFrame {
    let frame = generate_l1_ca_multi(config, scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        scenario,
        &frame,
        "2026-07-14T00:00:00Z",
        Some(description.to_string()),
    );
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    )
}
