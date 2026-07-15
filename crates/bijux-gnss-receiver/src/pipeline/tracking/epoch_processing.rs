fn default_tracking_assumptions(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
) -> TrackingAssumptions {
    tracking_assumptions(signal_model, resolve_signal_tracking_params(config, signal_model))
}

fn tracking_assumptions(
    signal_model: &TrackingSignalModel,
    params: TrackingParams,
) -> TrackingAssumptions {
    TrackingAssumptions {
        integration_ms: params.integration_ms,
        early_late_spacing_chips: params.early_late_spacing_chips,
        dll_bw_hz: params.dll_bw_hz,
        pll_bw_hz: params.pll_bw_hz,
        fll_bw_hz: params.fll_bw_hz,
        discriminator_family: signal_model.discriminator_family.label().to_string(),
        aiding_mode: signal_model.aiding_mode.label().to_string(),
    }
}

fn tracking_frame_start_offset(
    frame: &SamplesFrame,
    start_sample_index: Option<u64>,
) -> Option<usize> {
    let Some(start_sample_index) = start_sample_index else {
        return Some(0);
    };
    let frame_start = frame.t0.sample_index;
    let frame_end = frame_start + frame.len() as u64;
    if start_sample_index >= frame_end {
        return None;
    }
    if start_sample_index <= frame_start {
        return Some(0);
    }
    Some((start_sample_index - frame_start) as usize)
}

fn tracked_signal_center_hz(intermediate_freq_hz: f64, signal: SignalSpec) -> f64 {
    intermediate_freq_hz + (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())
}

fn tracked_signal_doppler_hz(
    intermediate_freq_hz: f64,
    tracked_carrier_hz: f64,
    signal: SignalSpec,
) -> f64 {
    tracked_carrier_hz - tracked_signal_center_hz(intermediate_freq_hz, signal)
}

fn normalize_acquisition_carrier_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    acquisition: &bijux_gnss_core::api::AcqResult,
) -> f64 {
    let acquisition_carrier_hz = acquisition.carrier_hz.0;
    let tracked_center_hz =
        tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec);
    let looks_like_doppler_only_seed = tracked_center_hz.abs() > 1.0
        && (acquisition_carrier_hz - acquisition.doppler_hz.0).abs() <= 1.0e-9;
    if looks_like_doppler_only_seed {
        tracked_center_hz + acquisition.doppler_hz.0
    } else {
        acquisition_carrier_hz
    }
}

fn carrier_aided_code_rate_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> f64 {
    carrier_aiding_reference(config, signal_model, tracked_carrier_hz)
        .map(|reference| reference.code_rate_hz)
        .unwrap_or(signal_model.code_rate_hz)
}

fn next_code_rate_reference_hz(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
    previous_reference_hz: f64,
    carrier_lock_ready: bool,
) -> f64 {
    if carrier_lock_ready {
        carrier_aided_code_rate_hz(config, signal_model, tracked_carrier_hz)
    } else {
        previous_reference_hz
    }
}

fn carrier_aiding_reference(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> Result<CarrierAidingReference, &'static str> {
    let tracked_doppler_hz = tracked_signal_doppler_hz(
        config.intermediate_freq_hz,
        tracked_carrier_hz,
        signal_model.signal_spec,
    );
    if !tracked_carrier_hz.is_finite() || !tracked_doppler_hz.is_finite() {
        return Err("carrier_aiding_non_finite_seed");
    }
    if tracked_doppler_hz.abs() > carrier_aiding_doppler_window_hz(config) {
        return Err("carrier_aiding_incompatible_signal_center");
    }
    let Some(code_rate_hz) = shared_path_code_rate_hz(
        tracked_doppler_hz,
        signal_model.signal_spec,
        signal_model.signal_spec,
    ) else {
        return Err("carrier_aiding_invalid_signal_metadata");
    };
    if !code_rate_hz.is_finite() || code_rate_hz <= 0.0 {
        return Err("carrier_aiding_invalid_code_rate");
    }
    Ok(CarrierAidingReference { tracked_carrier_hz, tracked_doppler_hz, code_rate_hz })
}

fn carrier_aiding_validation_required(signal_model: &TrackingSignalModel) -> bool {
    !matches!(signal_model.signal_spec.constellation, Constellation::Glonass)
}

fn code_rate_reference_label(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
    tracked_carrier_hz: f64,
) -> &'static str {
    if carrier_aiding_reference(config, signal_model, tracked_carrier_hz).is_ok() {
        "carrier_doppler"
    } else {
        "nominal"
    }
}

fn carrier_aiding_doppler_window_hz(config: &ReceiverPipelineConfig) -> f64 {
    (config.acquisition_doppler_search_hz.unsigned_abs() as f64
        + CARRIER_AID_DOPPLER_WINDOW_MARGIN_HZ)
        .max(CARRIER_AID_MIN_DOPPLER_WINDOW_HZ)
}

fn frame_slice(frame: &SamplesFrame, start: usize, end: usize) -> SamplesFrame {
    SamplesFrame::new(
        SampleTime {
            sample_index: frame.t0.sample_index + start as u64,
            sample_rate_hz: frame.t0.sample_rate_hz,
        },
        frame.dt_s,
        frame.iq[start..end].to_vec(),
    )
}

fn code_periods_in_frame(frame_len: usize, samples_per_code: usize) -> u64 {
    if frame_len == 0 {
        return 0;
    }
    (frame_len / samples_per_code.max(1)).max(1) as u64
}

fn classify_prompt_phase(
    raw_phase_cycles: f64,
    previous_aligned_phase_cycles: Option<f64>,
    nav_bit_phase_offset_cycles: f64,
    phase_transition_source: TrackingPhaseTransitionSource,
) -> PromptPhaseDecision {
    let same_offset_phase =
        wrap_phase_cycles_signed(raw_phase_cycles - nav_bit_phase_offset_cycles);
    let Some(previous_aligned_phase_cycles) = previous_aligned_phase_cycles else {
        return PromptPhaseDecision {
            aligned_phase_cycles: same_offset_phase,
            aligned_phase_delta_cycles: 0.0,
            nav_bit_phase_offset_cycles,
            nav_bit_transition: false,
            cycle_slip: false,
        };
    };

    let same_delta = wrapped_phase_delta_cycles(same_offset_phase, previous_aligned_phase_cycles);
    let flipped_offset =
        wrap_phase_cycles_signed(nav_bit_phase_offset_cycles + NAV_BIT_PHASE_STEP_CYCLES);
    let flipped_phase = wrap_phase_cycles_signed(raw_phase_cycles - flipped_offset);
    let flipped_delta = wrapped_phase_delta_cycles(flipped_phase, previous_aligned_phase_cycles);

    let nav_bit_transition = phase_transition_source.allows_half_cycle_transition()
        && same_delta.abs() > CYCLE_SLIP_PHASE_DELTA_CYCLES
        && (same_delta.abs() - NAV_BIT_PHASE_STEP_CYCLES).abs()
            <= NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES
        && flipped_delta.abs() <= NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES
        && same_delta.abs() - flipped_delta.abs() >= NAV_BIT_PHASE_MIN_IMPROVEMENT_CYCLES
        && flipped_delta.abs() < same_delta.abs();

    let (aligned_phase_cycles, nav_bit_phase_offset_cycles, chosen_delta) = if nav_bit_transition {
        (flipped_phase, flipped_offset, flipped_delta)
    } else {
        (same_offset_phase, nav_bit_phase_offset_cycles, same_delta)
    };

    PromptPhaseDecision {
        aligned_phase_cycles,
        aligned_phase_delta_cycles: chosen_delta,
        nav_bit_phase_offset_cycles,
        nav_bit_transition,
        cycle_slip: chosen_delta.abs() > CYCLE_SLIP_PHASE_DELTA_CYCLES,
    }
}

fn carrier_prompt_discriminators(
    prompt: Complex<f32>,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, bool) {
    let pll = prompt.im.atan2(prompt.re);
    let fll = if let Some(prev) = prev_prompt {
        let dot = prompt.re * prev.re + prompt.im * prev.im;
        let det = prompt.im * prev.re - prompt.re * prev.im;
        det.atan2(dot)
    } else {
        0.0
    };
    let lock = prompt.norm() > 0.1;
    (pll, fll, lock)
}

fn recover_epoch_navigation_bit_sign(
    signal_model: &TrackingSignalModel,
    data_prompt: Option<Complex<f32>>,
    carrier_prompt: Complex<f32>,
    carrier_prompt_source: CarrierPromptSource,
    carrier_phase_offset_cycles: f64,
    allow_decision: bool,
) -> Option<i8> {
    if !allow_decision || !signal_model.supports_epoch_data_symbol_sign_recovery() {
        return None;
    }
    let prompt = data_prompt?;
    let prompt_norm = prompt.norm();
    if !prompt_norm.is_finite() || prompt_norm <= f32::EPSILON {
        return None;
    }
    let carrier_reference =
        align_prompt_with_phase_offset(carrier_prompt, carrier_phase_offset_cycles);
    let carrier_norm = carrier_reference.norm();
    if !carrier_norm.is_finite() || carrier_norm <= f32::EPSILON {
        return None;
    }
    let data_axis_reference = align_prompt_with_phase_offset(
        carrier_reference,
        carrier_to_data_phase_offset_cycles(signal_model, carrier_prompt_source),
    );
    let aligned_data_prompt = prompt * data_axis_reference.conj();
    if !aligned_data_prompt.re.is_finite() || !aligned_data_prompt.im.is_finite() {
        return None;
    }
    (aligned_data_prompt.re.abs() >= aligned_data_prompt.im.abs())
        .then_some(if aligned_data_prompt.re >= 0.0 { 1 } else { -1 })
}

fn carrier_to_data_phase_offset_cycles(
    signal_model: &TrackingSignalModel,
    carrier_prompt_source: CarrierPromptSource,
) -> f64 {
    if carrier_prompt_source != CarrierPromptSource::Pilot {
        return 0.0;
    }
    pilot_carrier_phase_offset_cycles(signal_model)
}

fn requires_dedicated_pilot_carrier(signal_model: &TrackingSignalModel) -> bool {
    pilot_carrier_phase_offset_cycles(signal_model).abs() > f64::EPSILON
}

fn pilot_carrier_phase_offset_cycles(signal_model: &TrackingSignalModel) -> f64 {
    match signal_model.pilot_component.as_ref().map(|component| &component.local_code_model) {
        Some(
            TrackingComponentLocalCodeModel::GalileoE5aQ { .. }
            | TrackingComponentLocalCodeModel::GalileoE5bQ { .. },
        ) => 0.25,
        _ => 0.0,
    }
}

fn align_prompt_with_phase_offset(prompt: Complex<f32>, phase_offset_cycles: f64) -> Complex<f32> {
    let rotation_radians = -carrier_phase_offset_radians(phase_offset_cycles) as f32;
    let rotation = Complex::new(rotation_radians.cos(), rotation_radians.sin());
    prompt * rotation
}

fn apply_dll_code_loop(input: CodeLoopInput) -> CodeLoopUpdate {
    let update = signal_apply_code_loop(bijux_gnss_signal::api::CodeLoopInput {
        current_code_rate_hz: input.current_code_rate_hz,
        previous_reference_code_rate_hz: input.previous_reference_code_rate_hz,
        reference_code_rate_hz: input.reference_code_rate_hz,
        current_code_phase_samples: input.current_code_phase_samples,
        epoch_len_samples: input.epoch_len_samples,
        coherent_integration_s: input.coherent_integration_s,
        nominal_code_rate_hz: input.nominal_code_rate_hz,
        dll_bw_hz: input.dll_bw_hz,
        dll_err: input.dll_err,
        samples_per_chip: input.samples_per_chip,
        samples_per_code: input.samples_per_code,
    });
    CodeLoopUpdate {
        code_rate_hz: update.code_rate_hz,
        code_phase_samples: update.code_phase_samples,
    }
}

fn tracking_stability_signature(epochs: &[TrackEpoch]) -> String {
    let mut rows = epochs
        .iter()
        .map(|epoch| {
            format!(
                "{}|{}|{:.3}|{:.3}|{}|{}|{}",
                epoch.epoch.index,
                epoch.sample_index,
                epoch.carrier_hz.0,
                epoch.code_phase_samples.0,
                epoch.lock_state,
                epoch.lock,
                epoch.cycle_slip
            )
        })
        .collect::<Vec<_>>();
    rows.sort();
    rows.join(";")
}
