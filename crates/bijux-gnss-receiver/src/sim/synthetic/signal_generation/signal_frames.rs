fn generate_l1_ca_with_carrier_dynamics_signal_only(
    config: &ReceiverPipelineConfig,
    params: SyntheticCarrierDynamicsParams,
    duration_s: f64,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (duration_s * config.sampling_freq_hz).round() as usize;
    let sat_state = SatState::new_with_carrier_dynamics_and_receiver_oscillator(
        config,
        params.signal,
        receiver_oscillator_model_from_legacy_bias(0.0),
        params.doppler_rate_hz_per_s,
        params.doppler_jerk_hz_per_s2,
        0,
    );
    let mut iq = Vec::with_capacity(sample_count);
    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        iq.push(sat_state.sample_at(t));
    }
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

fn generate_l1_ca_signal_only(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    duration_s: f64,
) -> SamplesFrame {
    generate_l1_ca_multi_signal_only(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed: 0,
            satellites: vec![params],
            ephemerides: Vec::new(),
            id: "synthetic".to_string(),
        },
    )
}

fn apply_synthetic_fade_windows(frame: &mut SamplesFrame, fade_windows: &[SyntheticFadeWindow]) {
    if fade_windows.is_empty() {
        return;
    }

    let sample_rate_hz = frame.t0.sample_rate_hz;
    for (offset, sample) in frame.iq.iter_mut().enumerate() {
        let sample_time_s = (frame.t0.sample_index as f64 + offset as f64) / sample_rate_hz;
        let signal_scale = synthetic_signal_scale_at_time_s(fade_windows, sample_time_s);
        *sample *= signal_scale;
    }
}

fn apply_synthetic_phase_windows(frame: &mut SamplesFrame, phase_windows: &[SyntheticPhaseWindow]) {
    if phase_windows.is_empty() {
        return;
    }

    let sample_rate_hz = frame.t0.sample_rate_hz;
    for (offset, sample) in frame.iq.iter_mut().enumerate() {
        let sample_time_s = (frame.t0.sample_index as f64 + offset as f64) / sample_rate_hz;
        let phase_offset_rad = synthetic_phase_offset_at_time_s(phase_windows, sample_time_s);
        if phase_offset_rad.abs() <= f64::EPSILON {
            continue;
        }
        let rotation = Complex::from_polar(1.0_f32, phase_offset_rad as f32);
        *sample *= rotation;
    }
}

fn synthetic_signal_scale_at_time_s(
    fade_windows: &[SyntheticFadeWindow],
    sample_time_s: f64,
) -> f32 {
    fade_windows
        .iter()
        .filter(|window| sample_time_s >= window.start_s && sample_time_s < window.end_s)
        .fold(1.0_f32, |signal_scale, window| signal_scale * window.signal_scale)
}

fn synthetic_phase_offset_at_time_s(
    phase_windows: &[SyntheticPhaseWindow],
    sample_time_s: f64,
) -> f64 {
    phase_windows
        .iter()
        .filter(|window| sample_time_s >= window.start_s && sample_time_s < window.end_s)
        .map(|window| window.phase_offset_rad)
        .sum()
}

fn generate_l1_ca_multi_signal_only(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    generate_l1_ca_multi_signal_only_with_source_front_end(config, scenario, None)
}

pub fn generate_l1_ca_multi_signal_only_with_source_front_end(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SamplesFrame {
    let receiver_oscillator_model =
        receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz);
    generate_l1_ca_multi_signal_only_with_capture_effects(
        config,
        scenario,
        &receiver_oscillator_model,
        source_front_end_filter,
    )
}

/// Generate a synthetic multi-satellite signal component with explicit receiver oscillator effects.
pub fn generate_l1_ca_multi_signal_only_with_receiver_oscillator(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> SamplesFrame {
    generate_l1_ca_multi_signal_only_with_capture_effects(
        config,
        scenario,
        receiver_oscillator,
        None,
    )
}

fn generate_l1_ca_multi_signal_only_with_capture_effects(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
    let sat_states: Vec<SatState> = scenario
        .satellites
        .iter()
        .map(|sat| {
            SatState::new_with_receiver_oscillator(
                config,
                sat.clone(),
                receiver_oscillator.clone(),
                sample_count as u64,
            )
        })
        .collect();
    let mut iq = Vec::with_capacity(sample_count);
    for n in 0..sample_count {
        let t = n as f64 * dt_s;
        let mut sample = Complex::new(0.0f32, 0.0f32);
        for sat in &sat_states {
            sample += sat.sample_at(t);
        }
        iq.push(sample);
    }
    apply_source_front_end_filter_in_place(
        source_front_end_filter,
        config.sampling_freq_hz,
        &mut iq,
    );
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

fn apply_source_front_end_filter_in_place(
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
    sample_rate_hz: f64,
    iq: &mut [Complex<f32>],
) {
    let Some(spec) = source_front_end_filter else {
        return;
    };
    let mut filter = spec
        .design(sample_rate_hz)
        .expect("validated synthetic source front-end filter must design");
    filter.apply_in_place(iq);
}
