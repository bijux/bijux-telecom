/// Generate a synthetic GPS L1 C/A signal at the receiver sample rate.
///
/// The C/N0 control is approximate and intended for test harnesses.
pub fn generate_l1_ca(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    generate_l1_ca_multi(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed,
            satellites: vec![params],
            ephemerides: Vec::new(),
            id: "synthetic".to_string(),
        },
    )
}

/// Generate a synthetic GPS L1 C/A signal whose Doppler evolves linearly over time.
///
/// The `doppler_rate_hz_per_s` term models a moving-receiver or moving-satellite scenario
/// where the instantaneous carrier Doppler shifts continuously during the capture.
pub fn generate_l1_ca_with_doppler_ramp(
    config: &ReceiverPipelineConfig,
    params: SyntheticDopplerRampParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_with_carrier_dynamics_signal_only(
        config,
        SyntheticCarrierDynamicsParams {
            signal: params.signal,
            doppler_rate_hz_per_s: params.doppler_rate_hz_per_s,
            doppler_jerk_hz_per_s2: 0.0,
        },
        duration_s,
    );
    add_synthetic_noise(config, signal_only, seed)
}

/// Generate a synthetic GPS L1 C/A signal with acceleration and jerk in carrier Doppler.
pub fn generate_l1_ca_with_carrier_dynamics(
    config: &ReceiverPipelineConfig,
    params: SyntheticCarrierDynamicsParams,
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_with_carrier_dynamics_signal_only(config, params, duration_s);
    add_synthetic_noise(config, signal_only, seed)
}

fn add_synthetic_noise(
    config: &ReceiverPipelineConfig,
    signal_only: SamplesFrame,
    seed: u64,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}
/// Generate a synthetic GPS L1 C/A signal with deterministic signal-power fades.
pub fn generate_l1_ca_with_fades(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    fade_windows: &[SyntheticFadeWindow],
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let mut signal_only = generate_l1_ca_signal_only(config, params, duration_s);
    apply_synthetic_fade_windows(&mut signal_only, fade_windows);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

/// Generate a synthetic GPS L1 C/A signal with deterministic carrier phase-offset windows.
pub fn generate_l1_ca_with_phase_windows(
    config: &ReceiverPipelineConfig,
    params: SyntheticSignalParams,
    phase_windows: &[SyntheticPhaseWindow],
    seed: u64,
    duration_s: f64,
) -> SamplesFrame {
    let mut signal_only = generate_l1_ca_signal_only(config, params, duration_s);
    apply_synthetic_phase_windows(&mut signal_only, phase_windows);
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}

pub fn generate_l1_ca_multi(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
) -> SamplesFrame {
    generate_l1_ca_multi_with_receiver_oscillator(
        config,
        scenario,
        &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
    )
}

/// Generate a synthetic multi-satellite capture with explicit receiver oscillator effects.
pub fn generate_l1_ca_multi_with_receiver_oscillator(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
) -> SamplesFrame {
    generate_l1_ca_multi_with_capture_effects(config, scenario, receiver_oscillator, None)
}

/// Generate a synthetic multi-satellite capture with an optional source-side front-end filter.
pub fn generate_l1_ca_multi_with_source_front_end(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SamplesFrame {
    generate_l1_ca_multi_with_capture_effects(
        config,
        scenario,
        &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
        source_front_end_filter,
    )
}

fn generate_l1_ca_multi_with_capture_effects(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    receiver_oscillator: &SyntheticReceiverOscillatorModel,
    source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
) -> SamplesFrame {
    let signal_only = generate_l1_ca_multi_signal_only_with_receiver_oscillator(
        config,
        scenario,
        receiver_oscillator,
    );
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let noise_std = SYNTHETIC_NOISE_STD_PER_COMPONENT;
    let mut rng = XorShift64::new(scenario.seed);
    let mut iq = Vec::with_capacity(signal_only.len());
    for sample in signal_only.iq {
        let noise_i = rng.next_gaussian() * noise_std;
        let noise_q = rng.next_gaussian() * noise_std;
        let noise = Complex::new(noise_i, noise_q);
        iq.push(sample + noise);
    }
    apply_source_front_end_filter_in_place(
        source_front_end_filter,
        config.sampling_freq_hz,
        &mut iq,
    );
    SamplesFrame::new(signal_only.t0, Seconds(dt_s), iq)
}
