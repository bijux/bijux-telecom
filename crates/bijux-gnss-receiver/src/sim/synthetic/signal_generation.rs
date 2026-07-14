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

/// Streaming synthetic GPS L1 C/A signal source for long-duration receiver tests.
pub struct SyntheticSignalSource {
    sample_rate_hz: f64,
    dt_s: f64,
    remaining_samples: usize,
    next_sample_index: u64,
    noise_std: f32,
    synthetic_signals: Vec<SyntheticSignalParams>,
    sat_states: Vec<SatState>,
    gps_ephemerides: Vec<GpsEphemeris>,
    signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
    source_front_end_filter: Option<bijux_gnss_signal::api::FrontEndFirFilter>,
    rng: XorShift64,
}

/// Source-provided whole-code alignment for one synthetic signal.
#[derive(Debug, Clone)]
pub struct SyntheticSignalDelayAlignment {
    pub sat: SatId,
    pub signal_delay_alignment: SignalDelayAlignment,
}

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

impl SyntheticSignalSource {
    /// Build a streaming synthetic source from a scenario without materializing the full capture.
    pub fn new(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::new_with_receiver_oscillator(
            config,
            scenario,
            &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
        )
    }

    /// Build a streaming synthetic source that emits only the deterministic signal component.
    pub fn new_signal_only(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::new_signal_only_with_receiver_oscillator(
            config,
            scenario,
            &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
        )
    }

    /// Build a streaming synthetic source from a scenario with explicit receiver oscillator effects.
    pub fn new_with_receiver_oscillator(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        receiver_oscillator: &SyntheticReceiverOscillatorModel,
    ) -> Self {
        Self::with_capture_effects(
            config,
            scenario,
            receiver_oscillator,
            SYNTHETIC_NOISE_STD_PER_COMPONENT,
            Vec::new(),
            None,
        )
    }

    /// Build a signal-only streaming source with explicit receiver oscillator effects.
    pub fn new_signal_only_with_receiver_oscillator(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        receiver_oscillator: &SyntheticReceiverOscillatorModel,
    ) -> Self {
        Self::with_capture_effects(config, scenario, receiver_oscillator, 0.0, Vec::new(), None)
    }

    /// Build a streaming synthetic source with explicit whole-code signal-delay alignments.
    pub fn new_with_signal_delay_alignments(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
    ) -> Self {
        Self::with_capture_effects(
            config,
            scenario,
            &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
            SYNTHETIC_NOISE_STD_PER_COMPONENT,
            signal_delay_alignments,
            None,
        )
    }

    /// Build a streaming synthetic source with explicit whole-code alignments and a source filter.
    pub fn new_with_signal_delay_alignments_and_source_front_end(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
        source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
    ) -> Self {
        Self::with_capture_effects(
            config,
            scenario,
            &receiver_oscillator_model_from_legacy_bias(scenario.receiver_clock_frequency_bias_hz),
            SYNTHETIC_NOISE_STD_PER_COMPONENT,
            signal_delay_alignments,
            source_front_end_filter,
        )
    }

    fn with_capture_effects(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        receiver_oscillator: &SyntheticReceiverOscillatorModel,
        noise_std: f32,
        signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
        source_front_end_filter: Option<&bijux_gnss_signal::api::FrontEndFilterSpec>,
    ) -> Self {
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;

        Self {
            sample_rate_hz: config.sampling_freq_hz,
            dt_s: 1.0 / config.sampling_freq_hz,
            remaining_samples: sample_count,
            next_sample_index: 0,
            noise_std,
            synthetic_signals: scenario.satellites.clone(),
            sat_states: scenario
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
                .collect(),
            gps_ephemerides: scenario.ephemerides.clone(),
            signal_delay_alignments,
            source_front_end_filter: source_front_end_filter.cloned().map(|spec| {
                spec.design(config.sampling_freq_hz)
                    .expect("validated synthetic source front-end filter must design")
            }),
            rng: XorShift64::new(scenario.seed),
        }
    }

    /// Broadcast ephemerides carried alongside this synthetic source.
    pub fn gps_ephemerides(&self) -> &[GpsEphemeris] {
        &self.gps_ephemerides
    }

    /// Declared synthetic signal specifications carried by this source.
    pub fn synthetic_signals(&self) -> &[SyntheticSignalParams] {
        &self.synthetic_signals
    }

    /// Whole-code signal-delay alignment for one tracked satellite, when available.
    pub fn signal_delay_alignment(&self, sat: SatId) -> Option<&SignalDelayAlignment> {
        self.signal_delay_alignments
            .iter()
            .find(|alignment| alignment.sat == sat)
            .map(|alignment| &alignment.signal_delay_alignment)
    }
}

/// Run a synthetic scenario through the canonical receiver pipeline.
pub fn run_synthetic_scenario(
    config: &ReceiverPipelineConfig,
    runtime: crate::engine::runtime::ReceiverRuntime,
    scenario: &SyntheticScenario,
    capture_start_gps_time: Option<GpsTime>,
) -> Result<crate::api::RunArtifacts, crate::engine::receiver_config::ReceiverError> {
    let runtime = match (runtime.config.capture_start_gps_time, capture_start_gps_time) {
        (Some(_), _) | (None, None) => runtime,
        (None, Some(capture_start_gps_time)) => {
            runtime.with_capture_start_gps_time(capture_start_gps_time)
        }
    };
    let receiver = crate::api::Receiver::new(config.clone(), runtime);
    let mut source = SyntheticSignalSource::new(config, scenario);
    receiver.run(&mut source)
}

impl SignalSource for SyntheticSignalSource {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        if self.remaining_samples == 0 {
            return Ok(None);
        }
        let count = self.remaining_samples.min(frame_len.max(1));
        let t0 = SampleTime {
            sample_index: self.next_sample_index,
            sample_rate_hz: self.sample_rate_hz,
        };
        let mut iq = Vec::with_capacity(count);
        for offset in 0..count {
            let t = (self.next_sample_index + offset as u64) as f64 * self.dt_s;
            let mut sample = Complex::new(0.0f32, 0.0f32);
            for sat in &self.sat_states {
                sample += sat.sample_at(t);
            }
            if self.noise_std <= f32::EPSILON {
                iq.push(sample);
            } else {
                let noise_i = self.rng.next_gaussian() * self.noise_std;
                let noise_q = self.rng.next_gaussian() * self.noise_std;
                iq.push(sample + Complex::new(noise_i, noise_q));
            }
        }
        if let Some(filter) = self.source_front_end_filter.as_mut() {
            filter.apply_in_place(&mut iq);
        }
        self.next_sample_index += count as u64;
        self.remaining_samples -= count;
        Ok(Some(SamplesFrame::new(t0, Seconds(self.dt_s), iq)))
    }

    fn is_done(&self) -> bool {
        self.remaining_samples == 0
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[derive(Debug, Clone)]
struct SatState {
    doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    doppler_jerk_hz_per_s2: f64,
    receiver_oscillator_model: SyntheticReceiverOscillatorModel,
    receiver_oscillator_phase_noise_knots: Vec<(u64, f64)>,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    navigation_data: SyntheticNavigationData,
    nav_bit_mode: SyntheticNavBitMode,
    nav_symbol_period_s: Option<f64>,
    signal_model: SyntheticSignalModel,
    if_hz: f64,
    sample_rate_hz: f64,
}

type SyntheticSignalModel = bijux_gnss_signal::api::ReplicaCodeModel;

fn synthetic_replica_model(params: &SyntheticSignalParams) -> SyntheticSignalModel {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            return SyntheticSignalModel::galileo_e5a_qpsk(params.sat.prn)
                .unwrap_or_else(|_| SyntheticSignalModel::galileo_e5a_qpsk_or_ones(params.sat.prn));
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            return SyntheticSignalModel::galileo_e5b_qpsk(params.sat.prn)
                .unwrap_or_else(|_| SyntheticSignalModel::galileo_e5b_qpsk_or_ones(params.sat.prn));
        }
        _ => {}
    }
    SyntheticSignalModel::for_sat_signal(params.sat, Some(params.signal_band), signal_code)
        .ok()
        .flatten()
        .unwrap_or_else(|| SyntheticSignalModel::gps_l1_ca_or_ones(params.sat.prn))
}

impl SatState {
    fn new_with_receiver_clock_frequency_bias_hz(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_clock_frequency_bias_hz: f64,
    ) -> Self {
        Self::new_with_receiver_oscillator(
            config,
            params,
            receiver_oscillator_model_from_legacy_bias(receiver_clock_frequency_bias_hz),
            0,
        )
    }

    fn new_with_receiver_oscillator(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_oscillator_model: SyntheticReceiverOscillatorModel,
        sample_count: u64,
    ) -> Self {
        Self::new_with_doppler_rate_and_receiver_oscillator(
            config,
            params,
            receiver_oscillator_model,
            0.0,
            sample_count,
        )
    }

    fn new_with_doppler_rate_and_receiver_oscillator(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_oscillator_model: SyntheticReceiverOscillatorModel,
        doppler_rate_hz_per_s: f64,
        sample_count: u64,
    ) -> Self {
        Self::new_with_carrier_dynamics_and_receiver_oscillator(
            config,
            params,
            receiver_oscillator_model,
            doppler_rate_hz_per_s,
            0.0,
            sample_count,
        )
    }

    fn new_with_carrier_dynamics_and_receiver_oscillator(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_oscillator_model: SyntheticReceiverOscillatorModel,
        doppler_rate_hz_per_s: f64,
        doppler_jerk_hz_per_s2: f64,
        sample_count: u64,
    ) -> Self {
        let receiver_oscillator_phase_noise_knots =
            receiver_oscillator_phase_noise_knots(&receiver_oscillator_model, sample_count);
        let signal_model = synthetic_replica_model(&params);
        let nav_bit_mode = nav_bit_mode(&params);
        Self {
            doppler_hz: params.doppler_hz,
            doppler_rate_hz_per_s,
            doppler_jerk_hz_per_s2,
            receiver_oscillator_model,
            receiver_oscillator_phase_noise_knots,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            navigation_data: params.navigation_data.clone(),
            nav_bit_mode,
            nav_symbol_period_s: nav_symbol_period_for_signal(&params, nav_bit_mode),
            signal_model,
            if_hz: synthetic_intermediate_frequency_hz(
                config.intermediate_freq_hz,
                params.sat,
                params.signal_band,
                params.signal_code,
                params.glonass_frequency_channel,
            ),
            sample_rate_hz: config.sampling_freq_hz,
        }
    }

    #[cfg(test)]
    fn new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_clock_frequency_bias_hz: f64,
        doppler_rate_hz_per_s: f64,
    ) -> Self {
        Self::new_with_doppler_rate_and_receiver_oscillator(
            config,
            params,
            receiver_oscillator_model_from_legacy_bias(receiver_clock_frequency_bias_hz),
            doppler_rate_hz_per_s,
            0,
        )
    }

    fn initial_carrier_hz(&self) -> f64 {
        self.if_hz + self.doppler_hz + self.receiver_oscillator_model.carrier_frequency_bias_hz
    }

    fn effective_elapsed_s(&self, nominal_elapsed_s: f64) -> f64 {
        nominal_elapsed_s * (1.0 + self.receiver_oscillator_model.sampling_clock_fractional_error)
            + 0.5
                * self.receiver_oscillator_model.sampling_clock_fractional_drift_per_s
                * nominal_elapsed_s
                * nominal_elapsed_s
    }

    fn receiver_oscillator_phase_noise_rad_at_nominal_time_s(&self, nominal_elapsed_s: f64) -> f64 {
        let sample_index = ((nominal_elapsed_s * self.sample_rate_hz).round() as u64).min(
            self.receiver_oscillator_phase_noise_knots.last().map(|(index, _)| *index).unwrap_or(0),
        );
        receiver_oscillator_phase_noise_rad_from_knots(
            &self.receiver_oscillator_phase_noise_knots,
            sample_index,
        )
    }

    #[cfg(test)]
    fn carrier_hz_at(&self, t: f64) -> f64 {
        let effective_elapsed_s = self.effective_elapsed_s(t);
        let carrier_hz = bijux_gnss_signal::api::carrier_hz_at_time_with_jerk(
            self.initial_carrier_hz(),
            self.doppler_rate_hz_per_s
                + self.receiver_oscillator_model.carrier_frequency_drift_hz_per_s,
            self.doppler_jerk_hz_per_s2,
            effective_elapsed_s,
        );
        carrier_hz * (1.0 + self.receiver_oscillator_model.sampling_clock_fractional_error)
    }

    #[cfg(test)]
    fn total_chip_phase_at(&self, t: f64) -> f64 {
        self.code_phase_chips + self.signal_model.code_rate_hz() * self.effective_elapsed_s(t)
    }

    fn carrier_phase_rad_at(&self, t: f64) -> f64 {
        let effective_elapsed_s = self.effective_elapsed_s(t);
        carrier_phase_radians_at_time_with_jerk(
            self.carrier_phase_rad,
            self.initial_carrier_hz(),
            self.doppler_rate_hz_per_s
                + self.receiver_oscillator_model.carrier_frequency_drift_hz_per_s,
            self.doppler_jerk_hz_per_s2,
            effective_elapsed_s,
        ) + self.receiver_oscillator_phase_noise_rad_at_nominal_time_s(t)
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let effective_elapsed_s = self.effective_elapsed_s(t);
        let data_bit = nav_bit_sign_with_symbol_period_at_time_s(
            &self.navigation_data,
            self.nav_bit_mode,
            self.nav_symbol_period_s,
            effective_elapsed_s,
        );
        let amplitude = signal_amplitude_from_cn0(self.cn0_db_hz, self.sample_rate_hz);
        let total_chip_phase =
            self.code_phase_chips + self.signal_model.code_rate_hz() * effective_elapsed_s;
        let code_length = self.signal_model.code_length().max(1) as f64;
        let primary_code_period_index = if total_chip_phase <= 0.0 {
            0
        } else {
            (total_chip_phase / code_length).floor() as usize
        };
        let code_phase = total_chip_phase.rem_euclid(code_length);
        let signal_value = self
            .signal_model
            .sample_value(code_phase, primary_code_period_index, data_bit)
            .unwrap_or(Complex::new(0.0, 0.0));
        let phase = self.carrier_phase_rad_at(t) as f32;
        let carrier = Complex::new(phase.cos(), phase.sin());
        carrier * signal_value * amplitude
    }
}

fn signal_amplitude_from_cn0(cn0_db_hz: f32, sample_rate_hz: f64) -> f32 {
    bijux_gnss_signal::api::signal_amplitude_from_cn0_db_hz(
        cn0_db_hz,
        sample_rate_hz,
        SYNTHETIC_COMPLEX_NOISE_POWER,
    )
}

fn regenerate_isolated_scaled_satellite_signal_only_frame(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_signal_only_with_capture_effects(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
        &truth.receiver_oscillator_model,
        truth.source_front_end_filter.as_ref(),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(
        measured_frame.t0,
        measured_frame.dt_s,
        quantize_samples_for_storage(&iq, truth.quantization),
    )
}

fn regenerate_isolated_scaled_satellite_frame_with_noise(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi_with_capture_effects(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
        &truth.receiver_oscillator_model,
        truth.source_front_end_filter.as_ref(),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn isolated_satellite_scenario(
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: truth.sample_rate_hz,
        intermediate_freq_hz: truth.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: truth.receiver_clock_frequency_bias_hz,
        duration_s: measured_frame.len() as f64 * measured_frame.dt_s.0,
        seed: truth.seed,
        satellites: vec![SyntheticSignalParams {
            sat: sat_truth.sat,
            glonass_frequency_channel: sat_truth.glonass_frequency_channel,
            signal_band: sat_truth.signal_band,
            signal_code: sat_truth.signal_code,
            doppler_hz: sat_truth.doppler_hz,
            code_phase_chips: sat_truth.code_phase_chips,
            carrier_phase_rad: sat_truth.carrier_phase_rad,
            cn0_db_hz: sat_truth.cn0_db_hz,
            navigation_data: sat_truth.navigation_data.clone(),
        }],
        ephemerides: Vec::new(),
        id: sat_truth.sat.prn.to_string(),
    }
}

#[cfg(test)]
fn code_phase_samples_at_epoch_start(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    bijux_gnss_signal::api::code_phase_samples_at_sample_index(
        frame.t0.sample_rate_hz,
        config.code_freq_basis_hz,
        config.code_length,
        frame.t0.sample_index,
        code_phase_chips,
    )
    .expect("synthetic epoch alignment requires a valid code phase model")
}

fn synthetic_intermediate_frequency_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    intermediate_freq_hz
        + (synthetic_constellation_carrier_hz(
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        ) - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}

fn synthetic_carrier_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    doppler_hz: f64,
) -> f64 {
    carrier_hz_from_doppler_hz(
        synthetic_intermediate_frequency_hz(
            intermediate_freq_hz,
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        ),
        doppler_hz,
    )
}

fn synthetic_truth_measured_doppler_hz(
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> f64 {
    sat_truth.doppler_hz + truth.receiver_clock_frequency_bias_hz
}

fn synthetic_constellation_carrier_hz(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    match bijux_gnss_signal::api::default_signal_carrier_hz_for_signal(
        sat,
        Some(signal_band),
        resolved_signal_code(sat, signal_band, signal_code),
        glonass_frequency_channel,
    ) {
        Ok(Some(carrier_hz)) => carrier_hz.value(),
        Ok(None) => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        Err(bijux_gnss_signal::api::SignalError::MissingGlonassFrequencyChannel(_)) => {
            panic!(
                "GLONASS synthetic signal for {} requires glonass_frequency_channel",
                bijux_gnss_core::api::format_sat(sat)
            )
        }
        Err(_) => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
    }
}

fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => {
            SyntheticNavBitMode::GpsL5QNh20
        }
        (bijux_gnss_core::api::Constellation::Glonass, SignalBand::L1, _) => {
            match &params.navigation_data {
                SyntheticNavigationData::ConstantPositive => {
                    SyntheticNavBitMode::GlonassL1FixedDataString
                }
                SyntheticNavigationData::AlternatingStartPositive => {
                    SyntheticNavBitMode::GlonassL1AlternatingDataString
                }
                SyntheticNavigationData::GlonassL1String { raw_data_bits } => {
                    if raw_data_bits.iter().all(|&bit| bit == 1) {
                        SyntheticNavBitMode::GlonassL1FixedDataString
                    } else {
                        SyntheticNavBitMode::GlonassL1CustomDataString
                    }
                }
                _ => SyntheticNavBitMode::GlonassL1CustomDataString,
            }
        }
        _ => match &params.navigation_data {
            SyntheticNavigationData::ConstantPositive => SyntheticNavBitMode::ConstantPositive,
            SyntheticNavigationData::ConstantNegative => SyntheticNavBitMode::ConstantNegative,
            SyntheticNavigationData::AlternatingStartPositive => {
                match (params.sat.constellation, params.signal_band, signal_code) {
                    (
                        bijux_gnss_core::api::Constellation::Galileo,
                        SignalBand::E5,
                        SignalCode::E5b,
                    ) => SyntheticNavBitMode::AlternatingGalileoInav4ms,
                    (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I) => {
                        SyntheticNavBitMode::AlternatingGpsL5I10ms
                    }
                    _ => SyntheticNavBitMode::AlternatingGpsLnav20ms,
                }
            }
            SyntheticNavigationData::AlternatingStartNegative
            | SyntheticNavigationData::SymbolSequence(_)
            | SyntheticNavigationData::GlonassL1String { .. } => {
                SyntheticNavBitMode::NativeSymbolSequence
            }
        },
    }
}

#[cfg(test)]
fn nav_bit_sign_for_mode_at_time_s(nav_bit_mode: SyntheticNavBitMode, time_s: f64) -> i8 {
    let navigation_data = navigation_data_for_nav_mode(nav_bit_mode);
    nav_bit_sign_with_navigation_data_for_mode_at_time_s(&navigation_data, nav_bit_mode, time_s)
}

#[cfg(test)]
fn nav_bit_sign_with_navigation_data_for_mode_at_time_s(
    navigation_data: &SyntheticNavigationData,
    nav_bit_mode: SyntheticNavBitMode,
    time_s: f64,
) -> i8 {
    nav_bit_sign_with_symbol_period_at_time_s(
        navigation_data,
        nav_bit_mode,
        legacy_nav_symbol_period_s(nav_bit_mode),
        time_s,
    )
}

fn nav_bit_sign_with_symbol_period_at_time_s(
    navigation_data: &SyntheticNavigationData,
    nav_bit_mode: SyntheticNavBitMode,
    symbol_period_s: Option<f64>,
    time_s: f64,
) -> i8 {
    match nav_bit_mode {
        SyntheticNavBitMode::GlonassL1FixedDataString
        | SyntheticNavBitMode::GlonassL1AlternatingDataString
        | SyntheticNavBitMode::GlonassL1CustomDataString => {
            bijux_gnss_signal::api::glonass_l1_string_symbol_at_time_s(
                &glonass_l1_raw_data_bits(navigation_data),
                time_s.max(0.0),
            )
            .expect("synthetic GLONASS L1 string schedule must be valid")
        }
        SyntheticNavBitMode::GpsL5QNh20 => bijux_gnss_signal::api::gps_l5_q_epoch_symbol(
            navigation_symbol_index_at_time_s(symbol_period_s, time_s),
        ),
        _ => navigation_symbol_at_index(
            navigation_data,
            navigation_symbol_index_at_time_s(symbol_period_s, time_s),
        ),
    }
}

fn navigation_symbol_index_at_time_s(symbol_period_s: Option<f64>, time_s: f64) -> usize {
    let Some(symbol_period_s) = symbol_period_s else {
        return 0;
    };
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / symbol_period_s).floor() as usize
}

#[cfg(test)]
fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    let navigation_data = SyntheticNavigationData::from(data_bit_flip);
    nav_bit_sign_with_navigation_data_for_mode_at_time_s(
        &navigation_data,
        nav_bit_mode_from_flip(data_bit_flip),
        time_s,
    )
}

#[cfg(test)]
fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    navigation_symbol_index_at_time_s(Some(GPS_L1_CA_NAV_BIT_PERIOD_S), time_s) as u64
}

fn navigation_symbol_at_index(
    navigation_data: &SyntheticNavigationData,
    symbol_index: usize,
) -> i8 {
    match navigation_data {
        SyntheticNavigationData::ConstantPositive => 1,
        SyntheticNavigationData::ConstantNegative => -1,
        SyntheticNavigationData::AlternatingStartPositive => {
            if symbol_index % 2 == 0 {
                1
            } else {
                -1
            }
        }
        SyntheticNavigationData::AlternatingStartNegative => {
            if symbol_index % 2 == 0 {
                -1
            } else {
                1
            }
        }
        SyntheticNavigationData::SymbolSequence(symbols) => {
            assert!(!symbols.is_empty(), "synthetic navigation symbol sequence must not be empty");
            let symbol = symbols.get(symbol_index % symbols.len()).copied().unwrap_or_else(|| {
                panic!("synthetic navigation symbol sequence must not be empty")
            });
            validate_navigation_symbol(symbol)
        }
        SyntheticNavigationData::GlonassL1String { .. } => {
            panic!("glonass l1 raw data strings are only valid on glonass l1 signals")
        }
    }
}

fn validate_navigation_symbol(symbol: i8) -> i8 {
    match symbol {
        -1 | 1 => symbol,
        other => panic!("synthetic navigation symbol must be -1 or 1, found {other}"),
    }
}

fn receiver_oscillator_model_from_legacy_bias(
    receiver_clock_frequency_bias_hz: f64,
) -> SyntheticReceiverOscillatorModel {
    SyntheticReceiverOscillatorModel::with_carrier_frequency_bias_hz(
        receiver_clock_frequency_bias_hz,
    )
}

fn receiver_oscillator_truth_for_capture(
    model: &SyntheticReceiverOscillatorModel,
    sample_rate_hz: f64,
    sample_count: u64,
) -> SyntheticReceiverOscillatorTruth {
    let sample_indices =
        receiver_oscillator_truth_sample_indices(model, sample_rate_hz, sample_count);
    SyntheticReceiverOscillatorTruth {
        carrier_frequency_bias_hz: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: model.carrier_frequency_bias_hz,
            })
            .collect(),
        carrier_frequency_drift_hz_per_s: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: model.carrier_frequency_drift_hz_per_s,
            })
            .collect(),
        phase_noise_rad: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: receiver_oscillator_phase_noise_rad_at_sample(
                    model,
                    sample_index,
                    sample_count,
                ),
            })
            .collect(),
        sampling_clock_time_error_s: sample_indices
            .iter()
            .copied()
            .map(|sample_index| SyntheticReceiverOscillatorTruthPoint {
                sample_index,
                time_s: sample_index as f64 / sample_rate_hz,
                value: sampling_clock_time_error_s_at_nominal_time(
                    sample_index as f64 / sample_rate_hz,
                    model,
                ),
            })
            .collect(),
    }
}

fn sampling_clock_time_error_s_at_nominal_time(
    nominal_elapsed_s: f64,
    model: &SyntheticReceiverOscillatorModel,
) -> f64 {
    nominal_elapsed_s * model.sampling_clock_fractional_error
        + 0.5 * model.sampling_clock_fractional_drift_per_s * nominal_elapsed_s * nominal_elapsed_s
}

fn receiver_oscillator_truth_sample_indices(
    model: &SyntheticReceiverOscillatorModel,
    sample_rate_hz: f64,
    sample_count: u64,
) -> Vec<u64> {
    if sample_count == 0 {
        return Vec::new();
    }

    let mut sample_indices = vec![0];
    let last_sample_index = sample_count.saturating_sub(1);
    let truth_step_samples = ((sample_rate_hz * 0.001).round() as u64).max(1);
    let phase_noise_step_samples = model.phase_noise.knot_interval_samples.max(1);
    let stride = if model.phase_noise.is_enabled() {
        truth_step_samples.min(phase_noise_step_samples)
    } else {
        truth_step_samples
    };
    let mut sample_index = stride;
    while sample_index < last_sample_index {
        sample_indices.push(sample_index);
        sample_index += stride;
    }
    if last_sample_index > 0 {
        sample_indices.push(last_sample_index);
    }
    sample_indices
}

fn receiver_oscillator_phase_noise_rad_at_sample(
    model: &SyntheticReceiverOscillatorModel,
    sample_index: u64,
    sample_count: u64,
) -> f64 {
    if !model.phase_noise.is_enabled() || sample_count == 0 {
        return 0.0;
    }

    let knots = receiver_oscillator_phase_noise_knots(model, sample_count);
    receiver_oscillator_phase_noise_rad_from_knots(&knots, sample_index)
}

fn receiver_oscillator_phase_noise_knots(
    model: &SyntheticReceiverOscillatorModel,
    sample_count: u64,
) -> Vec<(u64, f64)> {
    if !model.phase_noise.is_enabled() || sample_count == 0 {
        return vec![(0, 0.0)];
    }

    let last_sample_index = sample_count.saturating_sub(1);
    let step_samples = model.phase_noise.knot_interval_samples.max(1);
    let mut rng = XorShift64::new(model.phase_noise.seed);
    let mut knots = vec![(0, 0.0)];
    let mut phase_noise_rad = 0.0;
    let mut sample_index = step_samples;
    while sample_index <= last_sample_index {
        phase_noise_rad += rng.next_gaussian() as f64 * model.phase_noise.step_std_rad;
        knots.push((sample_index, phase_noise_rad));
        sample_index += step_samples;
    }
    if knots.last().map(|(sample_index, _)| *sample_index) != Some(last_sample_index) {
        phase_noise_rad += rng.next_gaussian() as f64 * model.phase_noise.step_std_rad;
        knots.push((last_sample_index, phase_noise_rad));
    }
    knots
}

fn receiver_oscillator_phase_noise_rad_from_knots(knots: &[(u64, f64)], sample_index: u64) -> f64 {
    if let Some((_, value)) =
        knots.iter().find(|(knot_sample_index, _)| *knot_sample_index == sample_index)
    {
        return *value;
    }

    for window in knots.windows(2) {
        let (start_sample_index, start_value) = window[0];
        let (end_sample_index, end_value) = window[1];
        if sample_index >= start_sample_index && sample_index <= end_sample_index {
            let span = (end_sample_index - start_sample_index).max(1) as f64;
            let alpha = (sample_index - start_sample_index) as f64 / span;
            return start_value + alpha * (end_value - start_value);
        }
    }

    knots.last().map(|(_, value)| *value).unwrap_or(0.0)
}

fn peak_component(samples: &[Complex<f32>]) -> f32 {
    samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max)
}

fn nav_bit_segments(
    sample_rate_hz: f64,
    sample_count: u64,
    params: &SyntheticSignalParams,
    nav_bit_mode: SyntheticNavBitMode,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }

    if matches!(
        nav_bit_mode,
        SyntheticNavBitMode::GlonassL1FixedDataString
            | SyntheticNavBitMode::GlonassL1AlternatingDataString
            | SyntheticNavBitMode::GlonassL1CustomDataString
    ) {
        return symbol_period_segments(
            sample_rate_hz,
            sample_count,
            Some(bijux_gnss_signal::api::GLONASS_L1_SYMBOL_PERIOD_S),
            |start_s| {
                nav_bit_sign_with_symbol_period_at_time_s(
                    &params.navigation_data,
                    nav_bit_mode,
                    None,
                    start_s,
                )
            },
        );
    }

    if let Some(primary_epoch_period_s) = native_epoch_period_for_signal(params) {
        return symbol_period_segments(
            sample_rate_hz,
            sample_count,
            Some(primary_epoch_period_s),
            |start_s| {
                let primary_code_period_index = (start_s / primary_epoch_period_s).floor() as usize;
                emitted_epoch_symbol(params, primary_code_period_index)
            },
        );
    }

    symbol_period_segments(
        sample_rate_hz,
        sample_count,
        nav_symbol_period_for_signal(params, nav_bit_mode),
        |start_s| {
            nav_bit_sign_with_symbol_period_at_time_s(
                &params.navigation_data,
                nav_bit_mode,
                nav_symbol_period_for_signal(params, nav_bit_mode),
                start_s,
            )
        },
    )
}

fn symbol_period_segments<F>(
    sample_rate_hz: f64,
    sample_count: u64,
    symbol_period_s: Option<f64>,
    symbol_at_time_s: F,
) -> Vec<SyntheticNavBitSegment>
where
    F: Fn(f64) -> i8,
{
    let Some(symbol_period_s) = symbol_period_s else {
        return vec![SyntheticNavBitSegment {
            start_sample: 0,
            end_sample: sample_count,
            start_s: 0.0,
            end_s: sample_count as f64 / sample_rate_hz,
            bit: 1,
        }];
    };

    let mut segments = Vec::new();
    let mut bit_index = 0u64;
    loop {
        let start_sample = ((bit_index as f64 * symbol_period_s * sample_rate_hz).ceil()) as u64;
        if start_sample >= sample_count {
            break;
        }
        let end_sample =
            ((((bit_index + 1) as f64) * symbol_period_s * sample_rate_hz).ceil()) as u64;
        let clamped_end = end_sample.min(sample_count);
        let start_s = start_sample as f64 / sample_rate_hz;
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: symbol_at_time_s(start_s),
        });
        bit_index += 1;
    }
    segments
}

fn native_epoch_period_for_signal(params: &SyntheticSignalParams) -> Option<f64> {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        params.signal_band,
        signal_code,
        params.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let component = registry_entry.default_component()?;

    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I)
        | (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            Some(component.primary_code_period_s)
        }
        _ => None,
    }
}

fn emitted_epoch_symbol(params: &SyntheticSignalParams, primary_code_period_index: usize) -> i8 {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    match (params.sat.constellation, params.signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I) => {
            bijux_gnss_signal::api::gps_l5_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::gps_l5_i_data_symbol_index(primary_code_period_index),
                )],
                primary_code_period_index,
            )
            .expect("synthetic GPS L5I epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q) => {
            bijux_gnss_signal::api::gps_l5_q_epoch_symbol(primary_code_period_index)
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            bijux_gnss_signal::api::galileo_e5a_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::galileo_e5a_i_data_symbol_index(
                        primary_code_period_index,
                    ),
                )],
                primary_code_period_index,
            )
            .expect("synthetic Galileo E5a epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            bijux_gnss_signal::api::galileo_e5b_i_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::galileo_e5b_i_data_symbol_index(
                        primary_code_period_index,
                    ),
                )],
                primary_code_period_index,
            )
            .expect("synthetic Galileo E5b epoch symbols must be valid")
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            bijux_gnss_signal::api::beidou_d1_epoch_symbol(
                &[navigation_symbol_at_index(
                    &params.navigation_data,
                    bijux_gnss_signal::api::beidou_d1_data_symbol_index(primary_code_period_index),
                )],
                primary_code_period_index,
            )
            .expect("synthetic BeiDou D1 epoch symbols must be valid")
        }
        _ => {
            let nav_bit_mode = nav_bit_mode(params);
            nav_bit_sign_with_symbol_period_at_time_s(
                &params.navigation_data,
                nav_bit_mode,
                nav_symbol_period_for_signal(params, nav_bit_mode),
                0.0,
            )
        }
    }
}

fn nav_symbol_period_for_signal(
    params: &SyntheticSignalParams,
    nav_bit_mode: SyntheticNavBitMode,
) -> Option<f64> {
    if matches!(
        nav_bit_mode,
        SyntheticNavBitMode::ConstantPositive | SyntheticNavBitMode::ConstantNegative
    ) {
        return None;
    }

    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
    let registry_entry = bijux_gnss_signal::api::resolved_signal_registry_entry(
        params.sat,
        params.signal_band,
        signal_code,
        params.glonass_frequency_channel,
    )
    .ok()
    .flatten()?;
    let component = registry_entry.default_component()?;

    match nav_bit_mode {
        SyntheticNavBitMode::GpsL5QNh20 => {
            component.secondary_code.map(|secondary| secondary.chip_period_s)
        }
        _ => component.symbol_period_s,
    }
}

#[cfg(test)]
fn legacy_nav_symbol_period_s(nav_bit_mode: SyntheticNavBitMode) -> Option<f64> {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive | SyntheticNavBitMode::ConstantNegative => None,
        SyntheticNavBitMode::GlonassL1FixedDataString
        | SyntheticNavBitMode::GlonassL1AlternatingDataString
        | SyntheticNavBitMode::GlonassL1CustomDataString => {
            Some(bijux_gnss_signal::api::GLONASS_L1_SYMBOL_PERIOD_S)
        }
        SyntheticNavBitMode::AlternatingGpsLnav20ms => Some(GPS_L1_CA_NAV_BIT_PERIOD_S),
        SyntheticNavBitMode::AlternatingGalileoInav4ms => Some(0.004),
        SyntheticNavBitMode::AlternatingGpsL5I10ms => Some(0.010),
        SyntheticNavBitMode::NativeSymbolSequence => Some(GPS_L1_CA_NAV_BIT_PERIOD_S),
        SyntheticNavBitMode::GpsL5QNh20 => Some(0.001),
    }
}

fn resolved_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
) -> bijux_gnss_core::api::SignalCode {
    if signal_code != bijux_gnss_core::api::SignalCode::Unknown {
        return signal_code;
    }
    match (sat.constellation, signal_band) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L1) => {
            bijux_gnss_core::api::SignalCode::Ca
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L2) => {
            bijux_gnss_core::api::SignalCode::L2C
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5) => {
            bijux_gnss_core::api::SignalCode::L5I
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1) => {
            bijux_gnss_core::api::SignalCode::E1B
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5) => {
            bijux_gnss_core::api::SignalCode::E5a
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1) => {
            bijux_gnss_core::api::SignalCode::B1I
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2) => {
            bijux_gnss_core::api::SignalCode::B2I
        }
        _ => bijux_gnss_core::api::SignalCode::Unknown,
    }
}

#[cfg(test)]
fn nav_bit_mode_from_flip(data_bit_flip: bool) -> SyntheticNavBitMode {
    if data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

#[cfg(test)]
fn navigation_data_for_nav_mode(nav_bit_mode: SyntheticNavBitMode) -> SyntheticNavigationData {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::ConstantNegative => SyntheticNavigationData::ConstantNegative,
        SyntheticNavBitMode::GlonassL1FixedDataString => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::GlonassL1AlternatingDataString => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::GlonassL1CustomDataString => {
            SyntheticNavigationData::GlonassL1String {
                raw_data_bits: glonass_l1_raw_data_bits(
                    &SyntheticNavigationData::AlternatingStartPositive,
                )
                .to_vec(),
            }
        }
        SyntheticNavBitMode::AlternatingGpsLnav20ms
        | SyntheticNavBitMode::AlternatingGalileoInav4ms
        | SyntheticNavBitMode::AlternatingGpsL5I10ms => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::NativeSymbolSequence => {
            SyntheticNavigationData::SymbolSequence(vec![1, -1])
        }
        SyntheticNavBitMode::GpsL5QNh20 => SyntheticNavigationData::ConstantPositive,
    }
}

fn glonass_l1_raw_data_bits(
    navigation_data: &SyntheticNavigationData,
) -> [i8; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS] {
    let len = bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS;
    match navigation_data {
        SyntheticNavigationData::ConstantPositive => {
            [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS]
        }
        SyntheticNavigationData::AlternatingStartPositive => {
            let mut raw_data_bits = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (index, bit) in raw_data_bits.iter_mut().enumerate().skip(1) {
                *bit = if index % 2 == 0 { 1 } else { -1 };
            }
            raw_data_bits
        }
        SyntheticNavigationData::GlonassL1String { raw_data_bits } => {
            assert!(
                raw_data_bits.len() == len,
                "synthetic glonass l1 raw data string must contain {len} bits"
            );
            let mut validated = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (slot, bit) in validated.iter_mut().zip(raw_data_bits.iter().copied()) {
                *slot = validate_navigation_symbol(bit);
            }
            validated
        }
        _ => {
            panic!("glonass l1 string modulation requires constant positive, alternating start positive, or an explicit raw data string")
        }
    }
}

#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}

#[cfg(test)]
mod signal_generation_tests {
    use super::{synthetic_replica_model, SyntheticNavigationData, SyntheticSignalParams};
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
    use bijux_gnss_signal::api::ReplicaCodeModel;

    fn synthetic_signal_params(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
    ) -> SyntheticSignalParams {
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: SyntheticNavigationData::ConstantPositive,
        }
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5a_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5a,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5aQpsk { .. }), "{model:?}");
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5b_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5b,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5bQpsk { .. }), "{model:?}");
    }
}
