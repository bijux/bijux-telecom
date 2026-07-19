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
