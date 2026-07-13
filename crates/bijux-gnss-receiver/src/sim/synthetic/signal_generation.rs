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
    let signal_only = generate_l1_ca_with_doppler_ramp_signal_only(config, params, duration_s);
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
    let signal_only = generate_l1_ca_multi_signal_only(config, scenario);
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
    rng: XorShift64,
}

/// Source-provided whole-code alignment for one synthetic signal.
#[derive(Debug, Clone)]
pub struct SyntheticSignalDelayAlignment {
    pub sat: SatId,
    pub signal_delay_alignment: SignalDelayAlignment,
}

fn generate_l1_ca_with_doppler_ramp_signal_only(
    config: &ReceiverPipelineConfig,
    params: SyntheticDopplerRampParams,
    duration_s: f64,
) -> SamplesFrame {
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (duration_s * config.sampling_freq_hz).round() as usize;
    let sat_state = SatState::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        config,
        params.signal,
        0.0,
        params.doppler_rate_hz_per_s,
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
    let clock = SampleClock::new(config.sampling_freq_hz);
    let dt_s = clock.dt_s();
    let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;
    let sat_states: Vec<SatState> = scenario
        .satellites
        .iter()
        .map(|sat| {
            SatState::new_with_receiver_clock_frequency_bias_hz(
                config,
                *sat,
                scenario.receiver_clock_frequency_bias_hz,
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
    let t0 = SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz };
    SamplesFrame::new(t0, Seconds(dt_s), iq)
}

impl SyntheticSignalSource {
    /// Build a streaming synthetic source from a scenario without materializing the full capture.
    pub fn new(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::with_noise_std(config, scenario, SYNTHETIC_NOISE_STD_PER_COMPONENT, Vec::new())
    }

    /// Build a streaming synthetic source that emits only the deterministic signal component.
    pub fn new_signal_only(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::with_noise_std(config, scenario, 0.0, Vec::new())
    }

    /// Build a streaming synthetic source with explicit whole-code signal-delay alignments.
    pub fn new_with_signal_delay_alignments(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
    ) -> Self {
        Self::with_noise_std(
            config,
            scenario,
            SYNTHETIC_NOISE_STD_PER_COMPONENT,
            signal_delay_alignments,
        )
    }

    fn with_noise_std(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        noise_std: f32,
        signal_delay_alignments: Vec<SyntheticSignalDelayAlignment>,
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
                    SatState::new_with_receiver_clock_frequency_bias_hz(
                        config,
                        *sat,
                        scenario.receiver_clock_frequency_bias_hz,
                    )
                })
                .collect(),
            gps_ephemerides: scenario.ephemerides.clone(),
            signal_delay_alignments,
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
    receiver_clock_frequency_bias_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
    nav_bit_mode: SyntheticNavBitMode,
    signal_model: SyntheticSignalModel,
    if_hz: f64,
    sample_rate_hz: f64,
}

type SyntheticSignalModel = bijux_gnss_signal::api::ReplicaCodeModel;

fn synthetic_replica_model(params: SyntheticSignalParams) -> SyntheticSignalModel {
    let signal_code = resolved_signal_code(params.sat, params.signal_band, params.signal_code);
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
        Self::new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
            config,
            params,
            receiver_clock_frequency_bias_hz,
            0.0,
        )
    }

    fn new_with_doppler_rate_and_receiver_clock_frequency_bias_hz(
        config: &ReceiverPipelineConfig,
        params: SyntheticSignalParams,
        receiver_clock_frequency_bias_hz: f64,
        doppler_rate_hz_per_s: f64,
    ) -> Self {
        let signal_model = synthetic_replica_model(params);
        Self {
            doppler_hz: params.doppler_hz,
            doppler_rate_hz_per_s,
            receiver_clock_frequency_bias_hz,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            nav_bit_mode: nav_bit_mode(&params),
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

    fn initial_carrier_hz(&self) -> f64 {
        self.if_hz + self.doppler_hz + self.receiver_clock_frequency_bias_hz
    }

    #[cfg(test)]
    fn carrier_hz_at(&self, t: f64) -> f64 {
        bijux_gnss_signal::api::carrier_hz_at_time(
            self.initial_carrier_hz(),
            self.doppler_rate_hz_per_s,
            t,
        )
    }

    fn carrier_phase_rad_at(&self, t: f64) -> f64 {
        bijux_gnss_signal::api::carrier_phase_radians_at_time(
            self.carrier_phase_rad,
            self.initial_carrier_hz(),
            self.doppler_rate_hz_per_s,
            t,
        )
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let data_bit = nav_bit_sign_for_mode_at_time_s(self.nav_bit_mode, t);
        let amplitude = signal_amplitude_from_cn0(self.cn0_db_hz, self.sample_rate_hz);
        bijux_gnss_signal::api::sample_modulated_replica_at_time(
            &self.signal_model,
            self.code_phase_chips,
            self.carrier_phase_rad,
            self.initial_carrier_hz(),
            self.doppler_rate_hz_per_s,
            t,
            data_bit,
            amplitude,
        )
        .unwrap_or_else(|_| Complex::new(0.0, 0.0))
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
    let isolated_frame = generate_l1_ca_multi_signal_only(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
    );
    let iq = isolated_frame
        .iq
        .iter()
        .map(|sample| *sample * truth.output_scale_applied)
        .collect::<Vec<_>>();
    SamplesFrame::new(measured_frame.t0, measured_frame.dt_s, iq)
}

fn regenerate_isolated_scaled_satellite_frame_with_noise(
    config: &ReceiverPipelineConfig,
    measured_frame: &SamplesFrame,
    truth: &SyntheticIqTruthBundle,
    sat_truth: &SyntheticSatelliteTruth,
) -> SamplesFrame {
    let isolated_frame = generate_l1_ca_multi(
        config,
        &isolated_satellite_scenario(measured_frame, truth, sat_truth),
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
            data_bit_flip: sat_truth.nav_bit_mode != SyntheticNavBitMode::ConstantPositive,
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
        )
            - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
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
    match (
        resolved_signal_code(params.sat, params.signal_band, params.signal_code),
        params.data_bit_flip,
        params.sat.constellation,
        params.signal_band,
    ) {
        (bijux_gnss_core::api::SignalCode::L5Q, _, bijux_gnss_core::api::Constellation::Gps, SignalBand::L5) => {
            SyntheticNavBitMode::GpsL5QNh20
        }
        (_, false, _, _) => SyntheticNavBitMode::ConstantPositive,
        (
            bijux_gnss_core::api::SignalCode::E5b,
            true,
            bijux_gnss_core::api::Constellation::Galileo,
            SignalBand::E5,
        ) => SyntheticNavBitMode::AlternatingGalileoInav4ms,
        (bijux_gnss_core::api::SignalCode::L5I, true, bijux_gnss_core::api::Constellation::Gps, SignalBand::L5) => {
            SyntheticNavBitMode::AlternatingGpsL5I10ms
        }
        (_, true, _, _) => SyntheticNavBitMode::AlternatingGpsLnav20ms,
    }
}

fn nav_bit_sign_for_mode_at_time_s(nav_bit_mode: SyntheticNavBitMode, time_s: f64) -> i8 {
    match nav_bit_mode {
        SyntheticNavBitMode::GpsL5QNh20 => bijux_gnss_signal::api::gps_l5_q_epoch_symbol(
            nav_bit_index_for_mode_at_time_s(nav_bit_mode, time_s) as usize,
        ),
        _ => nav_bit_sign_for_index(nav_bit_index_for_mode_at_time_s(nav_bit_mode, time_s)),
    }
}

fn nav_bit_index_for_mode_at_time_s(nav_bit_mode: SyntheticNavBitMode, time_s: f64) -> u64 {
    let Some(symbol_period_s) = nav_symbol_period_s(nav_bit_mode) else {
        return 0;
    };
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / symbol_period_s).floor() as u64
}

#[cfg(test)]
fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    nav_bit_sign_for_mode_at_time_s(nav_bit_mode_from_flip(data_bit_flip), time_s)
}

#[cfg(test)]
fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    nav_bit_index_for_mode_at_time_s(SyntheticNavBitMode::AlternatingGpsLnav20ms, time_s)
}

fn nav_bit_sign_for_index(bit_index: u64) -> i8 {
    if bit_index % 2 == 0 {
        1
    } else {
        -1
    }
}

fn peak_component(samples: &[Complex<f32>]) -> f32 {
    samples.iter().flat_map(|sample| [sample.re.abs(), sample.im.abs()]).fold(0.0f32, f32::max)
}

fn encode_iq16_le_bytes(samples: &[Complex<f32>], scale: f32) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(samples.len() * 4);
    for sample in samples {
        encoded.extend_from_slice(&quantize_i16_component(sample.re * scale).to_le_bytes());
        encoded.extend_from_slice(&quantize_i16_component(sample.im * scale).to_le_bytes());
    }
    encoded
}

fn quantize_i16_component(value: f32) -> i16 {
    let scaled = (value * 32768.0).round();
    scaled.clamp(-32768.0, 32767.0) as i16
}

fn nav_bit_segments(
    sample_rate_hz: f64,
    sample_count: u64,
    nav_bit_mode: SyntheticNavBitMode,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }
    let Some(symbol_period_s) = nav_symbol_period_s(nav_bit_mode) else {
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
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s: start_sample as f64 / sample_rate_hz,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: nav_bit_sign_for_mode_at_time_s(nav_bit_mode, start_sample as f64 / sample_rate_hz),
        });
        bit_index += 1;
    }
    segments
}

fn nav_symbol_period_s(nav_bit_mode: SyntheticNavBitMode) -> Option<f64> {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive => None,
        SyntheticNavBitMode::AlternatingGpsLnav20ms => Some(GPS_L1_CA_NAV_BIT_PERIOD_S),
        SyntheticNavBitMode::AlternatingGalileoInav4ms => Some(0.004),
        SyntheticNavBitMode::AlternatingGpsL5I10ms => Some(0.010),
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
