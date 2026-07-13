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
    sat_states: Vec<SatState>,
    gps_ephemerides: Vec<GpsEphemeris>,
    rng: XorShift64,
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
        Self::with_noise_std(config, scenario, SYNTHETIC_NOISE_STD_PER_COMPONENT)
    }

    /// Build a streaming synthetic source that emits only the deterministic signal component.
    pub fn new_signal_only(config: &ReceiverPipelineConfig, scenario: &SyntheticScenario) -> Self {
        Self::with_noise_std(config, scenario, 0.0)
    }

    fn with_noise_std(
        config: &ReceiverPipelineConfig,
        scenario: &SyntheticScenario,
        noise_std: f32,
    ) -> Self {
        let sample_count = (scenario.duration_s * config.sampling_freq_hz).round() as usize;

        Self {
            sample_rate_hz: config.sampling_freq_hz,
            dt_s: 1.0 / config.sampling_freq_hz,
            remaining_samples: sample_count,
            next_sample_index: 0,
            noise_std,
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
            rng: XorShift64::new(scenario.seed),
        }
    }

    /// Broadcast ephemerides carried alongside this synthetic source.
    pub fn gps_ephemerides(&self) -> &[GpsEphemeris] {
        &self.gps_ephemerides
    }
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
    data_bit_flip: bool,
    signal_model: SyntheticSignalModel,
    if_hz: f64,
    sample_rate_hz: f64,
}

#[derive(Debug, Clone)]
enum SyntheticSignalModel {
    GpsL1Ca { code: Vec<i8> },
    GalileoE1 { e1b_code: Vec<i8>, e1c_code: Vec<i8> },
    BeidouB1I { code: Vec<i8> },
    GlonassL1 { code: Vec<i8> },
}

impl SyntheticSignalModel {
    fn for_satellite(params: SyntheticSignalParams) -> Self {
        match params.sat.constellation {
            Constellation::Galileo => Self::GalileoE1 {
                e1b_code: bijux_gnss_signal::api::generate_galileo_e1b_code(params.sat.prn)
                    .unwrap_or_else(|_| vec![1; 4092]),
                e1c_code: bijux_gnss_signal::api::generate_galileo_e1c_code(params.sat.prn)
                    .unwrap_or_else(|_| vec![1; 4092]),
            },
            Constellation::Beidou => Self::BeidouB1I {
                code: bijux_gnss_signal::api::generate_beidou_b1i_code(params.sat.prn)
                    .unwrap_or_else(|_| vec![1; 2046]),
            },
            Constellation::Glonass => Self::GlonassL1 {
                code: bijux_gnss_signal::api::generate_glonass_l1_st_code(),
            },
            _ => Self::GpsL1Ca {
                code: generate_ca_code(Prn(params.sat.prn)).unwrap_or_else(|_| vec![1; 1023]),
            },
        }
    }

    fn code_rate_hz(&self) -> f64 {
        match self {
            Self::GpsL1Ca { .. } => 1_023_000.0,
            Self::GalileoE1 { .. } => bijux_gnss_signal::api::GALILEO_E1_CODE_RATE_HZ,
            Self::BeidouB1I { .. } => bijux_gnss_signal::api::BEIDOU_B1I_CODE_RATE_HZ,
            Self::GlonassL1 { .. } => bijux_gnss_signal::api::GLONASS_L1_ST_CODE_RATE_HZ,
        }
    }

    fn code_length(&self) -> usize {
        match self {
            Self::GpsL1Ca { code } => code.len(),
            Self::GalileoE1 { e1b_code, .. } => e1b_code.len(),
            Self::BeidouB1I { code } => code.len(),
            Self::GlonassL1 { code } => code.len(),
        }
    }

    fn sample_value(&self, chip_phase: f64, primary_code_period_index: usize, data_bit: i8) -> f32 {
        match self {
            Self::GpsL1Ca { code } => code_value_at_phase(code, chip_phase).unwrap_or(1.0) * data_bit as f32,
            Self::GalileoE1 { e1b_code, e1c_code } => {
                bijux_gnss_signal::api::galileo_e1_cboc_value(
                    e1b_code,
                    e1c_code,
                    chip_phase,
                    primary_code_period_index,
                    data_bit,
                )
                .unwrap_or(1.0)
            }
            Self::BeidouB1I { code } | Self::GlonassL1 { code } => {
                code_value_at_phase(code, chip_phase).unwrap_or(1.0) * data_bit as f32
            }
        }
    }
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
        Self {
            doppler_hz: params.doppler_hz,
            doppler_rate_hz_per_s,
            receiver_clock_frequency_bias_hz,
            code_phase_chips: params.code_phase_chips,
            carrier_phase_rad: params.carrier_phase_rad,
            cn0_db_hz: params.cn0_db_hz,
            data_bit_flip: params.data_bit_flip,
            signal_model: SyntheticSignalModel::for_satellite(params),
            if_hz: synthetic_intermediate_frequency_hz(
                config.intermediate_freq_hz,
                params.sat,
                params.glonass_frequency_channel,
            ),
            sample_rate_hz: config.sampling_freq_hz,
        }
    }

    fn carrier_hz_at(&self, t: f64) -> f64 {
        self.if_hz
            + self.doppler_hz
            + self.receiver_clock_frequency_bias_hz
            + self.doppler_rate_hz_per_s * t
    }

    fn carrier_phase_rad_at(&self, t: f64) -> f64 {
        let initial_carrier_hz = self.carrier_hz_at(0.0);
        self.carrier_phase_rad
            + std::f64::consts::TAU
                * (initial_carrier_hz * t + 0.5 * self.doppler_rate_hz_per_s * t * t)
    }

    fn sample_at(&self, t: f64) -> Complex<f32> {
        let total_chip_phase = self.code_phase_chips + (self.signal_model.code_rate_hz() * t);
        let code_length = self.signal_model.code_length() as f64;
        let primary_code_period_index = if total_chip_phase <= 0.0 {
            0
        } else {
            (total_chip_phase / code_length).floor() as usize
        };
        let code_phase = total_chip_phase.rem_euclid(code_length);
        let data_bit = nav_bit_sign_at_time_s(self.data_bit_flip, t);
        let signal_value =
            self.signal_model.sample_value(code_phase, primary_code_period_index, data_bit);

        let phase = self.carrier_phase_rad_at(t) as f32;
        let carrier = Complex::new(phase.cos(), phase.sin());

        let amplitude = signal_amplitude_from_cn0(self.cn0_db_hz, self.sample_rate_hz);

        carrier * (signal_value * amplitude)
    }
}

fn signal_amplitude_from_cn0(cn0_db_hz: f32, sample_rate_hz: f64) -> f32 {
    let cn0_linear = 10.0_f64.powf(cn0_db_hz as f64 / 10.0).max(1e-12);
    ((cn0_linear * SYNTHETIC_COMPLEX_NOISE_POWER) / sample_rate_hz).sqrt() as f32
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
            doppler_hz: sat_truth.doppler_hz,
            code_phase_chips: sat_truth.code_phase_chips,
            carrier_phase_rad: sat_truth.carrier_phase_rad,
            cn0_db_hz: sat_truth.cn0_db_hz,
            data_bit_flip: sat_truth.nav_bit_mode == SyntheticNavBitMode::AlternatingGpsLnav20ms,
        }],
        ephemerides: Vec::new(),
        id: sat_truth.sat.prn.to_string(),
    }
}

fn code_phase_samples_at_epoch_start(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    code_phase_chips: f64,
) -> f64 {
    code_phase_samples_at_sample_index(
        config,
        frame.t0.sample_rate_hz,
        frame.t0.sample_index,
        code_phase_chips,
    )
}

fn synthetic_intermediate_frequency_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    intermediate_freq_hz
        + (synthetic_constellation_carrier_hz(sat, glonass_frequency_channel)
            - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}

fn synthetic_carrier_hz(
    intermediate_freq_hz: f64,
    sat: SatId,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    doppler_hz: f64,
) -> f64 {
    carrier_hz_from_doppler_hz(
        synthetic_intermediate_frequency_hz(intermediate_freq_hz, sat, glonass_frequency_channel),
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
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> f64 {
    match sat.constellation {
        Constellation::Galileo => bijux_gnss_core::api::GALILEO_E1_CARRIER_HZ.value(),
        Constellation::Gps => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
        Constellation::Beidou => bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ.value(),
        Constellation::Glonass => bijux_gnss_core::api::glonass_l1_carrier_hz(
            glonass_frequency_channel.unwrap_or_else(|| {
                panic!(
                    "GLONASS synthetic signal for {} requires glonass_frequency_channel",
                    bijux_gnss_core::api::format_sat(sat)
                )
            }),
        )
        .value(),
        _ => bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value(),
    }
}

fn nav_bit_mode(params: &SyntheticSignalParams) -> SyntheticNavBitMode {
    if params.data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

fn nav_bit_sign_at_time_s(data_bit_flip: bool, time_s: f64) -> i8 {
    if !data_bit_flip {
        return 1;
    }
    nav_bit_sign_for_index(nav_bit_index_at_time_s(time_s))
}

fn nav_bit_index_at_time_s(time_s: f64) -> u64 {
    if !time_s.is_finite() || time_s <= 0.0 {
        return 0;
    }
    (time_s / GPS_L1_CA_NAV_BIT_PERIOD_S).floor() as u64
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
    data_bit_flip: bool,
) -> Vec<SyntheticNavBitSegment> {
    if sample_count == 0 {
        return Vec::new();
    }
    if !data_bit_flip {
        return vec![SyntheticNavBitSegment {
            start_sample: 0,
            end_sample: sample_count,
            start_s: 0.0,
            end_s: sample_count as f64 / sample_rate_hz,
            bit: 1,
        }];
    }

    let mut segments = Vec::new();
    let mut bit_index = 0u64;
    loop {
        let start_sample =
            ((bit_index as f64 * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz).ceil()) as u64;
        if start_sample >= sample_count {
            break;
        }
        let end_sample = ((((bit_index + 1) as f64) * GPS_L1_CA_NAV_BIT_PERIOD_S * sample_rate_hz)
            .ceil()) as u64;
        let clamped_end = end_sample.min(sample_count);
        segments.push(SyntheticNavBitSegment {
            start_sample,
            end_sample: clamped_end,
            start_s: start_sample as f64 / sample_rate_hz,
            end_s: clamped_end as f64 / sample_rate_hz,
            bit: nav_bit_sign_for_index(bit_index),
        });
        bit_index += 1;
    }
    segments
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
