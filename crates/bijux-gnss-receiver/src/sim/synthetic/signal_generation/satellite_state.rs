#[derive(Debug, Clone)]
struct SatState {
    doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
    doppler_jerk_hz_per_s2: f64,
    receiver_oscillator_model: SyntheticReceiverOscillatorModel,
    receiver_oscillator_phase_noise_knots: Vec<(u64, f64)>,
    receiver_oscillator_noise_knots: ReceiverOscillatorNoiseKnots,
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

#[derive(Debug, Clone, Default)]
struct ReceiverOscillatorNoiseKnots {
    white_phase_rad: Vec<(u64, f64)>,
    frequency_noise_hz: Vec<(u64, f64)>,
    frequency_noise_phase_rad: Vec<(u64, f64)>,
}

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
        let receiver_oscillator_noise_knots =
            receiver_oscillator_noise_knots(&receiver_oscillator_model, sample_count, config.sampling_freq_hz);
        let signal_model = synthetic_replica_model(&params);
        let nav_bit_mode = nav_bit_mode(&params);
        Self {
            doppler_hz: params.doppler_hz,
            doppler_rate_hz_per_s,
            doppler_jerk_hz_per_s2,
            receiver_oscillator_model,
            receiver_oscillator_phase_noise_knots,
            receiver_oscillator_noise_knots,
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

    fn receiver_oscillator_noise_phase_rad_at_nominal_time_s(&self, nominal_elapsed_s: f64) -> f64 {
        let sample_index = ((nominal_elapsed_s * self.sample_rate_hz).round() as u64).min(
            self.receiver_oscillator_noise_knots
                .frequency_noise_phase_rad
                .last()
                .map(|(index, _)| *index)
                .unwrap_or(0),
        );
        receiver_oscillator_noise_phase_rad_from_knots(
            &self.receiver_oscillator_noise_knots,
            sample_index,
            self.sample_rate_hz,
        )
    }

    #[cfg(test)]
    fn receiver_oscillator_frequency_noise_hz_at_nominal_time_s(
        &self,
        nominal_elapsed_s: f64,
    ) -> f64 {
        let sample_index = ((nominal_elapsed_s * self.sample_rate_hz).round() as u64).min(
            self.receiver_oscillator_noise_knots
                .frequency_noise_hz
                .last()
                .map(|(index, _)| *index)
                .unwrap_or(0),
        );
        receiver_oscillator_frequency_noise_hz_from_knots(
            &self.receiver_oscillator_noise_knots,
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
            + self.receiver_oscillator_frequency_noise_hz_at_nominal_time_s(t)
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
            + self.receiver_oscillator_noise_phase_rad_at_nominal_time_s(t)
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
