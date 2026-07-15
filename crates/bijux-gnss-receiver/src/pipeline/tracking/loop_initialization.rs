impl Tracking {
    fn tracking_loop_profile(tracking_params: TrackingParams) -> SignalTrackingLoopProfile {
        SignalTrackingLoopProfile {
            dll_bw_hz: tracking_params.dll_bw_hz,
            pll_bw_hz: tracking_params.pll_bw_hz,
            fll_bw_hz: tracking_params.fll_bw_hz,
            integration_ms: tracking_params.integration_ms.max(1),
        }
    }

    fn tracking_params_for_state(
        &self,
        base_tracking_params: TrackingParams,
        state: &LoopState,
    ) -> TrackingParams {
        if !self.config.adaptive_tracking_enabled {
            return base_tracking_params;
        }
        TrackingParams {
            early_late_spacing_chips: base_tracking_params.early_late_spacing_chips,
            dll_bw_hz: state.tracking_loop_profile.dll_bw_hz,
            pll_bw_hz: state.tracking_loop_profile.pll_bw_hz,
            fll_bw_hz: state.tracking_loop_profile.fll_bw_hz,
            integration_ms: state.tracking_loop_profile.integration_ms.max(1),
        }
    }

    fn initial_loop_state(
        &self,
        signal_model: &TrackingSignalModel,
        carrier_hz: f64,
        code_phase_samples: f64,
        acquisition_cn0_proxy_dbhz: f64,
        signal_delay_alignment: Option<SignalDelayAlignment>,
        subcarrier_code_phase_refined: bool,
        tracking_params: TrackingParams,
        reacquisition_pending: bool,
    ) -> LoopState {
        let code_rate_reference_hz =
            carrier_aided_code_rate_hz(&self.config, signal_model, carrier_hz);
        LoopState {
            carrier_hz,
            carrier_phase_cycles: pilot_carrier_phase_offset_cycles(signal_model),
            carrier_rate_hz_per_s: 0.0,
            code_rate_hz: code_rate_reference_hz,
            code_rate_reference_hz,
            code_phase_samples,
            tracking_adaptation_state: SignalTrackingAdaptationState::default(),
            tracking_loop_profile: Self::tracking_loop_profile(tracking_params),
            signal_delay_alignment,
            subcarrier_code_phase_refined,
            acquisition_cn0_proxy_dbhz,
            lock_reference_cn0_dbhz: acquisition_cn0_proxy_dbhz,
            prev_prompt: None,
            prev_prompt_phase_cycles: None,
            secondary_code_prompt_history: VecDeque::new(),
            secondary_code_sync: None,
            nav_bit_phase_offset_cycles: 0.0,
            nav_bit_transition_count: 0,
            pull_in_stable_epochs: 0,
            weak_cn0_epochs: 0,
            degraded_epochs: 0,
            prompt_power_reference: 0.0,
            prompt_cn0_window: VecDeque::new(),
            code_error_window_samples: VecDeque::new(),
            carrier_phase_error_window_cycles: VecDeque::new(),
            doppler_error_window_hz: VecDeque::new(),
            cn0_estimate_window_dbhz: VecDeque::new(),
            unstable_discriminator_epochs: 0,
            state: ChannelState::Acquired,
            unlocked_count: 0,
            lost_reason: None,
            reacquisition_candidate: None,
            reacquisition_candidate_streak: 0,
            reacquisition_pending,
            reacquisition_attempt_epochs: 0,
            reacquisition_stable_tracking_epochs: 0,
        }
    }
}
