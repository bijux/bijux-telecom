impl Tracking {
    fn try_reacquire_channel(
        &self,
        channel: &mut IncrementalTrackingChannel,
        frame: &SamplesFrame,
    ) -> ReacquisitionOutcome {
        if channel.state.state != ChannelState::Lost {
            return ReacquisitionOutcome::NotNeeded;
        }
        let Some(sustained_loss_seed) = sustained_lock_loss_reacquire_seed(&channel.epochs) else {
            channel.state.reacquisition_candidate = None;
            channel.state.reacquisition_candidate_streak = 0;
            return ReacquisitionOutcome::SeedUnavailable;
        };
        let reacquisition_code_phase_samples = project_reacquisition_code_phase_samples(
            sustained_loss_seed,
            frame.t0.sample_index,
            self.config.code_freq_basis_hz,
            samples_per_code(
                self.config.sampling_freq_hz,
                self.config.code_freq_basis_hz,
                self.config.code_length,
            ),
        );
        let Some(seed) = self.quick_reacquire(ReacquisitionSearchRequest {
            frame,
            sat: channel.sat,
            carrier_hz: sustained_loss_seed.carrier_hz,
            code_phase_samples: reacquisition_code_phase_samples,
            lock_reference_cn0_dbhz: channel.state.lock_reference_cn0_dbhz,
            min_prompt_power: channel.state.prompt_power_reference
                * REACQUISITION_REFERENCE_PROMPT_POWER_RATIO,
            acquisition_uncertainty: channel.acquisition_uncertainty.as_ref(),
        }) else {
            channel.state.reacquisition_candidate = None;
            channel.state.reacquisition_candidate_streak = 0;
            return ReacquisitionOutcome::Failed;
        };
        let candidate_streak = if channel.state.reacquisition_candidate.is_some_and(|candidate| {
            self.reacquisition_seed_matches(
                candidate,
                seed,
                channel.acquisition_uncertainty.as_ref(),
            )
        }) {
            channel.state.reacquisition_candidate_streak.saturating_add(1)
        } else {
            1
        };
        channel.state.reacquisition_candidate = Some(seed);
        channel.state.reacquisition_candidate_streak = candidate_streak;
        if candidate_streak < REACQUISITION_CONFIRMATION_EPOCHS {
            return ReacquisitionOutcome::Failed;
        }

        channel.state = self.initial_loop_state(TrackingLoopInitialization {
            signal_model: &channel.signal_model,
            carrier_hz: seed.carrier_hz,
            code_phase_samples: seed.code_phase_samples,
            acquisition_cn0_proxy_dbhz: seed.cn0_dbhz,
            signal_delay_alignment: channel.state.signal_delay_alignment.clone(),
            subcarrier_code_phase_refined: channel.state.subcarrier_code_phase_refined,
            tracking_params: channel.tracking_params,
            reacquisition_pending: true,
        });
        channel.state.carrier_phase_cycles = wrap_phase_cycles_signed(
            channel.state.carrier_phase_cycles + seed.carrier_sign.phase_offset_cycles(),
        );
        ReacquisitionOutcome::Started
    }

    fn reacquisition_seed_matches(
        &self,
        left: ReacquisitionSeed,
        right: ReacquisitionSeed,
        acquisition_uncertainty: Option<&AcqUncertainty>,
    ) -> bool {
        let doppler_step_hz = self.config.acquisition_doppler_step_hz.max(1) as f64;
        let doppler_tolerance_hz = acquisition_uncertainty
            .map(|uncertainty| uncertainty.doppler_hz.clamp(doppler_step_hz / 2.0, doppler_step_hz))
            .unwrap_or(doppler_step_hz);
        let code_tolerance_samples = acquisition_uncertainty
            .map(|uncertainty| uncertainty.code_phase_samples.clamp(0.5, 2.0))
            .unwrap_or(2.0);
        (left.carrier_hz - right.carrier_hz).abs() <= doppler_tolerance_hz
            && (left.code_phase_samples - right.code_phase_samples).abs() <= code_tolerance_samples
            && left.carrier_sign == right.carrier_sign
            && left.secondary_code_phase_periods == right.secondary_code_phase_periods
    }

    fn quick_reacquire(
        &self,
        request: ReacquisitionSearchRequest<'_>,
    ) -> Option<ReacquisitionSeed> {
        let ReacquisitionSearchRequest {
            frame,
            sat,
            carrier_hz,
            code_phase_samples,
            lock_reference_cn0_dbhz,
            min_prompt_power,
            acquisition_uncertainty,
        } = request;
        let doppler_step_hz = self.config.acquisition_doppler_step_hz.max(1) as f64;
        let doppler_step = acquisition_uncertainty
            .map(|uncertainty| uncertainty.doppler_hz.clamp(doppler_step_hz / 2.0, doppler_step_hz))
            .unwrap_or(doppler_step_hz);
        let code_step = acquisition_uncertainty
            .map(|uncertainty| uncertainty.code_phase_samples.clamp(0.5, 2.0))
            .unwrap_or(1.0);
        let doppler_bins = [-2_i8, -1, 0, 1, 2];
        let code_bins = [-2_i8, -1, 0, 1, 2];
        let signal_model = TrackingSignalModel::for_sat(&self.config, sat);
        let carrier_signs = reacquisition_carrier_signs(&signal_model);
        let secondary_code_phases = reacquisition_secondary_code_phase_periods(&signal_model);
        let mut candidates = Vec::new();
        let min_cn0_dbhz = reacquisition_min_cn0_dbhz(lock_reference_cn0_dbhz);
        for doppler_bin in doppler_bins {
            for code_bin in code_bins {
                let doppler_offset_hz = f64::from(doppler_bin) * doppler_step;
                let code_offset_samples = f64::from(code_bin) * code_step;
                let candidate_carrier_hz = carrier_hz + doppler_offset_hz;
                let candidate_code_phase_samples = code_phase_samples + code_offset_samples;
                for &carrier_sign in carrier_signs {
                    for &secondary_code_phase_periods in &secondary_code_phases {
                        let corr = self.correlate_reacquisition_epoch(
                            frame,
                            &signal_model,
                            candidate_carrier_hz,
                            carrier_sign.phase_offset_cycles(),
                            self.config.code_freq_basis_hz,
                            candidate_code_phase_samples,
                            0.5,
                            secondary_code_phase_periods,
                        );
                        let cn0_dbhz = estimate_cn0_dbhz(
                            corr.prompt,
                            corr.early - corr.late,
                            self.config.sampling_freq_hz,
                            frame.len() as f64,
                            corr.early_late_noise_weight_energy,
                        );
                        let anti_false_lock =
                            anti_false_lock_detected(corr.early, corr.prompt, corr.late);
                        if !anti_false_lock {
                            candidates.push(ReacquisitionCandidate {
                                hypothesis: ReacquisitionHypothesis {
                                    doppler_bin,
                                    code_bin,
                                    carrier_sign,
                                    secondary_code_phase_periods,
                                },
                                carrier_hz: candidate_carrier_hz,
                                code_phase_samples: candidate_code_phase_samples,
                                cn0_dbhz,
                                prompt_power: corr.prompt.norm_sqr(),
                            });
                        }
                    }
                }
            }
        }
        match select_reacquisition_candidate(&candidates, min_cn0_dbhz, min_prompt_power) {
            ReacquisitionSelection::Accepted(seed) => Some(seed),
            ReacquisitionSelection::Refused => None,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn correlate_reacquisition_epoch(
        &self,
        frame: &SamplesFrame,
        signal_model: &TrackingSignalModel,
        carrier_freq_hz: f64,
        carrier_phase_cycles: f64,
        code_rate_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
        secondary_code_phase_periods: Option<usize>,
    ) -> CorrelatorOutput {
        let sample_rate_hz = self.config.sampling_freq_hz;
        let primary_code_period_samples = signal_model.samples_per_code(sample_rate_hz);
        let epoch_primary_code_period_index =
            frame.t0.sample_index as usize / primary_code_period_samples.max(1);
        let primary_code_period_index = secondary_code_phase_periods
            .and_then(|phase_periods| {
                let component = secondary_code_sync_component(signal_model)?;
                let period = component.secondary_code_period()?.max(1);
                let period_base =
                    epoch_primary_code_period_index - (epoch_primary_code_period_index % period);
                Some(period_base + (phase_periods % period))
            })
            .unwrap_or(epoch_primary_code_period_index);
        let tracked_chips_per_sample = code_rate_hz / sample_rate_hz;
        let epoch_start_code_phase_samples = epoch_start_code_phase_samples_from_receiver_phase(
            code_phase_samples,
            primary_code_period_samples,
        );
        let base_chip_phase =
            epoch_start_code_phase_samples * signal_model.nominal_chips_per_sample(sample_rate_hz);
        correlate_early_prompt_late(
            EarlyPromptLateCorrelatorInput {
                samples: &frame.iq,
                sample_rate_hz,
                carrier_hz: carrier_freq_hz,
                carrier_phase_offset_radians: carrier_phase_offset_radians(carrier_phase_cycles),
                base_chip_phase,
                chips_per_sample: tracked_chips_per_sample,
                early_late_spacing_chips,
            },
            |chip_phase| signal_model.value_at_phase(chip_phase, primary_code_period_index),
        )
    }
}

#[derive(Debug, Clone, Copy)]
struct ReacquisitionSearchRequest<'a> {
    frame: &'a SamplesFrame,
    sat: SatId,
    carrier_hz: f64,
    code_phase_samples: f64,
    lock_reference_cn0_dbhz: f64,
    min_prompt_power: f32,
    acquisition_uncertainty: Option<&'a AcqUncertainty>,
}
