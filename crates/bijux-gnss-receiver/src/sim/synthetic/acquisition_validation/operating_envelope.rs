/// Build deterministic operating-envelope cases for every acquisition-supported signal.
pub fn default_supported_acquisition_operating_envelope_signal_cases(
) -> Vec<SyntheticAcquisitionOperatingEnvelopeSignalCase> {
    crate::engine::support_matrix::build_support_matrix()
        .rows
        .into_iter()
        .filter(|row| row.stage_support.acquisition == crate::api::core::SupportStatus::Supported)
        .map(default_supported_acquisition_operating_envelope_signal_case)
        .collect()
}

/// Measure per-signal acquisition operating envelopes across supported sweep axes.
pub fn measure_truth_guided_acquisition_operating_envelopes(
    cases: &[SyntheticAcquisitionOperatingEnvelopeSignalCase],
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionOperatingEnvelopeReport {
    let signals = cases
        .iter()
        .map(|case| {
            measure_truth_guided_acquisition_operating_envelope_signal(
                case,
                trial_seeds,
                scenario_id_prefix,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            )
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionOperatingEnvelopeReport {
        scenario_id_prefix: scenario_id_prefix.to_string(),
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
        signals,
    }
}

fn default_supported_acquisition_operating_envelope_signal_case(
    row: bijux_gnss_core::api::SignalSupportRow,
) -> SyntheticAcquisitionOperatingEnvelopeSignalCase {
    let signal = default_operating_envelope_signal_seed(
        row.constellation,
        row.band,
        row.code,
        default_operating_envelope_glonass_frequency_channel(row.constellation),
    );
    let config = default_operating_envelope_config_for_signal(
        signal.sat,
        signal.signal_band,
        signal.signal_code,
        signal.glonass_frequency_channel,
    );
    let integration_profiles = default_operating_envelope_integration_profiles(&config, &signal);
    let code_length = config.code_length.max(1) as f64;
    let shifted_code_phase_chips =
        wrap_code_phase_chips(signal.code_phase_chips + (code_length / 3.0), code_length);
    let acquisition_doppler_step_hz = config.acquisition_doppler_step_hz;

    SyntheticAcquisitionOperatingEnvelopeSignalCase {
        config,
        signal: signal.clone(),
        integration_profiles,
        cn0_db_hz_points: vec![signal.cn0_db_hz - 8.0, signal.cn0_db_hz],
        doppler_hz_points: vec![
            0.0,
            baseline_profile_doppler_offset_hz(&signal, acquisition_doppler_step_hz),
        ],
        receiver_clock_frequency_bias_hz_points: vec![0.0, acquisition_doppler_step_hz as f64],
        code_phase_chips_points: vec![signal.code_phase_chips, shifted_code_phase_chips],
    }
}

fn measure_truth_guided_acquisition_operating_envelope_signal(
    case: &SyntheticAcquisitionOperatingEnvelopeSignalCase,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionOperatingEnvelopeSignalReport {
    let baseline_profile = operating_envelope_baseline_integration_profile(case);
    let mut false_alarm_cache = BTreeMap::new();
    for profile in operating_envelope_effective_integration_profiles(case) {
        false_alarm_cache.insert(
            (profile.coherent_ms, profile.noncoherent),
            measure_targeted_noise_only_acquisition_false_alarm_profile(
                &case.config,
                &case.signal,
                profile.coherent_ms,
                profile.noncoherent,
                trial_seeds,
                &operating_envelope_false_alarm_case_id(scenario_id_prefix, &case.signal, profile),
            ),
        );
    }

    let points = operating_envelope_variants(case, baseline_profile)
        .into_iter()
        .map(|variant| {
            let profile_config = operating_envelope_profile_config(
                &case.config,
                variant.coherent_ms,
                variant.noncoherent,
            );
            let trials = measure_truth_guided_targeted_acquisition_sensitivity_trials(
                &profile_config,
                &variant.signal,
                variant.receiver_clock_frequency_bias_hz,
                trial_seeds,
                &operating_envelope_point_case_id(
                    scenario_id_prefix,
                    variant.axis,
                    &variant.signal,
                    variant.coherent_ms,
                    variant.noncoherent,
                    variant.receiver_clock_frequency_bias_hz,
                ),
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
            );
            let sensitivity = synthetic_acquisition_sensitivity_report(
                scenario_id_prefix,
                variant.signal.sat,
                Some(variant.signal.cn0_db_hz),
                variant.coherent_ms,
                variant.noncoherent,
                code_phase_tolerance_samples,
                doppler_tolerance_bins,
                profile_config.acquisition_doppler_step_hz.max(1),
                trials,
            );
            let false_alarm = false_alarm_cache
                .get(&(variant.coherent_ms, variant.noncoherent))
                .expect("operating envelope false-alarm profile");

            SyntheticAcquisitionOperatingEnvelopePoint {
                axis: variant.axis,
                sat: variant.signal.sat,
                signal_band: variant.signal.signal_band,
                signal_code: variant.signal.signal_code,
                glonass_frequency_channel: variant.signal.glonass_frequency_channel,
                cn0_db_hz: variant.signal.cn0_db_hz,
                coherent_ms: variant.coherent_ms,
                noncoherent: variant.noncoherent,
                doppler_hz: variant.signal.doppler_hz,
                receiver_clock_frequency_bias_hz: variant.receiver_clock_frequency_bias_hz,
                code_phase_chips: variant.signal.code_phase_chips,
                trial_count: sensitivity.trial_count,
                accepted_count: sensitivity.accepted_count,
                detected_count: sensitivity.detected_count,
                false_alarm_trial_count: false_alarm.trial_count,
                false_alarm_count: false_alarm.false_alarm_count,
                acceptance_probability: sensitivity.acceptance_probability,
                detection_probability: sensitivity.detection_probability,
                false_alarm_rate: false_alarm.false_alarm_rate,
                mean_peak_mean_ratio: sensitivity.mean_peak_mean_ratio,
            }
        })
        .collect::<Vec<_>>();

    SyntheticAcquisitionOperatingEnvelopeSignalReport {
        sat: case.signal.sat,
        signal_band: case.signal.signal_band,
        signal_code: case.signal.signal_code,
        glonass_frequency_channel: case.signal.glonass_frequency_channel,
        sampling_freq_hz: case.config.sampling_freq_hz,
        intermediate_freq_hz: case.config.intermediate_freq_hz,
        code_freq_basis_hz: case.config.code_freq_basis_hz,
        code_length: case.config.code_length,
        acquisition_doppler_search_hz: case.config.acquisition_doppler_search_hz,
        acquisition_doppler_step_hz: case.config.acquisition_doppler_step_hz.max(1),
        points,
    }
}
