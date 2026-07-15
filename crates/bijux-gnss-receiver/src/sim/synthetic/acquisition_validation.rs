include!("acquisition_validation/truth_table.rs");

include!("acquisition_validation/code_phase_refinement.rs");

include!("acquisition_validation/frequency_assistance.rs");

include!("acquisition_validation/detection_rate.rs");

include!("acquisition_validation/operating_envelope.rs");

include!("acquisition_validation/acquisition_trials.rs");

include!("acquisition_validation/interference.rs");

fn measure_truth_guided_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    measure_truth_guided_acquisition_trial_with_expected_measured_doppler_hz(
        config,
        frame,
        signal,
        signal.doppler_hz,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
    )
}

fn measure_truth_guided_acquisition_trial_with_expected_measured_doppler_hz(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    expected_measured_doppler_hz: f64,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    acquisition_trial_measurement_from_result(
        config,
        frame,
        signal,
        result,
        expected_measured_doppler_hz,
        code_phase_tolerance_samples,
        doppler_tolerance_bins,
    )
}

fn acquisition_trial_measurement_from_result(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
    result: crate::api::core::AcqResult,
    expected_measured_doppler_hz: f64,
    code_phase_tolerance_samples: usize,
    doppler_tolerance_bins: usize,
) -> SyntheticAcquisitionTrialMeasurement {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples(config, frame, signal.code_phase_chips);
    let code_phase_error_samples = wrapped_code_phase_error_samples(
        result.code_phase_samples,
        expected_code_phase_samples,
        period_samples,
    );
    let measured_doppler_hz = synthetic_measured_doppler_hz_from_carrier_hz(
        config.intermediate_freq_hz,
        signal.sat,
        signal.signal_band,
        signal.signal_code,
        signal.glonass_frequency_channel,
        result.carrier_hz.0,
    );
    let doppler_step_hz = config.acquisition_doppler_step_hz.max(1) as f64;
    let doppler_error_bins =
        (measured_doppler_hz - expected_measured_doppler_hz).abs() / doppler_step_hz;
    let accepted = matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted);
    let detected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected)
        && code_phase_error_samples <= code_phase_tolerance_samples
        && doppler_error_bins <= doppler_tolerance_bins as f64 + f64::EPSILON;

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted,
        detected,
        code_phase_error_samples: Some(code_phase_error_samples),
        doppler_error_bins: Some(doppler_error_bins),
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

fn measure_target_absent_acquisition_trial(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> SyntheticAcquisitionTrialMeasurement {
    let result = acquisition_result_for_target_signal(config, frame, signal);
    let non_rejected = !matches!(result.hypothesis, crate::api::core::AcqHypothesis::Rejected);

    SyntheticAcquisitionTrialMeasurement {
        hypothesis: result.hypothesis.to_string(),
        accepted: non_rejected,
        detected: false,
        code_phase_error_samples: None,
        doppler_error_bins: None,
        peak_mean_ratio: result.peak_mean_ratio,
    }
}

include!("acquisition_validation/uncertainty_coverage.rs");

fn acquisition_result_for_target_signal(
    config: &ReceiverPipelineConfig,
    frame: &SamplesFrame,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqResult {
    let acquisition = crate::pipeline::acquisition::Acquisition::new(
        config.clone(),
        crate::engine::runtime::ReceiverRuntime::default(),
    );
    acquisition
        .run_fft_for_requests(frame, &[acquisition_request_for_signal(config, signal)])
        .remove(0)
}

fn acquisition_request_for_signal(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
) -> crate::api::core::AcqRequest {
    crate::api::core::AcqRequest {
        sat: signal.sat,
        glonass_frequency_channel: signal.glonass_frequency_channel,
        signal_band: signal.signal_band,
        signal_code: signal.signal_code,
        doppler_center_hz: 0.0,
        doppler_rate_center_hz_per_s: 0.0,
        expected_line_of_sight_doppler_hz: Some(signal.doppler_hz),
        assistance_bounds: None,
        doppler_search_hz: config.acquisition_doppler_search_hz,
        doppler_step_hz: config.acquisition_doppler_step_hz,
        doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
        doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }
}

fn scale_synthetic_capture_frame(frame: &SamplesFrame) -> SamplesFrame {
    let peak_component_before_scaling = peak_component(&frame.iq);
    let output_scale_applied = if peak_component_before_scaling <= 0.999 {
        1.0
    } else {
        0.999 / peak_component_before_scaling
    };
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * output_scale_applied).collect(),
    )
}

fn scaled_synthetic_acquisition_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    description: &str,
) -> SamplesFrame {
    let frame = generate_l1_ca_multi(config, scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        scenario,
        &frame,
        "2026-07-14T00:00:00Z",
        Some(description.to_string()),
    );
    SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    )
}

fn interference_case_id(
    scenario_id_prefix: &str,
    case: &SyntheticAcquisitionInterferenceCase,
) -> String {
    format!(
        "{scenario_id_prefix}_prn_{}_band_{:?}_code_{:?}_interferers_{}_coherent_{}ms_noncoherent_{}",
        case.target_signal.sat.prn,
        case.target_signal.signal_band,
        case.target_signal.signal_code,
        case.interfering_signals.len(),
        case.coherent_ms,
        case.noncoherent,
    )
}

fn measure_targeted_noise_only_acquisition_false_alarm_profile(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
    coherent_ms: u32,
    noncoherent: u32,
    trial_seeds: &[u64],
    scenario_id_prefix: &str,
) -> SyntheticAcquisitionFalseAlarmProfileMeasurement {
    let profile_config = operating_envelope_profile_config(config, coherent_ms, noncoherent);
    let duration_s = (coherent_ms.saturating_mul(noncoherent).max(1) as f64) / 1000.0;
    let false_alarm_count = trial_seeds
        .iter()
        .enumerate()
        .filter(|(trial_index, seed)| {
            let scenario = SyntheticScenario {
                sample_rate_hz: profile_config.sampling_freq_hz,
                intermediate_freq_hz: profile_config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: **seed,
                satellites: Vec::new(),
                ephemerides: Vec::new(),
                id: format!("{scenario_id_prefix}_trial_{trial_index}"),
            };
            let scaled_frame = scaled_synthetic_acquisition_frame(
                &profile_config,
                &scenario,
                "synthetic targeted acquisition false-alarm trial",
            );
            let result =
                acquisition_result_for_target_signal(&profile_config, &scaled_frame, signal);
            matches!(result.hypothesis, crate::api::core::AcqHypothesis::Accepted)
        })
        .count();

    SyntheticAcquisitionFalseAlarmProfileMeasurement {
        trial_count: trial_seeds.len(),
        false_alarm_count,
        false_alarm_rate: probability(false_alarm_count, trial_seeds.len()),
    }
}

fn default_operating_envelope_glonass_frequency_channel(
    constellation: bijux_gnss_core::api::Constellation,
) -> Option<bijux_gnss_core::api::GlonassFrequencyChannel> {
    (constellation == bijux_gnss_core::api::Constellation::Glonass).then(|| {
        bijux_gnss_core::api::GlonassFrequencyChannel::new(-4)
            .expect("GLONASS operating-envelope channel -4 must be valid")
    })
}

fn default_operating_envelope_signal_seed(
    constellation: bijux_gnss_core::api::Constellation,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> SyntheticSignalParams {
    let sat = match constellation {
        bijux_gnss_core::api::Constellation::Gps => {
            let prn = if signal_band == SignalBand::L5 && signal_code == SignalCode::L5Q {
                24
            } else if signal_band == SignalBand::L5 {
                12
            } else {
                7
            };
            SatId { constellation, prn }
        }
        bijux_gnss_core::api::Constellation::Galileo => SatId { constellation, prn: 11 },
        bijux_gnss_core::api::Constellation::Beidou => SatId { constellation, prn: 11 },
        bijux_gnss_core::api::Constellation::Glonass => bijux_gnss_core::api::glonass_slot_sat(
            bijux_gnss_core::api::GlonassSlot::new(8)
                .expect("GLONASS operating-envelope slot 8 must be valid"),
        ),
        _ => SatId { constellation, prn: 7 },
    };
    let code_phase_chips = match (constellation, signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L1, SignalCode::Ca) => 300.0,
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, _) => 2_048.375,
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => 321.0,
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, _) => 2_048.375,
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        | (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => 321.375,
        (bijux_gnss_core::api::Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            147.25
        }
        _ => 256.0,
    };

    SyntheticSignalParams {
        sat,
        glonass_frequency_channel,
        signal_band,
        signal_code,
        doppler_hz: 0.0,
        code_phase_chips,
        carrier_phase_rad: 0.25,
        cn0_db_hz: 38.0,
        navigation_data: false.into(),
    }
}

fn default_operating_envelope_config_for_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
) -> ReceiverPipelineConfig {
    let mut config = ReceiverPipelineConfig::default();
    config.acquisition_doppler_search_hz = 1_500;
    config.channels = 4;
    config.tracking_budget_ms = 100.0;
    config.tracking_over_budget_action = "continue".to_string();

    match (sat.constellation, signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
            config.sampling_freq_hz = 4_092_000.0;
            config.intermediate_freq_hz = 0.0;
            config.code_freq_basis_hz = 1_023_000.0;
            config.code_length = 1_023;
            config.acquisition_doppler_step_hz = 250;
            config.acquisition_integration_ms = 1;
            config.acquisition_noncoherent = 1;
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5I)
        | (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        | (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            config.sampling_freq_hz = 10_230_000.0;
            config.intermediate_freq_hz = 0.0;
            config.code_freq_basis_hz = 10_230_000.0;
            config.code_length = 10_230;
            config.acquisition_doppler_step_hz = 250;
            config.acquisition_integration_ms = 1;
            config.acquisition_noncoherent = 1;
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
            config.sampling_freq_hz = 4_092_000.0;
            config.intermediate_freq_hz = 0.0;
            config.code_freq_basis_hz = 1_023_000.0;
            config.code_length = 4_092;
            config.acquisition_doppler_step_hz = 500;
            config.acquisition_integration_ms = 20;
            config.acquisition_noncoherent = 1;
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
            config.sampling_freq_hz = 4_092_000.0;
            config.intermediate_freq_hz = bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
                - bijux_gnss_core::api::BEIDOU_B1_CARRIER_HZ.value();
            config.code_freq_basis_hz = 2_046_000.0;
            config.code_length = 2_046;
            config.acquisition_doppler_step_hz = 250;
            config.acquisition_integration_ms = 1;
            config.acquisition_noncoherent = 1;
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            config.sampling_freq_hz = 4_092_000.0;
            config.intermediate_freq_hz = 0.0;
            config.code_freq_basis_hz = 2_046_000.0;
            config.code_length = 2_046;
            config.acquisition_doppler_step_hz = 500;
            config.acquisition_integration_ms = 1;
            config.acquisition_noncoherent = 1;
        }
        (bijux_gnss_core::api::Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            let channel = glonass_frequency_channel
                .expect("GLONASS operating-envelope signal requires channel");
            config.sampling_freq_hz = 2_044_000.0;
            config.intermediate_freq_hz = bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()
                - bijux_gnss_signal::api::glonass_l1_carrier_hz(channel).value();
            config.code_freq_basis_hz = 511_000.0;
            config.code_length = 511;
            config.acquisition_doppler_step_hz = 250;
            config.acquisition_integration_ms = 1;
            config.acquisition_noncoherent = 1;
            config.channels = 2;
        }
        _ => panic!(
            "unsupported operating-envelope acquisition signal {:?} {:?} {:?}",
            sat.constellation, signal_band, signal_code
        ),
    }

    config
}

fn default_operating_envelope_integration_profiles(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
) -> Vec<SyntheticAcquisitionIntegrationProfile> {
    let mut profiles = vec![SyntheticAcquisitionIntegrationProfile {
        coherent_ms: config.acquisition_integration_ms,
        noncoherent: config.acquisition_noncoherent,
    }];
    let comparison_profile =
        match (signal.sat.constellation, signal.signal_band, signal.signal_code) {
            (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
                SyntheticAcquisitionIntegrationProfile { coherent_ms: 4, noncoherent: 1 }
            }
            _ => SyntheticAcquisitionIntegrationProfile { coherent_ms: 5, noncoherent: 1 },
        };
    if !profiles.contains(&comparison_profile) {
        profiles.push(comparison_profile);
    }
    profiles
}

fn baseline_profile_doppler_offset_hz(
    _signal: &SyntheticSignalParams,
    acquisition_doppler_step_hz: i32,
) -> f64 {
    acquisition_doppler_step_hz.max(1) as f64 / 2.0
}

fn operating_envelope_baseline_integration_profile(
    case: &SyntheticAcquisitionOperatingEnvelopeSignalCase,
) -> SyntheticAcquisitionIntegrationProfile {
    case.integration_profiles.first().copied().unwrap_or(SyntheticAcquisitionIntegrationProfile {
        coherent_ms: case.config.acquisition_integration_ms,
        noncoherent: case.config.acquisition_noncoherent,
    })
}

fn operating_envelope_effective_integration_profiles(
    case: &SyntheticAcquisitionOperatingEnvelopeSignalCase,
) -> Vec<SyntheticAcquisitionIntegrationProfile> {
    if case.integration_profiles.is_empty() {
        vec![operating_envelope_baseline_integration_profile(case)]
    } else {
        case.integration_profiles.clone()
    }
}

fn operating_envelope_profile_config(
    config: &ReceiverPipelineConfig,
    coherent_ms: u32,
    noncoherent: u32,
) -> ReceiverPipelineConfig {
    let mut profile_config = config.clone();
    profile_config.acquisition_integration_ms = coherent_ms;
    profile_config.acquisition_noncoherent = noncoherent;
    profile_config
}

fn operating_envelope_variants(
    case: &SyntheticAcquisitionOperatingEnvelopeSignalCase,
    baseline_profile: SyntheticAcquisitionIntegrationProfile,
) -> Vec<SyntheticAcquisitionOperatingEnvelopeVariant> {
    let mut variants = Vec::new();

    for cn0_db_hz in &case.cn0_db_hz_points {
        let mut signal = case.signal.clone();
        signal.cn0_db_hz = *cn0_db_hz;
        variants.push(SyntheticAcquisitionOperatingEnvelopeVariant {
            axis: SyntheticAcquisitionOperatingEnvelopeAxis::Cn0DbHz,
            signal,
            coherent_ms: baseline_profile.coherent_ms,
            noncoherent: baseline_profile.noncoherent,
            receiver_clock_frequency_bias_hz: 0.0,
        });
    }
    for profile in operating_envelope_effective_integration_profiles(case) {
        variants.push(SyntheticAcquisitionOperatingEnvelopeVariant {
            axis: SyntheticAcquisitionOperatingEnvelopeAxis::IntegrationProfile,
            signal: case.signal.clone(),
            coherent_ms: profile.coherent_ms,
            noncoherent: profile.noncoherent,
            receiver_clock_frequency_bias_hz: 0.0,
        });
    }
    for doppler_hz in &case.doppler_hz_points {
        let mut signal = case.signal.clone();
        signal.doppler_hz = *doppler_hz;
        variants.push(SyntheticAcquisitionOperatingEnvelopeVariant {
            axis: SyntheticAcquisitionOperatingEnvelopeAxis::DopplerHz,
            signal,
            coherent_ms: baseline_profile.coherent_ms,
            noncoherent: baseline_profile.noncoherent,
            receiver_clock_frequency_bias_hz: 0.0,
        });
    }
    for receiver_clock_frequency_bias_hz in &case.receiver_clock_frequency_bias_hz_points {
        variants.push(SyntheticAcquisitionOperatingEnvelopeVariant {
            axis: SyntheticAcquisitionOperatingEnvelopeAxis::ReceiverClockFrequencyBiasHz,
            signal: case.signal.clone(),
            coherent_ms: baseline_profile.coherent_ms,
            noncoherent: baseline_profile.noncoherent,
            receiver_clock_frequency_bias_hz: *receiver_clock_frequency_bias_hz,
        });
    }
    for code_phase_chips in &case.code_phase_chips_points {
        let mut signal = case.signal.clone();
        signal.code_phase_chips = *code_phase_chips;
        variants.push(SyntheticAcquisitionOperatingEnvelopeVariant {
            axis: SyntheticAcquisitionOperatingEnvelopeAxis::CodePhaseChips,
            signal,
            coherent_ms: baseline_profile.coherent_ms,
            noncoherent: baseline_profile.noncoherent,
            receiver_clock_frequency_bias_hz: 0.0,
        });
    }

    variants
}

fn operating_envelope_point_case_id(
    scenario_id_prefix: &str,
    axis: SyntheticAcquisitionOperatingEnvelopeAxis,
    signal: &SyntheticSignalParams,
    coherent_ms: u32,
    noncoherent: u32,
    receiver_clock_frequency_bias_hz: f64,
) -> String {
    format!(
        "{scenario_id_prefix}_{}_{}_{}_{}_channel_{:+03}_prn_{}_coherent_{}ms_noncoherent_{}_cn0_{:03}_doppler_{:+05}_clock_bias_{:+05}_code_phase_{:05}",
        operating_envelope_axis_name(axis),
        format!("{:?}", signal.sat.constellation).to_lowercase(),
        format!("{:?}", signal.signal_band).to_lowercase(),
        format!("{:?}", signal.signal_code).to_lowercase(),
        signal
            .glonass_frequency_channel
            .map(|channel| channel.value())
            .unwrap_or_default(),
        signal.sat.prn,
        coherent_ms,
        noncoherent,
        (signal.cn0_db_hz * 10.0).round() as i32,
        signal.doppler_hz.round() as i32,
        receiver_clock_frequency_bias_hz.round() as i32,
        signal.code_phase_chips.round() as i32,
    )
}

fn operating_envelope_false_alarm_case_id(
    scenario_id_prefix: &str,
    signal: &SyntheticSignalParams,
    profile: SyntheticAcquisitionIntegrationProfile,
) -> String {
    format!(
        "{scenario_id_prefix}_false_alarm_{}_{}_{}_{}_coherent_{}ms_noncoherent_{}",
        format!("{:?}", signal.sat.constellation).to_lowercase(),
        format!("{:?}", signal.signal_band).to_lowercase(),
        format!("{:?}", signal.signal_code).to_lowercase(),
        signal.sat.prn,
        profile.coherent_ms,
        profile.noncoherent,
    )
}

fn operating_envelope_axis_name(axis: SyntheticAcquisitionOperatingEnvelopeAxis) -> &'static str {
    match axis {
        SyntheticAcquisitionOperatingEnvelopeAxis::Cn0DbHz => "cn0",
        SyntheticAcquisitionOperatingEnvelopeAxis::IntegrationProfile => "integration",
        SyntheticAcquisitionOperatingEnvelopeAxis::DopplerHz => "doppler",
        SyntheticAcquisitionOperatingEnvelopeAxis::ReceiverClockFrequencyBiasHz => "clock_bias",
        SyntheticAcquisitionOperatingEnvelopeAxis::CodePhaseChips => "code_phase",
    }
}

fn wrap_code_phase_chips(code_phase_chips: f64, code_length: f64) -> f64 {
    if code_length <= f64::EPSILON {
        code_phase_chips
    } else {
        code_phase_chips.rem_euclid(code_length)
    }
}

fn mean_peak_mean_ratio(values: impl Iterator<Item = f32>) -> f64 {
    let values = values.map(|value| value as f64).collect::<Vec<_>>();
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn probability(count: usize, total: usize) -> f64 {
    if total == 0 {
        0.0
    } else {
        count as f64 / total as f64
    }
}
