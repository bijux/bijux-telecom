use super::*;

fn galileo_e1_subcarrier_guard_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn galileo_e1_subcarrier_guard_for_seed(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    code_phase_chips: f64,
    seed_offset_chips: f64,
) -> super::SubcarrierAmbiguityGuard {
    let sample_count =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
        Seconds(1.0 / config.sampling_freq_hz),
        sample_galileo_e1_cboc(
            sat.prn,
            config.sampling_freq_hz,
            code_phase_chips,
            sample_count,
            0,
            1,
        )
        .expect("Galileo E1 CBOC samples")
        .into_iter()
        .map(|value| Complex::new(value, 0.0))
        .collect(),
    );
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        None,
    );
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let code_phase_samples = (code_phase_chips + seed_offset_chips) * samples_per_chip;
    tracking
        .tracking_epoch_correlation(
            &frame,
            0,
            frame.len(),
            0,
            &signal_model,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            code_phase_samples,
            config.early_late_spacing_chips,
        )
        .subcarrier_ambiguity_guard
        .expect("Galileo E1 requires subcarrier ambiguity guard")
}

#[test]
fn tracking_signal_model_uses_registry_metadata_for_gps_l5q() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5Q,
        None,
    );

    assert_eq!(signal_model.component_role, SignalComponentRole::Pilot);
    assert!(signal_model.secondary_code.is_some());
    assert_eq!(
        signal_model.phase_transition_source,
        super::TrackingPhaseTransitionSource::SecondaryCode
    );
    assert_eq!(
        signal_model.discriminator_family,
        super::TrackingDiscriminatorFamily::EarlyPromptLate
    );
    assert!((signal_model.nominal_carrier_hz() - 1_176_450_000.0).abs() <= f64::EPSILON);
    assert!(!signal_model.supports_navigation_bit_sign_recovery());
}

#[test]
fn tracked_signal_center_hz_rebases_non_l1_signals_into_receiver_if() {
    let signal = signal_spec_gps_l5_i();
    let center_hz = super::tracked_signal_center_hz(0.0, signal);

    assert!(
        (center_hz - (signal.carrier_hz.value() - GPS_L1_CA_CARRIER_HZ.value())).abs()
            <= f64::EPSILON
    );
}

#[test]
fn tracked_signal_doppler_hz_removes_signal_specific_center_offset() {
    let signal = signal_spec_gps_l5_i();
    let tracked_carrier_hz = super::tracked_signal_center_hz(0.0, signal) + 875.0;

    assert!(
        (super::tracked_signal_doppler_hz(0.0, tracked_carrier_hz, signal) - 875.0).abs()
            <= f64::EPSILON
    );
}

#[test]
fn normalize_acquisition_carrier_hz_rebases_doppler_only_l5_seed() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5I,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(180.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(180.0),
        code_phase_samples: 0,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_joint_component_tracking".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    let normalized = super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition);

    assert!(
        (normalized
            - (super::tracked_signal_center_hz(
                config.intermediate_freq_hz,
                signal_model.signal_spec
            ) + acquisition.doppler_hz.0))
            .abs()
            <= f64::EPSILON
    );
}

#[test]
fn normalize_acquisition_carrier_hz_keeps_absolute_wideband_carrier() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E5,
        SignalCode::E5b,
        None,
    );
    let absolute_carrier_hz =
        super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
            + 250.0;
    let acquisition = bijux_gnss_core::api::AcqResult {
        sat,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5b,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(250.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(absolute_carrier_hz),
        code_phase_samples: 0,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    assert_eq!(
        super::normalize_acquisition_carrier_hz(&config, &signal_model, &acquisition),
        absolute_carrier_hz
    );
}

#[test]
fn carrier_aided_code_rate_hz_uses_tracked_signal_center_and_code_rate() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let tracked_carrier_hz =
        super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
            + 875.0;
    let aided_code_rate_hz =
        super::carrier_aided_code_rate_hz(&config, &signal_model, tracked_carrier_hz);
    let expected = signal_model.code_rate_hz
        + (875.0 * signal_model.code_rate_hz / signal_model.signal_spec.carrier_hz.value());

    assert!((aided_code_rate_hz - expected).abs() <= 1.0e-9);
}

#[test]
fn carrier_aiding_reference_rejects_cross_band_center() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - signal_spec_gps_l5_i().carrier_hz.value(),
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    let error = super::carrier_aiding_reference(
        &config,
        &signal_model,
        GPS_L1_CA_CARRIER_HZ.value() + 875.0,
    )
    .expect_err("GPS L1 carrier must not aid a GPS L5 code loop");

    assert_eq!(error, "carrier_aiding_incompatible_signal_center");
}

#[test]
fn next_code_rate_reference_hz_holds_previous_value_until_carrier_lock_is_ready() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let previous_reference_hz = signal_model.code_rate_hz;
    let tracked_carrier_hz =
        super::tracked_signal_center_hz(config.intermediate_freq_hz, signal_model.signal_spec)
            + 875.0;

    assert_eq!(
        super::next_code_rate_reference_hz(
            &config,
            &signal_model,
            tracked_carrier_hz,
            previous_reference_hz,
            false,
        ),
        previous_reference_hz,
    );
}

#[test]
fn tracking_signal_model_resolves_joint_components_for_gps_l2c() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 511_500.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L2,
        SignalCode::L2C,
        None,
    );

    assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
    assert_eq!(signal_model.component_role, SignalComponentRole::Data);
    assert!(signal_model.pilot_component.is_some());
    assert!(signal_model.data_symbol_component.is_some());
    assert_eq!(
        signal_model.carrier_phase_transition_source(),
        super::TrackingPhaseTransitionSource::None
    );
    assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
}

#[test]
fn tracking_signal_model_resolves_joint_components_for_gps_l5i() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
    assert_eq!(signal_model.component_role, SignalComponentRole::Data);
    assert!(signal_model.pilot_component.is_some());
    assert!(signal_model.data_symbol_component.is_some());
    assert_eq!(
        signal_model.carrier_phase_transition_source(),
        super::TrackingPhaseTransitionSource::SecondaryCode
    );
    assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
}

#[test]
fn tracking_signal_model_resolves_joint_components_for_galileo_e5b() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E5,
        SignalCode::E5b,
        None,
    );

    assert_eq!(signal_model.aiding_mode, super::TrackingAidingMode::PilotCarrier);
    assert_eq!(signal_model.component_role, SignalComponentRole::Data);
    assert!(signal_model.pilot_component.is_some());
    assert!(signal_model.data_symbol_component.is_some());
    assert_eq!(
        signal_model.carrier_phase_transition_source(),
        super::TrackingPhaseTransitionSource::SecondaryCode
    );
    assert!(signal_model.supports_epoch_data_symbol_sign_recovery());
}

#[test]
fn select_carrier_prompt_prefers_pilot_when_it_is_strong_enough() {
    let (selected, source) = super::select_carrier_prompt(
        Complex::new(0.25, 0.05),
        Some(Complex::new(0.60, -0.10)),
        super::TrackingAidingMode::PilotCarrier,
        false,
    );

    assert_eq!(selected, Complex::new(0.60, -0.10));
    assert_eq!(source, super::CarrierPromptSource::Pilot);
}

#[test]
fn recover_epoch_navigation_bit_sign_reads_data_prompt_polarity_for_joint_tracking() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(0.75, 0.05)),
            Complex::new(0.80, 0.02),
            super::CarrierPromptSource::Primary,
            0.0,
            true,
        ),
        Some(1)
    );
    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(-0.75, 0.05)),
            Complex::new(0.80, 0.02),
            super::CarrierPromptSource::Primary,
            0.0,
            true,
        ),
        Some(-1)
    );
}

#[test]
fn select_carrier_prompt_keeps_dedicated_galileo_pilot_even_when_primary_is_stronger() {
    let (selected, source) = super::select_carrier_prompt(
        Complex::new(0.90, 0.05),
        Some(Complex::new(0.20, 0.70)),
        super::TrackingAidingMode::PilotCarrier,
        true,
    );

    assert_eq!(selected, Complex::new(0.20, 0.70));
    assert_eq!(source, super::CarrierPromptSource::Pilot);
}

#[test]
fn recover_epoch_navigation_bit_sign_compensates_half_cycle_pilot_flips() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(0.72, -0.08)),
            Complex::new(-0.83, 0.04),
            super::CarrierPromptSource::Primary,
            0.5,
            true,
        ),
        Some(1)
    );
    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(-0.72, 0.08)),
            Complex::new(-0.83, 0.04),
            super::CarrierPromptSource::Primary,
            0.5,
            true,
        ),
        Some(-1)
    );
}

#[test]
fn recover_epoch_navigation_bit_sign_projects_onto_carrier_reference() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E5,
        SignalCode::E5b,
        None,
    );

    let carrier_reference = Complex::new(0.50, 0.50);
    let positive_data = Complex::new(0.62, 0.58);
    let negative_data = -positive_data;

    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(positive_data),
            carrier_reference,
            super::CarrierPromptSource::Primary,
            0.0,
            true,
        ),
        Some(1)
    );
    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(negative_data),
            carrier_reference,
            super::CarrierPromptSource::Primary,
            0.0,
            true,
        ),
        Some(-1)
    );
}

#[test]
fn recover_epoch_navigation_bit_sign_rotates_galileo_pilot_axis_into_data_axis() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E5,
        SignalCode::E5b,
        None,
    );

    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(0.70, 0.02)),
            Complex::new(0.01, 0.80),
            super::CarrierPromptSource::Pilot,
            0.0,
            true,
        ),
        Some(1)
    );
    assert_eq!(
        super::recover_epoch_navigation_bit_sign(
            &signal_model,
            Some(Complex::new(-0.70, -0.02)),
            Complex::new(0.01, 0.80),
            super::CarrierPromptSource::Pilot,
            0.0,
            true,
        ),
        Some(-1)
    );
}

#[test]
fn tracking_signal_model_uses_registry_metadata_for_galileo_e1b() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        None,
    );

    assert_eq!(signal_model.component_role, SignalComponentRole::Data);
    assert!(signal_model.secondary_code.is_none());
    assert_eq!(
        signal_model.discriminator_family,
        super::TrackingDiscriminatorFamily::CbocEarlyPromptLate
    );
    assert_eq!(
        signal_model.phase_transition_source,
        super::TrackingPhaseTransitionSource::DataSymbol
    );
}

#[test]
fn galileo_e1_subcarrier_guard_accepts_main_lobe_prompt() {
    let config = galileo_e1_subcarrier_guard_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let code_phase_chips = 321.375;
    let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.0);

    assert!(!guard.detected(), "guard={guard:?}");
    assert!(
        guard.prompt_relative_power >= super::SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER,
        "guard={guard:?}",
    );
}

#[test]
fn galileo_e1_subcarrier_guard_rejects_half_chip_side_lobe_prompt() {
    let config = galileo_e1_subcarrier_guard_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let code_phase_chips = 321.375;
    let guard = galileo_e1_subcarrier_guard_for_seed(&config, sat, code_phase_chips, 0.5);

    assert!(guard.detected(), "guard={guard:?}");
    assert!(
        guard.strongest_alternate_power > guard.prompt_power,
        "guard should find stronger half-chip evidence than the side-lobe prompt: {guard:?}",
    );
}

#[test]
fn tracking_signal_model_uses_registry_metadata_for_beidou_b1i() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };

    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::B1,
        SignalCode::B1I,
        None,
    );

    assert_eq!(signal_model.component_role, SignalComponentRole::Data);
    assert!(signal_model.secondary_code.is_some());
    assert_eq!(
        signal_model.discriminator_family,
        super::TrackingDiscriminatorFamily::EarlyPromptLate
    );
    assert_eq!(
        signal_model.phase_transition_source,
        super::TrackingPhaseTransitionSource::SecondaryCode
    );
    assert!((signal_model.nominal_carrier_hz() - 1_561_098_000.0).abs() <= f64::EPSILON);
    assert!(!signal_model.supports_navigation_bit_sign_recovery());
}

#[test]
fn signal_tracking_params_use_narrow_spacing_for_gps_l1_ca_defaults() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        None,
    );

    let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

    assert_eq!(tracking_params.early_late_spacing_chips, 0.25);
}

#[test]
fn code_discriminator_mode_uses_double_delta_for_bpsk_tracking() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        None,
    );

    assert_eq!(
        super::code_discriminator_mode(&signal_model),
        super::CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate
    );
}

#[test]
fn code_discriminator_mode_keeps_subcarrier_tracking_unambiguous() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        None,
    );

    assert_eq!(
        super::code_discriminator_mode(&signal_model),
        super::CodeDiscriminatorMode::EarlyPromptLate
    );
}

#[test]
fn code_discriminator_mode_keeps_secondary_code_tracking_unambiguous() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::B1,
        SignalCode::B1I,
        None,
    );

    assert_eq!(
        super::code_discriminator_mode(&signal_model),
        super::CodeDiscriminatorMode::EarlyPromptLate
    );
}

#[test]
fn signal_tracking_params_keep_configured_spacing_for_secondary_code_tracking() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::B1,
        SignalCode::B1I,
        None,
    );

    let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

    assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
}

#[test]
fn signal_tracking_params_use_tighter_spacing_for_high_rate_signals() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 18 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

    assert_eq!(tracking_params.early_late_spacing_chips, 0.10);
}

#[test]
fn signal_tracking_params_preserve_per_band_spacing_overrides() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig {
        tracking_per_band: vec![BandTrackingSpec {
            band: SignalBand::L1,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            integration_ms: 1,
        }],
        ..crate::engine::receiver_config::ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        None,
    );

    let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

    assert_eq!(tracking_params.early_late_spacing_chips, 0.5);
}

#[test]
fn tracking_assumptions_follow_signal_metadata() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        None,
    );
    let tracking_params = super::resolve_signal_tracking_params(&config, &signal_model);

    let assumptions = super::tracking_assumptions(&signal_model, tracking_params);

    assert_eq!(assumptions.discriminator_family, "unambiguous_cboc_early_prompt_late");
    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
}
