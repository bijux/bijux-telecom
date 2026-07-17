use super::*;

#[test]
fn tracking_lock_detector_thresholds_derive_from_spacing_and_dynamics() {
    let tracking_params = super::TrackingParams {
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        integration_ms: 1,
    };
    let high_resolution =
        super::tracking_lock_detector_thresholds(45.0, 0.001, 4.0, tracking_params, 0.0);
    let low_resolution =
        super::tracking_lock_detector_thresholds(45.0, 0.001, 1.0, tracking_params, 25_000.0);

    assert!(low_resolution.dll_lock > high_resolution.dll_lock);
    assert!(low_resolution.fll_lock_hz > high_resolution.fll_lock_hz);
    assert!(high_resolution.pll_hold_rad > high_resolution.pll_lock_rad);
}

#[test]
fn low_resolution_code_lock_retains_supported_tracking_when_carrier_is_stable() {
    assert!(super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, false));
    assert!(super::low_resolution_code_lock(1.0, 0.5, true, false, true, false, false));
    assert!(super::low_resolution_code_lock(2.0, 0.5, true, false, true, false, false));
}

#[test]
fn low_resolution_code_lock_requires_prompt_and_lock_safety_guards() {
    assert!(!super::low_resolution_code_lock(1.0, 0.5, false, true, true, false, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, false, false, false, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, true, false));
    assert!(!super::low_resolution_code_lock(1.0, 0.5, true, true, true, false, true));
    assert!(!super::low_resolution_code_lock(4.0, 0.5, true, true, true, false, false));
}

#[test]
fn low_resolution_false_lock_override_requires_consistent_tracking_evidence() {
    assert!(super::low_resolution_false_lock_override(super::LowResolutionFalseLockEvidence {
        samples_per_chip: 2.0,
        early_late_spacing_chips: 0.5,
        prompt_lock: true,
        prompt_power_supports_lock: true,
        fll_lock: true,
        doppler_consistent: true,
        pll_err_rad: 0.8,
        cycle_slip: false,
    }));
    assert!(!super::low_resolution_false_lock_override(super::LowResolutionFalseLockEvidence {
        samples_per_chip: 2.0,
        early_late_spacing_chips: 0.5,
        prompt_lock: true,
        prompt_power_supports_lock: false,
        fll_lock: true,
        doppler_consistent: true,
        pll_err_rad: 0.8,
        cycle_slip: false,
    }));
    assert!(!super::low_resolution_false_lock_override(super::LowResolutionFalseLockEvidence {
        samples_per_chip: 2.0,
        early_late_spacing_chips: 0.5,
        prompt_lock: true,
        prompt_power_supports_lock: true,
        fll_lock: true,
        doppler_consistent: false,
        pll_err_rad: 0.8,
        cycle_slip: false,
    }));
    assert!(!super::low_resolution_false_lock_override(super::LowResolutionFalseLockEvidence {
        samples_per_chip: 2.0,
        early_late_spacing_chips: 0.5,
        prompt_lock: true,
        prompt_power_supports_lock: true,
        fll_lock: true,
        doppler_consistent: true,
        pll_err_rad: 2.0,
        cycle_slip: false,
    }));
}

#[test]
fn clean_seeded_tracking_clears_false_lock_once_loops_converge() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: -750.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        0x710C_A000,
        0.012,
    );
    let code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples(&config, &frame, 211.25);
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(-750.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: bijux_gnss_core::api::Hertz(-750.0),
            code_phase_samples,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 52.0,
            score: 1.0,
            hypothesis: bijux_gnss_core::api::AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("clean_seeded_tracking".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }],
    );
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs
            .iter()
            .skip(4)
            .filter(|epoch| epoch.pll_lock && epoch.fll_lock)
            .all(|epoch| !epoch.anti_false_lock),
        "clean, converged tracking epochs must not remain marked as false lock: {epochs:?}"
    );
}

#[test]
fn low_rate_seeded_tracking_clears_low_resolution_false_lock_when_carrier_is_consistent() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.25,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: -1_000.0,
            code_phase_chips: 321.5,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            navigation_data: false.into(),
        },
        0x710C_A000,
        0.04,
    );
    let code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples(&config, &frame, 321.5);
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[bijux_gnss_core::api::AcqResult {
            sat,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: bijux_gnss_core::api::ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: bijux_gnss_core::api::Hertz(-1_000.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: bijux_gnss_core::api::Hertz(-1_000.0),
            code_phase_samples,
            peak: 1.0,
            second_peak: 0.1,
            mean: 0.01,
            peak_mean_ratio: 20.0,
            peak_second_ratio: 10.0,
            cn0_proxy: 52.0,
            score: 1.0,
            hypothesis: bijux_gnss_core::api::AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: Some("low_rate_seeded_tracking".to_string()),
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        }],
    );
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs.iter().any(|epoch| epoch.lock_state == "tracking"),
        "low-rate seeded tracking must reach tracking: {epochs:?}"
    );
    assert!(
        epochs
            .iter()
            .skip(4)
            .filter(|epoch| epoch.fll_lock && epoch.lock_state == "tracking")
            .all(|epoch| !epoch.anti_false_lock),
        "low-rate stable tracking must not retain false-lock flags: {epochs:?}"
    );
}

#[test]
fn correlate_epoch_honors_receiver_code_phase_seed_at_low_sample_rate() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 1,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 750.0,
            code_phase_chips: 200.25,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            navigation_data: false.into(),
        },
        0x330C_2000,
        0.04,
    );
    let refined_code_phase_samples =
        crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(&config, &frame, 200.25);
    let tracking = Tracking::new(config, ReceiverRuntime::default());
    let correlator = tracking.correlate_epoch(super::TrackingCorrelationRequest {
        frame: &frame,
        sat,
        carrier_hz: 750.0,
        carrier_phase_cycles: 0.0,
        code_rate_hz: 1_023_000.0,
        code_phase_samples: refined_code_phase_samples,
        early_late_spacing_chips: 0.5,
    });

    assert!(
        correlator.prompt.norm() > 10_000.0,
        "receiver-seeded code phase must produce a strong prompt correlation: {:?}",
        correlator.prompt
    );
    assert!(
        correlator.prompt.re.abs() > correlator.prompt.im.abs() * 100.0,
        "receiver-seeded code phase must align carrier phase near the real axis: {:?}",
        correlator.prompt
    );
}

#[test]
fn tracking_recovery_from_loss_of_lock() {
    let lost = Tracking::transition_state(1, ChannelState::Tracking, ChannelState::Lost);
    assert_eq!(lost, ChannelState::Lost);
    let pull_in = Tracking::transition_state(1, lost, ChannelState::PullIn);
    assert_eq!(pull_in, ChannelState::PullIn);
    let tracking = Tracking::transition_state(1, pull_in, ChannelState::Tracking);
    assert_eq!(tracking, ChannelState::Tracking);
}

#[test]
fn tracking_reset_after_gap() {
    let lost = Tracking::transition_state(2, ChannelState::Tracking, ChannelState::Lost);
    let reset = Tracking::transition_state(2, lost, ChannelState::Idle);
    assert_eq!(reset, ChannelState::Idle);
}

#[test]
fn ambiguous_hypothesis_is_degraded_for_tracking() {
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Accepted), "accepted");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Ambiguous), "degraded");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Rejected), "rejected");
    assert_eq!(super::acq_to_track_state(&AcqHypothesis::Deferred), "deferred");
}

#[test]
fn deterministic_transition_rule_handles_cycle_slip_first() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Tracking,
        lock: false,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: Some(super::LossOfLockCause::PhaseJump),
        unlocked_count: 1,
        degraded_epochs: 0,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "phase_jump");
    assert_eq!(decision.next_unlocked_count, 2);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_promotes_lock() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::PullIn,
        lock: true,
        ready_for_tracking: true,
        anti_false_lock: false,
        loss_of_lock_cause: None,
        unlocked_count: 2,
        degraded_epochs: 0,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Tracking);
    assert_eq!(decision.reason, "carrier_converged");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_degrades_tracking_during_short_fade() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Tracking,
        lock: false,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: None,
        unlocked_count: 1,
        degraded_epochs: 0,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "signal_fade");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 1);
}

#[test]
fn deterministic_transition_rule_reports_doppler_estimator_divergence() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Tracking,
        lock: true,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: None,
        unlocked_count: 0,
        degraded_epochs: 0,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: Some("doppler_estimator_divergence"),
    });

    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "doppler_estimator_divergence");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 1);
}

#[test]
fn deterministic_transition_rule_recovers_after_short_fade() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Degraded,
        lock: true,
        ready_for_tracking: true,
        anti_false_lock: false,
        loss_of_lock_cause: None,
        unlocked_count: 0,
        degraded_epochs: 4,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Tracking);
    assert_eq!(decision.reason, "fade_recovered");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_preserves_doppler_reason_while_degraded() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Degraded,
        lock: true,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: None,
        unlocked_count: 0,
        degraded_epochs: 4,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: Some("doppler_estimator_divergence"),
    });

    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "doppler_estimator_divergence");
    assert_eq!(decision.next_unlocked_count, 0);
    assert_eq!(decision.next_degraded_epochs, 5);
}

#[test]
fn deterministic_transition_rule_keeps_degraded_state_during_fade_cycle_slip() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Degraded,
        lock: false,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: Some(super::LossOfLockCause::PhaseJump),
        unlocked_count: 0,
        degraded_epochs: 2,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "phase_jump");
    assert_eq!(decision.next_unlocked_count, 1);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_marks_loss_after_fade_budget_exhaustion() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Degraded,
        lock: false,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: Some(super::LossOfLockCause::PromptPowerDrop),
        unlocked_count: 0,
        degraded_epochs: 100,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Lost);
    assert_eq!(decision.reason, "prompt_power_drop");
    assert_eq!(decision.next_unlocked_count, 1);
    assert_eq!(decision.next_degraded_epochs, 0);
}

#[test]
fn deterministic_transition_rule_grants_short_fade_grace_to_degraded_instability() {
    let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
        from_state: ChannelState::Degraded,
        lock: false,
        ready_for_tracking: false,
        anti_false_lock: false,
        loss_of_lock_cause: Some(super::LossOfLockCause::DiscriminatorInstability),
        unlocked_count: 0,
        degraded_epochs: super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS,
        short_fade_epoch_budget: 100,
        short_fade_relock_evidence: false,
        degraded_tracking_reason: None,
    });
    assert_eq!(decision.to_state, ChannelState::Degraded);
    assert_eq!(decision.reason, "signal_fade");
    assert_eq!(decision.next_degraded_epochs, super::DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS + 1);
}

#[test]
fn classify_loss_of_lock_cause_prioritizes_phase_jump() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.9), 3);

    assert_eq!(cause, Some(super::LossOfLockCause::PhaseJump));
}

#[test]
fn classify_loss_of_lock_cause_treats_weak_prompt_cycle_slips_as_prompt_power_drop() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, true, Some(0.1), 0);

    assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
}

#[test]
fn classify_loss_of_lock_cause_detects_prompt_power_drop() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.1), 0);

    assert_eq!(cause, Some(super::LossOfLockCause::PromptPowerDrop));
}

#[test]
fn classify_loss_of_lock_cause_detects_discriminator_instability() {
    let cause = super::classify_loss_of_lock_cause(ChannelState::Tracking, false, Some(0.8), 2);

    assert_eq!(cause, Some(super::LossOfLockCause::DiscriminatorInstability));
}

#[test]
fn update_discriminator_instability_epochs_requires_strong_prompt() {
    let epochs = super::update_discriminator_instability_epochs(
        1,
        ChannelState::Tracking,
        TrackingPhaseTransitionSource::None,
        Some(0.7),
        false,
        false,
        false,
        false,
    );
    assert_eq!(epochs, 2);

    let reset = super::update_discriminator_instability_epochs(
        epochs,
        ChannelState::Tracking,
        TrackingPhaseTransitionSource::None,
        Some(0.1),
        false,
        false,
        false,
        false,
    );
    assert_eq!(reset, 0);
}

#[test]
fn update_discriminator_instability_epochs_accumulates_when_either_carrier_lock_breaks() {
    let epochs = super::update_discriminator_instability_epochs(
        1,
        ChannelState::Tracking,
        TrackingPhaseTransitionSource::None,
        Some(0.7),
        true,
        false,
        false,
        false,
    );

    assert_eq!(epochs, 2);
}

#[test]
fn update_discriminator_instability_epochs_ignores_single_lock_break_for_secondary_code() {
    let epochs = super::update_discriminator_instability_epochs(
        1,
        ChannelState::Tracking,
        TrackingPhaseTransitionSource::SecondaryCode,
        Some(0.7),
        true,
        false,
        false,
        false,
    );

    assert_eq!(epochs, 0);
}
