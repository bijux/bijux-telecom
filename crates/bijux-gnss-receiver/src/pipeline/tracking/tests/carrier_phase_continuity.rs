use super::*;

#[test]
fn classify_prompt_phase_recovers_continuity_across_nav_bit_flip() {
    let decision = super::classify_prompt_phase(
        -0.39,
        Some(0.11),
        0.0,
        super::TrackingPhaseTransitionSource::DataSymbol,
    );

    assert!(decision.nav_bit_transition);
    assert!(!decision.cycle_slip);
    assert!((decision.aligned_phase_cycles - 0.11).abs() <= 0.02);
    assert!(decision.aligned_phase_delta_cycles.abs() <= 0.02);
    assert!((decision.nav_bit_phase_offset_cycles - 0.5).abs() <= f64::EPSILON);
}

#[test]
fn classify_prompt_phase_preserves_cycle_slip_for_non_nav_jump() {
    let decision = super::classify_prompt_phase(
        0.36,
        Some(0.0),
        0.0,
        super::TrackingPhaseTransitionSource::DataSymbol,
    );

    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
    assert!((decision.aligned_phase_delta_cycles - 0.36).abs() <= 1.0e-9);
    assert!((decision.nav_bit_phase_offset_cycles - 0.0).abs() <= f64::EPSILON);
}

#[test]
fn classify_prompt_phase_keeps_half_cycle_jump_as_slip_without_transition_metadata() {
    let decision = super::classify_prompt_phase(
        -0.39,
        Some(0.11),
        0.0,
        super::TrackingPhaseTransitionSource::None,
    );

    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
    assert!(decision.aligned_phase_delta_cycles.abs() > 0.35);
}

#[test]
fn secondary_code_phase_transition_waits_for_accepted_sync() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    let transition_source = super::carrier_phase_transition_source_for_prompt(&signal_model, None);
    let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

    assert_eq!(transition_source, super::TrackingPhaseTransitionSource::None);
    assert!(!decision.nav_bit_transition);
    assert!(decision.cycle_slip);
}

#[test]
fn secondary_code_phase_transition_accepts_synchronized_half_cycle_jump() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let sync = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 1.0,
        best_likelihood: 1.0,
        next_best_likelihood: 0.0,
        observed_periods: 20,
        accepted: true,
    };

    let transition_source =
        super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
    let decision = super::classify_prompt_phase(-0.39, Some(0.11), 0.0, transition_source);

    assert_eq!(transition_source, super::TrackingPhaseTransitionSource::SecondaryCode);
    assert!(decision.nav_bit_transition);
    assert!(!decision.cycle_slip);
}

#[test]
fn secondary_code_phase_transitions_preserve_continuity_across_multiple_flips() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let sync = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 1.0,
        best_likelihood: 1.0,
        next_best_likelihood: 0.0,
        observed_periods: 20,
        accepted: true,
    };
    let transition_source =
        super::carrier_phase_transition_source_for_prompt(&signal_model, Some(sync));
    let raw_phase_cycles = [0.10, -0.39, -0.38, 0.13, 0.14];
    let expected_transitions = [false, true, false, true, false];
    let mut previous_aligned_phase_cycles = None;
    let mut secondary_code_phase_offset_cycles = 0.0;

    for (raw_phase_cycles, expected_transition) in
        raw_phase_cycles.into_iter().zip(expected_transitions)
    {
        let decision = super::classify_prompt_phase(
            raw_phase_cycles,
            previous_aligned_phase_cycles,
            secondary_code_phase_offset_cycles,
            transition_source,
        );

        assert_eq!(decision.nav_bit_transition, expected_transition, "{decision:?}");
        assert!(!decision.cycle_slip, "{decision:?}");
        if previous_aligned_phase_cycles.is_some() {
            assert!(decision.aligned_phase_delta_cycles.abs() <= 0.03, "{decision:?}");
        }
        previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
        secondary_code_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
    }
}

#[test]
fn classify_prompt_phase_handles_real_synthetic_nav_bit_transitions() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 7 };
    let code_phase_chips = 321.0;
    let samples_per_epoch =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 70.0,
            navigation_data: true.into(),
        },
        0x7A91B17,
        0.05,
    );
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let mut previous_aligned_phase_cycles = None;
    let mut nav_bit_phase_offset_cycles = 0.0;
    let mut transition_epochs = Vec::new();

    for (epoch_index, epoch_samples) in frame.iq.chunks_exact(samples_per_epoch).enumerate() {
        let sample_index = epoch_index as u64 * samples_per_epoch as u64;
        let epoch_frame = SamplesFrame::new(
            SampleTime { sample_index, sample_rate_hz: config.sampling_freq_hz },
            Seconds(1.0 / config.sampling_freq_hz),
            epoch_samples.to_vec(),
        );
        let epoch_code_phase_chips = advance_code_phase_seconds(
            code_phase_chips,
            config.code_freq_basis_hz,
            sample_index as f64 / config.sampling_freq_hz,
            config.code_length,
        )
        .expect("valid epoch code phase");
        let epoch_code_phase_samples =
            crate::sim::synthetic::expected_acquisition_code_phase_samples_f64(
                &config,
                &epoch_frame,
                epoch_code_phase_chips,
            );
        let (epoch, _) = tracking.track_epoch(
            &epoch_frame,
            0,
            sat,
            0.0,
            0.0,
            config.code_freq_basis_hz,
            epoch_code_phase_samples,
            0.5,
        );
        let raw_phase_cycles =
            (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / (2.0 * std::f64::consts::PI);
        let decision = super::classify_prompt_phase(
            raw_phase_cycles,
            previous_aligned_phase_cycles,
            nav_bit_phase_offset_cycles,
            super::TrackingPhaseTransitionSource::DataSymbol,
        );
        if decision.nav_bit_transition {
            transition_epochs.push(epoch_index);
        }
        assert!(
            !decision.cycle_slip,
            "unexpected cycle slip at epoch {epoch_index}: raw_phase_cycles={raw_phase_cycles}"
        );
        previous_aligned_phase_cycles = Some(decision.aligned_phase_cycles);
        nav_bit_phase_offset_cycles = decision.nav_bit_phase_offset_cycles;
    }

    assert_eq!(transition_epochs, vec![20, 40]);
}
