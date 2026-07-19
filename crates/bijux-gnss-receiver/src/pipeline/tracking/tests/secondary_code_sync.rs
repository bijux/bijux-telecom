use super::*;

fn gps_l5q_secondary_code_component() -> super::TrackingComponentModel {
    super::TrackingComponentModel {
        role: SignalComponentRole::Pilot,
        code_length: 10_230,
        phase_transition_source: super::TrackingPhaseTransitionSource::SecondaryCode,
        local_code_model: super::TrackingComponentLocalCodeModel::Local(
            LocalCodeModel::gps_l5_q(18).expect("GPS L5Q local code"),
        ),
    }
}

fn secondary_code_prompt_history(
    component: &super::TrackingComponentModel,
    phase_periods: usize,
    observed_periods: usize,
    prompt: Complex<f32>,
) -> Vec<super::SecondaryCodePromptSample> {
    (0..observed_periods)
        .map(|period_offset| {
            let primary_code_period_index = phase_periods + period_offset;
            assert!(component.secondary_code_symbol(primary_code_period_index).is_some());
            super::SecondaryCodePromptSample { primary_code_period_index, prompt }
        })
        .collect()
}

#[test]
fn secondary_code_sync_accepts_best_likelihood_phase() {
    let component = gps_l5q_secondary_code_component();
    let history = secondary_code_prompt_history(&component, 7, 20, Complex::new(2.0, 0.25));

    let sync = super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

    assert!(sync.accepted, "sync result should be accepted: {sync:?}");
    assert_eq!(sync.phase_periods, 7);
    assert_eq!(sync.observed_periods, 20);
    assert!(sync.best_likelihood > sync.next_best_likelihood);
    assert!(sync.confidence >= super::SECONDARY_CODE_SYNC_MIN_CONFIDENCE);
}

#[test]
fn secondary_code_sync_scores_incorrect_phase_lower() {
    let component = gps_l5q_secondary_code_component();
    let history = secondary_code_prompt_history(&component, 11, 20, Complex::new(1.5, -0.5));

    let correct = super::secondary_code_phase_score(&component, &history, 11);
    let incorrect = super::secondary_code_phase_score(&component, &history, 12);

    assert!(correct.likelihood > incorrect.likelihood);
}

#[test]
fn secondary_code_sync_waits_for_enough_observations() {
    let component = gps_l5q_secondary_code_component();
    let history = secondary_code_prompt_history(&component, 3, 3, Complex::new(1.0, 0.0));

    assert!(super::secondary_code_sync_from_prompt_history(&component, &history).is_none());
}

#[test]
fn secondary_code_sync_rejects_zero_energy_prompt_history() {
    let component = gps_l5q_secondary_code_component();
    let history = secondary_code_prompt_history(&component, 5, 20, Complex::new(0.0, 0.0));

    let sync = super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

    assert!(!sync.accepted);
    assert_eq!(sync.confidence, 0.0);
}

#[test]
fn secondary_code_sync_updates_from_pilot_carrier_prompt_history() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let mut state = empty_loop_state();

    for period_offset in 0..20 {
        super::update_secondary_code_synchronization(
            &signal_model,
            &mut state,
            7 + period_offset,
            Complex::new(1.0, 0.125),
        );
    }

    let sync = state.secondary_code_sync.expect("secondary code sync");
    assert!(sync.accepted, "pilot prompt sync should be accepted: {sync:?}");
    assert_eq!(sync.phase_periods, 7);
    assert_eq!(sync.observed_periods, 20);
    assert_eq!(state.secondary_code_prompt_history.len(), 20);
}

#[test]
fn secondary_code_sync_accepts_galileo_e5_pilot_phase() {
    let config = ReceiverPipelineConfig::default();
    for (signal_code, phase_periods) in [(SignalCode::E5a, 37), (SignalCode::E5b, 61)] {
        let signal_model = super::TrackingSignalModel::for_sat_signal_band(
            &config,
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            signal_code,
            None,
        );
        let component = signal_model.carrier_component();
        let history =
            secondary_code_prompt_history(&component, phase_periods, 100, Complex::new(1.0, 0.2));

        let sync =
            super::secondary_code_sync_from_prompt_history(&component, &history).expect("sync");

        assert!(sync.accepted, "Galileo {signal_code:?} sync should be accepted: {sync:?}");
        assert_eq!(sync.phase_periods, phase_periods);
        assert!(sync.best_likelihood > sync.next_best_likelihood);
    }
}

#[test]
fn secondary_code_sync_ignores_signals_without_secondary_code() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat(
        &config,
        SatId { constellation: Constellation::Gps, prn: 1 },
    );
    let mut state = empty_loop_state();

    let sync = super::update_secondary_code_synchronization(
        &signal_model,
        &mut state,
        0,
        Complex::new(1.0, 0.0),
    );

    assert!(sync.is_none());
    assert!(state.secondary_code_sync.is_none());
    assert!(state.secondary_code_prompt_history.is_empty());
}

#[test]
fn secondary_code_sync_provenance_reports_accepted_phase() {
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
        confidence: 0.5,
        best_likelihood: 1.0,
        next_best_likelihood: 0.5,
        observed_periods: 20,
        accepted: true,
    };

    let provenance = super::secondary_code_sync_provenance(&signal_model, Some(sync))
        .expect("secondary-code provenance");

    assert!(provenance.contains("secondary_code_sync=accepted"));
    assert!(provenance.contains("secondary_code_phase_periods=7"));
    assert!(provenance.contains("secondary_code_sync_confidence=0.500000"));
    assert!(provenance.contains("secondary_code_observed_periods=20"));
}

#[test]
fn secondary_code_sync_provenance_reports_insufficient_evidence() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );

    let provenance =
        super::secondary_code_sync_provenance(&signal_model, None).expect("provenance");

    assert_eq!(provenance, " secondary_code_sync=insufficient");
}

#[test]
fn secondary_code_sync_keeps_accepted_phase_over_weaker_candidate() {
    let accepted = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 0.25,
        best_likelihood: 1.0,
        next_best_likelihood: 0.75,
        observed_periods: 20,
        accepted: true,
    };
    let rejected = super::SecondaryCodeSyncResult {
        phase_periods: 3,
        confidence: 0.01,
        best_likelihood: 0.6,
        next_best_likelihood: 0.594,
        observed_periods: 20,
        accepted: false,
    };

    let selected = super::select_secondary_code_synchronization(Some(accepted), Some(rejected))
        .expect("selected sync");

    assert_eq!(selected, accepted);
}

#[test]
fn secondary_code_sync_replaces_rejected_phase_with_accepted_candidate() {
    let rejected = super::SecondaryCodeSyncResult {
        phase_periods: 3,
        confidence: 0.01,
        best_likelihood: 0.6,
        next_best_likelihood: 0.594,
        observed_periods: 20,
        accepted: false,
    };
    let accepted = super::SecondaryCodeSyncResult {
        phase_periods: 7,
        confidence: 0.25,
        best_likelihood: 1.0,
        next_best_likelihood: 0.75,
        observed_periods: 20,
        accepted: true,
    };

    let selected = super::select_secondary_code_synchronization(Some(rejected), Some(accepted))
        .expect("selected sync");

    assert_eq!(selected, accepted);
}

#[test]
fn prompt_center_period_index_follows_code_phase_across_epoch_boundary() {
    assert_eq!(super::prompt_center_primary_code_period_index(0, 100.0, 1.0, 10_230, 10_230), 0);
    assert_eq!(super::prompt_center_primary_code_period_index(0, 8_182.0, 1.0, 10_230, 10_230), 1);
}
