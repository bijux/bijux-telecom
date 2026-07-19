use super::*;

#[test]
fn best_coherent_data_correlation_restores_unknown_symbol_flip_gain() {
    let per_period = vec![
        vec![Complex::new(1.0, 0.0), Complex::new(0.5, 0.0)],
        vec![Complex::new(-1.0, 0.0), Complex::new(0.5, 0.0)],
    ];
    let signs = coherent_data_sign_hypotheses(2, 20);

    let coherent = best_coherent_data_correlation(&per_period, &signs);

    assert!((coherent[0].norm() - 2.0).abs() <= f32::EPSILON);
    assert!((coherent[1].norm() - 1.0).abs() <= f32::EPSILON);
}

#[test]
fn best_coherent_secondary_code_phase_correlation_selects_global_phase() {
    let per_period = vec![
        vec![Complex::new(-3.0, 0.0)],
        vec![Complex::new(-2.0, 0.0)],
        vec![Complex::new(5.0, 0.0)],
        vec![Complex::new(-4.0, 0.0)],
    ];

    let accumulation = best_coherent_secondary_code_phase_correlation(&per_period, 2, &[1, -1, -1]);

    assert_eq!(accumulation.secondary_code_phase_periods, Some(1));
    assert_eq!(accumulation.per_noncoherent.len(), 2);
    assert!((accumulation.noncoherent_accumulator[0] - 14.0).abs() <= f32::EPSILON);
}

#[test]
fn coherent_correlation_with_signs_preserves_fixed_sign_sum_without_hypotheses() {
    let per_period = vec![
        vec![Complex::new(1.0, 0.0), Complex::new(0.25, -0.25)],
        vec![Complex::new(2.0, 0.0), Complex::new(0.75, 0.25)],
    ];

    let coherent = coherent_correlation_with_signs(&per_period, &[1, 1]);

    assert_eq!(coherent, vec![Complex::new(3.0, 0.0), Complex::new(1.0, 0.0)]);
}

#[test]
fn strategy_uses_data_sign_hypotheses_for_long_gps_l1_coherent_windows() {
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let strategy = acquisition_strategies_for_signal(sat, SignalBand::L1, SignalCode::Ca, None, 20)
        .expect("GPS L1 acquisition strategies")
        .into_iter()
        .next()
        .expect("GPS L1 acquisition strategy");

    assert!(strategy_uses_data_sign_hypotheses(&strategy, 20));
    assert!(!strategy_uses_data_sign_hypotheses(&strategy, 1));
}

#[test]
fn candidate_uses_data_sign_hypotheses_matches_strategy_provenance() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let strategies =
        acquisition_strategies_for_signal(sat, SignalBand::E5, SignalCode::E5b, None, 20)
            .expect("Galileo E5b acquisition strategies");
    let candidate = AcqResult {
        sat,
        signal_band: SignalBand::E5,
        signal_code: SignalCode::E5b,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(0.0),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(0.0),
        code_phase_samples: 0,
        peak: 10.0,
        second_peak: 5.0,
        mean: 2.0,
        peak_mean_ratio: 5.0,
        peak_second_ratio: 2.0,
        cn0_proxy: 50.0,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: None,
        evidence: vec![AcqEvidence {
            rank: 1,
            code_phase_samples: 0,
            doppler_hz: 0.0,
            doppler_rate_hz_per_s: 0.0,
            peak: 10.0,
            second_peak: 5.0,
            peak_mean_ratio: 5.0,
            peak_second_ratio: 2.0,
            mean: 2.0,
            component_provenance: Some(AcqComponentProvenance {
                combination_mode: AcqComponentCombinationMode::SingleComponent,
                components: vec![AcqComponentStatistic {
                    role: SignalComponentRole::Data,
                    peak: 10.0,
                    second_peak: 5.0,
                    mean: 2.0,
                    peak_mean_ratio: 5.0,
                    peak_second_ratio: 2.0,
                    secondary_code_phase_periods: None,
                }],
            }),
        }],
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    };

    assert!(candidate_uses_data_sign_hypotheses(&candidate, &strategies, 20));
    assert!(!candidate_uses_data_sign_hypotheses(&candidate, &strategies, 1));
}
