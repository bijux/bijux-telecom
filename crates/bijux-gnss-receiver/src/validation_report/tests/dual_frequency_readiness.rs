use super::*;

#[test]
fn validation_report_marks_l1_l2_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            40,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, true),
            ],
        )],
        &[fixture_solution(40, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.l1_l2_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
    assert_eq!(report.ppp_readiness.maturity, AdvancedMaturity::NotReady);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}

#[test]
fn validation_report_marks_l1_l5_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            41,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L5, SignalCode::L5I, true, true),
            ],
        )],
        &[fixture_solution(41, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.l1_l5_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_e1_e5_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            44,
            vec![
                dual_frequency_satellite_for_constellation(
                    Constellation::Galileo,
                    SignalBand::E1,
                    SignalCode::E1B,
                    true,
                    true,
                ),
                dual_frequency_satellite_for_constellation(
                    Constellation::Galileo,
                    SignalBand::E5,
                    SignalCode::E5a,
                    true,
                    true,
                ),
            ],
        )],
        &[fixture_solution(44, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.e1_e5_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_b1_b2_dual_frequency_pairs_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            45,
            vec![
                dual_frequency_satellite_for_constellation(
                    Constellation::Beidou,
                    SignalBand::B1,
                    SignalCode::B1I,
                    true,
                    true,
                ),
                dual_frequency_satellite_for_constellation(
                    Constellation::Beidou,
                    SignalBand::B2,
                    SignalCode::B2I,
                    true,
                    true,
                ),
            ],
        )],
        &[fixture_solution(45, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 1);
    assert_eq!(report.dual_frequency_observations.b1_b2_pairs, 1);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
}

#[test]
fn validation_report_marks_incomplete_dual_frequency_pairs_not_ready_for_combinations() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            42,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, true),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, false, true),
            ],
        )],
        &[fixture_solution(42, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
    assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(!report.ppp_readiness.combinations_valid);
    assert!(!report.ppp_readiness.prerequisites_met);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.refusal_class, Some(AdvancedRefusalClass::UnsupportedModel));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}

#[test]
fn validation_report_accepts_code_ready_pairs_without_carrier_lock() {
    let report = build_validation_report(
        &[],
        &[dual_frequency_epoch(
            43,
            vec![
                dual_frequency_satellite(SignalBand::L1, SignalCode::Ca, true, false),
                dual_frequency_satellite(SignalBand::L2, SignalCode::Py, true, false),
            ],
        )],
        &[fixture_solution(43, 1.0, 0.5, 4)],
        &[],
        1.0,
        true,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("validation report");

    assert_eq!(report.dual_frequency_observations.complete_pairs, 0);
    assert_eq!(report.dual_frequency_observations.incomplete_pairs, 2);
    assert!(report.ppp_readiness.multi_freq_present);
    assert!(report.ppp_readiness.combinations_valid);
    assert_eq!(report.ppp_readiness.status, "not_ready");
    assert_eq!(report.ppp_readiness.status_reason.as_deref(), Some("missing_reference_frame"));
    assert_eq!(report.ppp_readiness.claim, AdvancedSolutionClaim::NotReady);
}
