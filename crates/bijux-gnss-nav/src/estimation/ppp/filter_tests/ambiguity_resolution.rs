use super::*;

fn append_ppp_state(filter: &mut PppFilter, identity: PppStateIdentity, value: f64) {
    filter.ekf.add_state(&ppp_state_label(&identity), value, 9.0);
    filter.state_identities.push(identity);
    filter.indices =
        ppp_indices_from_state_identities(&filter.state_identities).expect("PPP state layout");
}

fn ppp_ar_test_filter() -> PppFilter {
    PppFilter::new(PppConfig {
        ar_mode: PppArMode::PppArWideLane,
        ar_ratio_threshold: 3.0,
        ar_stability_epochs: 1,
        ar_max_sats: 8,
        ..PppConfig::default()
    })
}

fn ppp_narrow_lane_ar_test_filter(signal: SigId) -> PppFilter {
    let mut filter = PppFilter::new(PppConfig {
        ar_mode: PppArMode::PppArNarrowLane,
        ar_ratio_threshold: 3.0,
        ar_stability_epochs: 1,
        ar_max_sats: 8,
        ..PppConfig::default()
    });
    filter.set_biases(
        Box::new(ZeroBiases),
        Box::new(SignalPhaseBiases::from_biases([PhaseBias { sig: signal, bias_cycles: 0.0 }])),
    );
    filter
}

#[test]
fn ppp_wide_lane_integer_acceptance_requires_phase_bias_provenance() {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 22 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let observation = ppp_test_signal_satellite(sig.sat, sig.band, sig.code);
    let mut filter = ppp_ar_test_filter();
    filter.wl_state.insert(
        sig.sat,
        WlAmbiguity {
            float_cycles: 8.05,
            variance: 0.001,
            fixed: false,
            integer_cycles: None,
            ratio: None,
            phase_bias_provenance_complete: false,
            last_update_epoch: 1,
        },
    );

    let fixed_count =
        filter.try_fix_wide_lane(&ppp_test_epoch(vec![observation.clone()]), &[&observation]);

    assert_eq!(fixed_count, 0);
    assert_eq!(filter.ar_evidence.accepted_count, 0);
    assert!(!filter.ar_integer_ambiguities[0].accepted);
    assert!(filter.ar_integer_ambiguities[0]
        .validation_reasons
        .contains(&"missing_phase_bias_provenance".to_string()));
}

#[test]
fn ppp_wide_lane_integer_acceptance_records_validated_candidate_evidence() {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 23 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let observation = ppp_test_signal_satellite(sig.sat, sig.band, sig.code);
    let mut filter = ppp_ar_test_filter();
    filter.wl_state.insert(
        sig.sat,
        WlAmbiguity {
            float_cycles: 8.05,
            variance: 0.001,
            fixed: false,
            integer_cycles: None,
            ratio: None,
            phase_bias_provenance_complete: true,
            last_update_epoch: 1,
        },
    );

    let fixed_count =
        filter.try_fix_wide_lane(&ppp_test_epoch(vec![observation.clone()]), &[&observation]);

    assert_eq!(fixed_count, 1);
    assert_eq!(filter.ar_evidence.candidate_count, 1);
    assert_eq!(filter.ar_evidence.accepted_count, 1);
    assert!(filter.ar_evidence.phase_bias_provenance_complete);
    assert!(filter.ar_evidence.wide_lane_validated);
    assert!(filter.ar_integer_ambiguities[0].accepted);
    assert_eq!(filter.ar_integer_ambiguities[0].integer_cycles, 8);
}

#[test]
fn ppp_narrow_lane_integer_acceptance_requires_wide_lane_validation() {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 24 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let observation = ppp_test_signal_satellite(sig.sat, sig.band, sig.code);
    let mut filter = ppp_narrow_lane_ar_test_filter(sig);
    append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 12.05);
    let ambiguity_index = filter.indices.ambiguity[&sig];
    filter.ekf.p[(ambiguity_index, ambiguity_index)] = 0.001;

    let fixed_count =
        filter.try_fix_wide_lane(&ppp_test_epoch(vec![observation.clone()]), &[&observation]);

    assert_eq!(fixed_count, 0);
    assert_eq!(filter.ar_evidence.accepted_count, 0);
    assert!(!filter.ar_integer_ambiguities.iter().any(|candidate| candidate.kind
        == PppIntegerAmbiguityKind::NarrowLane
        && candidate.accepted));
    assert!(filter.ar_integer_ambiguities.iter().any(|candidate| {
        candidate.kind == PppIntegerAmbiguityKind::NarrowLane
            && candidate.validation_reasons.contains(&"missing_wide_lane_validation".to_string())
    }));
}

#[test]
fn ppp_narrow_lane_integer_acceptance_conditions_signal_ambiguity_state() {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 25 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    let observation = ppp_test_signal_satellite(sig.sat, sig.band, sig.code);
    let mut filter = ppp_narrow_lane_ar_test_filter(sig);
    append_ppp_state(&mut filter, PppStateIdentity::CarrierAmbiguity(sig), 12.05);
    let ambiguity_index = filter.indices.ambiguity[&sig];
    filter.ekf.p[(ambiguity_index, ambiguity_index)] = 0.001;
    filter.wl_state.insert(
        sig.sat,
        WlAmbiguity {
            float_cycles: 8.05,
            variance: 0.001,
            fixed: false,
            integer_cycles: None,
            ratio: None,
            phase_bias_provenance_complete: true,
            last_update_epoch: 1,
        },
    );

    let fixed_count =
        filter.try_fix_wide_lane(&ppp_test_epoch(vec![observation.clone()]), &[&observation]);

    assert_eq!(fixed_count, 1);
    assert_eq!(filter.ar_evidence.accepted_count, 2);
    assert!(filter.ar_evidence.wide_lane_validated);
    assert!(filter.ar_evidence.narrow_lane_validated);
    assert_eq!(filter.ekf.x[ambiguity_index], 12.0);
    assert!(filter.ar_integer_ambiguities.iter().any(|candidate| {
        candidate.kind == PppIntegerAmbiguityKind::NarrowLane && candidate.accepted
    }));
}
