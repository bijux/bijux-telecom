use super::*;

#[test]
fn ppp_filter_aligns_dynamic_state_identities_with_indices() {
    let gps = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 7 });
    let galileo = ppp_test_satellite(SatId { constellation: Constellation::Galileo, prn: 11 });
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });

    filter.ensure_states(&[&gps, &galileo], &PppPhaseBreaks::default());

    assert_eq!(filter.ekf.x.len(), filter.state_identities.len());
    assert_eq!(filter.ekf.labels.len(), filter.state_identities.len());
    for (constellation, index) in &filter.indices.isb {
        assert_eq!(
            filter.state_identities[*index],
            super::PppStateIdentity::InterSystemBias(*constellation)
        );
        assert_eq!(
            filter.ekf.labels[*index],
            super::ppp_state_label(&filter.state_identities[*index])
        );
    }
    for (sat, index) in &filter.indices.iono {
        assert_eq!(filter.state_identities[*index], super::PppStateIdentity::SlantIonosphere(*sat));
    }
    for (sig, index) in &filter.indices.ambiguity {
        assert_eq!(
            filter.state_identities[*index],
            super::PppStateIdentity::CarrierAmbiguity(*sig)
        );
    }
}

#[test]
fn ppp_measurement_observations_keep_direct_signals_for_uncombined_mode() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ppp_test_signal_satellite(sat, SignalBand::L1, SignalCode::Ca);
    let l2 = ppp_test_signal_satellite(sat, SignalBand::L2, SignalCode::Py);
    let obs = ppp_test_epoch(vec![l2.clone(), l1.clone()]);

    let direct = super::ppp_measurement_observations(&obs, false);
    let iono_free = super::ppp_measurement_observations(&obs, true);

    assert_eq!(direct.len(), 2);
    assert_eq!(direct[0].signal_id, l1.signal_id);
    assert_eq!(direct[1].signal_id, l2.signal_id);
    assert_eq!(iono_free.len(), 1);
    assert_eq!(iono_free[0].signal_id.sat, sat);
}

#[test]
fn ppp_code_sigma_combines_observation_and_declared_product_components() {
    let sat = ppp_test_signal_satellite(
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        SignalCode::Ca,
    );
    let state = GpsSatState {
        x_m: 0.0,
        y_m: 0.0,
        z_m: 0.0,
        vx_mps: 0.0,
        vy_mps: 0.0,
        vz_mps: 0.0,
        clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
        uncertainty: SatelliteStateUncertainty {
            orbit_sigma_m: Some(3.0),
            clock_sigma_s: Some(2.0 / SPEED_OF_LIGHT_MPS),
            orbit_source: SatelliteOrbitUncertaintySource::Sp3Accuracy,
            clock_source: SatelliteClockUncertaintySource::ClkSigma,
            health_status: SatelliteHealthStatus::Healthy,
            health_source: SatelliteHealthSource::Sp3Flags,
        },
    };
    let config = PppConfig {
        measurement_noise: PppMeasurementNoise {
            code_floor_m: 0.3,
            phase_floor_cycles: 0.01,
            orbit_sigma_scale: 1.0,
            clock_sigma_scale: 1.0,
            troposphere_residual_m: 0.25,
            antenna_residual_m: 0.1,
        },
        ..PppConfig::default()
    };

    let common_sigma_m = ppp_common_range_sigma_m(&state, 2.0, &config);
    let sigma_m = ppp_code_sigma_m(&sat, common_sigma_m, &config);
    let expected_common_sigma_m =
        (3.0_f64.powi(2) + 2.0_f64.powi(2) + 0.5_f64.powi(2) + 0.1_f64.powi(2)).sqrt();
    let expected_sigma_m = (sat.pseudorange_var_m2 + expected_common_sigma_m.powi(2)).sqrt();

    assert!((common_sigma_m - expected_common_sigma_m).abs() < 1.0e-12);
    assert!((sigma_m - expected_sigma_m).abs() < 1.0e-12);
}

#[test]
fn ppp_phase_sigma_maps_range_components_to_cycles() {
    let mut sat = ppp_test_signal_satellite(
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L2,
        SignalCode::Py,
    );
    sat.carrier_phase_var_cycles2 = 0.04;
    let wavelength_m = signal_wavelength_m(sat.metadata.signal).0;
    let config = PppConfig {
        measurement_noise: PppMeasurementNoise {
            phase_floor_cycles: 0.01,
            ..PppMeasurementNoise::default()
        },
        ..PppConfig::default()
    };

    let sigma_cycles = ppp_phase_sigma_cycles(&sat, wavelength_m, 1.5, &config);
    let expected_sigma_cycles = (0.04_f64 + (1.5 / wavelength_m).powi(2)).sqrt();

    assert!((sigma_cycles - expected_sigma_cycles).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_creates_uncombined_dual_frequency_states() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ppp_test_signal_satellite(sat, SignalBand::L1, SignalCode::Ca);
    let l2 = ppp_test_signal_satellite(sat, SignalBand::L2, SignalCode::Py);
    let mut filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        enable_iono_state: true,
        ..PppConfig::default()
    });

    filter.ensure_states(&[&l1, &l2], &PppPhaseBreaks::default());

    assert_eq!(filter.indices.iono.len(), 1);
    assert!(filter.indices.iono.contains_key(&sat));
    assert_eq!(filter.indices.ambiguity.len(), 2);
    assert!(filter.indices.ambiguity.contains_key(&l1.signal_id));
    assert!(filter.indices.ambiguity.contains_key(&l2.signal_id));
    assert_eq!(
        filter.state_identities[filter.indices.iono[&sat]],
        super::PppStateIdentity::SlantIonosphere(sat)
    );
    assert_eq!(
        filter.state_identities[filter.indices.ambiguity[&l1.signal_id]],
        super::PppStateIdentity::CarrierAmbiguity(l1.signal_id)
    );
    assert_eq!(
        filter.state_identities[filter.indices.ambiguity[&l2.signal_id]],
        super::PppStateIdentity::CarrierAmbiguity(l2.signal_id)
    );
}

#[test]
fn ppp_solution_reports_uncombined_state_support() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ppp_test_signal_satellite(sat, SignalBand::L1, SignalCode::Ca);
    let l2 = ppp_test_signal_satellite(sat, SignalBand::L2, SignalCode::Py);
    let mut filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        enable_iono_state: true,
        ..PppConfig::default()
    });
    filter.ensure_states(&[&l1, &l2], &PppPhaseBreaks::default());

    let solution = filter.solution_epoch(
        5,
        12.0,
        Vec::new(),
        0,
        ppp_stochastic_evidence_from_config(&filter.config),
    );

    assert_eq!(solution.constellation_clock_state_count, 1);
    assert_eq!(solution.slant_ionosphere_state_count, 1);
    assert_eq!(solution.carrier_ambiguity_state_count, 2);
    assert!(solution.stochastic_evidence.process_covariance_supported);
    assert!(solution.stochastic_evidence.atmosphere_residual_supported);
    assert!(solution.stochastic_evidence.antenna_residual_supported);
}

#[test]
fn ppp_filter_compacts_removed_satellite_states_without_covariance_index_corruption() {
    let stale = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 7 });
    let retained = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 11 });
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&stale, &retained], &PppPhaseBreaks::default());
    let retained_ambiguity_index = filter.indices.ambiguity[&retained.signal_id];
    let retained_iono_index = filter.indices.iono[&retained.signal_id.sat];
    filter.ekf.x[retained_ambiguity_index] = 42.0;
    filter.ekf.p[(retained_ambiguity_index, retained_ambiguity_index)] = 123.0;
    filter.ekf.p[(retained_iono_index, retained_ambiguity_index)] = 7.5;
    filter.ekf.p[(retained_ambiguity_index, retained_iono_index)] = 7.5;
    let original_state_count = filter.ekf.x.len();

    let mut removed = BTreeSet::new();
    removed.insert(super::PppStateIdentity::SlantIonosphere(stale.signal_id.sat));
    removed.insert(super::PppStateIdentity::CarrierAmbiguity(stale.signal_id));
    let removed_count = filter.remove_state_identities(&removed);

    assert_eq!(removed_count, 2);
    assert_eq!(filter.ekf.x.len(), original_state_count - 2);
    assert!(!filter.indices.iono.contains_key(&stale.signal_id.sat));
    assert!(!filter.indices.ambiguity.contains_key(&stale.signal_id));
    let compacted_ambiguity_index = filter.indices.ambiguity[&retained.signal_id];
    let compacted_iono_index = filter.indices.iono[&retained.signal_id.sat];
    assert_eq!(filter.ekf.x[compacted_ambiguity_index], 42.0);
    assert_eq!(filter.ekf.p[(compacted_ambiguity_index, compacted_ambiguity_index)], 123.0);
    assert!((filter.ekf.p[(compacted_iono_index, compacted_ambiguity_index)] - 7.5).abs() < 1e-9);
    assert_eq!(
        filter.state_identities[compacted_ambiguity_index],
        super::PppStateIdentity::CarrierAmbiguity(retained.signal_id)
    );
    assert_eq!(filter.ekf.labels.len(), filter.ekf.x.len());
}

#[test]
fn ppp_filter_does_not_create_ambiguity_for_slipped_phase() {
    let mut slipped = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 17 });
    slipped.lock_flags.cycle_slip = true;
    let mut phase_breaks = PppPhaseBreaks::default();
    phase_breaks.signals.insert(slipped.signal_id);
    phase_breaks.satellites.insert(slipped.signal_id.sat);
    let mut filter = PppFilter::new(PppConfig::default());

    filter.ensure_states(&[&slipped], &phase_breaks);

    assert!(!filter.indices.ambiguity.contains_key(&slipped.signal_id));
    assert!(!filter.state_identities.iter().any(|identity| {
        *identity == super::PppStateIdentity::CarrierAmbiguity(slipped.signal_id)
    }));
}

#[test]
fn ppp_filter_resets_phase_state_on_carrier_discontinuity() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 18 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig::default());
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    filter.phase_windup.insert(sig.sat, PhaseWindupState { previous_cycles: Some(0.25) });
    filter.wl_state.insert(
        sig.sat,
        WlAmbiguity {
            float_cycles: 8.0,
            variance: 0.2,
            fixed: true,
            integer_cycles: Some(8),
            ratio: Some(12.0),
            phase_bias_provenance_complete: true,
            last_update_epoch: 2,
        },
    );
    let mut signals = BTreeSet::new();
    signals.insert(sig);
    let mut satellites = BTreeSet::new();
    satellites.insert(sig.sat);

    let removed = filter.reset_phase_continuity(9, &signals, &satellites, "cycle_slip");

    assert_eq!(removed, 1);
    assert!(!filter.indices.ambiguity.contains_key(&sig));
    assert!(!filter.phase_windup.contains_key(&sig.sat));
    assert!(!filter.wl_state.contains_key(&sig.sat));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::CarrierDiscontinuity
            && event.epoch_idx == Some(9)
            && event.signal == Some(sig)
            && event.removed_states == vec![super::PppStateIdentity::CarrierAmbiguity(sig)]
    }));
}

#[test]
fn ppp_filter_prunes_stale_satellite_lifecycle_state() {
    let stale = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 19 });
    let retained = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 20 });
    let mut filter = PppFilter::new(PppConfig {
        enable_iono_state: true,
        prune_after_epochs: 1,
        ..PppConfig::default()
    });
    filter.ensure_states(&[&stale, &retained], &PppPhaseBreaks::default());
    filter.last_seen_iono.insert(stale.signal_id.sat, 1);
    filter.last_seen_iono.insert(retained.signal_id.sat, 5);
    filter.last_seen_amb.insert(stale.signal_id, 1);
    filter.last_seen_amb.insert(retained.signal_id, 5);
    filter
        .phase_windup
        .insert(stale.signal_id.sat, PhaseWindupState { previous_cycles: Some(0.4) });
    filter.wl_state.insert(
        stale.signal_id.sat,
        WlAmbiguity {
            float_cycles: 2.0,
            variance: 0.1,
            fixed: false,
            integer_cycles: None,
            ratio: None,
            phase_bias_provenance_complete: false,
            last_update_epoch: 1,
        },
    );
    filter.residual_history.insert(stale.signal_id, vec![4.0]);

    filter.prune_states(5);

    assert!(!filter.indices.iono.contains_key(&stale.signal_id.sat));
    assert!(!filter.indices.ambiguity.contains_key(&stale.signal_id));
    assert!(filter.indices.iono.contains_key(&retained.signal_id.sat));
    assert!(filter.indices.ambiguity.contains_key(&retained.signal_id));
    assert!(!filter.phase_windup.contains_key(&stale.signal_id.sat));
    assert!(!filter.wl_state.contains_key(&stale.signal_id.sat));
    assert!(!filter.residual_history.contains_key(&stale.signal_id));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::SatelliteStatePruned
            && event.epoch_idx == Some(5)
            && event.sat == Some(stale.signal_id.sat)
            && event
                .removed_states
                .contains(&super::PppStateIdentity::SlantIonosphere(stale.signal_id.sat))
            && event
                .removed_states
                .contains(&super::PppStateIdentity::CarrierAmbiguity(stale.signal_id))
    }));
}

#[test]
fn ppp_filter_resets_satellite_state_when_product_support_changes() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 21 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    let ambiguity_index = filter.indices.ambiguity[&sig];
    filter.ekf.x[ambiguity_index] = 12.0;
    filter.last_seen_iono.insert(sig.sat, 3);
    filter.last_seen_amb.insert(sig, 3);
    filter
        .product_support
        .insert(sig.sat, PppProductSupport { precise_orbit: true, precise_clock: true });

    let recreated = filter.handle_product_support(
        4,
        sig.sat,
        PppProductSupport { precise_orbit: false, precise_clock: true },
        &PppPhaseBreaks::default(),
        &observation,
    );
    if recreated {
        filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    }

    assert!(recreated);
    assert!(filter.indices.iono.contains_key(&sig.sat));
    assert!(filter.indices.ambiguity.contains_key(&sig));
    let new_ambiguity_index = filter.indices.ambiguity[&sig];
    assert_eq!(filter.ekf.x[new_ambiguity_index], 0.0);
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::ProductSupportChanged
            && event.epoch_idx == Some(4)
            && event.sat == Some(sig.sat)
            && event.removed_states.contains(&super::PppStateIdentity::SlantIonosphere(sig.sat))
            && event.removed_states.contains(&super::PppStateIdentity::CarrierAmbiguity(sig))
    }));
}
