use super::*;

fn precise_product_discontinuity(
    sat: SatId,
    surface: PreciseProductSurface,
    kind: PreciseProductDiscontinuityKind,
) -> PreciseProductDiscontinuity {
    PreciseProductDiscontinuity { sat, t_s: 10.0, surface, kind }
}

#[test]
fn ppp_precise_product_policy_bridges_missing_support_without_state_reset() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 22 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    let ambiguity_index = filter.indices.ambiguity[&sig];
    let ionosphere_index = filter.indices.iono[&sig.sat];

    let usable = filter.apply_precise_product_policy(
        5,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::MissingSatellite,
        )],
    );

    assert!(usable);
    assert_eq!(filter.indices.ambiguity.get(&sig), Some(&ambiguity_index));
    assert_eq!(filter.indices.iono.get(&sig.sat), Some(&ionosphere_index));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason
                == "precise_product_orbit_missing_satellite_action_bridge_with_broadcast"
            && event.removed_states.is_empty()
    }));
}

#[test]
fn ppp_precise_product_policy_inflates_satellite_state_covariance() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 23 });
    let sig = observation.signal_id;
    let mut config = PppConfig { enable_iono_state: true, ..PppConfig::default() };
    config.precise_product_policy.orbit_gap_action = PppPreciseProductAction::InflateSatelliteState;
    config.precise_product_policy.satellite_state_inflation = 12.0;
    let mut filter = PppFilter::new(config);
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    let ambiguity_index = filter.indices.ambiguity[&sig];
    let ionosphere_index = filter.indices.iono[&sig.sat];
    let ambiguity_variance = filter.ekf.p[(ambiguity_index, ambiguity_index)];
    let ionosphere_variance = filter.ekf.p[(ionosphere_index, ionosphere_index)];

    let usable = filter.apply_precise_product_policy(
        6,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::OrbitGap,
        )],
    );

    assert!(usable);
    assert_eq!(filter.indices.ambiguity.get(&sig), Some(&ambiguity_index));
    assert_eq!(filter.indices.iono.get(&sig.sat), Some(&ionosphere_index));
    assert!(
        (filter.ekf.p[(ambiguity_index, ambiguity_index)] - ambiguity_variance * 12.0).abs()
            < 1.0e-9
    );
    assert!(
        (filter.ekf.p[(ionosphere_index, ionosphere_index)] - ionosphere_variance * 12.0).abs()
            < 1.0e-9
    );
}

#[test]
fn ppp_precise_product_policy_resets_satellite_states_on_clock_jump() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 24 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    filter.last_seen_iono.insert(sig.sat, 4);
    filter.last_seen_amb.insert(sig, 4);
    filter.residual_history.insert(sig, vec![3.0]);

    let usable = filter.apply_precise_product_policy(
        7,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Clock,
            PreciseProductDiscontinuityKind::ClockJump,
        )],
    );

    assert!(usable);
    assert!(!filter.indices.ambiguity.contains_key(&sig));
    assert!(!filter.indices.iono.contains_key(&sig.sat));
    assert!(!filter.last_seen_amb.contains_key(&sig));
    assert!(!filter.last_seen_iono.contains_key(&sig.sat));
    assert!(!filter.residual_history.contains_key(&sig));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason == "precise_product_clock_clock_jump_action_reset_satellite_state"
            && event.removed_states.contains(&super::PppStateIdentity::SlantIonosphere(sig.sat))
            && event.removed_states.contains(&super::PppStateIdentity::CarrierAmbiguity(sig))
    }));
}

#[test]
fn ppp_precise_product_policy_refuses_flagged_orbit_records() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 25 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig::default());

    let usable = filter.apply_precise_product_policy(
        8,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::OrbitFlag,
        )],
    );

    assert!(!usable);
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason == "precise_product_orbit_orbit_flag_action_refuse_satellite"
            && event.sat == Some(sig.sat)
            && event.signal == Some(sig)
    }));
}
