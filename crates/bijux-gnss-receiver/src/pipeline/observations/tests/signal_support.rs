use super::*;

#[test]
fn observation_signal_support_follows_tracked_signal_mapping() {
    assert!(supports_observation_signal(Constellation::Gps, SignalBand::L1, SignalCode::Ca));
    assert!(supports_observation_signal(Constellation::Gps, SignalBand::L2, SignalCode::L2C));
    assert!(supports_observation_signal(Constellation::Gps, SignalBand::L5, SignalCode::L5I));
    assert!(supports_observation_signal(Constellation::Galileo, SignalBand::E5, SignalCode::E5a));
    assert!(supports_observation_signal(Constellation::Beidou, SignalBand::B2, SignalCode::B2I));
    assert!(supports_observation_signal(
        Constellation::Glonass,
        SignalBand::L1,
        SignalCode::Unknown
    ));
    assert!(!supports_observation_signal(Constellation::Gps, SignalBand::L2, SignalCode::Ca));
    assert_eq!(tracked_signal_code_for_band(Constellation::Unknown, SignalBand::Unknown), None);
}

#[test]
fn observation_signal_support_matches_registered_signal_inventory() {
    let registered = registered_signal_registry_entries();
    let supported = registered
        .iter()
        .filter(|entry| {
            supports_observation_signal(entry.spec.constellation, entry.spec.band, entry.spec.code)
        })
        .map(|entry| (entry.spec.constellation, entry.spec.band, entry.spec.code))
        .collect::<BTreeSet<_>>();
    let unsupported = registered
        .iter()
        .filter(|entry| {
            !supports_observation_signal(entry.spec.constellation, entry.spec.band, entry.spec.code)
        })
        .map(|entry| (entry.spec.constellation, entry.spec.band, entry.spec.code))
        .collect::<BTreeSet<_>>();

    assert_eq!(
        supported,
        BTreeSet::from([
            (Constellation::Gps, SignalBand::L1, SignalCode::Ca),
            (Constellation::Gps, SignalBand::L2, SignalCode::L2C),
            (Constellation::Gps, SignalBand::L5, SignalCode::L5I),
            (Constellation::Gps, SignalBand::L5, SignalCode::L5Q),
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
            (Constellation::Galileo, SignalBand::E5, SignalCode::E5b),
            (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown),
            (Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
            (Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
        ])
    );
    assert_eq!(
        unsupported,
        BTreeSet::from([
            (Constellation::Gps, SignalBand::L2, SignalCode::Py),
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1C),
        ])
    );
}
