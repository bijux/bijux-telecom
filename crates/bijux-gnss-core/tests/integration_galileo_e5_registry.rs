use bijux_gnss_core::api::{
    signal_registry, signal_spec_galileo_e5a, Constellation, SignalBand, SignalCode,
};

#[test]
fn galileo_e5a_registry_declares_primary_code_length() {
    let e5a = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        .expect("galileo e5-a signal must be registered");

    assert_eq!(e5a.code_length, Some(10230));
    assert_eq!(e5a.spec.code_rate_hz, 10_230_000.0);
}

#[test]
fn galileo_e5a_signal_spec_matches_registry_entry() {
    let registry = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        .expect("galileo e5-a signal must be registered");
    let spec = signal_spec_galileo_e5a();

    assert_eq!(registry.spec.constellation, spec.constellation);
    assert_eq!(registry.spec.band, spec.band);
    assert_eq!(registry.spec.code, spec.code);
    assert_eq!(registry.spec.code_rate_hz, spec.code_rate_hz);
    assert_eq!(registry.spec.carrier_hz, spec.carrier_hz);
}
