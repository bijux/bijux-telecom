use bijux_gnss_core::api::{
    default_acquisition_signal, signal_registry, signal_spec_galileo_e1b, signal_spec_galileo_e1c,
    Constellation, SignalBand, SignalCode,
};

#[test]
fn galileo_e1_registry_declares_primary_code_length() {
    let e1b = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
        .expect("galileo e1-b signal must be registered");
    let e1c = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1C)
        .expect("galileo e1-c signal must be registered");

    assert_eq!(e1b.code_length, Some(4092));
    assert_eq!(e1c.code_length, Some(4092));
    assert_eq!(e1b.spec.code_rate_hz, 1_023_000.0);
    assert_eq!(e1c.spec.code_rate_hz, 1_023_000.0);
}

#[test]
fn default_acquisition_signal_uses_galileo_e1b() {
    let signal = default_acquisition_signal(Constellation::Galileo)
        .expect("galileo default acquisition signal must exist");
    let spec = signal_spec_galileo_e1b();

    assert_eq!(signal.spec.constellation, spec.constellation);
    assert_eq!(signal.spec.band, spec.band);
    assert_eq!(signal.spec.code, spec.code);
    assert_eq!(signal.spec.code_rate_hz, spec.code_rate_hz);
    assert_eq!(signal.spec.carrier_hz.value(), spec.carrier_hz.value());
    assert_eq!(signal.code_length, Some(4092));
}

#[test]
fn galileo_e1_signal_specs_are_distinct() {
    let e1b = signal_spec_galileo_e1b();
    let e1c = signal_spec_galileo_e1c();

    assert_eq!(e1b.band, SignalBand::E1);
    assert_eq!(e1c.band, SignalBand::E1);
    assert_eq!(e1b.code, SignalCode::E1B);
    assert_eq!(e1c.code, SignalCode::E1C);
    assert_eq!(e1b.carrier_hz, e1c.carrier_hz);
}
