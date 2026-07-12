use bijux_gnss_core::api::{
    signal_registry, signal_spec_beidou_b1i, signal_spec_beidou_b2i, Constellation, SignalBand,
    SignalCode,
};

#[test]
fn beidou_b2i_registry_exposes_dual_frequency_ready_signal_contract() {
    let b2 = signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
        .expect("BeiDou B2I registry entry");

    assert_eq!(b2.spec, signal_spec_beidou_b2i());
    assert_eq!(b2.code_length, Some(2046));
}

#[test]
fn beidou_b1i_and_b2i_registry_entries_preserve_distinct_carriers() {
    let b1 = signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        .expect("BeiDou B1I registry entry");
    let b2 = signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
        .expect("BeiDou B2I registry entry");

    assert_eq!(b1.spec, signal_spec_beidou_b1i());
    assert_eq!(b2.spec, signal_spec_beidou_b2i());
    assert_ne!(b1.spec.carrier_hz, b2.spec.carrier_hz);
    assert_eq!(b1.spec.code_rate_hz, b2.spec.code_rate_hz);
}
