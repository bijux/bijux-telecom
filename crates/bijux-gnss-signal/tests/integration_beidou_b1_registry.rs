use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, BEIDOU_B1_CARRIER_HZ};
use bijux_gnss_signal::api::{
    default_acquisition_sats, default_acquisition_signal, signal_registry, signal_spec_beidou_b1i,
};

#[test]
fn beidou_b1_registry_declares_primary_code_length() {
    let signal = signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        .expect("beidou b1i signal must be registered");

    assert_eq!(signal.spec.code_rate_hz, 2_046_000.0);
    assert_eq!(signal.code_length, Some(2046));
    assert_eq!(signal.spec.carrier_hz, BEIDOU_B1_CARRIER_HZ);
    assert_eq!(signal.spec.code, SignalCode::B1I);
}

#[test]
fn default_acquisition_signal_uses_beidou_b1i() {
    let signal = default_acquisition_signal(Constellation::Beidou)
        .expect("beidou default acquisition signal must exist");
    let spec = signal_spec_beidou_b1i();

    assert_eq!(signal.spec.constellation, spec.constellation);
    assert_eq!(signal.spec.band, spec.band);
    assert_eq!(signal.spec.code, spec.code);
    assert_eq!(signal.spec.code_rate_hz, spec.code_rate_hz);
    assert_eq!(signal.spec.carrier_hz.value(), spec.carrier_hz.value());
    assert_eq!(signal.code_length, Some(2046));
}

#[test]
fn default_acquisition_catalog_covers_registered_beidou_prns() {
    let sats = default_acquisition_sats(Constellation::Beidou);

    assert_eq!(sats.len(), 37);
    assert_eq!(sats.first().copied(), Some(SatId { constellation: Constellation::Beidou, prn: 1 }));
    assert_eq!(sats.last().copied(), Some(SatId { constellation: Constellation::Beidou, prn: 37 }));
    assert!(sats.iter().all(|sat| sat.constellation == Constellation::Beidou));
}
