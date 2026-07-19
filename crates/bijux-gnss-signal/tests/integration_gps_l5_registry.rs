use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode};
use bijux_gnss_signal::api::{
    signal_registry, signal_spec_gps_l5, signal_spec_gps_l5_i, signal_spec_gps_l5_q,
};

#[test]
fn gps_l5_registry_declares_both_data_and_pilot_codes() {
    let l5i = signal_registry(Constellation::Gps, SignalBand::L5, SignalCode::L5I)
        .expect("gps l5-i signal must be registered");
    let l5q = signal_registry(Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        .expect("gps l5-q signal must be registered");

    assert_eq!(l5i.code_length, Some(10_230));
    assert_eq!(l5q.code_length, Some(10_230));
    assert_eq!(l5i.spec.code_rate_hz, 10_230_000.0);
    assert_eq!(l5q.spec.code_rate_hz, 10_230_000.0);
    assert_eq!(l5i.spec.carrier_hz, l5q.spec.carrier_hz);
}

#[test]
fn gps_l5_signal_specs_keep_l5i_as_the_legacy_alias() {
    let l5 = signal_spec_gps_l5();
    let l5i = signal_spec_gps_l5_i();
    let l5q = signal_spec_gps_l5_q();

    assert_eq!(l5, l5i);
    assert_eq!(l5i.code, SignalCode::L5I);
    assert_eq!(l5q.code, SignalCode::L5Q);
    assert_eq!(l5i.carrier_hz, l5q.carrier_hz);
}
