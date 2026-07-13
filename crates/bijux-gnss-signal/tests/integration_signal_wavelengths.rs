use bijux_gnss_core::api::{Constellation, SatId, SigId, SignalBand, SignalCode};
use bijux_gnss_signal::api::{signal_id_wavelength_m, signal_registry, signal_wavelength_m};

fn assert_signal_wavelength_matches_registry(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) {
    let signal = signal_registry(constellation, band, code)
        .unwrap_or_else(|| panic!("missing registry entry for {constellation:?} {band:?} {code:?}"))
        .spec;
    let signal_id = SigId { sat: SatId { constellation, prn: 11 }, band, code };

    let from_signal = signal_wavelength_m(signal);
    let from_id = signal_id_wavelength_m(signal_id).expect("signal-id wavelength");

    assert!((from_signal.0 - from_id.0).abs() < 1.0e-15, "{signal_id:?}");
}

#[test]
fn signal_wavelength_helpers_cover_all_registered_phase_signals() {
    for (constellation, band, code) in [
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca),
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C),
        (Constellation::Gps, SignalBand::L2, SignalCode::Py),
        (Constellation::Gps, SignalBand::L5, SignalCode::L5I),
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1C),
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown),
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
    ] {
        assert_signal_wavelength_matches_registry(constellation, band, code);
    }
}

#[test]
fn signal_id_wavelength_returns_none_for_unregistered_signal_identity() {
    let signal_id = SigId {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        band: SignalBand::B2,
        code: SignalCode::Unknown,
    };

    assert!(signal_id_wavelength_m(signal_id).is_none());
}
