#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, FreqHz, GlonassFrequencyChannel, Meters, SigId, SignalBand,
    SignalCode, SignalRegistryEntry, SignalSpec, BEIDOU_B1_CARRIER_HZ, BEIDOU_B2_CARRIER_HZ,
    GALILEO_E1_CARRIER_HZ, GALILEO_E5A_CARRIER_HZ, GALILEO_E5B_CARRIER_HZ,
    GLONASS_L1_CARRIER_HZ, GLONASS_L1_CHANNEL_SPACING_HZ, GPS_L1_CA_CARRIER_HZ,
    GPS_L2_PY_CARRIER_HZ, GPS_L2C_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

pub fn signal_spec_gps_l1_ca() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L1,
        code: SignalCode::Ca,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GPS_L1_CA_CARRIER_HZ,
    }
}

pub fn signal_spec_gps_l2c() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L2,
        code: SignalCode::L2C,
        code_rate_hz: 511_500.0,
        carrier_hz: GPS_L2C_CARRIER_HZ,
    }
}

pub fn signal_spec_gps_l2_py() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L2,
        code: SignalCode::Py,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GPS_L2_PY_CARRIER_HZ,
    }
}

pub fn signal_spec_gps_l5() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Gps,
        band: SignalBand::L5,
        code: SignalCode::Unknown,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GPS_L5_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e1b() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E1,
        code: SignalCode::E1B,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GALILEO_E1_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e1c() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E1,
        code: SignalCode::E1C,
        code_rate_hz: 1_023_000.0,
        carrier_hz: GALILEO_E1_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e5a() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E5,
        code: SignalCode::E5a,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GALILEO_E5A_CARRIER_HZ,
    }
}

pub fn signal_spec_galileo_e5b() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Galileo,
        band: SignalBand::E5,
        code: SignalCode::E5b,
        code_rate_hz: 10_230_000.0,
        carrier_hz: GALILEO_E5B_CARRIER_HZ,
    }
}

pub fn signal_spec_glonass_l1(frequency_channel: GlonassFrequencyChannel) -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Glonass,
        band: SignalBand::L1,
        code: SignalCode::Unknown,
        code_rate_hz: 511_000.0,
        carrier_hz: glonass_l1_carrier_hz(frequency_channel),
    }
}

pub fn signal_spec_beidou_b1i() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Beidou,
        band: SignalBand::B1,
        code: SignalCode::B1I,
        code_rate_hz: 2_046_000.0,
        carrier_hz: BEIDOU_B1_CARRIER_HZ,
    }
}

pub fn signal_spec_beidou_b2i() -> SignalSpec {
    SignalSpec {
        constellation: Constellation::Beidou,
        band: SignalBand::B2,
        code: SignalCode::B2I,
        code_rate_hz: 2_046_000.0,
        carrier_hz: BEIDOU_B2_CARRIER_HZ,
    }
}

pub fn carrier_wavelength_m(carrier_hz: FreqHz) -> Meters {
    Meters(SPEED_OF_LIGHT_MPS / carrier_hz.value())
}

pub fn signal_wavelength_m(signal: SignalSpec) -> Meters {
    carrier_wavelength_m(signal.carrier_hz)
}

pub fn signal_id_wavelength_m(signal_id: SigId) -> Option<Meters> {
    let signal = signal_registry(signal_id.sat.constellation, signal_id.band, signal_id.code)?.spec;
    Some(signal_wavelength_m(signal))
}

pub fn signal_cycles_to_meters(cycles: Cycles, signal: SignalSpec) -> Meters {
    Meters(cycles.0 * signal_wavelength_m(signal).0)
}

pub fn signal_id_cycles_to_meters(cycles: Cycles, signal_id: SigId) -> Option<Meters> {
    Some(Meters(cycles.0 * signal_id_wavelength_m(signal_id)?.0))
}

pub fn signal_meters_to_cycles(distance_m: Meters, signal: SignalSpec) -> Cycles {
    Cycles(distance_m.0 / signal_wavelength_m(signal).0)
}

pub fn signal_id_meters_to_cycles(distance_m: Meters, signal_id: SigId) -> Option<Cycles> {
    Some(Cycles(distance_m.0 / signal_id_wavelength_m(signal_id)?.0))
}

pub fn first_order_ionosphere_code_delay_m(
    reference_delay_m: Meters,
    reference_signal: SignalSpec,
    target_signal: SignalSpec,
) -> Option<Meters> {
    scaled_inverse_square_delay_m(
        reference_delay_m,
        reference_signal.carrier_hz,
        target_signal.carrier_hz,
    )
}

pub fn first_order_ionosphere_phase_advance_m(
    reference_delay_m: Meters,
    reference_signal: SignalSpec,
    target_signal: SignalSpec,
) -> Option<Meters> {
    Some(Meters(
        -first_order_ionosphere_code_delay_m(reference_delay_m, reference_signal, target_signal)?.0,
    ))
}

fn scaled_inverse_square_delay_m(
    reference_delay_m: Meters,
    reference_carrier_hz: FreqHz,
    target_carrier_hz: FreqHz,
) -> Option<Meters> {
    let reference_delay_m = reference_delay_m.0;
    let reference_carrier_hz = reference_carrier_hz.value();
    let target_carrier_hz = target_carrier_hz.value();
    if !reference_delay_m.is_finite()
        || !reference_carrier_hz.is_finite()
        || !target_carrier_hz.is_finite()
        || reference_carrier_hz <= 0.0
        || target_carrier_hz <= 0.0
    {
        return None;
    }

    let scaled_delay_m = reference_delay_m * (reference_carrier_hz * reference_carrier_hz)
        / (target_carrier_hz * target_carrier_hz);
    scaled_delay_m.is_finite().then_some(Meters(scaled_delay_m))
}

pub fn glonass_l1_carrier_hz(frequency_channel: GlonassFrequencyChannel) -> FreqHz {
    FreqHz::new(
        GLONASS_L1_CARRIER_HZ.value()
            + f64::from(frequency_channel.value()) * GLONASS_L1_CHANNEL_SPACING_HZ.value(),
    )
}

pub fn signal_registry(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> Option<SignalRegistryEntry> {
    let spec =
        SignalSpec { constellation, band, code, code_rate_hz: 0.0, carrier_hz: FreqHz::new(0.0) };
    let (carrier_hz, code_rate_hz, code_length) = match (constellation, band, code) {
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca) => {
            (GPS_L1_CA_CARRIER_HZ, 1_023_000.0, Some(1023))
        }
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C) => {
            (GPS_L2C_CARRIER_HZ, 511_500.0, Some(10_230))
        }
        (Constellation::Gps, SignalBand::L2, SignalCode::Py) => {
            (GPS_L2_PY_CARRIER_HZ, 10_230_000.0, None)
        }
        (Constellation::Gps, SignalBand::L5, SignalCode::Unknown) => {
            (GPS_L5_CARRIER_HZ, 10_230_000.0, None)
        }
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B) => {
            (GALILEO_E1_CARRIER_HZ, 1_023_000.0, Some(4092))
        }
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1C) => {
            (GALILEO_E1_CARRIER_HZ, 1_023_000.0, Some(4092))
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            (GALILEO_E5A_CARRIER_HZ, 10_230_000.0, Some(10_230))
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            (GALILEO_E5B_CARRIER_HZ, 10_230_000.0, Some(10_230))
        }
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown) => {
            (GLONASS_L1_CARRIER_HZ, 511_000.0, Some(511))
        }
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I) => {
            (BEIDOU_B1_CARRIER_HZ, 2_046_000.0, Some(2046))
        }
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I) => {
            (BEIDOU_B2_CARRIER_HZ, 2_046_000.0, Some(2046))
        }
        _ => return None,
    };
    Some(SignalRegistryEntry { spec: SignalSpec { carrier_hz, code_rate_hz, ..spec }, code_length })
}

pub fn registered_signal_registry_entries() -> Vec<SignalRegistryEntry> {
    [
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca),
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C),
        (Constellation::Gps, SignalBand::L2, SignalCode::Py),
        (Constellation::Gps, SignalBand::L5, SignalCode::Unknown),
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1B),
        (Constellation::Galileo, SignalBand::E1, SignalCode::E1C),
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a),
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5b),
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown),
        (Constellation::Beidou, SignalBand::B1, SignalCode::B1I),
        (Constellation::Beidou, SignalBand::B2, SignalCode::B2I),
    ]
    .into_iter()
    .map(|(constellation, band, code)| {
        signal_registry(constellation, band, code).expect("registered signal registry entry")
    })
    .collect()
}

pub fn default_acquisition_signal(constellation: Constellation) -> Option<SignalRegistryEntry> {
    match constellation {
        Constellation::Gps => signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca),
        Constellation::Galileo => {
            signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
        }
        Constellation::Glonass => {
            signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
        }
        Constellation::Beidou => {
            signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        }
        Constellation::Unknown => None,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        carrier_wavelength_m, default_acquisition_signal, first_order_ionosphere_code_delay_m,
        first_order_ionosphere_phase_advance_m, glonass_l1_carrier_hz,
        registered_signal_registry_entries, signal_cycles_to_meters, signal_id_cycles_to_meters,
        signal_id_meters_to_cycles, signal_id_wavelength_m, signal_meters_to_cycles,
        signal_registry, signal_spec_beidou_b2i, signal_spec_galileo_e1b, signal_spec_glonass_l1,
        signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l2c, signal_wavelength_m,
    };
    use bijux_gnss_core::api::{
        Constellation, Cycles, FreqHz, GlonassFrequencyChannel, GlonassL1FdmaSignal, GlonassSlot,
        Meters, SatId, SigId, SignalBand, SignalCode, BEIDOU_B1_CARRIER_HZ, GLONASS_L1_CARRIER_HZ,
        GLONASS_L1_CHANNEL_SPACING_HZ,
    };

    #[test]
    fn gps_l2c_signal_spec_matches_civil_code_timing() {
        let signal = signal_spec_gps_l2c();

        assert_eq!(signal.code, SignalCode::L2C);
        assert!((signal.code_rate_hz - 511_500.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::L2C)
            .expect("GPS L2C registry entry");
        assert_eq!(registry.code_length, Some(10_230));
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn registered_signal_registry_entries_match_lookup_inventory() {
        let entries = registered_signal_registry_entries();
        assert_eq!(entries.len(), 11);

        for entry in entries {
            let lookup =
                signal_registry(entry.spec.constellation, entry.spec.band, entry.spec.code)
                    .expect("registered signal lookup");
            assert_eq!(lookup.spec, entry.spec);
            assert_eq!(lookup.code_length, entry.code_length);
        }
    }

    #[test]
    fn gps_l2_py_signal_spec_preserves_legacy_rate_without_short_code_length() {
        let signal = signal_spec_gps_l2_py();

        assert_eq!(signal.code, SignalCode::Py);
        assert!((signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::Py)
            .expect("GPS L2 P(Y) registry entry");
        assert_eq!(registry.code_length, None);
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn beidou_b2i_signal_spec_matches_open_service_timing() {
        let signal = signal_spec_beidou_b2i();

        assert_eq!(signal.code, SignalCode::B2I);
        assert!((signal.code_rate_hz - 2_046_000.0).abs() <= f64::EPSILON);
        let registry = signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
            .expect("BeiDou B2I registry entry");
        assert_eq!(registry.code_length, Some(2046));
        assert_eq!(registry.spec, signal);
    }

    #[test]
    fn carrier_wavelength_matches_speed_of_light_ratio() {
        let wavelength = carrier_wavelength_m(FreqHz::new(1_575_420_000.0));

        assert!((wavelength.0 - 0.190_293_672_798_364_87).abs() < 1.0e-15);
    }

    #[test]
    fn signal_wavelength_uses_signal_carrier_frequency() {
        let signal = signal_spec_beidou_b2i();
        let wavelength = signal_wavelength_m(signal);

        assert!((wavelength.0 - 0.248_349_369_584_306_9).abs() < 1.0e-15);
    }

    #[test]
    fn signal_id_wavelength_resolves_registered_signal_identity() {
        let wavelength = signal_id_wavelength_m(SigId {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            band: SignalBand::B2,
            code: SignalCode::B2I,
        })
        .expect("BeiDou B2I wavelength");

        assert!((wavelength.0 - 0.248_349_369_584_306_9).abs() < 1.0e-15);
    }

    #[test]
    fn signal_phase_conversion_helpers_round_trip_registered_signal() {
        let signal_id = SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: SignalBand::L2,
            code: SignalCode::Py,
        };
        let phase_m = signal_id_cycles_to_meters(Cycles(12_345.25), signal_id)
            .expect("GPS L2 P(Y) phase meters");
        let phase_cycles =
            signal_id_meters_to_cycles(phase_m, signal_id).expect("GPS L2 P(Y) phase cycles");

        assert!((phase_cycles.0 - 12_345.25).abs() < 1.0e-12);
    }

    #[test]
    fn signal_phase_conversion_helpers_round_trip_signal_spec() {
        let signal = signal_spec_beidou_b2i();
        let phase_m = signal_cycles_to_meters(Cycles(512.5), signal);
        let phase_cycles = signal_meters_to_cycles(Meters(phase_m.0), signal);

        assert!((phase_cycles.0 - 512.5).abs() < 1.0e-12);
    }

    #[test]
    fn first_order_ionosphere_delay_scales_with_inverse_frequency_square() {
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2c();
        let reference_delay_m = Meters(5.0);

        let scaled_delay_m =
            first_order_ionosphere_code_delay_m(reference_delay_m, l1, l2).expect("delay");
        let expected_delay_m = reference_delay_m.0
            * (l1.carrier_hz.value() * l1.carrier_hz.value())
            / (l2.carrier_hz.value() * l2.carrier_hz.value());

        assert!((scaled_delay_m.0 - expected_delay_m).abs() < 1.0e-12);
    }

    #[test]
    fn first_order_ionosphere_phase_advance_is_negative_code_delay() {
        let l1 = signal_spec_gps_l1_ca();
        let e1 = signal_spec_galileo_e1b();
        let reference_delay_m = Meters(4.0);

        let code_delay_m =
            first_order_ionosphere_code_delay_m(reference_delay_m, l1, e1).expect("code delay");
        let phase_advance_m = first_order_ionosphere_phase_advance_m(reference_delay_m, l1, e1)
            .expect("phase advance");

        assert!((phase_advance_m.0 + code_delay_m.0).abs() < 1.0e-12);
    }

    #[test]
    fn default_acquisition_signal_matches_supported_band_per_constellation() {
        assert_eq!(
            default_acquisition_signal(Constellation::Beidou)
                .expect("BeiDou default signal")
                .spec
                .carrier_hz,
            BEIDOU_B1_CARRIER_HZ
        );
    }

    #[test]
    fn glonass_l1_spec_uses_frequency_channel_spacing() {
        let slot = GlonassSlot::new(8).expect("slot");
        let channel = GlonassFrequencyChannel::new(5).expect("channel");
        let signal = GlonassL1FdmaSignal { slot, frequency_channel: channel };
        let spec = signal_spec_glonass_l1(channel);

        assert_eq!(spec.constellation, Constellation::Glonass);
        assert_eq!(spec.band, SignalBand::L1);
        assert_eq!(spec.carrier_hz, glonass_l1_carrier_hz(channel));
        assert_eq!(
            spec.carrier_hz.value(),
            GLONASS_L1_CARRIER_HZ.value() + 5.0 * GLONASS_L1_CHANNEL_SPACING_HZ.value()
        );
        assert_eq!(signal.signal_id().sat, signal.sat_id());
    }
}
