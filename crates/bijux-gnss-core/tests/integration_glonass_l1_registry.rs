use bijux_gnss_core::api::{
    default_acquisition_sats, glonass_l1_carrier_hz, glonass_slot_from_sat, glonass_slot_sat,
    signal_registry,
    signal_spec_glonass_l1, Constellation, GlonassFrequencyChannel, GlonassL1FdmaSignal,
    GlonassSlot, SignalBand, SignalCode, GLONASS_L1_CARRIER_HZ, GLONASS_L1_CHANNEL_SPACING_HZ,
};

#[test]
fn glonass_l1_registry_declares_standard_precision_code_length() {
    let signal = signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
        .expect("GLONASS L1 must be registered");

    assert_eq!(signal.spec.code_rate_hz, 511_000.0);
    assert_eq!(signal.code_length, Some(511));
    assert_eq!(signal.spec.carrier_hz, GLONASS_L1_CARRIER_HZ);
}

#[test]
fn glonass_slot_round_trips_through_satellite_identity() {
    let slot = GlonassSlot::new(12).expect("slot 12 must be valid");
    let sat = glonass_slot_sat(slot);

    assert_eq!(sat.constellation, Constellation::Glonass);
    assert_eq!(sat.prn, 12);
    assert_eq!(glonass_slot_from_sat(sat), Some(slot));
    assert_eq!(
        glonass_slot_from_sat(bijux_gnss_core::api::SatId {
            constellation: Constellation::Gps,
            prn: 12
        }),
        None
    );
}

#[test]
fn glonass_l1_carrier_spacing_tracks_frequency_channel_number() {
    let lower = GlonassFrequencyChannel::new(-7).expect("channel -7 must be valid");
    let upper = GlonassFrequencyChannel::new(13).expect("channel 13 must be valid");

    assert_eq!(glonass_l1_carrier_hz(lower).value(), 1_598_062_500.0);
    assert_eq!(glonass_l1_carrier_hz(upper).value(), 1_609_312_500.0);
    assert_eq!(GLONASS_L1_CHANNEL_SPACING_HZ.value(), 562_500.0);
}

#[test]
fn glonass_l1_signal_descriptor_preserves_slot_channel_and_carrier() {
    let slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let signal = GlonassL1FdmaSignal { slot, frequency_channel: channel };
    let spec = signal_spec_glonass_l1(channel);

    assert_eq!(signal.sat_id(), glonass_slot_sat(slot));
    assert_eq!(signal.signal_id().band, SignalBand::L1);
    assert_eq!(signal.signal_id().code, SignalCode::Unknown);
    assert_eq!(signal.carrier_hz(), spec.carrier_hz);
    assert_eq!(spec.code_rate_hz, 511_000.0);
}

#[test]
fn default_glonass_acquisition_catalog_covers_all_slots() {
    let sats = default_acquisition_sats(Constellation::Glonass);

    assert_eq!(sats.len(), GlonassSlot::MAX as usize);
    assert_eq!(
        sats.first().copied(),
        Some(glonass_slot_sat(GlonassSlot::new(GlonassSlot::MIN).expect("slot 1 must be valid")))
    );
    assert_eq!(
        sats.last().copied(),
        Some(glonass_slot_sat(
            GlonassSlot::new(GlonassSlot::MAX).expect("slot 24 must be valid")
        ))
    );
}
