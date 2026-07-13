use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, BEIDOU_B2_CARRIER_HZ};
use bijux_gnss_signal::api::{
    default_local_code_model_for_signal, default_signal_carrier_hz_for_signal,
    generate_beidou_b1i_code, generate_beidou_b2i_code, sample_modulated_replica_at_time,
    AcquisitionSignalModel, ReplicaCodeModel,
};

#[test]
fn beidou_b2i_public_code_generator_matches_open_service_family() {
    let b1 = generate_beidou_b1i_code(6).expect("valid BeiDou B1I PRN");
    let b2 = generate_beidou_b2i_code(6).expect("valid BeiDou B2I PRN");

    assert_eq!(b2.len(), 2046);
    assert_eq!(b2, b1);
}

#[test]
fn beidou_b2i_default_local_code_model_is_available_for_tracking() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let model = default_local_code_model_for_signal(sat, SignalBand::B2, SignalCode::B2I)
        .expect("local code model result")
        .expect("BeiDou B2I local code");

    assert_eq!(model.code_length(), 2046);
    assert_eq!(model.code_rate_hz(), 2_046_000.0);
}

#[test]
fn beidou_b2i_acquisition_model_reports_b2_carrier_identity() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let model = AcquisitionSignalModel::for_sat_signal(sat, Some(SignalBand::B2), SignalCode::B2I, None)
        .expect("acquisition model result")
        .expect("BeiDou B2I acquisition model");

    assert_eq!(model.signal_band, SignalBand::B2);
    assert_eq!(model.code_rate_hz, 2_046_000.0);
    assert_eq!(model.code_length, 2046);
    assert_eq!(model.carrier_hz, BEIDOU_B2_CARRIER_HZ);
}

#[test]
fn beidou_b2i_replica_samples_supported_power_on_b2_carrier() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::B2), SignalCode::B2I)
        .expect("replica result")
        .expect("BeiDou B2I replica");
    let carrier_hz = default_signal_carrier_hz_for_signal(sat, Some(SignalBand::B2), SignalCode::B2I, None)
        .expect("carrier result")
        .expect("BeiDou B2I carrier");
    let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, carrier_hz.value(), 0.0, 0.0, 1, 1.0)
        .expect("BeiDou B2I sample");

    assert!((sample.norm() - 1.0).abs() < 1.0e-6, "{sample:?}");
}
