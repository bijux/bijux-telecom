use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_signal::api::{
    default_local_code_model_for_signal, galileo_e5a_i_secondary_code,
    galileo_e5a_q_secondary_code, generate_galileo_e5a_i_code, generate_galileo_e5a_q_code,
    sample_modulated_replica_at_time, ReplicaCodeModel,
};

fn first_24_chips_hex(code: &[i8]) -> String {
    let mut value = 0_u32;
    for chip in code.iter().take(24) {
        value <<= 1;
        if *chip == -1 {
            value |= 1;
        }
    }
    format!("{value:06X}")
}

#[test]
fn galileo_e5a_public_code_generators_match_published_prefixes() {
    let e5ai = generate_galileo_e5a_i_code(1).expect("valid Galileo E5a-I PRN");
    let e5aq = generate_galileo_e5a_q_code(1).expect("valid Galileo E5a-Q PRN");

    assert_eq!(e5ai.len(), 10_230);
    assert_eq!(e5aq.len(), 10_230);
    assert_eq!(first_24_chips_hex(&e5ai), "3CEA9D");
    assert_eq!(first_24_chips_hex(&e5aq), "515537");
}

#[test]
fn galileo_e5a_public_secondary_code_generators_publish_expected_lengths() {
    let e5ai = galileo_e5a_i_secondary_code();
    let e5aq = galileo_e5a_q_secondary_code(1).expect("valid Galileo E5a-Q PRN");

    assert_eq!(e5ai.len(), 20);
    assert_eq!(e5aq.len(), 100);
}

#[test]
fn galileo_e5a_default_local_code_model_is_available_for_tracking() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let model = default_local_code_model_for_signal(sat, SignalBand::E5, SignalCode::E5a)
        .expect("local code model result")
        .expect("Galileo E5a local code");

    assert_eq!(model.code_length(), 10_230);
    assert_eq!(model.code_rate_hz(), 10_230_000.0);
}

#[test]
fn galileo_e5a_public_replica_samples_supported_data_component_power() {
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let model = ReplicaCodeModel::for_sat_signal(sat, Some(SignalBand::E5), SignalCode::E5a)
        .expect("replica result")
        .expect("Galileo E5a replica");

    let sample = sample_modulated_replica_at_time(&model, 0.0, 0.0, 0.0, 0.0, 0.0, 1, 1.0)
        .expect("Galileo E5a sample");

    assert!((sample.norm() - std::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-6, "{sample:?}");
}
