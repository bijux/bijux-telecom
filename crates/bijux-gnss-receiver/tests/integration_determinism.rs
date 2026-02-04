use bijux_gnss_receiver::api::sim::{
    generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams,
};
use bijux_gnss_receiver::api::ReceiverProfile;

fn hash_samples(samples: &[num_complex::Complex<f32>]) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    for sample in samples {
        let i = sample.re.to_bits() as u64;
        let q = sample.im.to_bits() as u64;
        hash ^= i;
        hash = hash.wrapping_mul(0x100000001b3);
        hash ^= q;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[test]
fn deterministic_synthetic_runs_match() {
    let profile = ReceiverProfile::default();
    let config = profile.to_receiver_config();
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        duration_s: 0.01,
        seed: 42,
        satellites: vec![SyntheticSignalParams {
            sat: bijux_gnss_core::api::SatId {
                constellation: bijux_gnss_core::api::Constellation::Gps,
                prn: 1,
            },
            doppler_hz: 1000.0,
            code_phase_chips: 100.0,
            carrier_phase_rad: 0.1,
            cn0_db_hz: 45.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "determinism".to_string(),
    };

    let frame_a = generate_l1_ca_multi(&config, &scenario);
    let frame_b = generate_l1_ca_multi(&config, &scenario);

    assert_eq!(hash_samples(&frame_a.iq), hash_samples(&frame_b.iq));
}
