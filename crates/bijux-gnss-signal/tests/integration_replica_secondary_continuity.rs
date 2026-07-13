mod support;

use bijux_gnss_signal::api::ReplicaCodeModel;
use support::chunked_sampling::deterministic_chunk_lengths;

const ARBITRARY_SAMPLE_RATE_HZ: f64 = 1_500_001.0;
const LONG_SESSION_START_SAMPLE_INDEX: u64 = 150_000_321;
const INITIAL_CARRIER_PHASE_RADIANS: f64 = 0.125;
const INITIAL_CARRIER_HZ: f64 = 4_100.0;
const CARRIER_RATE_HZ_PER_S: f64 = 2.5;
const DATA_BIT: i8 = 1;
const AMPLITUDE: f32 = 0.75;

#[test]
fn gps_l5_q_replica_matches_random_chunk_stitching() {
    let model = ReplicaCodeModel::gps_l5_q(24).expect("valid GPS L5-Q PRN");
    assert_chunked_replica_matches_full(&model, 1_024.75, 40_960);
}

#[test]
fn galileo_e5a_qpsk_replica_matches_random_chunk_stitching() {
    let model = ReplicaCodeModel::galileo_e5a_qpsk(11).expect("valid Galileo E5a-Q PRN");
    assert_chunked_replica_matches_full(&model, 1_536.125, 40_960);
}

#[test]
fn galileo_e5b_qpsk_replica_matches_random_chunk_stitching() {
    let model = ReplicaCodeModel::galileo_e5b_qpsk(11).expect("valid Galileo E5b-Q PRN");
    assert_chunked_replica_matches_full(&model, 768.875, 40_960);
}

fn assert_chunked_replica_matches_full(
    model: &ReplicaCodeModel,
    initial_code_phase_chips: f64,
    sample_count: usize,
) {
    let full = model
        .sample_block(
            ARBITRARY_SAMPLE_RATE_HZ,
            initial_code_phase_chips,
            INITIAL_CARRIER_PHASE_RADIANS,
            INITIAL_CARRIER_HZ,
            CARRIER_RATE_HZ_PER_S,
            LONG_SESSION_START_SAMPLE_INDEX,
            DATA_BIT,
            AMPLITUDE,
            sample_count,
        )
        .expect("full replica block");

    let mut stitched = Vec::with_capacity(sample_count);
    let mut next_sample_index = LONG_SESSION_START_SAMPLE_INDEX;
    for chunk_len in deterministic_chunk_lengths(sample_count) {
        let chunk = model
            .sample_block(
                ARBITRARY_SAMPLE_RATE_HZ,
                initial_code_phase_chips,
                INITIAL_CARRIER_PHASE_RADIANS,
                INITIAL_CARRIER_HZ,
                CARRIER_RATE_HZ_PER_S,
                next_sample_index,
                DATA_BIT,
                AMPLITUDE,
                chunk_len,
            )
            .expect("chunked replica block");
        stitched.extend(chunk);
        next_sample_index += chunk_len as u64;
    }

    assert_eq!(stitched, full);
}
