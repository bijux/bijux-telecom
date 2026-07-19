mod support;

use bijux_gnss_signal::api::{ReplicaBlockRequest, ReplicaCodeModel};
use support::chunked_sampling::deterministic_chunk_lengths;

const ARBITRARY_SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const START_SAMPLE_INDEX: u64 = 31;
const INITIAL_CARRIER_PHASE_RADIANS: f64 = 0.125;
const INITIAL_CARRIER_HZ: f64 = 4_100.0;
const CARRIER_RATE_HZ_PER_S: f64 = 2.5;
const DATA_BIT: i8 = 1;
const AMPLITUDE: f32 = 0.75;

#[test]
fn gps_l2c_replica_matches_random_chunk_stitching() {
    let model = ReplicaCodeModel::gps_l2c_time_multiplexed(38).expect("valid GPS L2C PRN");
    assert_chunked_replica_matches_full(&model, 412.375, 20_000);
}

#[test]
fn galileo_e1_replica_matches_random_chunk_stitching() {
    let model = ReplicaCodeModel::galileo_e1_cboc(11).expect("valid Galileo E1 PRN");
    assert_chunked_replica_matches_full(&model, 137.625, 32_768);
}

fn assert_chunked_replica_matches_full(
    model: &ReplicaCodeModel,
    initial_code_phase_chips: f64,
    sample_count: usize,
) {
    let full = model
        .sample_block(ReplicaBlockRequest {
            sample_rate_hz: ARBITRARY_SAMPLE_RATE_HZ,
            initial_code_phase_chips,
            initial_carrier_phase_radians: INITIAL_CARRIER_PHASE_RADIANS,
            initial_carrier_hz: INITIAL_CARRIER_HZ,
            carrier_rate_hz_per_s: CARRIER_RATE_HZ_PER_S,
            start_sample_index: START_SAMPLE_INDEX,
            data_bit: DATA_BIT,
            amplitude: AMPLITUDE,
            sample_count,
        })
        .expect("full replica block");

    let mut stitched = Vec::with_capacity(sample_count);
    let mut next_sample_index = START_SAMPLE_INDEX;
    for chunk_len in deterministic_chunk_lengths(sample_count) {
        let chunk = model
            .sample_block(ReplicaBlockRequest {
                sample_rate_hz: ARBITRARY_SAMPLE_RATE_HZ,
                initial_code_phase_chips,
                initial_carrier_phase_radians: INITIAL_CARRIER_PHASE_RADIANS,
                initial_carrier_hz: INITIAL_CARRIER_HZ,
                carrier_rate_hz_per_s: CARRIER_RATE_HZ_PER_S,
                start_sample_index: next_sample_index,
                data_bit: DATA_BIT,
                amplitude: AMPLITUDE,
                sample_count: chunk_len,
            })
            .expect("chunked replica block");
        stitched.extend(chunk);
        next_sample_index += chunk_len as u64;
    }

    assert_eq!(stitched, full);
}
