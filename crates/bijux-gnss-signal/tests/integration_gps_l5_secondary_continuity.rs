mod support;

use bijux_gnss_signal::api::LocalCodeModel;
use support::chunked_sampling::deterministic_chunk_lengths;

const ARBITRARY_SAMPLE_RATE_HZ: f64 = 1_500_001.0;
const LONG_SESSION_START_SAMPLE_INDEX: u64 = 90_000_123;

#[test]
fn gps_l5_i_tracking_code_matches_random_chunk_stitching() {
    let model = LocalCodeModel::gps_l5_i(18).expect("valid GPS L5-I PRN");
    assert_chunked_tracking_code_matches_full(&model, 2_048.25, 40_960);
}

#[test]
fn gps_l5_q_tracking_code_matches_random_chunk_stitching() {
    let model = LocalCodeModel::gps_l5_q(24).expect("valid GPS L5-Q PRN");
    assert_chunked_tracking_code_matches_full(&model, 1_024.75, 40_960);
}

fn assert_chunked_tracking_code_matches_full(
    model: &LocalCodeModel,
    initial_code_phase_chips: f64,
    sample_count: usize,
) {
    let full = model
        .sample_tracking_block(
            ARBITRARY_SAMPLE_RATE_HZ,
            initial_code_phase_chips,
            LONG_SESSION_START_SAMPLE_INDEX,
            sample_count,
        )
        .expect("full tracking-local block");

    let mut stitched = Vec::with_capacity(sample_count);
    let mut next_sample_index = LONG_SESSION_START_SAMPLE_INDEX;
    for chunk_len in deterministic_chunk_lengths(sample_count) {
        let chunk = model
            .sample_tracking_block(
                ARBITRARY_SAMPLE_RATE_HZ,
                initial_code_phase_chips,
                next_sample_index,
                chunk_len,
            )
            .expect("chunked tracking-local block");
        stitched.extend(chunk);
        next_sample_index += chunk_len as u64;
    }

    assert_eq!(stitched, full);
}
