mod support;

use bijux_gnss_signal::api::LocalCodeModel;
use support::chunked_sampling::deterministic_chunk_lengths;

const ARBITRARY_SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const START_SAMPLE_INDEX: u64 = 31;

#[test]
fn gps_l1_local_code_matches_random_chunk_stitching() {
    let model = LocalCodeModel::gps_l1_ca(11).expect("valid GPS L1 PRN");
    assert_chunked_local_code_matches_full(&model, 412.375, 20_000);
}

#[test]
fn galileo_e1_boc_local_code_matches_random_chunk_stitching() {
    let model = LocalCodeModel::galileo_e1_boc11(11).expect("valid Galileo E1 PRN");
    assert_chunked_local_code_matches_full(&model, 137.625, 32_768);
}

fn assert_chunked_local_code_matches_full(
    model: &LocalCodeModel,
    initial_code_phase_chips: f64,
    sample_count: usize,
) {
    let full = model
        .sample_block(
            ARBITRARY_SAMPLE_RATE_HZ,
            initial_code_phase_chips,
            START_SAMPLE_INDEX,
            sample_count,
        )
        .expect("full local-code block");

    let mut stitched = Vec::with_capacity(sample_count);
    let mut next_sample_index = START_SAMPLE_INDEX;
    for chunk_len in deterministic_chunk_lengths(sample_count) {
        let chunk = model
            .sample_block(
                ARBITRARY_SAMPLE_RATE_HZ,
                initial_code_phase_chips,
                next_sample_index,
                chunk_len,
            )
            .expect("chunked local-code block");
        stitched.extend(chunk);
        next_sample_index += chunk_len as u64;
    }

    assert_eq!(stitched, full);
}
