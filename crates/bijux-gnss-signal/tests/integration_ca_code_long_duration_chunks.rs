#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    advance_code_phase_chips, advance_code_phase_seconds, sample_ca_code, samples_per_code, Prn,
    CA_CODE_PERIOD_CHIPS,
};

const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const SIXTY_SECONDS: f64 = 60.0;
const PHASE_TOLERANCE_CHIPS: f64 = 1e-8;
const CHUNK_PATTERN: [usize; 4] = [97_531, 131_071, 262_139, 524_287];

#[test]
fn chunked_phase_accumulation_matches_theoretical_phase_after_sixty_seconds() {
    let start_chip_phase = 412.375;
    let total_samples = (SIXTY_SECONDS * SAMPLE_RATE_HZ) as usize;
    let phase_from_chunks = advance_in_chunks(start_chip_phase, total_samples);
    let phase_from_seconds = advance_code_phase_seconds(
        start_chip_phase,
        GPS_L1_CA_CODE_RATE_HZ,
        SIXTY_SECONDS,
        CA_CODE_PERIOD_CHIPS,
    )
    .expect("valid elapsed-time phase advance");

    assert_phase_close(
        phase_from_chunks,
        phase_from_seconds,
        "chunked phase accumulation drifted from the theoretical sixty-second phase",
    );

    let block_len = samples_per_code(SAMPLE_RATE_HZ, GPS_L1_CA_CODE_RATE_HZ, CA_CODE_PERIOD_CHIPS);
    let chunked_block = sample_ca_code(
        Prn(11),
        SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        phase_from_chunks,
        block_len,
    )
    .expect("valid sampled code block from chunked phase");
    let theoretical_block = sample_ca_code(
        Prn(11),
        SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        phase_from_seconds,
        block_len,
    )
    .expect("valid sampled code block from theoretical phase");

    assert_eq!(
        chunked_block, theoretical_block,
        "final sampled code block drifted after sixty seconds of chunked advancement"
    );
}

fn advance_in_chunks(start_chip_phase: f64, total_samples: usize) -> f64 {
    let mut phase = start_chip_phase;
    let mut remaining = total_samples;
    let mut chunk_index = 0usize;

    while remaining > 0 {
        let step = CHUNK_PATTERN[chunk_index % CHUNK_PATTERN.len()].min(remaining);
        phase = advance_code_phase_chips(
            phase,
            SAMPLE_RATE_HZ,
            GPS_L1_CA_CODE_RATE_HZ,
            step,
            CA_CODE_PERIOD_CHIPS,
        )
        .expect("valid chunk phase advance");
        remaining -= step;
        chunk_index += 1;
    }

    phase
}

fn assert_phase_close(actual: f64, expected: f64, message: &str) {
    let delta = (actual - expected).abs();
    assert!(
        delta <= PHASE_TOLERANCE_CHIPS,
        "{message}: actual={actual:.12}, expected={expected:.12}, delta={delta:.12}"
    );
}
