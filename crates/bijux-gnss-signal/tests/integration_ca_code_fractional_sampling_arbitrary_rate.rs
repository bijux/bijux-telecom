#![allow(missing_docs)]

use bijux_gnss_signal::api::{
    advance_code_phase_chips, generate_ca_code, sample_ca_code, Prn, CA_CODE_PERIOD_CHIPS,
};

const ARBITRARY_SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;

#[test]
fn sampled_ca_code_matches_analytic_chip_phase_model_at_arbitrary_rate() {
    let start_chip_phase = 137.625;
    let sample_count = 4096;

    for prn in [1_u8, 7, 19, 32] {
        let chips = generate_ca_code(Prn(prn)).expect("valid PRN");
        let samples = sample_ca_code(
            Prn(prn),
            ARBITRARY_SAMPLE_RATE_HZ,
            GPS_L1_CA_CODE_RATE_HZ,
            start_chip_phase,
            sample_count,
        )
        .expect("valid arbitrary-rate sampling");

        for (sample_index, sample) in samples.iter().enumerate() {
            let chip_phase =
                start_chip_phase + sample_index as f64 * GPS_L1_CA_CODE_RATE_HZ / ARBITRARY_SAMPLE_RATE_HZ;
            let wrapped = chip_phase.rem_euclid(CA_CODE_PERIOD_CHIPS as f64);
            let expected = chips[wrapped.floor() as usize] as f32;
            assert!(
                (*sample - expected).abs() < f32::EPSILON,
                "sample mismatch for PRN {prn} at sample {sample_index}"
            );
        }
    }
}

#[test]
fn sampled_ca_code_preserves_phase_continuity_across_chunks() {
    let start_chip_phase = 412.375;
    let first_chunk_len = 1733;
    let second_chunk_len = 2459;
    let total_len = first_chunk_len + second_chunk_len;

    let full = sample_ca_code(
        Prn(11),
        ARBITRARY_SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        start_chip_phase,
        total_len,
    )
    .expect("valid full-block sampling");

    let first = sample_ca_code(
        Prn(11),
        ARBITRARY_SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        start_chip_phase,
        first_chunk_len,
    )
    .expect("valid first chunk");
    let second_start = advance_code_phase_chips(
        start_chip_phase,
        ARBITRARY_SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        first_chunk_len,
        CA_CODE_PERIOD_CHIPS,
    )
    .expect("valid phase advance");
    let second = sample_ca_code(
        Prn(11),
        ARBITRARY_SAMPLE_RATE_HZ,
        GPS_L1_CA_CODE_RATE_HZ,
        second_start,
        second_chunk_len,
    )
    .expect("valid second chunk");

    let stitched = first.into_iter().chain(second).collect::<Vec<_>>();
    assert_eq!(stitched, full, "chunked sampling drifted from full-block sampling");
}
