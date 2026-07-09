#![allow(missing_docs)]

use std::f64::consts::TAU;

use bijux_gnss_signal::api::Nco;

const SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const CARRIER_HZ: f64 = -12_345.625;
const PHASE_OFFSET_RAD: f64 = 0.375;
const SIXTY_SECONDS: f64 = 60.0;
const SAMPLE_BLOCK_LEN: usize = 4092;
const PHASE_TOLERANCE_RAD: f64 = 1e-12;
const SAMPLE_TOLERANCE: f64 = 1e-12;

#[test]
fn nco_from_sample_index_matches_analytic_phase_after_sixty_seconds() {
    let start_sample_index = (SIXTY_SECONDS * SAMPLE_RATE_HZ) as u64;
    let expected_phase_rad = analytic_phase_rad(start_sample_index);
    let nco =
        Nco::from_sample_index(CARRIER_HZ, SAMPLE_RATE_HZ, start_sample_index, PHASE_OFFSET_RAD);

    assert_phase_close(nco.phase_rad(), expected_phase_rad, "start phase drifted at sixty seconds");
}

#[test]
fn nco_outputs_match_analytic_carrier_block_after_sixty_seconds() {
    let start_sample_index = (SIXTY_SECONDS * SAMPLE_RATE_HZ) as u64;
    let mut nco =
        Nco::from_sample_index(CARRIER_HZ, SAMPLE_RATE_HZ, start_sample_index, PHASE_OFFSET_RAD);

    for sample_offset in 0..SAMPLE_BLOCK_LEN {
        let absolute_sample_index = start_sample_index + sample_offset as u64;
        let expected_phase_rad = analytic_phase_rad(absolute_sample_index);
        let (sin, cos) = nco.next_sin_cos();
        assert!(
            (sin - expected_phase_rad.sin()).abs() <= SAMPLE_TOLERANCE,
            "sin drifted at sample {absolute_sample_index}: actual={sin:.15}, expected={:.15}",
            expected_phase_rad.sin()
        );
        assert!(
            (cos - expected_phase_rad.cos()).abs() <= SAMPLE_TOLERANCE,
            "cos drifted at sample {absolute_sample_index}: actual={cos:.15}, expected={:.15}",
            expected_phase_rad.cos()
        );
    }
}

fn analytic_phase_rad(sample_index: u64) -> f64 {
    let elapsed_seconds = sample_index as f64 / SAMPLE_RATE_HZ;
    (PHASE_OFFSET_RAD + TAU * CARRIER_HZ * elapsed_seconds).rem_euclid(TAU)
}

fn assert_phase_close(actual: f64, expected: f64, message: &str) {
    let delta = (actual - expected).abs();
    assert!(
        delta <= PHASE_TOLERANCE_RAD,
        "{message}: actual={actual:.15}, expected={expected:.15}, delta={delta:.15}"
    );
}
