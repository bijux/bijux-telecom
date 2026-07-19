#![allow(missing_docs)]

use std::f64::consts::TAU;

use bijux_gnss_signal::api::wipeoff_carrier;
use num_complex::Complex;

const SAMPLE_RATE_HZ: f64 = 4_000_000.0;
const CARRIER_HZ: f64 = 12_345.625;
const CARRIER_PHASE_OFFSET_RAD: f64 = 0.375;
const START_SAMPLE_INDEX: u64 = 240_000_000;
const BLOCK_LEN: usize = 4092;
const WIPEOFF_TOLERANCE: f32 = 1e-5;

#[test]
fn carrier_wipeoff_recovers_constant_baseband_after_sixty_seconds() {
    let samples = synthetic_carrier_block(START_SAMPLE_INDEX, BLOCK_LEN);
    let mixed = wipeoff_carrier(
        &samples,
        CARRIER_HZ,
        SAMPLE_RATE_HZ,
        START_SAMPLE_INDEX,
        CARRIER_PHASE_OFFSET_RAD,
    )
    .expect("valid carrier wipeoff");

    for (index, sample) in mixed.iter().enumerate() {
        assert!(
            (sample.re - 1.0).abs() <= WIPEOFF_TOLERANCE,
            "residual I drifted at sample {index}: actual={}",
            sample.re
        );
        assert!(
            sample.im.abs() <= WIPEOFF_TOLERANCE,
            "residual Q drifted at sample {index}: actual={}",
            sample.im
        );
    }
}

#[test]
fn carrier_wipeoff_is_continuous_across_adjacent_long_offset_blocks() {
    let first = wipeoff_carrier(
        &synthetic_carrier_block(START_SAMPLE_INDEX, BLOCK_LEN),
        CARRIER_HZ,
        SAMPLE_RATE_HZ,
        START_SAMPLE_INDEX,
        CARRIER_PHASE_OFFSET_RAD,
    )
    .expect("valid first wipeoff block");
    let second_start = START_SAMPLE_INDEX + BLOCK_LEN as u64;
    let second = wipeoff_carrier(
        &synthetic_carrier_block(second_start, BLOCK_LEN),
        CARRIER_HZ,
        SAMPLE_RATE_HZ,
        second_start,
        CARRIER_PHASE_OFFSET_RAD,
    )
    .expect("valid second wipeoff block");

    assert_eq!(first.len(), second.len());
    for (index, (left, right)) in first.iter().zip(second.iter()).enumerate() {
        assert!(
            (left.re - right.re).abs() <= WIPEOFF_TOLERANCE,
            "adjacent wipeoff I mismatch at sample {index}: left={}, right={}",
            left.re,
            right.re
        );
        assert!(
            (left.im - right.im).abs() <= WIPEOFF_TOLERANCE,
            "adjacent wipeoff Q mismatch at sample {index}: left={}, right={}",
            left.im,
            right.im
        );
    }
}

fn synthetic_carrier_block(start_sample_index: u64, sample_count: usize) -> Vec<Complex<f32>> {
    (0..sample_count)
        .map(|offset| {
            let sample_index = start_sample_index + offset as u64;
            let phase_rad = (CARRIER_PHASE_OFFSET_RAD
                + TAU * CARRIER_HZ * (sample_index as f64 / SAMPLE_RATE_HZ))
                .rem_euclid(TAU);
            Complex::new(phase_rad.cos() as f32, phase_rad.sin() as f32)
        })
        .collect()
}
