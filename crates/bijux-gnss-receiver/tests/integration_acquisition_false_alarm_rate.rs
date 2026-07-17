#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_noise_only_acquisition_false_alarm_rates, SyntheticAcquisitionFalseAlarmRateCase,
    },
    AcquisitionThresholdMode, ReceiverPipelineConfig,
};

const FALSE_ALARM_RATE_TRIAL_COUNT: usize = 24;
const FALSE_ALARM_RATE_THRESHOLD_TRIAL_COUNT: usize = 32;
const FALSE_ALARM_RATE_THRESHOLD: f64 = 0.05;
const CALIBRATED_FALSE_ALARM_TARGET: f64 = 0.01;
const CALIBRATED_FALSE_ALARM_CALIBRATION_TRIAL_COUNT: usize = 24;
const CALIBRATED_FALSE_ALARM_HELD_OUT_TRIAL_COUNT: usize = 32;
const CALIBRATED_FALSE_ALARM_CONFIDENCE_LEVEL: f64 = 0.95;

#[test]
fn acquisition_false_alarm_rate_report_runs_multiple_measurement_points() {
    let config = acquisition_profile();
    let report = measure_noise_only_acquisition_false_alarm_rates(
        &config,
        &[false_alarm_rate_case(1, 1), false_alarm_rate_case(5, 4)],
        &trial_seeds(0x2407_1993, FALSE_ALARM_RATE_TRIAL_COUNT),
        "acquisition_false_alarm_rate_smoke",
    );

    assert_eq!(report.points.len(), 2);
    assert_eq!(report.points[0].trial_count, FALSE_ALARM_RATE_TRIAL_COUNT);
    assert_eq!(report.points[1].trial_count, FALSE_ALARM_RATE_TRIAL_COUNT);
    assert_eq!(report.points[0].coherent_ms, 1);
    assert_eq!(report.points[1].noncoherent, 4);
}

#[test]
fn acquisition_false_alarm_rate_reports_coherent_integration_profiles() {
    let config = acquisition_profile();
    let report = measure_noise_only_acquisition_false_alarm_rates(
        &config,
        &[
            false_alarm_rate_case(1, 1),
            false_alarm_rate_case(2, 1),
            false_alarm_rate_case(5, 1),
            false_alarm_rate_case(10, 1),
            false_alarm_rate_case(20, 1),
        ],
        &trial_seeds(0x2407_1994, FALSE_ALARM_RATE_TRIAL_COUNT),
        "acquisition_false_alarm_rate_coherent",
    );

    assert_eq!(report.points.len(), 5);
    assert_eq!(
        report.points.iter().map(|point| point.coherent_ms).collect::<Vec<_>>(),
        vec![1, 2, 5, 10, 20]
    );
    assert!(
        report.points.iter().all(|point| point.noncoherent == 1
            && point.false_alarm_count <= point.trial_count
            && point.false_alarm_rate.is_finite()
            && (0.0..=1.0).contains(&point.false_alarm_rate)),
        "{report:?}"
    );
}

#[test]
fn acquisition_false_alarm_rate_reports_noncoherent_profiles() {
    let config = acquisition_profile();
    let report = measure_noise_only_acquisition_false_alarm_rates(
        &config,
        &[
            false_alarm_rate_case(1, 1),
            false_alarm_rate_case(1, 2),
            false_alarm_rate_case(1, 4),
            false_alarm_rate_case(1, 8),
        ],
        &trial_seeds(0x2407_1995, FALSE_ALARM_RATE_TRIAL_COUNT),
        "acquisition_false_alarm_rate_noncoherent",
    );

    assert_eq!(report.points.len(), 4);
    assert_eq!(
        report.points.iter().map(|point| point.noncoherent).collect::<Vec<_>>(),
        vec![1, 2, 4, 8]
    );
    assert!(
        report.points.iter().all(|point| point.coherent_ms == 1
            && point.false_alarm_count <= point.trial_count
            && point.false_alarm_rate.is_finite()
            && (0.0..=1.0).contains(&point.false_alarm_rate)),
        "{report:?}"
    );
}

#[test]
fn acquisition_noise_only_false_alarm_rate_stays_below_threshold() {
    let config = acquisition_profile();
    let report = measure_noise_only_acquisition_false_alarm_rates(
        &config,
        &[
            false_alarm_rate_case(1, 1),
            false_alarm_rate_case(2, 1),
            false_alarm_rate_case(5, 1),
            false_alarm_rate_case(10, 1),
            false_alarm_rate_case(20, 1),
            false_alarm_rate_case(1, 2),
            false_alarm_rate_case(1, 4),
            false_alarm_rate_case(1, 8),
        ],
        &trial_seeds(0x2407_1996, FALSE_ALARM_RATE_THRESHOLD_TRIAL_COUNT),
        "acquisition_false_alarm_rate_threshold",
    );

    assert!(
        report
            .points
            .iter()
            .all(|point| point.false_alarm_rate <= FALSE_ALARM_RATE_THRESHOLD),
        "expected every noise-only profile to stay at or below the false-alarm threshold: {report:?}"
    );
}

#[test]
fn acquisition_calibrated_false_alarm_rate_matches_target_within_confidence_interval() {
    let mut config = acquisition_profile();
    config.acquisition_threshold_policy.mode = AcquisitionThresholdMode::CalibratedFalseAlarm;
    config.acquisition_threshold_policy.false_alarm_probability = CALIBRATED_FALSE_ALARM_TARGET;
    config.acquisition_threshold_policy.calibration_trial_count =
        CALIBRATED_FALSE_ALARM_CALIBRATION_TRIAL_COUNT;
    config.acquisition_threshold_policy.confidence_level = CALIBRATED_FALSE_ALARM_CONFIDENCE_LEVEL;

    let report = measure_noise_only_acquisition_false_alarm_rates(
        &config,
        &[false_alarm_rate_case(1, 1)],
        &trial_seeds(0x2407_1997, CALIBRATED_FALSE_ALARM_HELD_OUT_TRIAL_COUNT),
        "acquisition_false_alarm_rate_calibrated",
    );

    assert_eq!(report.points.len(), 1);
    for point in &report.points {
        let (interval_low, interval_high) = wilson_confidence_interval(
            point.false_alarm_count,
            point.trial_count,
            CALIBRATED_FALSE_ALARM_CONFIDENCE_LEVEL,
        );
        assert!(
            interval_low <= CALIBRATED_FALSE_ALARM_TARGET
                && CALIBRATED_FALSE_ALARM_TARGET <= interval_high,
            "expected calibrated target {:.6} to lie within [{:.6}, {:.6}] for coherent_ms={} noncoherent={}: {:?}",
            CALIBRATED_FALSE_ALARM_TARGET,
            interval_low,
            interval_high,
            point.coherent_ms,
            point.noncoherent,
            point
        );
    }
}

fn acquisition_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    }
}

fn false_alarm_rate_case(
    coherent_ms: u32,
    noncoherent: u32,
) -> SyntheticAcquisitionFalseAlarmRateCase {
    SyntheticAcquisitionFalseAlarmRateCase {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        coherent_ms,
        noncoherent,
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}

fn wilson_confidence_interval(
    false_alarm_count: usize,
    trial_count: usize,
    confidence_level: f64,
) -> (f64, f64) {
    if trial_count == 0 {
        return (0.0, 0.0);
    }
    let p_hat = false_alarm_count as f64 / trial_count as f64;
    let z = inverse_standard_normal_cdf(0.5 + (confidence_level * 0.5));
    let n = trial_count as f64;
    let z2 = z * z;
    let denominator = 1.0 + (z2 / n);
    let center = (p_hat + (z2 / (2.0 * n))) / denominator;
    let margin = (z * ((p_hat * (1.0 - p_hat) / n) + (z2 / (4.0 * n * n))).sqrt()) / denominator;
    ((center - margin).max(0.0), (center + margin).min(1.0))
}

fn inverse_standard_normal_cdf(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const LOW: f64 = 0.02425;
    const HIGH: f64 = 1.0 - LOW;

    if p < LOW {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    if p > HIGH {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    let q = p - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}
