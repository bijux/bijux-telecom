#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_noise_only_acquisition_false_alarm_rates, SyntheticAcquisitionFalseAlarmRateCase,
    },
    ReceiverPipelineConfig,
};

const FALSE_ALARM_RATE_TRIAL_COUNT: usize = 24;
const FALSE_ALARM_RATE_THRESHOLD_TRIAL_COUNT: usize = 48;
const FALSE_ALARM_RATE_THRESHOLD: f64 = 0.05;

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
