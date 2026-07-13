#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_truth_guided_acquisition_detection_rate, SyntheticAcquisitionDetectionRateCase,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

const DETECTION_RATE_TRIAL_COUNT: usize = 24;

#[test]
fn acquisition_detection_rate_report_runs_multiple_measurement_points() {
    let config = acquisition_profile();
    let report = measure_truth_guided_acquisition_detection_rate(
        &config,
        &[detection_rate_case(30.0, 250.0, 1, 1), detection_rate_case(34.0, 250.0, 5, 1)],
        &trial_seeds(0x2407_1989, DETECTION_RATE_TRIAL_COUNT),
        "acquisition_detection_rate_smoke",
        2,
        1,
    );

    assert_eq!(report.points.len(), 2);
    assert_eq!(report.points[0].trial_count, DETECTION_RATE_TRIAL_COUNT);
    assert_eq!(report.points[1].trial_count, DETECTION_RATE_TRIAL_COUNT);
    assert_eq!(report.points[0].doppler_hz, 250.0);
    assert_eq!(report.points[1].coherent_ms, 5);
}

#[test]
fn acquisition_detection_rate_reports_cn0_sensitivity() {
    let config = acquisition_profile();
    let report = measure_truth_guided_acquisition_detection_rate(
        &config,
        &[
            detection_rate_case(26.0, 250.0, 1, 1),
            detection_rate_case(30.0, 250.0, 1, 1),
            detection_rate_case(34.0, 250.0, 1, 1),
            detection_rate_case(38.0, 250.0, 1, 1),
        ],
        &trial_seeds(0x2407_1990, DETECTION_RATE_TRIAL_COUNT),
        "acquisition_detection_rate_cn0",
        2,
        1,
    );

    assert_eq!(report.points.len(), 4);
    assert!(
        report.points.last().expect("strongest cn0").detection_probability
            > report.points.first().expect("weakest cn0").detection_probability,
        "expected detection probability to improve across the C/N0 sweep: {report:?}"
    );
}

#[test]
fn acquisition_detection_rate_reports_doppler_sensitivity() {
    let config = acquisition_profile();
    let report = measure_truth_guided_acquisition_detection_rate(
        &config,
        &[
            detection_rate_case(34.0, 0.0, 20, 1),
            detection_rate_case(34.0, 125.0, 20, 1),
            detection_rate_case(34.0, 250.0, 20, 1),
            detection_rate_case(34.0, 375.0, 20, 1),
        ],
        &trial_seeds(0x2407_1991, DETECTION_RATE_TRIAL_COUNT),
        "acquisition_detection_rate_doppler",
        2,
        1,
    );

    let aligned_detection_probability =
        (report.points[0].detection_probability + report.points[2].detection_probability) / 2.0;
    let half_bin_detection_probability =
        (report.points[1].detection_probability + report.points[3].detection_probability) / 2.0;

    assert_eq!(report.points.len(), 4);
    assert_eq!(
        report.points.iter().map(|point| point.doppler_hz).collect::<Vec<_>>(),
        vec![0.0, 125.0, 250.0, 375.0]
    );
    assert!(
        aligned_detection_probability > half_bin_detection_probability,
        "expected exact-bin Doppler hypotheses to outperform half-bin offsets: {report:?}"
    );
}

#[test]
fn acquisition_detection_rate_reports_integration_length_sensitivity() {
    let config = acquisition_profile();
    let report = measure_truth_guided_acquisition_detection_rate(
        &config,
        &[
            detection_rate_case(30.0, 250.0, 1, 1),
            detection_rate_case(30.0, 250.0, 2, 1),
            detection_rate_case(30.0, 250.0, 5, 1),
            detection_rate_case(30.0, 250.0, 10, 1),
            detection_rate_case(30.0, 250.0, 20, 1),
        ],
        &trial_seeds(0x2407_1992, DETECTION_RATE_TRIAL_COUNT),
        "acquisition_detection_rate_integration_length",
        2,
        1,
    );

    assert_eq!(report.points.len(), 5);
    assert_eq!(
        report.points.iter().map(|point| point.coherent_ms).collect::<Vec<_>>(),
        vec![1, 2, 5, 10, 20]
    );
    assert!(
        report.points.last().expect("longest integration").detection_probability
            > report.points.first().expect("shortest integration").detection_probability,
        "expected longer coherent integration to improve detection probability: {report:?}"
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

fn detection_rate_case(
    cn0_db_hz: f32,
    doppler_hz: f64,
    coherent_ms: u32,
    noncoherent: u32,
) -> SyntheticAcquisitionDetectionRateCase {
    SyntheticAcquisitionDetectionRateCase {
        signal: SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            doppler_hz,
            code_phase_chips: 300.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            data_bit_flip: false,
        },
        coherent_ms,
        noncoherent,
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}
