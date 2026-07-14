#![allow(missing_docs)]

use std::collections::BTreeSet;

use bijux_gnss_receiver::api::{
    sim::{
        measure_tracking_lock_detector_calibration, SyntheticTrackingLockDetectorCalibrationCase,
    },
    ReceiverPipelineConfig,
};

#[test]
fn tracking_lock_detector_calibration_reports_probabilities_over_cn0_and_dynamics() {
    let report = measure_tracking_lock_detector_calibration(
        &tracking_calibration_profile(),
        &[
            lock_detector_case(34.0, 0.0),
            lock_detector_case(44.0, 0.0),
            lock_detector_case(44.0, 40.0),
            lock_detector_case(54.0, 80.0),
        ],
        "tracking_lock_detector_calibration",
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.point_count, 4);
    assert_eq!(report.points.len(), report.point_count);
    assert!(report.points.iter().all(|point| {
        probability_is_valid(point.false_unlock_probability)
            && probability_is_valid(point.false_lock_probability)
            && probability_is_valid(point.missed_unlock_probability)
            && probability_is_valid(point.dll_false_unlock_probability)
            && probability_is_valid(point.pll_false_unlock_probability)
            && probability_is_valid(point.fll_false_unlock_probability)
            && probability_is_valid(point.dll_false_lock_probability)
            && probability_is_valid(point.pll_false_lock_probability)
            && probability_is_valid(point.fll_false_lock_probability)
    }));

    let cn0_levels = report
        .points
        .iter()
        .map(|point| (point.cn0_db_hz * 10.0).round() as i64)
        .collect::<BTreeSet<_>>();
    let dynamic_levels = report
        .points
        .iter()
        .map(|point| point.dynamic_stress_hz.round() as i64)
        .collect::<BTreeSet<_>>();
    assert!(cn0_levels.len() >= 3, "{report:?}");
    assert!(dynamic_levels.len() >= 3, "{report:?}");

    let calm = report
        .points
        .iter()
        .find(|point| point.cn0_db_hz == 44.0 && point.dynamic_stress_hz == 0.0)
        .expect("calm point");
    let dynamic = report
        .points
        .iter()
        .find(|point| point.cn0_db_hz == 44.0 && point.dynamic_stress_hz == 40.0)
        .expect("dynamic point");
    assert!(dynamic.fll_lock_threshold_hz > calm.fll_lock_threshold_hz, "{report:?}");
    assert!(dynamic.fll_false_lock_probability > calm.fll_false_lock_probability, "{report:?}");
}

#[test]
fn tracking_lock_detector_calibration_tightens_thresholds_with_stronger_cn0() {
    let report = measure_tracking_lock_detector_calibration(
        &tracking_calibration_profile(),
        &[lock_detector_case(34.0, 0.0), lock_detector_case(54.0, 0.0)],
        "tracking_lock_detector_cn0_thresholds",
    );

    assert!(report.pass, "{report:?}");
    let weak = &report.points[0];
    let strong = &report.points[1];
    assert!(strong.coherent_snr_linear > weak.coherent_snr_linear, "{report:?}");
    assert!(strong.dll_lock_threshold < weak.dll_lock_threshold, "{report:?}");
    assert!(strong.pll_lock_threshold_rad < weak.pll_lock_threshold_rad, "{report:?}");
    assert!(strong.fll_lock_threshold_hz < weak.fll_lock_threshold_hz, "{report:?}");
    assert!(strong.false_lock_probability < weak.false_lock_probability, "{report:?}");
}

fn tracking_calibration_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        early_late_spacing_chips: 0.5,
        tracking_integration_ms: 1,
        ..ReceiverPipelineConfig::default()
    }
}

fn lock_detector_case(
    cn0_db_hz: f64,
    dynamic_stress_hz: f64,
) -> SyntheticTrackingLockDetectorCalibrationCase {
    SyntheticTrackingLockDetectorCalibrationCase {
        cn0_db_hz,
        coherent_ms: 1,
        early_late_spacing_chips: 0.5,
        dynamic_stress_hz,
        unlocked_fll_half_width_hz: 1_000.0,
        missed_unlock_bias_sigma: 4.0,
    }
}

fn probability_is_valid(probability: f64) -> bool {
    probability.is_finite() && (0.0..=1.0).contains(&probability)
}
