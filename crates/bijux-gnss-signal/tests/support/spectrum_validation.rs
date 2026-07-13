#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::SignalComponentSpec;
use bijux_gnss_signal::api::{
    estimate_power_spectral_density, expected_component_power_spectral_density,
    find_deep_spectrum_nulls, summarize_power_spectral_density, PowerSpectralDensityPoint,
    PowerSpectralDensitySummary, SpectrumEstimatorConfig, SpectrumNull,
};

const MIN_SHAPE_POWER_RATIO: f64 = 1.0e-4;
const MIN_NULL_FREQUENCY_HZ: f64 = 1.0;

#[derive(Debug)]
pub struct SpectrumComparison {
    pub expected_points: Vec<PowerSpectralDensityPoint>,
    pub measured_points: Vec<PowerSpectralDensityPoint>,
    pub expected_summary: PowerSpectralDensitySummary,
    pub measured_summary: PowerSpectralDensitySummary,
    pub expected_nulls: Vec<SpectrumNull>,
    pub measured_nulls: Vec<SpectrumNull>,
    pub shape_rms_error_db: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SpectrumTolerance {
    pub center_frequency_hz: f64,
    pub occupied_bandwidth_ratio: f64,
    pub integrated_power: f64,
    pub symmetry_error: f64,
    pub null_frequency_hz: f64,
    pub shape_rms_error_db: f64,
}

impl SpectrumTolerance {
    pub const fn tight_bpsk() -> Self {
        Self {
            center_frequency_hz: 8_000.0,
            occupied_bandwidth_ratio: 0.25,
            integrated_power: 0.08,
            symmetry_error: 0.03,
            null_frequency_hz: 25_000.0,
            shape_rms_error_db: 2.2,
        }
    }

    pub const fn cboc() -> Self {
        Self {
            center_frequency_hz: 10_000.0,
            occupied_bandwidth_ratio: 0.30,
            integrated_power: 0.08,
            symmetry_error: 0.04,
            null_frequency_hz: 40_000.0,
            shape_rms_error_db: 2.0,
        }
    }
}

pub fn compare_component_spectrum(
    samples: &[f32],
    sample_rate_hz: f64,
    component: SignalComponentSpec,
    estimator_config: SpectrumEstimatorConfig,
    occupied_power_fraction: f64,
    null_threshold_db: f64,
) -> SpectrumComparison {
    let measured_points =
        estimate_power_spectral_density(samples, sample_rate_hz, estimator_config)
            .expect("measured power spectral density");
    let frequencies_hz = measured_points.iter().map(|point| point.frequency_hz).collect::<Vec<_>>();
    let expected_points = expected_component_power_spectral_density(component, &frequencies_hz)
        .expect("expected power spectral density");
    let measured_summary =
        summarize_power_spectral_density(&measured_points, occupied_power_fraction)
            .expect("measured PSD summary");
    let expected_summary =
        summarize_power_spectral_density(&expected_points, occupied_power_fraction)
            .expect("expected PSD summary");
    let measured_nulls = positive_nulls(
        &find_deep_spectrum_nulls(&measured_points, null_threshold_db).expect("measured nulls"),
    );
    let expected_nulls = positive_nulls(
        &find_deep_spectrum_nulls(&expected_points, null_threshold_db).expect("expected nulls"),
    );
    let shape_rms_error_db = spectrum_shape_rms_error_db(&expected_points, &measured_points);

    SpectrumComparison {
        expected_points,
        measured_points,
        expected_summary,
        measured_summary,
        expected_nulls,
        measured_nulls,
        shape_rms_error_db,
    }
}

pub fn assert_spectrum_matches(
    comparison: &SpectrumComparison,
    tolerance: SpectrumTolerance,
    minimum_positive_nulls: usize,
) {
    assert!(
        (comparison.measured_summary.center_frequency_hz
            - comparison.expected_summary.center_frequency_hz)
            .abs()
            <= tolerance.center_frequency_hz,
        "center frequency mismatch: measured={:?} expected={:?}",
        comparison.measured_summary,
        comparison.expected_summary
    );
    assert!(
        (comparison.measured_summary.occupied_bandwidth_hz
            - comparison.expected_summary.occupied_bandwidth_hz)
            .abs()
            / comparison.expected_summary.occupied_bandwidth_hz
            <= tolerance.occupied_bandwidth_ratio,
        "occupied bandwidth mismatch: measured={:?} expected={:?}",
        comparison.measured_summary,
        comparison.expected_summary
    );
    assert!(
        (comparison.measured_summary.integrated_power
            - comparison.expected_summary.integrated_power)
            .abs()
            <= tolerance.integrated_power,
        "integrated power mismatch: measured={:?} expected={:?}",
        comparison.measured_summary,
        comparison.expected_summary
    );
    assert!(
        comparison.measured_summary.symmetry_error <= tolerance.symmetry_error,
        "symmetry error too large: measured={:?}",
        comparison.measured_summary
    );
    assert!(
        comparison.shape_rms_error_db <= tolerance.shape_rms_error_db,
        "shape RMS error too large: comparison={comparison:?}"
    );
    assert!(
        comparison.expected_nulls.len() >= minimum_positive_nulls,
        "expected at least {minimum_positive_nulls} positive nulls, got {:?}",
        comparison.expected_nulls
    );
    assert!(
        comparison.measured_nulls.len() >= minimum_positive_nulls,
        "measured at least {minimum_positive_nulls} positive nulls, got {:?}",
        comparison.measured_nulls
    );

    for expected in comparison.expected_nulls.iter().take(minimum_positive_nulls) {
        let measured = comparison
            .measured_nulls
            .iter()
            .min_by(|left, right| {
                (left.frequency_hz - expected.frequency_hz)
                    .abs()
                    .partial_cmp(&(right.frequency_hz - expected.frequency_hz).abs())
                    .expect("finite null frequency distance")
            })
            .expect("measured null");
        assert!(
            (measured.frequency_hz - expected.frequency_hz).abs() <= tolerance.null_frequency_hz,
            "null frequency mismatch: expected={expected:?} measured={measured:?}"
        );
    }
}

pub fn default_estimator_config() -> SpectrumEstimatorConfig {
    SpectrumEstimatorConfig { segment_len: 4096, overlap_len: 2048 }
}

fn positive_nulls(nulls: &[SpectrumNull]) -> Vec<SpectrumNull> {
    nulls.iter().copied().filter(|null| null.frequency_hz >= MIN_NULL_FREQUENCY_HZ).collect()
}

fn spectrum_shape_rms_error_db(
    expected_points: &[PowerSpectralDensityPoint],
    measured_points: &[PowerSpectralDensityPoint],
) -> f64 {
    assert_eq!(expected_points.len(), measured_points.len(), "spectrum point count mismatch");
    let peak_expected_power =
        expected_points.iter().map(|point| point.power_density).fold(0.0_f64, f64::max);
    let minimum_expected_power = peak_expected_power * MIN_SHAPE_POWER_RATIO;

    let mut squared_error_sum = 0.0_f64;
    let mut error_count = 0usize;
    for (expected, measured) in expected_points.iter().zip(measured_points.iter()) {
        if expected.power_density < minimum_expected_power {
            continue;
        }
        let expected_db = 10.0 * expected.power_density.max(minimum_expected_power).log10();
        let measured_db = 10.0 * measured.power_density.max(minimum_expected_power).log10();
        squared_error_sum += (measured_db - expected_db).powi(2);
        error_count += 1;
    }

    assert!(error_count > 0, "no spectrum bins exceeded the shape-comparison threshold");
    (squared_error_sum / error_count as f64).sqrt()
}
