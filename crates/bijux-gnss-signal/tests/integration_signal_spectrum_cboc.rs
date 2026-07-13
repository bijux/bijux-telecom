mod support;

use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode};
use bijux_gnss_signal::api::{sample_galileo_e1_cboc, signal_registry, PowerSpectralDensityPoint};

use support::spectrum_validation::{
    assert_spectrum_matches, compare_component_spectrum, default_estimator_config,
    SpectrumTolerance,
};

const OCCUPIED_POWER_FRACTION: f64 = 0.99;
const NULL_THRESHOLD_DB: f64 = -24.0;
const SAMPLE_COUNT: usize = 49_152;

fn cboc_sample_rate_hz(chip_rate_hz: f64) -> f64 {
    chip_rate_hz * 48.0
}

#[test]
fn galileo_e1_cboc_spectrum_matches_expected_shape() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
        .expect("Galileo E1B registry entry");
    let component = entry.default_component().copied().expect("Galileo E1B component");
    let sample_rate_hz = cboc_sample_rate_hz(component.primary_code_rate_hz);
    let samples = sample_galileo_e1_cboc(11, sample_rate_hz, 0.0, SAMPLE_COUNT, 0, 1)
        .expect("Galileo E1 CBOC samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&comparison, SpectrumTolerance::cboc(), 1);
}

#[test]
fn galileo_e1_cboc_has_deep_baseband_notch_and_balanced_sidebands() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1C)
        .expect("Galileo E1C registry entry");
    let component = entry.default_component().copied().expect("Galileo E1C component");
    let sample_rate_hz = cboc_sample_rate_hz(component.primary_code_rate_hz);
    let samples = sample_galileo_e1_cboc(11, sample_rate_hz, 0.0, SAMPLE_COUNT, 0, 1)
        .expect("Galileo E1 CBOC samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    let dc_point = nearest_frequency_point(&comparison.measured_points, 0.0);
    let positive_lobe = nearest_frequency_point(&comparison.measured_points, 1_023_000.0);
    let negative_lobe = nearest_frequency_point(&comparison.measured_points, -1_023_000.0);
    let peak_power =
        comparison.measured_points.iter().map(|point| point.power_density).fold(0.0_f64, f64::max);

    assert!(dc_point.power_density <= peak_power * 1.0e-4, "{dc_point:?} peak={peak_power}");
    assert!(
        (positive_lobe.power_density - negative_lobe.power_density).abs() / peak_power <= 0.02,
        "positive_lobe={positive_lobe:?} negative_lobe={negative_lobe:?} peak={peak_power}"
    );
}

fn nearest_frequency_point(
    points: &[PowerSpectralDensityPoint],
    target_frequency_hz: f64,
) -> PowerSpectralDensityPoint {
    points
        .iter()
        .copied()
        .min_by(|left, right| {
            (left.frequency_hz - target_frequency_hz)
                .abs()
                .partial_cmp(&(right.frequency_hz - target_frequency_hz).abs())
                .expect("finite frequency distance")
        })
        .expect("PSD point")
}
