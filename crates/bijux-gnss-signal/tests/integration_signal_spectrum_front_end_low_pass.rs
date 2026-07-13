mod support;

use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode, SignalComponentRole};
use bijux_gnss_signal::api::{
    sample_ca_code, sample_galileo_e5a_i_primary_code, signal_registry, FrontEndFilterSpec, Prn,
};
use num_complex::Complex;

use support::spectrum_validation::{
    assert_spectrum_matches, compare_component_spectrum, compare_filtered_component_spectrum,
    default_estimator_config, SpectrumTolerance,
};

const OCCUPIED_POWER_FRACTION: f64 = 0.99;
const NULL_THRESHOLD_DB: f64 = -24.0;
const SAMPLE_COUNT: usize = 32_768;

fn oversampled_chip_rate(chip_rate_hz: f64) -> f64 {
    chip_rate_hz * 16.0
}

fn apply_filter(
    samples: &[f32],
    filter_spec: FrontEndFilterSpec,
    sample_rate_hz: f64,
) -> (Vec<Complex<f32>>, bijux_gnss_signal::api::FrontEndFirFilter) {
    let mut filtered = samples.iter().map(|sample| Complex::new(*sample, 0.0)).collect::<Vec<_>>();
    let mut filter = filter_spec.design(sample_rate_hz).expect("front-end filter");
    filter.apply_in_place(&mut filtered);
    (filtered, filter)
}

#[test]
fn gps_l1_ca_low_pass_spectrum_matches_filter_shaped_expectation() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca)
        .expect("GPS L1 C/A registry entry");
    let component = entry.default_component().copied().expect("GPS L1 C/A component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_ca_code(Prn(3), sample_rate_hz, component.primary_code_rate_hz, 0.0, SAMPLE_COUNT)
            .expect("GPS L1 C/A samples");
    let unfiltered = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    let (filtered_samples, filter) = apply_filter(
        &samples,
        FrontEndFilterSpec::LowPass { cutoff_hz: 650_000.0, taps: 81 },
        sample_rate_hz,
    );
    let filtered = compare_filtered_component_spectrum(
        &filtered_samples,
        sample_rate_hz,
        component,
        &filter,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );

    assert_spectrum_matches(&filtered, SpectrumTolerance::filtered_low_pass(), 1);
    assert!(
        filtered.measured_summary.occupied_bandwidth_hz
            < unfiltered.measured_summary.occupied_bandwidth_hz,
        "filtered={:?} unfiltered={:?}",
        filtered.measured_summary,
        unfiltered.measured_summary
    );
}

#[test]
fn galileo_e5a_data_low_pass_spectrum_matches_filter_shaped_expectation() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        .expect("Galileo E5a registry entry");
    let component =
        entry.component(SignalComponentRole::Data).copied().expect("Galileo E5a data component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples = sample_galileo_e5a_i_primary_code(7, sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("Galileo E5a-I samples");
    let unfiltered = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    let (filtered_samples, filter) = apply_filter(
        &samples,
        FrontEndFilterSpec::LowPass { cutoff_hz: 6_500_000.0, taps: 81 },
        sample_rate_hz,
    );
    let filtered = compare_filtered_component_spectrum(
        &filtered_samples,
        sample_rate_hz,
        component,
        &filter,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );

    assert_spectrum_matches(&filtered, SpectrumTolerance::filtered_low_pass(), 1);
    assert!(
        filtered.measured_summary.occupied_bandwidth_hz
            < unfiltered.measured_summary.occupied_bandwidth_hz,
        "filtered={:?} unfiltered={:?}",
        filtered.measured_summary,
        unfiltered.measured_summary
    );
}
