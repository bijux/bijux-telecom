mod support;

use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode};
use bijux_gnss_signal::api::{
    sample_ca_code, sample_galileo_e1_cboc, signal_registry, FrontEndFilterSpec, Prn,
};
use num_complex::Complex;

use support::spectrum_validation::{
    assert_spectrum_matches, compare_filtered_component_spectrum, default_estimator_config,
    SpectrumTolerance,
};

const OCCUPIED_POWER_FRACTION: f64 = 0.99;
const NULL_THRESHOLD_DB: f64 = -20.0;
const SAMPLE_COUNT: usize = 49_152;

fn oversampled_chip_rate(chip_rate_hz: f64) -> f64 {
    chip_rate_hz * 16.0
}

fn cboc_sample_rate_hz(chip_rate_hz: f64) -> f64 {
    chip_rate_hz * 48.0
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
fn gps_l1_ca_band_pass_spectrum_matches_filter_shaped_expectation() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca)
        .expect("GPS L1 C/A registry entry");
    let component = entry.default_component().copied().expect("GPS L1 C/A component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_ca_code(Prn(5), sample_rate_hz, component.primary_code_rate_hz, 0.0, SAMPLE_COUNT)
            .expect("GPS L1 C/A samples");
    let (filtered_samples, filter) = apply_filter(
        &samples,
        FrontEndFilterSpec::BandPass {
            center_hz: 700_000.0,
            bandwidth_hz: 900_000.0,
            taps: 81,
        },
        sample_rate_hz,
    );
    let comparison = compare_filtered_component_spectrum(
        &filtered_samples,
        sample_rate_hz,
        component,
        &filter,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );

    assert_spectrum_matches(&comparison, SpectrumTolerance::filtered_band_pass(), 0);
    assert!(comparison.measured_summary.center_frequency_hz >= 450_000.0, "{comparison:#?}");
}

#[test]
fn galileo_e1_cboc_band_pass_spectrum_matches_filter_shaped_expectation() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E1, SignalCode::E1C)
        .expect("Galileo E1C registry entry");
    let component = entry.default_component().copied().expect("Galileo E1C component");
    let sample_rate_hz = cboc_sample_rate_hz(component.primary_code_rate_hz);
    let samples = sample_galileo_e1_cboc(11, sample_rate_hz, 0.0, SAMPLE_COUNT, 0, 1)
        .expect("Galileo E1 CBOC samples");
    let (filtered_samples, filter) = apply_filter(
        &samples,
        FrontEndFilterSpec::BandPass {
            center_hz: 1_023_000.0,
            bandwidth_hz: 1_600_000.0,
            taps: 97,
        },
        sample_rate_hz,
    );
    let comparison = compare_filtered_component_spectrum(
        &filtered_samples,
        sample_rate_hz,
        component,
        &filter,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );

    assert_spectrum_matches(&comparison, SpectrumTolerance::filtered_band_pass(), 0);
    assert!(comparison.measured_summary.center_frequency_hz >= 800_000.0, "{comparison:#?}");
}
