mod support;

use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode, SignalComponentRole};
use bijux_gnss_signal::api::{
    sample_beidou_b1i_code, sample_beidou_b2i_code, sample_ca_code, sample_glonass_l1_st_code,
    sample_gps_l2c_cl_code, sample_gps_l2c_cm_code, signal_registry, Prn,
};

use support::spectrum_validation::{
    assert_spectrum_matches, compare_component_spectrum, default_estimator_config,
    SpectrumTolerance,
};

const OCCUPIED_POWER_FRACTION: f64 = 0.99;
const NULL_THRESHOLD_DB: f64 = -28.0;
const SAMPLE_COUNT: usize = 32_768;

fn oversampled_chip_rate(chip_rate_hz: f64) -> f64 {
    chip_rate_hz * 16.0
}

#[test]
fn gps_l1_ca_spectrum_matches_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L1, SignalCode::Ca)
        .expect("GPS L1 C/A registry entry");
    let component = entry.default_component().copied().expect("GPS L1 C/A component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_ca_code(Prn(1), sample_rate_hz, component.primary_code_rate_hz, 0.0, SAMPLE_COUNT)
            .expect("GPS L1 C/A samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn gps_l2c_data_component_spectrum_matches_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::L2C)
        .expect("GPS L2C registry entry");
    let component =
        entry.component(SignalComponentRole::Data).copied().expect("GPS L2C data component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_gps_l2c_cm_code(1, sample_rate_hz, 0.0, SAMPLE_COUNT).expect("GPS L2C CM samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn gps_l2c_pilot_component_spectrum_matches_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Gps, SignalBand::L2, SignalCode::L2C)
        .expect("GPS L2C registry entry");
    let component =
        entry.component(SignalComponentRole::Pilot).copied().expect("GPS L2C pilot component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_gps_l2c_cl_code(1, sample_rate_hz, 0.0, SAMPLE_COUNT).expect("GPS L2C CL samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn glonass_l1_spectrum_matches_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
        .expect("GLONASS L1 registry entry");
    let component = entry.default_component().copied().expect("GLONASS L1 component");
    let sample_rate_hz = oversampled_chip_rate(component.primary_code_rate_hz);
    let samples =
        sample_glonass_l1_st_code(sample_rate_hz, 0.0, SAMPLE_COUNT).expect("GLONASS L1 samples");

    let comparison = compare_component_spectrum(
        &samples,
        sample_rate_hz,
        component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn beidou_open_service_spectra_match_expected_bpsk_shape() {
    let b1_entry = signal_registry(Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
        .expect("BeiDou B1I registry entry");
    let b1_component = b1_entry.default_component().copied().expect("BeiDou B1I component");
    let b1_sample_rate_hz = oversampled_chip_rate(b1_component.primary_code_rate_hz);
    let b1_samples = sample_beidou_b1i_code(1, b1_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("BeiDou B1I samples");
    let b1_comparison = compare_component_spectrum(
        &b1_samples,
        b1_sample_rate_hz,
        b1_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&b1_comparison, SpectrumTolerance::tight_bpsk(), 2);

    let b2_entry = signal_registry(Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
        .expect("BeiDou B2I registry entry");
    let b2_component = b2_entry.default_component().copied().expect("BeiDou B2I component");
    let b2_sample_rate_hz = oversampled_chip_rate(b2_component.primary_code_rate_hz);
    let b2_samples = sample_beidou_b2i_code(1, b2_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("BeiDou B2I samples");
    let b2_comparison = compare_component_spectrum(
        &b2_samples,
        b2_sample_rate_hz,
        b2_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&b2_comparison, SpectrumTolerance::tight_bpsk(), 2);
}
