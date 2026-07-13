mod support;

use bijux_gnss_core::api::{Constellation, SignalBand, SignalCode, SignalComponentRole};
use bijux_gnss_signal::api::{
    sample_galileo_e5a_i_primary_code, sample_galileo_e5a_q_primary_code,
    sample_galileo_e5b_i_primary_code, sample_galileo_e5b_q_primary_code,
    sample_gps_l5_i_primary_code, sample_gps_l5_q_primary_code, signal_registry,
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
fn gps_l5_component_spectra_match_expected_bpsk_shape() {
    let l5i_entry = signal_registry(Constellation::Gps, SignalBand::L5, SignalCode::L5I)
        .expect("GPS L5I registry entry");
    let l5i_component = l5i_entry.default_component().copied().expect("GPS L5I component");
    let l5i_sample_rate_hz = oversampled_chip_rate(l5i_component.primary_code_rate_hz);
    let l5i_samples = sample_gps_l5_i_primary_code(1, l5i_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("GPS L5I primary samples");
    let l5i_comparison = compare_component_spectrum(
        &l5i_samples,
        l5i_sample_rate_hz,
        l5i_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&l5i_comparison, SpectrumTolerance::tight_bpsk(), 2);

    let l5q_entry = signal_registry(Constellation::Gps, SignalBand::L5, SignalCode::L5Q)
        .expect("GPS L5Q registry entry");
    let l5q_component = l5q_entry.default_component().copied().expect("GPS L5Q component");
    let l5q_sample_rate_hz = oversampled_chip_rate(l5q_component.primary_code_rate_hz);
    let l5q_samples = sample_gps_l5_q_primary_code(1, l5q_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("GPS L5Q primary samples");
    let l5q_comparison = compare_component_spectrum(
        &l5q_samples,
        l5q_sample_rate_hz,
        l5q_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&l5q_comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn galileo_e5a_component_spectra_match_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
        .expect("Galileo E5a registry entry");

    let data_component =
        entry.component(SignalComponentRole::Data).copied().expect("Galileo E5a data component");
    let data_sample_rate_hz = oversampled_chip_rate(data_component.primary_code_rate_hz);
    let data_samples = sample_galileo_e5a_i_primary_code(1, data_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("Galileo E5a-I samples");
    let data_comparison = compare_component_spectrum(
        &data_samples,
        data_sample_rate_hz,
        data_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&data_comparison, SpectrumTolerance::tight_bpsk(), 2);

    let pilot_component =
        entry.component(SignalComponentRole::Pilot).copied().expect("Galileo E5a pilot component");
    let pilot_sample_rate_hz = oversampled_chip_rate(pilot_component.primary_code_rate_hz);
    let pilot_samples =
        sample_galileo_e5a_q_primary_code(1, pilot_sample_rate_hz, 0.0, SAMPLE_COUNT)
            .expect("Galileo E5a-Q samples");
    let pilot_comparison = compare_component_spectrum(
        &pilot_samples,
        pilot_sample_rate_hz,
        pilot_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&pilot_comparison, SpectrumTolerance::tight_bpsk(), 2);
}

#[test]
fn galileo_e5b_component_spectra_match_expected_bpsk_shape() {
    let entry = signal_registry(Constellation::Galileo, SignalBand::E5, SignalCode::E5b)
        .expect("Galileo E5b registry entry");

    let data_component =
        entry.component(SignalComponentRole::Data).copied().expect("Galileo E5b data component");
    let data_sample_rate_hz = oversampled_chip_rate(data_component.primary_code_rate_hz);
    let data_samples = sample_galileo_e5b_i_primary_code(1, data_sample_rate_hz, 0.0, SAMPLE_COUNT)
        .expect("Galileo E5b-I samples");
    let data_comparison = compare_component_spectrum(
        &data_samples,
        data_sample_rate_hz,
        data_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&data_comparison, SpectrumTolerance::tight_bpsk(), 2);

    let pilot_component =
        entry.component(SignalComponentRole::Pilot).copied().expect("Galileo E5b pilot component");
    let pilot_sample_rate_hz = oversampled_chip_rate(pilot_component.primary_code_rate_hz);
    let pilot_samples =
        sample_galileo_e5b_q_primary_code(1, pilot_sample_rate_hz, 0.0, SAMPLE_COUNT)
            .expect("Galileo E5b-Q samples");
    let pilot_comparison = compare_component_spectrum(
        &pilot_samples,
        pilot_sample_rate_hz,
        pilot_component,
        default_estimator_config(),
        OCCUPIED_POWER_FRACTION,
        NULL_THRESHOLD_DB,
    );
    assert_spectrum_matches(&pilot_comparison, SpectrumTolerance::tight_bpsk(), 2);
}
