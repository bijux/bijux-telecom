#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{
        measure_synthetic_acquisition_uncertainty_coverage,
        SyntheticAcquisitionUncertaintyCoverageCase, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
fn stationary_acquisition_uncertainty_coverage_excludes_ambiguous_trials_from_rates() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        acquisition_peak_second_threshold: 1.01,
        ..ReceiverPipelineConfig::default()
    };
    let report = measure_synthetic_acquisition_uncertainty_coverage(
        &config,
        &[SyntheticAcquisitionUncertaintyCoverageCase {
            signal: SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Ca,
                doppler_hz: 375.0,
                code_phase_chips: 200.375,
                carrier_phase_rad: 0.0,
                cn0_db_hz: 70.0,
                navigation_data: false.into(),
            },
            coherent_ms: 1,
            noncoherent: 1,
            duration_s: 0.001,
            doppler_rate_hz_per_s: None,
        }],
        24,
    );
    let point = &report.points[0];

    assert!(point.successful_trial_count >= 16, "{report:?}");
    assert!(point.successful_trial_count < point.trial_count, "{report:?}");
    assert!(
        point.trials.iter().any(|trial| trial.reported_doppler_sigma_hz.is_some()),
        "{report:?}"
    );
    assert!(
        point.trials.iter().any(|trial| trial.reported_doppler_sigma_hz.is_none()),
        "{report:?}"
    );
    assert!(
        point.trials.iter().any(|trial| trial.doppler_within_one_sigma == Some(false)),
        "{report:?}"
    );
    assert!(
        point.trials.iter().any(|trial| trial.code_phase_within_one_sigma == Some(true)),
        "{report:?}"
    );
    for trial in point.trials.iter().filter(|trial| trial.reported_doppler_sigma_hz.is_some()) {
        assert!(trial
            .reported_doppler_sigma_hz
            .is_some_and(|sigma| sigma.is_finite() && sigma > 0.0));
        assert!(trial
            .reported_code_phase_sigma_samples
            .is_some_and(|sigma| sigma.is_finite() && sigma > 0.0));
        assert_eq!(trial.reported_doppler_rate_sigma_hz_per_s, None);
        assert!(trial.doppler_within_one_sigma.is_some());
        assert!(trial.code_phase_within_one_sigma.is_some());
    }
}

#[test]
fn doppler_rate_acquisition_uncertainty_coverage_stays_conservative_on_grid() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 250,
        acquisition_doppler_rate_search_hz_per_s: 25_000,
        acquisition_doppler_rate_step_hz_per_s: 5_000,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        acquisition_peak_mean_threshold: 8.0,
        acquisition_peak_second_threshold: 1.01,
        ..ReceiverPipelineConfig::default()
    };
    let report = measure_synthetic_acquisition_uncertainty_coverage(
        &config,
        &[SyntheticAcquisitionUncertaintyCoverageCase {
            signal: SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Gps, prn: 16 },
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Ca,
                doppler_hz: 250.0,
                code_phase_chips: 211.25,
                carrier_phase_rad: 0.40,
                cn0_db_hz: 75.0,
                navigation_data: false.into(),
            },
            coherent_ms: 20,
            noncoherent: 1,
            duration_s: 0.020,
            doppler_rate_hz_per_s: Some(20_000.0),
        }],
        8,
    );
    let point = &report.points[0];

    assert_eq!(point.successful_trial_count, point.trial_count, "{report:?}");
    assert_eq!(point.doppler_within_one_sigma_rate, 1.0, "{report:?}");
    assert_eq!(point.code_phase_within_one_sigma_rate, 1.0, "{report:?}");
    let doppler_rate_within_one_sigma_rate =
        point.doppler_rate_within_one_sigma_rate.expect("doppler-rate coverage rate");
    assert_eq!(doppler_rate_within_one_sigma_rate, 1.0, "{report:?}");
}
