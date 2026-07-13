#![allow(missing_docs)]

use bijux_gnss_core::api::{AcqHypothesis, Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

#[test]
fn acquisition_searches_first_and_last_code_phase_bins_at_configured_sample_rates() {
    for sampling_freq_hz in [4_092_000.0, 4_000_000.0] {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            ..ReceiverPipelineConfig::default()
        };
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let acquisition =
            AcquisitionEngine::new(config.clone(), ReceiverRuntime::default()).with_doppler(0, 500);

        for expected_code_phase_sample in [0, samples_per_code - 1] {
            let expected_reported_code_phase_sample =
                reported_code_phase_sample(expected_code_phase_sample, samples_per_code);
            let frame = generate_l1_ca(
                &config,
                SyntheticSignalParams {
                    sat,
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: 0.0,
                    code_phase_chips: code_phase_chips_for_sample(
                        &config,
                        expected_code_phase_sample,
                    ),
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: 60.0,
                    data_bit_flip: false,
                },
                0xC0DE_7000 + expected_code_phase_sample as u64,
                samples_per_code as f64 / config.sampling_freq_hz,
            );

            let result = acquisition.run_fft(&frame, &[sat]).remove(0);

            assert!(
                matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
                "expected trackable acquisition at {sampling_freq_hz} Hz for sample {expected_code_phase_sample}, got {}",
                result.hypothesis
            );
            assert!(
                wrapped_sample_distance(
                    result.code_phase_samples,
                    expected_reported_code_phase_sample,
                    samples_per_code,
                ) <= 1,
                "expected full-code search to recover reported sample {expected_reported_code_phase_sample} for injected sample {expected_code_phase_sample} at {sampling_freq_hz} Hz, got {}",
                result.code_phase_samples
            );
            assert!(result.peak_mean_ratio.is_finite());
            assert!(result.peak_mean_ratio > 2.0);

            let assumptions = result.assumptions.as_ref().expect("acquisition assumptions");
            assert_eq!(assumptions.code_phase_search_start_sample, 0);
            assert_eq!(assumptions.code_phase_search_step_samples, 1);
            assert_eq!(assumptions.code_phase_search_bins, samples_per_code);
            assert_eq!(assumptions.code_phase_search_mode, "full_code");
        }
    }
}

fn code_phase_chips_for_sample(config: &ReceiverPipelineConfig, sample_index: usize) -> f64 {
    sample_index as f64 * config.code_freq_basis_hz / config.sampling_freq_hz
}

fn reported_code_phase_sample(injected_sample: usize, period_samples: usize) -> usize {
    (period_samples - injected_sample) % period_samples.max(1)
}

fn wrapped_sample_distance(actual: usize, expected: usize, period_samples: usize) -> usize {
    let forward = actual.abs_diff(expected);
    let wrapped = period_samples.saturating_sub(forward);
    forward.min(wrapped)
}
