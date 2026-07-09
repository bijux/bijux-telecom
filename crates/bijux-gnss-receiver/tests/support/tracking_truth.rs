#![allow(missing_docs)]

use bijux_gnss_receiver::api::ReceiverPipelineConfig;
use bijux_gnss_signal::api::samples_per_code;

pub fn wrapped_code_phase_error_samples(
    config: &ReceiverPipelineConfig,
    measured_code_phase_samples: f64,
    expected_code_phase_samples: f64,
) -> f64 {
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            .max(1) as f64;
    let wrapped_error =
        (measured_code_phase_samples - expected_code_phase_samples).rem_euclid(period_samples);
    if wrapped_error > period_samples / 2.0 {
        period_samples - wrapped_error
    } else {
        wrapped_error
    }
}

#[cfg(test)]
mod tests {
    use super::wrapped_code_phase_error_samples;
    use bijux_gnss_receiver::api::ReceiverPipelineConfig;

    #[test]
    fn wrapped_code_phase_error_prefers_shortest_distance() {
        let config = ReceiverPipelineConfig {
            sampling_freq_hz: 1_023_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            ..ReceiverPipelineConfig::default()
        };

        let error = wrapped_code_phase_error_samples(&config, 1_020.0, 2.0);

        assert_eq!(error, 5.0);
    }
}
